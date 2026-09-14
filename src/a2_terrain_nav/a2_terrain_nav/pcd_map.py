"""Convert a FAST-LIO PCD map into a Nav2 occupancy image and YAML file."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO

import numpy as np


_PCD_DTYPES = {
    ('F', 4): '<f4',
    ('F', 8): '<f8',
    ('I', 1): 'i1',
    ('I', 2): '<i2',
    ('I', 4): '<i4',
    ('I', 8): '<i8',
    ('U', 1): 'u1',
    ('U', 2): '<u2',
    ('U', 4): '<u4',
    ('U', 8): '<u8',
}


@dataclass(frozen=True)
class ProjectionResult:
    yaml_path: Path
    image_path: Path
    point_count: int
    obstacle_point_count: int
    ground_height: float
    width: int
    height: int


def _read_header(stream: BinaryIO) -> tuple[dict[str, list[str]], str]:
    metadata: dict[str, list[str]] = {}
    while True:
        raw = stream.readline()
        if not raw:
            raise ValueError('PCD header is missing a DATA line')
        try:
            line = raw.decode('ascii').strip()
        except UnicodeDecodeError as exc:
            raise ValueError('PCD header is not ASCII') from exc
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        key = parts[0].upper()
        metadata[key] = parts[1:]
        if key == 'DATA':
            if not parts:
                raise ValueError('PCD DATA line has no storage type')
            return metadata, parts[1].lower()


def load_pcd_xyz(path: Path | str) -> np.ndarray:
    """Load XYZ from an ASCII or uncompressed-binary PCD file."""
    pcd_path = Path(path)
    with pcd_path.open('rb') as stream:
        metadata, storage = _read_header(stream)
        names = metadata.get('FIELDS', metadata.get('FIELD', []))
        sizes = [int(value) for value in metadata.get('SIZE', [])]
        types = metadata.get('TYPE', [])
        counts = [int(value) for value in metadata.get('COUNT', ['1'] * len(names))]
        if not names or not (len(names) == len(sizes) == len(types) == len(counts)):
            raise ValueError('PCD FIELDS/SIZE/TYPE/COUNT metadata is inconsistent')
        if not {'x', 'y', 'z'}.issubset(names):
            raise ValueError('PCD map must contain x, y and z fields')

        if storage == 'ascii':
            values = np.loadtxt(stream, dtype=np.float64, ndmin=2)
            offsets: dict[str, int] = {}
            offset = 0
            for name, count in zip(names, counts):
                offsets[name] = offset
                offset += count
            xyz = np.column_stack([values[:, offsets[name]] for name in ('x', 'y', 'z')])
        elif storage == 'binary':
            fields = []
            for name, size, type_code, count in zip(names, sizes, types, counts):
                try:
                    scalar_dtype = _PCD_DTYPES[(type_code.upper(), size)]
                except KeyError as exc:
                    raise ValueError(f'Unsupported PCD field type {type_code}{size}') from exc
                fields.append((name, scalar_dtype, (count,)) if count > 1 else (name, scalar_dtype))
            point_count = int(metadata.get('POINTS', metadata.get('WIDTH', ['0']))[0])
            cloud = np.fromfile(stream, dtype=np.dtype(fields), count=point_count)
            xyz = np.column_stack([cloud[name] for name in ('x', 'y', 'z')]).astype(
                np.float64, copy=False)
        elif storage == 'binary_compressed':
            raise ValueError('binary_compressed PCD is not supported; save an ASCII or binary map')
        else:
            raise ValueError(f'Unsupported PCD DATA type: {storage}')

    finite = np.isfinite(xyz).all(axis=1)
    return xyz[finite]


def estimate_ground_height(z_values: np.ndarray, bin_size: float = 0.04) -> float:
    """Estimate the dominant low surface without assuming map-frame Z is zero."""
    finite = np.asarray(z_values, dtype=np.float64)
    finite = finite[np.isfinite(finite)]
    if finite.size < 10:
        raise ValueError('At least 10 finite points are required to estimate ground height')
    low_surface = finite[finite <= np.quantile(finite, 0.60)]
    low = float(low_surface.min())
    high = float(low_surface.max())
    if high - low < bin_size:
        return float(np.median(low_surface))
    edges = np.arange(low, high + 2.0 * bin_size, bin_size)
    histogram, edges = np.histogram(low_surface, bins=edges)
    peak = int(np.argmax(histogram))
    in_peak = low_surface[(low_surface >= edges[peak]) & (low_surface < edges[peak + 1])]
    return float(np.median(in_peak))


def project_pcd_to_nav2(
    input_path: Path | str,
    output_yaml: Path | str,
    *,
    resolution: float = 0.05,
    min_obstacle_height: float = 0.12,
    max_obstacle_height: float = 1.50,
    margin: float = 1.0,
    minimum_points_per_cell: int = 1,
    ground_height: float | None = None,
) -> ProjectionResult:
    """Project a 3D map into the static 2D map used by Nav2."""
    if resolution <= 0.0:
        raise ValueError('resolution must be positive')
    if margin < 0.0:
        raise ValueError('margin must be non-negative')
    if min_obstacle_height < 0.0 or max_obstacle_height <= min_obstacle_height:
        raise ValueError('obstacle height limits are invalid')
    if minimum_points_per_cell < 1:
        raise ValueError('minimum_points_per_cell must be at least one')

    xyz = load_pcd_xyz(input_path)
    if xyz.shape[0] < 10:
        raise ValueError('PCD map contains too few finite points')
    floor_z = estimate_ground_height(xyz[:, 2]) if ground_height is None else ground_height
    relative_z = xyz[:, 2] - floor_z
    obstacle = xyz[
        (relative_z >= min_obstacle_height) & (relative_z <= max_obstacle_height)
    ]

    # Long-range floor returns can be far beyond the actually mapped room. Use
    # obstacle-bearing points for the static-map extent so those returns do not
    # create a huge, falsely-free planning area.
    extent_points = obstacle if obstacle.shape[0] >= 10 else xyz
    min_x = float(np.floor((extent_points[:, 0].min() - margin) / resolution) * resolution)
    min_y = float(np.floor((extent_points[:, 1].min() - margin) / resolution) * resolution)
    max_x = float(np.ceil((extent_points[:, 0].max() + margin) / resolution) * resolution)
    max_y = float(np.ceil((extent_points[:, 1].max() + margin) / resolution) * resolution)
    width = int(round((max_x - min_x) / resolution)) + 1
    height = int(round((max_y - min_y) / resolution)) + 1
    if width < 3 or height < 3 or width * height > 100_000_000:
        raise ValueError(f'Invalid output grid dimensions: {width} x {height}')

    counts = np.zeros((height, width), dtype=np.uint16)
    if obstacle.size:
        cols = np.floor((obstacle[:, 0] - min_x) / resolution).astype(np.int64)
        map_rows = np.floor((obstacle[:, 1] - min_y) / resolution).astype(np.int64)
        valid = (cols >= 0) & (cols < width) & (map_rows >= 0) & (map_rows < height)
        image_rows = height - 1 - map_rows[valid]
        np.add.at(counts, (image_rows, cols[valid]), 1)

    image = np.full((height, width), 254, dtype=np.uint8)
    image[counts >= minimum_points_per_cell] = 0
    # The PCD has no ray-origin history, so keep the rectangular map boundary closed.
    image[[0, -1], :] = 0
    image[:, [0, -1]] = 0

    yaml_path = Path(output_yaml)
    image_path = yaml_path.with_suffix('.pgm')
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    with image_path.open('wb') as stream:
        stream.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
        stream.write(image.tobytes(order='C'))
    yaml_path.write_text(
        '\n'.join([
            f'image: {image_path.name}',
            'mode: trinary',
            f'resolution: {resolution:.6f}',
            f'origin: [{min_x:.6f}, {min_y:.6f}, 0.0]',
            'negate: 0',
            'occupied_thresh: 0.65',
            'free_thresh: 0.196',
            '',
        ]),
        encoding='utf-8',
    )
    return ProjectionResult(
        yaml_path=yaml_path,
        image_path=image_path,
        point_count=int(xyz.shape[0]),
        obstacle_point_count=int(obstacle.shape[0]),
        ground_height=float(floor_z),
        width=width,
        height=height,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path, help='FAST-LIO PCD map')
    parser.add_argument('--output', required=True, type=Path, help='Output Nav2 YAML path')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--min-obstacle-height', type=float, default=0.12)
    parser.add_argument('--max-obstacle-height', type=float, default=1.50)
    parser.add_argument('--margin', type=float, default=1.0)
    parser.add_argument('--minimum-points-per-cell', type=int, default=1)
    parser.add_argument('--ground-height', type=float)
    args = parser.parse_args(argv)
    result = project_pcd_to_nav2(
        args.input,
        args.output,
        resolution=args.resolution,
        min_obstacle_height=args.min_obstacle_height,
        max_obstacle_height=args.max_obstacle_height,
        margin=args.margin,
        minimum_points_per_cell=args.minimum_points_per_cell,
        ground_height=args.ground_height,
    )
    print(
        f'generated {result.yaml_path} and {result.image_path}: '
        f'{result.width}x{result.height}, ground_z={result.ground_height:.3f}, '
        f'obstacles={result.obstacle_point_count}/{result.point_count}'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
