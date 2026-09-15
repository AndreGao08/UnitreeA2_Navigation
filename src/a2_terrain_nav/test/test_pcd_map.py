from pathlib import Path

import numpy as np

from a2_terrain_nav.pcd_map import estimate_ground_height, load_pcd_xyz, project_pcd_to_nav2


def _write_ascii_pcd(path: Path, points: np.ndarray) -> None:
    header = '\n'.join([
        '# .PCD v0.7',
        'VERSION 0.7',
        'FIELDS x y z',
        'SIZE 4 4 4',
        'TYPE F F F',
        'COUNT 1 1 1',
        f'WIDTH {len(points)}',
        'HEIGHT 1',
        f'POINTS {len(points)}',
        'DATA ascii',
    ])
    np.savetxt(path, points, header=header, comments='', fmt='%.6f')


def test_estimate_ground_height_is_offset_invariant():
    floor = np.full(100, 0.37)
    obstacles = np.linspace(0.7, 1.8, 40)
    assert abs(estimate_ground_height(np.concatenate([floor, obstacles])) - 0.37) < 0.03


def test_ascii_pcd_projection(tmp_path):
    floor = np.array([[x, y, -0.4] for x in (-1.0, 0.0, 1.0) for y in (-1.0, 0.0, 1.0)])
    floor = np.repeat(floor, 3, axis=0)
    wall = np.array([[0.5, y, 0.2] for y in np.linspace(-0.8, 0.8, 20)])
    pcd = tmp_path / 'map.pcd'
    output = tmp_path / 'map.yaml'
    _write_ascii_pcd(pcd, np.vstack([floor, wall]))

    loaded = load_pcd_xyz(pcd)
    result = project_pcd_to_nav2(
        pcd,
        output,
        resolution=0.1,
        minimum_points_per_cell=1,
    )

    assert loaded.shape == (47, 3)
    assert result.obstacle_point_count == 20
    assert result.image_path.exists()
    assert 'resolution: 0.100000' in output.read_text(encoding='utf-8')


def test_projection_rejects_sparse_body_ghosts(tmp_path):
    floor = np.repeat(
        np.array([[x, y, -0.4] for x in (-1.0, 0.0, 1.0) for y in (-1.0, 0.0, 1.0)]),
        3,
        axis=0,
    )
    sparse_ghost = np.array([[0.0, 0.0, 0.2]])
    # Ten obstacle-class returns also make the projector use obstacle extents,
    # keeping this synthetic grid compact and its origin deterministic.
    persistent_obstacle = np.repeat(np.array([[0.5, 0.0, 0.2]]), 9, axis=0)
    pcd = tmp_path / 'map.pcd'
    output = tmp_path / 'map.yaml'
    _write_ascii_pcd(pcd, np.vstack([floor, sparse_ghost, persistent_obstacle]))

    result = project_pcd_to_nav2(pcd, output, resolution=0.1, margin=0.1)

    with result.image_path.open('rb') as stream:
        assert stream.readline().strip() == b'P5'
        dimensions = stream.readline().split()
        width, height = int(dimensions[0]), int(dimensions[1])
        assert int(stream.readline()) == 255
        image = np.frombuffer(stream.read(), dtype=np.uint8).reshape(height, width)

    ghost_col = int(np.floor((0.0 - (-0.1)) / 0.1))
    obstacle_col = int(np.floor((0.5 - (-0.1)) / 0.1))
    row = height - 1 - int(np.floor((0.0 - (-0.1)) / 0.1))
    assert image[row, ghost_col] == 254
    assert image[row, obstacle_col] == 0
