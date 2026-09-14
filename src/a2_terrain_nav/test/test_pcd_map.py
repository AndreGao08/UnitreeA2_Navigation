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
    result = project_pcd_to_nav2(pcd, output, resolution=0.1)

    assert loaded.shape == (47, 3)
    assert result.obstacle_point_count == 20
    assert result.image_path.exists()
    assert 'resolution: 0.100000' in output.read_text(encoding='utf-8')
