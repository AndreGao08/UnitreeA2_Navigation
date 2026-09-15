import math
import re
from pathlib import Path
from xml.etree import ElementTree

import pytest

from a2_terrain_nav.localization_safety_monitor import localization_is_healthy


PROJECT_ROOT = Path(__file__).resolve().parents[3]
WORLD_FILES = (
    'a2_localization.sdf',
    'a2_localization_dynamic.sdf',
    'a2_localization_gait_demo.sdf',
)


def _model(root, name):
    return next(model for model in root.findall('.//model') if model.get('name') == name)


@pytest.mark.parametrize('world_name', WORLD_FILES)
def test_ramp_entry_is_flush_with_floor(world_name):
    root = ElementTree.parse(
        PROJECT_ROOT / 'driver' / 'a2_gazebo' / 'worlds' / world_name).getroot()
    floor = _model(root, 'ground_plane')
    ramp = _model(root, 'ramp')
    floor_size_z = float(floor.find('.//box/size').text.split()[2])
    ramp_size = [float(value) for value in ramp.find('.//box/size').text.split()]
    ramp_pose = [float(value) for value in ramp.find('pose').text.split()]
    pitch = ramp_pose[4]
    floor_top = floor_size_z / 2.0
    ramp_low_top = (
        ramp_pose[2]
        + ramp_size[2] * math.cos(pitch) / 2.0
        - ramp_size[0] * math.sin(pitch) / 2.0
    )
    assert ramp_low_top == pytest.approx(floor_top, abs=1.0e-3)


def test_gseg_contract_limits_traversable_ground_to_five_degrees():
    config = (PROJECT_ROOT / 'navigation' / 'a2_terrain_nav' / 'config' /
              'gseg3d_a2.yaml').read_text(encoding='utf-8')
    slope_limit = float(re.search(r'slopeThresholdDegrees:\s*([0-9.]+)', config).group(1))
    height_limit = float(re.search(r'maxGroundHeightDeviation:\s*([0-9.]+)', config).group(1))
    assert slope_limit == pytest.approx(5.0)
    assert 0.5 * math.tan(math.radians(slope_limit)) < height_limit < 0.20
    assert slope_limit < math.degrees(0.18)


def test_navigation_requires_both_localizers_to_be_healthy():
    assert localization_is_healthy('LOCALIZED', 'OK')
    assert not localization_is_healthy('LOST: registration failed', 'OK')
    assert not localization_is_healthy('LOCALIZED', 'LOST: odometry jump')


def test_navigation_uses_front_and_rear_lidar_with_180_degree_fov():
    combined_launch = (
        PROJECT_ROOT / 'navigation' / 'a2_terrain_nav' / 'launch' /
        'a2_terrain_navigation.launch.py'
    ).read_text(encoding='utf-8')
    perception_launch = (
        PROJECT_ROOT / 'navigation' / 'a2_terrain_nav' / 'launch' /
        'terrain_perception.launch.py'
    ).read_text(encoding='utf-8')

    assert "front_lidar_topic', default_value='/lidar_points'" in combined_launch
    assert "rear_lidar_topic', default_value='/lidar_points_2'" in combined_launch
    assert "'front_fov_deg': '180.0'" in combined_launch
    assert "'rear_fov_deg': '180.0'" in combined_launch
    assert "package='a2_dual_lidar_nav'" in perception_launch
    assert "default_value='/a2/navigation/lidar_points'" in perception_launch
    assert "('/ground_segmentation/input_pointcloud', navigation_pointcloud_topic)" in (
        perception_launch
    )
