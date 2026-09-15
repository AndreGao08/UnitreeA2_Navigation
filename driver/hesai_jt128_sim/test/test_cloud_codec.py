import math

import numpy as np
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from hesai_jt128_sim.cloud_codec import JT128_FIELDS, convert_cloud


def _organized_cloud(lines=128, columns=8):
    header = Header()
    header.stamp.sec = 12
    header.stamp.nanosec = 500_000_000
    header.frame_id = 'raw'
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    points = []
    for ring in range(lines):
        elevation = -0.4 + ring * 0.6 / (lines - 1)
        for column in range(columns):
            azimuth = -math.pi + column * 2.0 * math.pi / columns
            r = 5.0
            points.append((r * math.cos(elevation) * math.cos(azimuth),
                           r * math.cos(elevation) * math.sin(azimuth),
                           r * math.sin(elevation), float(ring)))
    msg = point_cloud2.create_cloud(header, fields, points)
    msg.height = lines
    msg.width = columns
    msg.row_step = msg.point_step * columns
    return msg


def test_conversion_has_exact_fast_lio_contract():
    converted = convert_cloud(_organized_cloud())
    assert tuple(field.name for field in converted.fields) == JT128_FIELDS
    assert converted.header.frame_id == 'hesai_jt128_link'
    assert converted.height == 1
    assert converted.width == 128 * 8
    assert converted.point_step == 32

    points = point_cloud2.read_points(converted, skip_nans=False)
    assert int(points['ring'].min()) == 0
    assert int(points['ring'].max()) == 127
    assert points['timestamp'][0] == 12.5
    assert points['timestamp'][-1] == 12.5


def test_rolling_scan_timestamps_are_opt_in():
    converted = convert_cloud(_organized_cloud(), rolling_scan_timestamps=True)
    points = point_cloud2.read_points(converted, skip_nans=False)
    assert np.all(np.diff(points['timestamp'].reshape(8, 128)[:, 0]) >= 0.0)
    assert points['timestamp'][0] == 12.5
    assert points['timestamp'][-1] < 12.6


def test_unorganized_cloud_infers_ring_and_monotonic_time():
    msg = _organized_cloud(lines=4, columns=16)
    msg.height = 1
    msg.width = 64
    msg.row_step = msg.point_step * msg.width
    converted = convert_cloud(msg, scan_lines=128)
    points = point_cloud2.read_points(converted, skip_nans=False)
    assert int(points['ring'].min()) >= 0
    assert int(points['ring'].max()) <= 127
    assert np.all(np.diff(points['timestamp']) >= 0.0)
