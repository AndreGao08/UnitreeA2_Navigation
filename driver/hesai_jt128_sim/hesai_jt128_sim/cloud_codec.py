"""Vectorized PointCloud2 conversion used by the JT128 simulation adapter."""

import array
from typing import Dict

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


JT128_FIELDS = ('x', 'y', 'z', 'intensity', 'ring', 'timestamp')


def stamp_seconds(msg: PointCloud2) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9


def _field_map(msg: PointCloud2) -> Dict[str, PointField]:
    return {field.name: field for field in msg.fields}


def _ordered_indices(msg: PointCloud2, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return point order and normalized firing phase in [0, 1).

    Gazebo GPU lidar clouds are normally organized as (vertical, horizontal).
    FAST-LIO expects the first point to be at the beginning of a scan and the
    last point near its end, so organized clouds are transposed into firing
    order: horizontal sample first, ring second.
    """
    count = x.size
    if msg.height > 1 and msg.width > 0 and msg.height * msg.width == count:
        order = np.arange(count, dtype=np.int64).reshape(msg.height, msg.width).T.reshape(-1)
        phase = np.repeat(np.arange(msg.width, dtype=np.float64) / float(msg.width), msg.height)
        return order, phase

    azimuth = np.arctan2(y, x)
    phase_raw = np.mod(azimuth + np.pi, 2.0 * np.pi) / (2.0 * np.pi)
    order = np.argsort(phase_raw, kind='stable')
    return order, phase_raw[order]


def convert_cloud(
    msg: PointCloud2,
    *,
    output_frame: str = 'hesai_jt128_link',
    scan_rate_hz: float = 10.0,
    scan_lines: int = 128,
    vertical_min_rad: float = -0.4363323129985824,
    vertical_max_rad: float = 0.2617993877991494,
    rolling_scan_timestamps: bool = False,
) -> PointCloud2:
    """Convert any XYZ PointCloud2 into the exact Hesai JT128 field layout.

    Existing intensity / ring / timestamp fields are preserved where possible.
    Missing ring is recovered from organized rows or elevation. Missing
    timestamp is set to the snapshot time. Set rolling_scan_timestamps only
    for a source that actually acquires columns sequentially.
    """
    if scan_rate_hz <= 0.0:
        raise ValueError('scan_rate_hz must be positive')
    if scan_lines <= 0:
        raise ValueError('scan_lines must be positive')

    names = _field_map(msg)
    missing_xyz = [name for name in ('x', 'y', 'z') if name not in names]
    if missing_xyz:
        raise ValueError(f'input cloud is missing fields: {missing_xyz}')

    # One structured NumPy view avoids decoding a 100k+ point cloud once per
    # field. This is important at the simulated JT128's 10 Hz frame rate.
    raw = point_cloud2.read_points(msg, skip_nans=False)
    x_raw = np.asarray(raw['x']).reshape(-1).astype(np.float32, copy=False)
    y_raw = np.asarray(raw['y']).reshape(-1).astype(np.float32, copy=False)
    z_raw = np.asarray(raw['z']).reshape(-1).astype(np.float32, copy=False)
    count = x_raw.size

    intensity_raw = (np.asarray(raw['intensity']).reshape(-1).astype(np.float32, copy=False)
                     if 'intensity' in names else np.zeros(count, dtype=np.float32))
    order, phase = _ordered_indices(msg, x_raw, y_raw)
    x = x_raw[order]
    y = y_raw[order]
    z = z_raw[order]
    intensity = intensity_raw[order]

    if 'ring' in names:
        ring = np.asarray(raw['ring']).reshape(-1)[order].astype(np.uint16, copy=False)
    elif msg.height == scan_lines and msg.width > 0 and msg.height * msg.width == count:
        ring = np.tile(np.arange(scan_lines, dtype=np.uint16), msg.width)
    else:
        elevation = np.arctan2(z, np.hypot(x, y))
        scale = (scan_lines - 1) / max(vertical_max_rad - vertical_min_rad, 1.0e-9)
        ring = np.rint((elevation - vertical_min_rad) * scale)
        ring = np.clip(ring, 0, scan_lines - 1).astype(np.uint16)

    timestamp_field = next((name for name in ('timestamp', 'time', 't') if name in names), None)
    if timestamp_field is not None:
        timestamp = np.asarray(raw[timestamp_field]).reshape(-1)[order].astype(np.float64, copy=False)
        if timestamp.size and np.nanmax(np.abs(timestamp)) < 1.0e6:
            timestamp = timestamp + stamp_seconds(msg)
    else:
        timestamp = np.full(x.size, stamp_seconds(msg), dtype=np.float64)
        if rolling_scan_timestamps:
            timestamp += phase / scan_rate_hz

    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & np.isfinite(timestamp)
    x, y, z = x[finite], y[finite], z[finite]
    intensity, ring, timestamp = intensity[finite], ring[finite], timestamp[finite]

    dtype = np.dtype({
        'names': list(JT128_FIELDS),
        'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f8'],
        'offsets': [0, 4, 8, 12, 16, 24],
        'itemsize': 32,
    })
    packed = np.empty(x.size, dtype=dtype)
    packed['x'], packed['y'], packed['z'] = x, y, z
    packed['intensity'], packed['ring'], packed['timestamp'] = intensity, ring, timestamp

    output = PointCloud2()
    output.header.stamp = msg.header.stamp
    output.header.frame_id = output_frame
    output.height = 1
    output.width = int(x.size)
    output.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
        PointField(name='timestamp', offset=24, datatype=PointField.FLOAT64, count=1),
    ]
    output.is_bigendian = False
    output.point_step = dtype.itemsize
    output.row_step = output.point_step * output.width
    output.is_dense = True
    # Assigning bytes invokes a generated per-byte ROS type check in Python.
    # array('B') takes the message's zero-copy fast path instead.
    output.data = array.array('B', packed.tobytes())
    return output
