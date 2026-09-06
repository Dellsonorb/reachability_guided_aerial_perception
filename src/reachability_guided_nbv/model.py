"""Small, ROS-independent level-hover and sensor geometry definitions."""

from dataclasses import dataclass, field
from numbers import Integral

import numpy as np


def readonly(value):
    result = np.array(value, copy=True)
    result.setflags(write=False)
    return result


def wrap_yaw(angle):
    return float((angle + np.pi) % (2 * np.pi) - np.pi)


def rotation_z(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def mid360_extrinsic():
    """Active P450 TF configuration, not a live TF measurement.

    base_link -> model mount: xyz=(.13,0,.23), pitch=.35.
    model mount -> ray sensor: xyz=(0,0,.05), identity rotation.
    """
    c, s = np.cos(.35), np.sin(.35)
    transform = np.eye(4)
    transform[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    transform[:3, 3] = [.13, 0, .23] + transform[:3, :3] @ [0, 0, .05]
    return transform


@dataclass(frozen=True)
class Viewpoint:
    """UAV base_link position in map, with roll=pitch=0; not a flight command."""

    position_xyz: tuple[float, float, float]
    yaw_rad: float
    frame_id: str = 'map'

    def __post_init__(self):
        xyz = np.asarray(self.position_xyz, dtype=float)
        if xyz.shape != (3,) or not np.all(np.isfinite(xyz)):
            raise ValueError('position_xyz must contain three finite coordinates')
        if not np.isscalar(self.yaw_rad) or not np.isfinite(self.yaw_rad):
            raise ValueError('yaw_rad must be finite')
        if self.frame_id != 'map':
            raise ValueError('viewpoint frame_id must be map')
        object.__setattr__(self, 'position_xyz', tuple(float(x) for x in xyz))
        object.__setattr__(self, 'yaw_rad', wrap_yaw(self.yaw_rad))


@dataclass(frozen=True, eq=False)
class SensorModel:
    """360-degree horizontal FOV; elevation measured in sensor XYZ coordinates."""

    T_uav_lidar: np.ndarray = field(default_factory=mid360_extrinsic)
    min_elevation_deg: float = -7.0
    max_elevation_deg: float = 52.0

    def __post_init__(self):
        transform = np.asarray(self.T_uav_lidar, dtype=float)
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            raise ValueError('T_uav_lidar must be a finite 4x4 rigid transform')
        rotation = transform[:3, :3]
        if (not np.allclose(transform[3], [0, 0, 0, 1], rtol=0, atol=1e-8)
                or not np.allclose(rotation.T @ rotation, np.eye(3), rtol=0, atol=1e-6)
                or not np.isclose(np.linalg.det(rotation), 1, rtol=0, atol=1e-6)):
            raise ValueError('T_uav_lidar must contain a proper rigid rotation')
        if (not np.isfinite(self.min_elevation_deg) or not np.isfinite(self.max_elevation_deg)
                or not -90 <= self.min_elevation_deg < self.max_elevation_deg <= 90):
            raise ValueError('require -90 <= min elevation < max elevation <= 90')
        object.__setattr__(self, 'T_uav_lidar', readonly(transform))

    @property
    def yaw_equivalent(self):
        # An XY lever arm also moves the sensing origin when the UAV rotates.
        return bool(np.allclose(self.T_uav_lidar[:2, 2], 0, rtol=0, atol=1e-12)
                    and np.allclose(self.T_uav_lidar[:2, 3], 0, rtol=0, atol=1e-12))


@dataclass(frozen=True)
class NBVConfig:
    xy_offsets_m: tuple[float, ...] = (-4, -2, 0, 2, 4)
    yaw_samples: int = 8
    assumed_height_m: float = 1.0
    flight_weight: float = .25
    yaw_cost_m_per_rad: float = .25

    def __post_init__(self):
        offsets = np.asarray(self.xy_offsets_m, dtype=float)
        if offsets.ndim != 1 or not offsets.size or not np.all(np.isfinite(offsets)):
            raise ValueError('xy_offsets_m must be a nonempty finite sequence')
        if isinstance(self.yaw_samples, bool) or not isinstance(self.yaw_samples, Integral) or self.yaw_samples < 1:
            raise ValueError('yaw_samples must be a positive integer')
        for name in ('assumed_height_m', 'flight_weight', 'yaw_cost_m_per_rad'):
            value = getattr(self, name)
            if not np.isfinite(value) or value < 0 or (name == 'assumed_height_m' and value == 0):
                raise ValueError(f'invalid {name}')
        object.__setattr__(self, 'xy_offsets_m', tuple(float(x) for x in offsets))


def generate_candidates(current, *, sensor=SensorModel(), config=NBVConfig()):
    """Task-independent local XY lattice, fixed altitude, current pose first."""
    x, y, z = current.position_xyz
    headings = 1 if sensor.yaw_equivalent else config.yaw_samples
    candidates, seen = [current], {current}
    for dx in config.xy_offsets_m:
        for dy in config.xy_offsets_m:
            for k in range(headings):
                pose = Viewpoint((x + dx, y + dy, z), current.yaw_rad + 2 * np.pi * k / headings)
                if pose not in seen:
                    candidates.append(pose)
                    seen.add(pose)
    return tuple(candidates)
