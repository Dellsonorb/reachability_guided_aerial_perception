"""Optimistic ground endpoint visibility through known occupied-prism geometry.

No ray carving, measured obstacle height, scan pattern or return probability.
"""

from dataclasses import dataclass

import numpy as np

from environment_belief import EnvironmentState
from .model import NBVConfig, SensorModel, readonly, rotation_z


def sensor_transform(viewpoint, sensor=SensorModel()):
    transform = np.eye(4)
    transform[:3, :3] = rotation_z(viewpoint.yaw_rad)
    transform[:3, 3] = viewpoint.position_xyz
    return transform @ sensor.T_uav_lidar


def ground_targets(belief):
    rows, cols = np.indices(belief.grid.shape)
    return np.column_stack((belief.origin_xy[0] + (cols.ravel() + .5) * belief.resolution_m,
                            belief.origin_xy[1] + (rows.ravel() + .5) * belief.resolution_m,
                            np.full(rows.size, belief.config.ground_z_m)))


def occupied_boxes(belief, assumed_height_m):
    rows, cols = np.nonzero(belief.state == EnvironmentState.OCCUPIED)
    lower = np.column_stack((belief.origin_xy[0] + cols * belief.resolution_m,
                             belief.origin_xy[1] + rows * belief.resolution_m,
                             np.full(rows.size, belief.config.ground_z_m)))
    return lower, lower + [belief.resolution_m, belief.resolution_m, assumed_height_m]


def segments_intersect_box(start, ends, lower, upper):
    """Closed segment / closed AABB slab test, vectorized over segment endpoints."""
    direction = np.asarray(ends) - start
    enter, leave = np.zeros(len(direction)), np.ones(len(direction))
    possible = np.ones(len(direction), dtype=bool)
    for axis in range(3):
        parallel = np.abs(direction[:, axis]) < 1e-12
        possible &= ~parallel | ((start[axis] >= lower[axis]) & (start[axis] <= upper[axis]))
        moving = ~parallel
        first = (lower[axis] - start[axis]) / direction[moving, axis]
        second = (upper[axis] - start[axis]) / direction[moving, axis]
        enter[moving] = np.maximum(enter[moving], np.minimum(first, second))
        leave[moving] = np.minimum(leave[moving], np.maximum(first, second))
    return possible & (enter <= leave + 1e-12)


@dataclass(frozen=True, eq=False)
class VisibilityPrediction:
    status: str
    sensor_xyz: tuple[float, float, float]
    visible: np.ndarray
    range_fov: np.ndarray
    occluded: np.ndarray


def predict_visibility(belief, viewpoint, *, sensor=SensorModel(), config=NBVConfig()):
    """An idealized NEW informative-endpoint opportunity for each visible cell.

    Only known OCCUPIED cells occlude, using assumed height, not A2-measured
    height. Candidate rejection concerns the sensor origin, NOT a safe path.
    """
    transform = sensor_transform(viewpoint, sensor)
    origin = transform[:3, 3]
    targets = ground_targets(belief)
    lower, upper = occupied_boxes(belief, config.assumed_height_m)
    status = 'VALID'
    if origin[2] <= belief.config.ground_z_m:
        status = 'SENSOR_AT_OR_BELOW_GROUND'
    elif np.any(np.all((origin >= lower) & (origin <= upper), axis=1)):
        status = 'SENSOR_INSIDE_ASSUMED_PRISM'
    visible = np.zeros(len(targets), dtype=bool)
    range_fov, occluded = visible.copy(), visible.copy()
    if status == 'VALID':
        points_sensor = (targets - origin) @ transform[:3, :3]
        distance = np.linalg.norm(points_sensor, axis=1)
        elevation = np.degrees(np.arctan2(points_sensor[:, 2], np.hypot(points_sensor[:, 0], points_sensor[:, 1])))
        range_fov = ((distance > belief.config.min_range_m) & (distance < belief.config.max_range_m)
                     & (elevation >= sensor.min_elevation_deg) & (elevation <= sensor.max_elevation_deg)
                     & (belief.state.ravel() != EnvironmentState.OCCUPIED))
        eligible = np.flatnonzero(range_fov)
        for lo, hi in zip(lower, upper):
            occluded[eligible] |= segments_intersect_box(origin, targets[eligible], lo, hi)
        visible = range_fov & ~occluded
    return VisibilityPrediction(status, tuple(float(x) for x in origin),
                                *(readonly(a.reshape(belief.grid.shape)) for a in (visible, range_fov, occluded)))
