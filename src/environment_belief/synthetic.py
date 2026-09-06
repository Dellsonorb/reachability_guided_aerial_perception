"""Small deterministic first-hit fixtures, not a MID360 scan-pattern emulator."""

from dataclasses import dataclass

import numpy as np

from .core import EnvironmentGridSpec, PointCloudObservation


@dataclass(frozen=True)
class Box:
    lower: tuple[float, float, float]
    upper: tuple[float, float, float]

    def __post_init__(self):
        lower, upper = np.asarray(self.lower, dtype=float), np.asarray(self.upper, dtype=float)
        if (lower.shape != (3,) or upper.shape != (3,)
                or not np.all(np.isfinite([lower, upper])) or not np.all(upper > lower)):
            raise ValueError('box requires finite lower < upper in three dimensions')


def sensor_pose(position, yaw=0.0, pitch=0.0):
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    transform = np.eye(4)
    transform[:3, :3] = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array(
        [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    transform[:3, 3] = position
    return transform


def _box_distance(origin, direction, box):
    """Distance to first positive slab intersection along a normalized ray."""
    enter, leave = -np.inf, np.inf
    for axis in range(3):
        if abs(direction[axis]) < 1e-12:
            if not box.lower[axis] <= origin[axis] <= box.upper[axis]:
                return np.inf
            continue
        a = (box.lower[axis] - origin[axis]) / direction[axis]
        b = (box.upper[axis] - origin[axis]) / direction[axis]
        enter, leave = max(enter, min(a, b)), min(leave, max(a, b))
        if enter > leave:
            return np.inf
    if enter > 1e-9:
        return enter
    return leave if leave > 1e-9 else np.inf


def cast_observation(targets_map, T_map_sensor, boxes=(), stamp_s=0.0,
                     ground_z_m=0.0, max_range_m=40.0):
    """Aim rays toward targets and retain only the nearest ground/box hit.

    Targets define directions, not range endpoints or belief labels. Missing
    hits are returned as zero XYZ with valid_return=False, like a filtered
    sensor adapter. All output XYZ is in the supplied sensor frame.
    """
    template = PointCloudObservation(np.empty((0, 3)), 'lidar', stamp_s, T_map_sensor)
    transform = template.T_map_sensor
    origin = transform[:3, 3]
    targets = np.asarray(targets_map, dtype=float)
    if targets.ndim != 2 or targets.shape[1] != 3 or not np.all(np.isfinite(targets)):
        raise ValueError('targets_map must have finite shape (N, 3)')
    sensor_points = np.zeros_like(targets)
    valid = np.zeros(len(targets), dtype=bool)
    for i, target in enumerate(targets):
        direction = target - origin
        length = np.linalg.norm(direction)
        if length == 0:
            raise ValueError('target must differ from sensor origin')
        direction /= length
        ground_distance = ((ground_z_m - origin[2]) / direction[2]
                           if abs(direction[2]) > 1e-12 else np.inf)
        distance = ground_distance if ground_distance > 1e-9 else np.inf
        for box in boxes:
            distance = min(distance, _box_distance(origin, direction, box))
        if distance < max_range_m:
            sensor_points[i] = (distance * direction) @ transform[:3, :3]
            valid[i] = True
    return PointCloudObservation(sensor_points, 'lidar', stamp_s, transform, valid)


@dataclass(frozen=True)
class SyntheticScenario:
    name: str
    grid: EnvironmentGridSpec
    boxes: tuple[Box, ...]
    observations: tuple[PointCloudObservation, ...]
    labels: tuple[str, ...]
    probe_cells: dict[str, tuple[int, int]]


def _targets(grid, rows, columns):
    x = grid.origin_xy[0] + (np.asarray(columns) + 0.5) * grid.resolution_m
    y = grid.origin_xy[1] + (np.asarray(rows) + 0.5) * grid.resolution_m
    xx, yy = np.meshgrid(x, y)
    return np.column_stack((xx.ravel(), yy.ravel(), np.zeros(xx.size)))


def make_scenarios():
    grid = EnvironmentGridSpec((-1.5, -1.5), 30, 30)
    clear_targets = _targets(grid, range(3, 27), range(3, 27))
    clear = SyntheticScenario(
        'clear', grid, (),
        (cast_observation(clear_targets, sensor_pose((-1.35, -0.85, 1.5), 0.3, 0.2), stamp_s=0),
         cast_observation(clear_targets, sensor_pose((1.35, 0.85, 1.8), -0.4, 0.15), stamp_s=1)),
        ('View 1: ground support', 'View 2: repeated ground'), {'ground_patch': (15, 15)},
    )

    low_box = (Box((-0.25, -0.25, 0), (0.25, 0.25, 0.4)),)
    far_targets = _targets(grid, range(13, 17), range(23, 28))
    wall_targets = _targets(grid, range(13, 17), [15])
    # The box face x=-0.25 and ground x=-0.28 share the cell [-0.3,-0.2).
    # Later ground votes are thus physically possible without moving the box.
    edge_ground = np.array([[-0.28, 0.05, 0]])
    overflight = SyntheticScenario(
        'overflight_obstacle', grid, low_box,
        (cast_observation(far_targets, sensor_pose((-1.25, 0.05, 2), 0.3, 0.2), low_box, 0),
         cast_observation(far_targets, sensor_pose((-1.15, 0.05, 2.1), 0.4, 0.15), low_box, 1),
         cast_observation(wall_targets, sensor_pose((-1.25, 0.05, 0.8), 0.2, 0.1), low_box, 2),
         cast_observation(edge_ground, sensor_pose((-1.25, 0.05, 1), 0.1, 0.2), low_box, 3),
         cast_observation(edge_ground, sensor_pose((-1.15, 0.05, 1.1), 0.2, 0.3), low_box, 4)),
        ('View 1: overflight', 'View 2: overflight', 'View 3: wall hit',
         'View 4: edge ground', 'View 5: edge ground'), {'box_edge': (15, 12)},
    )

    tall_box = (Box((-0.35, -0.55, 0), (0.05, 0.55, 1.2)),)
    hidden_targets = _targets(grid, range(11, 19), range(18, 27))
    first_targets = np.vstack((hidden_targets, [-1.25, 0.05, 3]))
    occlusion = SyntheticScenario(
        'occlusion_multiview', grid, tall_box,
        (cast_observation(first_targets, sensor_pose((-1.25, 0.05, 1.4), 0.3, 0.2), tall_box, 0),
         cast_observation(hidden_targets, sensor_pose((1.35, 0.05, 1.4), -0.3, 0.2), tall_box, 1),
         cast_observation(hidden_targets, sensor_pose((1.25, -0.15, 1.6), -0.4, 0.1), tall_box, 2)),
        ('View 1: occluded ground', 'View 2: ground revealed', 'View 3: ground confirmed'),
        {'hidden_ground': (15, 22)},
    )
    return clear, overflight, occlusion
