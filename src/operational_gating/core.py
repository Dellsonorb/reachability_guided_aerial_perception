"""Derived object-aware occupancy and ground evidence; raw A2 stays untouched."""

from dataclasses import dataclass
from enum import IntEnum
from numbers import Real

import numpy as np

from environment_belief import BeliefConfig, EnvironmentGridSpec, PointCloudObservation
from task_relevant_uncertainty.geometry import (
    CONTACT_TOLERANCE_M, FootprintSpec, footprint_cells, footprint_vertices,
)


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value):
        raise ValueError(f'{name} must be a finite real')
    return float(value)


def _readonly(value):
    array = np.array(value, copy=True)
    array.setflags(write=False)
    return array


class OccupiedClass(IntEnum):
    """Association labels apply to occupied-height return endpoints only."""

    ENVIRONMENT = 0
    AMBIGUOUS = 1
    TARGET = 2


@dataclass(frozen=True)
class PerceivedTarget:
    """Runtime-perceived map pose with known object dimensions and allowance.

    The same metric allowance expands both the association box and collision
    rectangle. It is supplied by the caller's sensor geometry budget.
    """

    center_xyz: tuple[float, float, float]
    yaw_rad: float
    size_xyz: tuple[float, float, float] = (.240, .053, .115)
    geometry_allowance_m: float = 0.
    frame_id: str = 'map'

    def __post_init__(self):
        for name in ('center_xyz', 'size_xyz'):
            values = np.asarray(getattr(self, name), dtype=float)
            if values.shape != (3,) or not np.all(np.isfinite(values)):
                raise ValueError(f'{name} must contain three finite coordinates')
            if name == 'size_xyz' and np.any(values <= 0):
                raise ValueError('size_xyz dimensions must be positive')
            object.__setattr__(self, name, tuple(float(value) for value in values))
        object.__setattr__(self, 'yaw_rad', _finite(self.yaw_rad, 'yaw_rad'))
        allowance = _finite(self.geometry_allowance_m, 'geometry_allowance_m')
        if allowance < 0:
            raise ValueError('geometry_allowance_m must be nonnegative')
        object.__setattr__(self, 'geometry_allowance_m', allowance)
        if self.frame_id != 'map':
            raise ValueError('perceived target frame_id must be map')

    @property
    def footprint(self):
        return FootprintSpec(self.size_xyz[0] / 2 + self.geometry_allowance_m,
                             self.size_xyz[1] / 2 + self.geometry_allowance_m)

    @property
    def xy_vertices(self):
        return _readonly(footprint_vertices(self.center_xyz[:2], self.yaw_rad, self.footprint))

    def contains(self, points_map):
        """Return an N-element mask for the expanded, closed oriented 3D box."""
        points = np.asarray(points_map, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError('points_map must have shape (N, 3)')
        if not np.all(np.isfinite(points)):
            raise ValueError('points_map must be finite')
        delta = points - np.asarray(self.center_xyz)
        c, s = np.cos(self.yaw_rad), np.sin(self.yaw_rad)
        local = np.column_stack((c * delta[:, 0] + s * delta[:, 1],
                                 -s * delta[:, 0] + c * delta[:, 1], delta[:, 2]))
        half_sizes = np.asarray(self.size_xyz) / 2 + self.geometry_allowance_m
        return np.all(np.abs(local) <= half_sizes + CONTACT_TOLERANCE_M, axis=1)


def _validate_context(grid, config, target):
    if not isinstance(grid, EnvironmentGridSpec) or not isinstance(config, BeliefConfig):
        raise ValueError('grid/config must be EnvironmentGridSpec/BeliefConfig')
    if not isinstance(target, PerceivedTarget):
        raise ValueError('target must be PerceivedTarget')
    if not np.all(np.isfinite(grid.extent)):
        raise ValueError('grid extent must be finite')


@dataclass(frozen=True, eq=False)
class OperationalEvidenceView:
    """Detached per-cell counts derived from a bounded observation history."""

    grid: EnvironmentGridSpec
    config: BeliefConfig
    target: PerceivedTarget
    environment_occupied_votes: np.ndarray
    ambiguous_occupied_votes: np.ndarray
    target_occupied_votes: np.ndarray
    ground_votes: np.ndarray

    def __post_init__(self):
        _validate_context(self.grid, self.config, self.target)
        for name in ('environment_occupied_votes', 'ambiguous_occupied_votes',
                     'target_occupied_votes', 'ground_votes'):
            values = np.asarray(getattr(self, name))
            if (values.shape != self.grid.shape or values.dtype.kind not in 'iu'
                    or np.any(values < 0) or np.any(values > np.iinfo(np.int64).max)):
                raise ValueError(f'{name} must contain nonnegative integer grid counts')
            object.__setattr__(self, name, _readonly(values.astype(np.int64)))


def derive_operational_evidence(grid, observations, target, *, labels=None, config=BeliefConfig()):
    """Replay endpoints with A2's filtering, retaining each occupied class.

    Labels are aligned to original rows, before valid-return/range/grid filters.
    Missing association is AMBIGUOUS. TARGET labels outside the target's 3D
    geometry are downgraded to AMBIGUOUS. A real ground endpoint contributes
    one vote per window unless ENVIRONMENT or AMBIGUOUS occupies that cell in
    that window; TARGET neither supplies nor suppresses ground evidence.
    """
    _validate_context(grid, config, target)
    observations = tuple(observations)
    if labels is None:
        label_windows = None
    else:
        label_windows = tuple(labels)
        if len(label_windows) != len(observations):
            raise ValueError('labels must have one array per observation')
    occupied = np.zeros((3,) + grid.shape, dtype=np.int64)
    ground_counts = np.zeros(grid.shape, dtype=np.int64)
    x_edges = grid.origin_xy[0] + np.arange(grid.width_cells + 1) * grid.resolution_m
    y_edges = grid.origin_xy[1] + np.arange(grid.height_cells + 1) * grid.resolution_m
    x0, x1, y0, y1 = grid.extent
    previous_stamp = -np.inf
    sensor_frame = None
    for index, observation in enumerate(observations):
        if not isinstance(observation, PointCloudObservation):
            raise ValueError('observations must be PointCloudObservation instances')
        if observation.stamp_s <= previous_stamp:
            raise ValueError('observation stamps must be strictly increasing')
        previous_stamp = observation.stamp_s
        if sensor_frame is None:
            sensor_frame = observation.frame_id
        if observation.frame_id != sensor_frame:
            raise ValueError('observation frame_id must match the first sensor frame')
        if label_windows is None:
            point_labels = np.full(len(observation.points_xyz), OccupiedClass.AMBIGUOUS, dtype=np.int8)
        else:
            point_labels = np.asarray(label_windows[index])
            if (point_labels.shape != (len(observation.points_xyz),)
                    or point_labels.dtype.kind not in 'iu'
                    or not np.all(np.isin(point_labels, tuple(OccupiedClass)))):
                raise ValueError('labels must be aligned integer arrays containing known OccupiedClass values')
            point_labels = np.array(point_labels, dtype=np.int8, copy=True)
        points = observation.points_xyz[observation.valid_return]
        point_labels = point_labels[observation.valid_return]
        finite = np.all(np.isfinite(points), axis=1)
        points, point_labels = points[finite], point_labels[finite]
        ranges = np.hypot.reduce(points, axis=1)
        usable = (ranges > config.min_range_m) & (ranges < config.max_range_m)
        transform = observation.T_map_sensor
        points = points[usable] @ transform[:3, :3].T + transform[:3, 3]
        point_labels = point_labels[usable]
        if not np.all(np.isfinite(points)):
            raise ValueError('transformed points must be finite')
        inside = ((points[:, 0] >= x0) & (points[:, 0] < x1)
                  & (points[:, 1] >= y0) & (points[:, 1] < y1))
        points, point_labels = points[inside], point_labels[inside]
        xy = np.column_stack((np.searchsorted(x_edges, points[:, 0], side='right') - 1,
                              np.searchsorted(y_edges, points[:, 1], side='right') - 1))
        heights = points[:, 2] - config.ground_z_m
        ground = np.abs(heights) <= config.ground_tolerance_m
        obstacle = heights >= config.obstacle_min_height_m
        invalid_target = ((point_labels == OccupiedClass.TARGET) & ~target.contains(points))
        point_labels[invalid_target] = OccupiedClass.AMBIGUOUS
        window_occupied = np.zeros_like(occupied, dtype=bool)
        for occupied_class in OccupiedClass:
            selected = obstacle & (point_labels == occupied_class)
            window_occupied[occupied_class, xy[selected, 1], xy[selected, 0]] = True
        window_ground = np.zeros(grid.shape, dtype=bool)
        window_ground[xy[ground, 1], xy[ground, 0]] = True
        window_ground &= ~(window_occupied[OccupiedClass.ENVIRONMENT]
                           | window_occupied[OccupiedClass.AMBIGUOUS])
        occupied += window_occupied
        ground_counts += window_ground
    return OperationalEvidenceView(grid, config, target, *occupied, ground_counts)


@dataclass(frozen=True)
class FootprintAssessment:
    covered_cells: tuple[int, ...]
    footprint_clipped: bool
    environment_cells: int
    ambiguous_cells: int
    target_cells: int
    ground_supported_cells: int
    ground_missing_cells: int
    target_collision: bool
    blocked: bool
    ground_supported: bool


def _rectangles_overlap(first, second):
    """Closed SAT on the edge normals of both continuous rectangles."""
    for vertices in (first, second):
        edges = np.roll(vertices, -1, axis=0) - vertices
        axes = np.column_stack((-edges[:, 1], edges[:, 0]))
        axes /= np.linalg.norm(axes, axis=1)[:, None]
        first_projection, second_projection = first @ axes.T, second @ axes.T
        if (np.any(first_projection.max(axis=0) < second_projection.min(axis=0) - CONTACT_TOLERANCE_M)
                or np.any(second_projection.max(axis=0) < first_projection.min(axis=0) - CONTACT_TOLERANCE_M)):
            return False
    return True


def assess_footprint(view, xy, yaw, footprint=FootprintSpec()):
    """Assess separate blocking and measured-ground support for a base pose."""
    if not isinstance(view, OperationalEvidenceView):
        raise ValueError('view must be OperationalEvidenceView')
    cells, clipped = footprint_cells(view.grid, xy, yaw, footprint)
    environment = int(np.count_nonzero(view.environment_occupied_votes.ravel()[cells]))
    ambiguous = int(np.count_nonzero(view.ambiguous_occupied_votes.ravel()[cells]))
    target = int(np.count_nonzero(view.target_occupied_votes.ravel()[cells]))
    supported = int(np.count_nonzero(view.ground_votes.ravel()[cells] >= view.config.free_observations))
    collision = _rectangles_overlap(footprint_vertices(xy, yaw, footprint), view.target.xy_vertices)
    return FootprintAssessment(
        tuple(int(cell) for cell in cells), bool(clipped), environment, ambiguous,
        target, supported, len(cells) - supported, collision,
        bool(environment or ambiguous or collision),
        bool(len(cells) and not clipped and supported == len(cells)),
    )
