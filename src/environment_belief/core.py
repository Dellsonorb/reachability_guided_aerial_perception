"""Static horizontal-ground belief from observed 3D return endpoints only."""

from dataclasses import dataclass
from enum import IntEnum
from numbers import Integral, Real

import numpy as np


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value):
        raise ValueError(f'{name} must be a finite real')
    return float(value)


def _positive_integer(value, name):
    if isinstance(value, bool) or not isinstance(value, Integral) or value <= 0:
        raise ValueError(f'{name} must be a positive integer')


def _copy_readonly(value):
    array = np.array(value, copy=True)
    array.setflags(write=False)
    return array


class EnvironmentState(IntEnum):
    UNKNOWN = -1
    FREE = 0
    OCCUPIED = 100


@dataclass(frozen=True)
class EnvironmentGridSpec:
    origin_xy: tuple[float, float]
    width_cells: int
    height_cells: int
    resolution_m: float = 0.10
    frame_id: str = 'map'

    def __post_init__(self):
        origin = np.asarray(self.origin_xy, dtype=float)
        if origin.shape != (2,) or not np.all(np.isfinite(origin)):
            raise ValueError('origin_xy must contain two finite coordinates')
        object.__setattr__(self, 'origin_xy', tuple(float(x) for x in origin))
        _positive_integer(self.width_cells, 'width_cells')
        _positive_integer(self.height_cells, 'height_cells')
        if _finite(self.resolution_m, 'resolution_m') <= 0:
            raise ValueError('resolution_m must be positive')
        if self.frame_id != 'map':
            raise ValueError('environment grid frame_id must be map')

    @property
    def shape(self):
        return self.height_cells, self.width_cells

    @property
    def extent(self):
        x, y = self.origin_xy
        return (x, x + self.width_cells * self.resolution_m,
                y, y + self.height_cells * self.resolution_m)


@dataclass(frozen=True)
class BeliefConfig:
    ground_z_m: float = 0.0
    ground_tolerance_m: float = 0.02
    obstacle_min_height_m: float = 0.05
    min_range_m: float = 0.20
    max_range_m: float = 40.0
    free_observations: int = 2
    unknown_scale: float = 2.0

    def __post_init__(self):
        for name in ('ground_z_m', 'ground_tolerance_m', 'obstacle_min_height_m',
                     'min_range_m', 'max_range_m', 'unknown_scale'):
            object.__setattr__(self, name, _finite(getattr(self, name), name))
        if not 0 <= self.ground_tolerance_m < self.obstacle_min_height_m:
            raise ValueError('require 0 <= ground tolerance < obstacle minimum height')
        if not 0 <= self.min_range_m < self.max_range_m:
            raise ValueError('require 0 <= min range < max range')
        if self.unknown_scale <= 0:
            raise ValueError('unknown_scale must be positive')
        _positive_integer(self.free_observations, 'free_observations')
        object.__setattr__(self, 'free_observations', int(self.free_observations))


@dataclass(frozen=True, eq=False)
class PointCloudObservation:
    """Endpoints in the named sensor frame, deskewed to stamp_s by the caller.

    Invalid return rows are allowed and filtered by update, not turned into
    free-space measurements. T_map_sensor includes the full sensor extrinsic.
    """

    points_xyz: np.ndarray
    frame_id: str
    stamp_s: float
    T_map_sensor: np.ndarray
    valid_return: np.ndarray | None = None

    def __post_init__(self):
        points = np.asarray(self.points_xyz, dtype=float)
        transform = np.asarray(self.T_map_sensor, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError('points_xyz must have shape (N, 3)')
        if not isinstance(self.frame_id, str) or not self.frame_id.strip():
            raise ValueError('frame_id must name the sensor frame')
        _finite(self.stamp_s, 'stamp_s')
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            raise ValueError('T_map_sensor must be a finite 4x4 rigid transform')
        rotation = transform[:3, :3]
        if (not np.allclose(transform[3], [0, 0, 0, 1], rtol=0, atol=1e-8)
                or not np.allclose(rotation.T @ rotation, np.eye(3), rtol=0, atol=1e-6)
                or not np.isclose(np.linalg.det(rotation), 1.0, rtol=0, atol=1e-6)):
            raise ValueError('T_map_sensor must contain a proper rigid rotation')
        valid = np.ones(len(points), dtype=bool) if self.valid_return is None else np.asarray(self.valid_return)
        if valid.shape != (len(points),) or valid.dtype != np.dtype(bool):
            raise ValueError('valid_return must be a bool array of shape (N,)')
        object.__setattr__(self, 'points_xyz', _copy_readonly(points))
        object.__setattr__(self, 'T_map_sensor', _copy_readonly(transform))
        object.__setattr__(self, 'valid_return', _copy_readonly(valid))


@dataclass(frozen=True)
class UpdateSummary:
    """Point rejection counts partition input_points; votes count cells."""

    stamp_s: float
    input_points: int
    masked_invalid_points: int
    nonfinite_points: int
    out_of_range_points: int
    out_of_grid_points: int
    ambiguous_points: int
    ground_points: int
    obstacle_points: int
    free_cells: int
    occupied_cells: int


@dataclass(frozen=True, eq=False)
class EnvironmentBeliefGrid:
    """Detached snapshot produced by EnvironmentBeliefMapper.snapshot()."""

    grid: EnvironmentGridSpec
    config: BeliefConfig
    state: np.ndarray
    occupied_evidence: np.ndarray
    free_evidence: np.ndarray
    observation_count: np.ndarray
    unknown_score: np.ndarray

    @property
    def frame_id(self):
        return self.grid.frame_id

    @property
    def origin_xy(self):
        return self.grid.origin_xy

    @property
    def resolution_m(self):
        return self.grid.resolution_m

    @property
    def width_cells(self):
        return self.grid.width_cells

    @property
    def height_cells(self):
        return self.grid.height_cells


class EnvironmentBeliefMapper:
    """Accumulate one exclusive endpoint vote per cell and observation.

    The scene is static: occupied evidence is never cleared. Each update must
    be a newly delivered observation; this class does not identify scan replays.
    """

    def __init__(self, grid, config=BeliefConfig(), sensor_frame='lidar'):
        if not isinstance(grid, EnvironmentGridSpec) or not isinstance(config, BeliefConfig):
            raise ValueError('grid/config must be EnvironmentGridSpec/BeliefConfig')
        if not isinstance(sensor_frame, str) or not sensor_frame.strip():
            raise ValueError('sensor_frame must be nonblank')
        self.grid, self.config, self.sensor_frame = grid, config, sensor_frame
        self._occupied = np.zeros(grid.shape, dtype=np.int64)
        self._free = np.zeros(grid.shape, dtype=np.int64)
        self._x_edges = grid.origin_xy[0] + np.arange(grid.width_cells + 1) * grid.resolution_m
        self._y_edges = grid.origin_xy[1] + np.arange(grid.height_cells + 1) * grid.resolution_m

    def update(self, observation):
        if not isinstance(observation, PointCloudObservation):
            raise ValueError('observation must be PointCloudObservation')
        if observation.frame_id != self.sensor_frame:
            raise ValueError('observation frame_id does not match mapper sensor_frame')
        points = observation.points_xyz[observation.valid_return]
        masked = len(observation.points_xyz) - len(points)
        finite = np.all(np.isfinite(points), axis=1)
        nonfinite = int(np.count_nonzero(~finite))
        points = points[finite]
        ranges = np.hypot.reduce(points, axis=1)
        usable = (ranges > self.config.min_range_m) & (ranges < self.config.max_range_m)
        out_of_range = int(np.count_nonzero(~usable))
        transform = observation.T_map_sensor
        points = points[usable] @ transform[:3, :3].T + transform[:3, 3]
        if not np.all(np.isfinite(points)):
            raise ValueError('transformed points must be finite')
        x0, x1, y0, y1 = self.grid.extent
        inside = ((points[:, 0] >= x0) & (points[:, 0] < x1)
                  & (points[:, 1] >= y0) & (points[:, 1] < y1))
        out_of_grid = int(np.count_nonzero(~inside))
        points = points[inside]
        # Compare against the defined edges directly: division followed by floor
        # can put an exact internal boundary in the preceding cell due to roundoff.
        xy = np.column_stack((np.searchsorted(self._x_edges, points[:, 0], side='right') - 1,
                              np.searchsorted(self._y_edges, points[:, 1], side='right') - 1))
        heights = points[:, 2] - self.config.ground_z_m
        ground = np.abs(heights) <= self.config.ground_tolerance_m
        obstacle = heights >= self.config.obstacle_min_height_m
        free_votes = np.zeros(self.grid.shape, dtype=bool)
        occupied_votes = np.zeros(self.grid.shape, dtype=bool)
        free_votes[xy[ground, 1], xy[ground, 0]] = True
        occupied_votes[xy[obstacle, 1], xy[obstacle, 0]] = True
        free_votes &= ~occupied_votes
        self._free += free_votes
        self._occupied += occupied_votes
        return UpdateSummary(
            float(observation.stamp_s), len(observation.points_xyz), masked, nonfinite,
            out_of_range, out_of_grid, int(np.count_nonzero(~(ground | obstacle))),
            int(np.count_nonzero(ground)), int(np.count_nonzero(obstacle)),
            int(np.count_nonzero(free_votes)), int(np.count_nonzero(occupied_votes)),
        )

    def snapshot(self):
        count = self._occupied + self._free
        state = np.full(self.grid.shape, EnvironmentState.UNKNOWN, dtype=np.int8)
        state[self._free >= self.config.free_observations] = EnvironmentState.FREE
        state[self._occupied > 0] = EnvironmentState.OCCUPIED
        unknown = np.exp(-count.astype(float) / self.config.unknown_scale)
        arrays = (state, self._occupied, self._free, count, unknown)
        return EnvironmentBeliefGrid(self.grid, self.config, *(_copy_readonly(a) for a in arrays))
