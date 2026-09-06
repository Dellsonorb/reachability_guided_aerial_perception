"""Pure footprint-aware combination of frozen A1 and A2 snapshots."""

from dataclasses import dataclass
from enum import IntEnum

import numpy as np

from environment_belief import EnvironmentBeliefGrid, EnvironmentGridSpec, EnvironmentState
from reachability_guided_aerial_perception.model import (
    AssessmentCoverage, CellState, FieldStatus, ManipulationInterestField,
)
from .geometry import FootprintSpec, footprint_cells


class SupportState(IntEnum):
    NO_VALIDATED_SUPPORT = -1
    BLOCKED_ONLY = 0
    SUPPORTED = 1


class PoseEnvironmentState(IntEnum):
    NOT_PROJECTED = -1
    A2_OCCUPIED_BLOCKED = 0
    UNCONFIRMED = 1
    OBSERVED_GROUND_SUPPORT = 2


@dataclass(frozen=True)
class PoseSupport:
    """Discrete cell-center pose, NOT the exact original IK-validated candidate.

    blocked means A2-occupied-blocked only, not navigation infeasible.
    source_id and covered_environment_cells are row-major flat cell ids.
    """

    source_id: int
    row: int
    col: int
    xy: tuple[float, float]
    yaw: float
    relevance: float
    environment_state: PoseEnvironmentState
    blocked: bool
    footprint_clipped: bool
    free_cells: int
    occupied_cells: int
    unknown_cells: int
    covered_environment_cells: tuple[int, ...]


@dataclass(frozen=True, eq=False)
class TaskRelevantUncertaintyField:
    grid: EnvironmentGridSpec
    grasp_id: str
    a1_status: FieldStatus
    a1_coverage: AssessmentCoverage
    footprint: FootprintSpec
    nominal_task_relevance: np.ndarray
    task_relevance_at_environment_cell: np.ndarray
    task_relevant_uncertainty: np.ndarray
    support_state: np.ndarray
    nominal_support_count: np.ndarray
    operational_support_count: np.ndarray
    best_nominal_source: np.ndarray
    best_operational_source: np.ndarray
    a1_cell_state: np.ndarray
    environment_state: np.ndarray
    unknown_score: np.ndarray
    pose_environment_state: np.ndarray
    poses: tuple[PoseSupport, ...]

    @property
    def frame_id(self):
        return self.grid.frame_id


FIELD_ARRAY_NAMES = (
    'nominal_task_relevance', 'task_relevance_at_environment_cell',
    'task_relevant_uncertainty', 'support_state', 'nominal_support_count',
    'operational_support_count', 'best_nominal_source', 'best_operational_source',
    'a1_cell_state', 'environment_state', 'unknown_score', 'pose_environment_state',
)


def _check_inputs(a1, a2):
    if not isinstance(a1, ManipulationInterestField) or not isinstance(a2, EnvironmentBeliefGrid):
        raise ValueError('inputs must be ManipulationInterestField and EnvironmentBeliefGrid')
    if a1.frame_id != 'map' or a2.frame_id != 'map':
        raise ValueError('both inputs must be in map')
    if (a1.grid.shape != a2.grid.shape
            or not np.allclose(a1.grid.origin_xy, a2.origin_xy, rtol=0, atol=1e-9)
            or not np.allclose([a1.grid.resolution_m, a2.resolution_m], 0.10, rtol=0, atol=1e-9)):
        raise ValueError('require aligned map origin/shape and 0.10 m resolution; no resampling')
    for name in ('state', 'unknown_score'):
        if np.asarray(getattr(a2, name)).shape != a2.grid.shape:
            raise ValueError(f'A2 {name} shape must match grid')
    if not np.all(np.isin(a2.state, [s.value for s in EnvironmentState])):
        raise ValueError('invalid A2 state')
    u = np.asarray(a2.unknown_score)
    if not np.issubdtype(u.dtype, np.number) or np.iscomplexobj(u):
        raise ValueError('unknown_score must be real numeric')
    if not np.all(np.isfinite(u) & (u >= 0) & (u <= 1)):
        raise ValueError('unknown_score must be finite in [0, 1]')
    eligible = np.isin(a1.cell_state, [CellState.HIGH, CellState.LOW])
    if (not np.all(np.isfinite(a1.relevance[eligible]) & (a1.relevance[eligible] > 0))
            or not np.all(np.isfinite(a1.best_yaw[eligible]))):
        raise ValueError('A1 HIGH/LOW cells require positive finite relevance and finite best_yaw')
    return eligible


def build_task_uncertainty(a1_field, a2_belief, footprint=FootprintSpec()):
    """Project stored best_yaw poses, then apply A2 occupied gating and score.

    UNKNOWN never blocks; FREE keeps its supplied unknown_score. No mutation,
    new IK, alternate yaw search, occupancy clearing, or clearance inference.
    """
    eligible = _check_inputs(a1_field, a2_belief)
    if not isinstance(footprint, FootprintSpec):
        raise ValueError('footprint must be FootprintSpec')
    grid = a2_belief.grid
    size = grid.width_cells * grid.height_cells
    nominal, operational = np.zeros(size), np.zeros(size)
    nominal_count, operational_count = np.zeros(size, dtype=np.int32), np.zeros(size, dtype=np.int32)
    nominal_source, operational_source = np.full(size, -1, dtype=np.int64), np.full(size, -1, dtype=np.int64)
    pose_state = np.full(size, PoseEnvironmentState.NOT_PROJECTED, dtype=np.int8)
    environment = np.asarray(a2_belief.state).ravel()
    poses = []
    for row, col in np.argwhere(eligible):
        row, col = int(row), int(col)
        source = row * grid.width_cells + col
        xy = (grid.origin_xy[0] + (col + 0.5) * grid.resolution_m,
              grid.origin_xy[1] + (row + 0.5) * grid.resolution_m)
        yaw, relevance = float(a1_field.best_yaw[row, col]), float(a1_field.relevance[row, col])
        cells, clipped = footprint_cells(grid, xy, yaw, footprint)
        states = environment[cells]
        occupied = int(np.count_nonzero(states == EnvironmentState.OCCUPIED))
        free = int(np.count_nonzero(states == EnvironmentState.FREE))
        unknown = int(np.count_nonzero(states == EnvironmentState.UNKNOWN))
        blocked = occupied > 0
        if blocked:
            status = PoseEnvironmentState.A2_OCCUPIED_BLOCKED
        elif clipped or unknown:
            status = PoseEnvironmentState.UNCONFIRMED
        else:
            status = PoseEnvironmentState.OBSERVED_GROUND_SUPPORT
        pose_state[source] = status
        poses.append(PoseSupport(source, row, col, xy, yaw, relevance, status, blocked,
                                 clipped, free, occupied, unknown, tuple(int(c) for c in cells)))
        nominal_count[cells] += 1
        better = cells[relevance > nominal[cells]]
        nominal[better], nominal_source[better] = relevance, source
        if not blocked:
            operational_count[cells] += 1
            better = cells[relevance > operational[cells]]
            operational[better], operational_source[better] = relevance, source

    support = np.full(size, SupportState.NO_VALIDATED_SUPPORT, dtype=np.int8)
    support[nominal_count > 0] = SupportState.BLOCKED_ONLY
    support[operational_count > 0] = SupportState.SUPPORTED
    no_support = nominal_count == 0
    nominal[no_support] = np.nan
    operational[no_support] = np.nan
    uncertainty = operational * np.asarray(a2_belief.unknown_score).ravel()
    values = (nominal, operational, uncertainty, support, nominal_count, operational_count,
              nominal_source, operational_source, a1_field.cell_state, a2_belief.state,
              a2_belief.unknown_score, pose_state)
    arrays = []
    for value in values:
        array = np.array(value, copy=True).reshape(grid.shape)
        array.setflags(write=False)
        arrays.append(array)
    return TaskRelevantUncertaintyField(grid, a1_field.grasp_id, a1_field.status,
                                        a1_field.coverage, footprint, *arrays, tuple(poses))
