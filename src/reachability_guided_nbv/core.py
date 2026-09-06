"""One-step task/generic predicted observation-gain ranking, not entropy EIG."""

from dataclasses import dataclass

import numpy as np

from environment_belief import EnvironmentBeliefGrid, EnvironmentGridSpec, EnvironmentState
from task_relevant_uncertainty import SupportState, TaskRelevantUncertaintyField
from .geometry import predict_visibility
from .model import NBVConfig, SensorModel, Viewpoint, generate_candidates, readonly, wrap_yaw


def flight_cost(current, candidate, config=NBVConfig()):
    """Straight-line distance + yaw proxy, NOT energy or collision-free path cost."""
    distance = np.linalg.norm(np.asarray(candidate.position_xyz) - current.position_xyz)
    cost = float(distance + config.yaw_cost_m_per_rad * abs(wrap_yaw(candidate.yaw_rad - current.yaw_rad)))
    if not np.isfinite(cost):
        raise ValueError('flight cost must be finite')
    return cost


@dataclass(frozen=True)
class CandidateEvaluation:
    candidate_id: int
    viewpoint: Viewpoint
    status: str
    task_gain: float
    generic_gain: float
    flight_cost: float
    task_score: float | None
    generic_score: float | None
    visible_cells: int
    visible_task_cells: int


@dataclass(frozen=True, eq=False)
class NBVResult:
    grid: EnvironmentGridSpec
    current: Viewpoint
    sensor: SensorModel
    config: NBVConfig
    status: str
    candidates: tuple[CandidateEvaluation, ...]
    task_order: tuple[int, ...]
    generic_order: tuple[int, ...]
    visibility: np.ndarray
    delta_unknown: np.ndarray
    marginal_task_gain: np.ndarray
    support_state: np.ndarray
    best_operational_source: np.ndarray
    a1_cell_state: np.ndarray

    @property
    def best_task(self):
        return self.candidates[self.task_order[0]] if self.status == 'RANKED' else None

    @property
    def best_generic(self):
        return self.candidates[self.generic_order[0]] if self.generic_order else None

    def task_contribution(self, candidate_id):
        """Unweighted-by-cost cell contributions; NaN keeps NO_VALIDATED_SUPPORT."""
        return self.marginal_task_gain * self.visibility[candidate_id]


def _check_inputs(task, belief):
    if not isinstance(task, TaskRelevantUncertaintyField) or not isinstance(belief, EnvironmentBeliefGrid):
        raise ValueError('require A3 TaskRelevantUncertaintyField and A2 EnvironmentBeliefGrid')
    if (task.frame_id != 'map' or belief.frame_id != 'map' or task.grid.shape != belief.grid.shape
            or not np.allclose(task.grid.origin_xy, belief.origin_xy, rtol=0, atol=1e-9)
            or not np.allclose([task.grid.resolution_m, belief.resolution_m], .1, rtol=0, atol=1e-9)):
        raise ValueError('require aligned map origin/shape and fixed 0.10 m resolution')
    for source, names in ((belief, ('state', 'unknown_score', 'observation_count')),
                          (task, ('environment_state', 'unknown_score', 'support_state',
                                  'task_relevance_at_environment_cell', 'task_relevant_uncertainty',
                                  'best_operational_source', 'a1_cell_state'))):
        for name in names:
            if np.asarray(getattr(source, name)).shape != belief.grid.shape:
                raise ValueError(f'{name} shape must match grid')
    if (not np.array_equal(task.environment_state, belief.state)
            or not np.allclose(task.unknown_score, belief.unknown_score, rtol=0, atol=1e-12)):
        raise ValueError('A3 and A2 must describe the same environment snapshot; rebuild A3 first')
    if not np.all(np.isin(belief.state, [s.value for s in EnvironmentState])):
        raise ValueError('invalid A2 environment state')
    n, u = np.asarray(belief.observation_count), np.asarray(belief.unknown_score)
    if (not np.all(np.isfinite(n) & (n >= 0) & (n == np.floor(n)))
            or not np.all(np.isfinite(u) & (u >= 0) & (u <= 1))
            or not np.allclose(u, np.exp(-n / belief.config.unknown_scale), rtol=1e-10, atol=1e-12)):
        raise ValueError('A2 unknown_score must be exp(-observation_count/unknown_scale)')
    if not np.all(np.isin(task.support_state, [s.value for s in SupportState])):
        raise ValueError('invalid A3 support_state')
    known_support = task.support_state != SupportState.NO_VALIDATED_SUPPORT
    m = np.asarray(task.task_relevance_at_environment_cell)
    uncertainty = np.asarray(task.task_relevant_uncertainty)
    if (not np.all(np.isfinite(m[known_support]) & (m[known_support] >= 0) & (m[known_support] <= 1))
            or not np.allclose(uncertainty[known_support], (m * u)[known_support], rtol=1e-10, atol=1e-12)
            or not np.all(m[task.support_state == SupportState.BLOCKED_ONLY] == 0)
            or not np.all(np.isnan(m[~known_support]) & np.isnan(uncertainty[~known_support]))):
        raise ValueError('A3 U_task must equal M_operational * unknown_score with no-support NaNs')


def rank_viewpoints(task, belief, current, *, candidates=None, sensor=SensorModel(), config=NBVConfig()):
    """Predict one new informative endpoint per visible cell, then rank.

    G_task = (1-exp(-1/tau)) * sum(V * U_task). This is an uncertainty-reduction
    surrogate with fixed A3 operational support, not calibrated expected IG.
    Supplying candidates overrides generation; caller should include stay/rescan.
    """
    _check_inputs(task, belief)
    if not isinstance(current, Viewpoint):
        raise ValueError('current must be a level-hover Viewpoint in map')
    viewpoints = (generate_candidates(current, sensor=sensor, config=config)
                  if candidates is None else tuple(candidates))
    if not viewpoints or not all(isinstance(v, Viewpoint) for v in viewpoints):
        raise ValueError('candidates must be a nonempty sequence of Viewpoint')
    alpha = -np.expm1(-1 / belief.config.unknown_scale)
    delta = alpha * belief.unknown_score
    marginal = alpha * task.task_relevant_uncertainty
    visibility, evaluations = [], []
    for index, viewpoint in enumerate(viewpoints):
        prediction = predict_visibility(belief, viewpoint, sensor=sensor, config=config)
        visible = prediction.visible
        task_gain = float(np.nansum(marginal * visible))
        generic_gain = float(np.sum(delta * visible))
        cost = flight_cost(current, viewpoint, config)
        valid = prediction.status == 'VALID'
        evaluations.append(CandidateEvaluation(
            index, viewpoint, prediction.status, task_gain, generic_gain, cost,
            task_gain - config.flight_weight * cost if valid else None,
            generic_gain - config.flight_weight * cost if valid else None,
            int(visible.sum()), int(np.count_nonzero(visible & (task.support_state == SupportState.SUPPORTED))),
        ))
        visibility.append(visible)
    valid_ids = [e.candidate_id for e in evaluations if e.status == 'VALID']
    task_order = tuple(sorted(valid_ids, key=lambda i: (-evaluations[i].task_score, evaluations[i].flight_cost, i)))
    generic_order = tuple(sorted(valid_ids, key=lambda i: (-evaluations[i].generic_score, evaluations[i].flight_cost, i)))
    status = ('NO_VALID_CANDIDATE' if not valid_ids else
              'NO_PREDICTED_TASK_GAIN' if not any(evaluations[i].task_gain > 0 for i in valid_ids) else 'RANKED')
    return NBVResult(belief.grid, current, sensor, config, status, tuple(evaluations), task_order, generic_order,
                     *(readonly(a) for a in (visibility, delta, marginal, task.support_state,
                                            task.best_operational_source, task.a1_cell_state)))
