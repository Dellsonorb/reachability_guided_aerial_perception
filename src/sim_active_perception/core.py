"""Compose frozen A1-A4 and recover exact, observed-ground-supported candidates."""

from dataclasses import asdict, dataclass
from numbers import Integral

import numpy as np

from environment_belief import BeliefConfig, EnvironmentBeliefMapper, EnvironmentState
from reachability_guided_aerial_perception import candidate_relevance
from reachability_guided_nbv import NBVConfig, generate_candidates, rank_viewpoints
from task_relevant_uncertainty import build_task_uncertainty
from task_relevant_uncertainty.geometry import footprint_cells


@dataclass(frozen=True)
class A5Config:
    max_viewpoints: int = 3
    grid_width_m: float = 4.0
    grid_height_m: float = 4.0
    ground_z_m: float = 0.0
    xy_offsets_m: tuple[float, ...] = (-2, 0, 2)
    flight_weight: float = .25
    flight_bounds: tuple[float, ...] = (-4, 4, -3, 3, .5, 3)
    facade_position_tolerance: float = .15

    def __post_init__(self):
        if isinstance(self.max_viewpoints, bool) or not isinstance(self.max_viewpoints, Integral) or self.max_viewpoints < 1:
            raise ValueError('max_viewpoints must be a positive integer')
        values = (self.grid_width_m, self.grid_height_m, self.ground_z_m,
                  self.flight_weight, self.facade_position_tolerance)
        if not np.all(np.isfinite(values)) or min(self.grid_width_m, self.grid_height_m) <= 0:
            raise ValueError('grid sizes must be positive and config scalars finite')
        if self.flight_weight < 0 or self.facade_position_tolerance < 0:
            raise ValueError('cost and position tolerance must be nonnegative')
        bounds = np.asarray(self.flight_bounds, dtype=float)
        if bounds.shape != (6,) or not np.all(np.isfinite(bounds)) or np.any(bounds[::2] >= bounds[1::2]):
            raise ValueError('flight_bounds must be finite ordered x/y/z limits')
        object.__setattr__(self, 'flight_bounds', tuple(bounds))
        NBVConfig(xy_offsets_m=self.xy_offsets_m, flight_weight=self.flight_weight)


def candidate_catalog(field, raw):
    """Recover the exact A1 per-cell winner, preserving original first-tie order."""
    winners = {}
    for index, candidate in enumerate(raw['evaluated_candidates']):
        relevance = candidate_relevance(candidate, field.config)
        cell = field.grid.cell_index(candidate['bunker_x'], candidate['bunker_y'])
        if cell is None or relevance <= 0:
            continue
        if cell in winners and relevance <= winners[cell]['relevance']:
            continue
        winners[cell] = dict(candidate_id=candidate['candidate_id'], x=float(candidate['bunker_x']),
                             y=float(candidate['bunker_y']), yaw=float(candidate['bunker_yaw']),
                             relevance=float(relevance), source_id=cell[0] * field.grid.width_cells + cell[1],
                             evaluation_index=index)
    return sorted(winners.values(), key=lambda c: c['evaluation_index'])


def assess_candidates(field, belief, catalog, *, task=None, operational=None):
    """Confirm measured ground support, never full navigation/clearance.

    v1 uses raw A2 FREE; v1.1 uses shared blocking plus real ground votes.
    """
    task = build_task_uncertainty(field, belief, operational=operational) if task is None else task
    expected = 'v1' if operational is None else 'object-aware-v1.1'
    if task.operational_semantics != expected:
        raise ValueError('task and exact candidates must use the same operational semantics')
    if operational is not None:
        from operational_gating import assess_footprint
        if operational.grid != belief.grid or operational.config != belief.config:
            raise ValueError('operational evidence must be aligned with the A2 grid/config')
    representative = {p.source_id: p for p in task.poses}
    assessments = []
    for candidate in catalog:
        cells, clipped = footprint_cells(belief.grid, (candidate['x'], candidate['y']), candidate['yaw'], task.footprint)
        states = belief.state.ravel()[cells]
        occupied = int(np.count_nonzero(states == EnvironmentState.OCCUPIED))
        free = int(np.count_nonzero(states == EnvironmentState.FREE))
        unknown = int(np.count_nonzero(states == EnvironmentState.UNKNOWN))
        source = representative.get(candidate['source_id'])
        source_blocked = source is None or source.blocked
        if operational is not None and source is not None:
            source_blocked = assess_footprint(operational, source.xy, source.yaw, task.footprint).blocked
        exact = None if operational is None else assess_footprint(
            operational, (candidate['x'], candidate['y']), candidate['yaw'], task.footprint)
        confirmed = (bool(len(cells) and not clipped and not source_blocked and free == len(cells))
                     if exact is None else bool(not source_blocked and not exact.blocked and exact.ground_supported))
        record = dict(candidate, footprint_clipped=bool(clipped), free_cells=free,
                                occupied_cells=occupied, unknown_cells=unknown,
                                representative_blocked=source_blocked,
                                confirmed=confirmed,
                                mean_unknown_score=float(np.mean(belief.unknown_score.ravel()[cells])) if len(cells) else None)
        if exact is not None:
            record['operational'] = asdict(exact)
        assessments.append(record)
    return assessments


def replay_observations(grid, observations, config=BeliefConfig()):
    """Replay bounded history into an EMPTY mapper, not into an accumulated one."""
    observations = tuple(observations)
    sensor_frame = observations[0].frame_id if observations else 'uav1/lidar_link'
    mapper = EnvironmentBeliefMapper(grid, config, sensor_frame=sensor_frame)
    previous = -np.inf
    for observation in observations:
        if observation.stamp_s <= previous:
            raise ValueError('observation stamps must be strictly increasing')
        previous = observation.stamp_s
        mapper.update(observation)
    return mapper.snapshot()


def decide(field, raw, belief, current, *, round_count, config=A5Config(), operational=None):
    """A bounded one-step decision; no simulated belief updates or forced flight."""
    if not isinstance(round_count, Integral) or round_count < 1:
        raise ValueError('round_count must count at least the initial observation')
    nbv_config = NBVConfig(xy_offsets_m=config.xy_offsets_m, flight_weight=config.flight_weight)
    candidates = []
    lo, hi = np.asarray(config.flight_bounds)[::2], np.asarray(config.flight_bounds)[1::2]
    for v in generate_candidates(current, config=nbv_config):
        if np.any(np.asarray(v.position_xyz) < lo) or np.any(np.asarray(v.position_xyz) > hi):
            continue
        distance = np.linalg.norm(np.asarray(v.position_xyz) - current.position_xyz)
        if distance <= config.facade_position_tolerance and v != current:
            continue  # Facade success does not certify yaw-only completion.
        candidates.append(v)
    if not candidates:
        raise ValueError('no viewpoint within the configured operating area')
    task = build_task_uncertainty(field, belief, operational=operational)
    ranking = rank_viewpoints(task, belief, current, candidates=candidates, config=nbv_config)
    assessments = assess_candidates(field, belief, candidate_catalog(field, raw), task=task, operational=operational)
    confirmed = [c for c in assessments if c['confirmed']]
    selected = max(confirmed, key=lambda c: c['relevance']) if confirmed else None
    best = ranking.best_task
    stop = (ranking.status if ranking.status != 'RANKED' else
            'NONPOSITIVE_SCORE' if best.task_score <= 0 else
            'VIEW_BUDGET_REACHED' if round_count >= config.max_viewpoints else None)
    next_pose = None if stop or best is None else [*best.viewpoint.position_xyz, best.viewpoint.yaw_rad]
    choice = dict(ok=True, round=int(round_count), stop_reason=stop, next_viewpoint=next_pose,
                selected_candidate=selected, candidate_count=len(assessments), confirmed_candidate_count=len(confirmed),
                best_task_score=None if best is None else best.task_score,
                best_task_gain=None if best is None else best.task_gain,
                assessments=assessments, environment_cells={s.name: int(np.count_nonzero(belief.state == s))
                                                           for s in EnvironmentState},
                total_observation_votes=int(belief.observation_count.sum()),
                task_uncertainty_mass=float(np.nansum(task.task_relevant_uncertainty)))
    if operational is not None:
        choice['operational_semantics'] = task.operational_semantics
    return choice, ranking
