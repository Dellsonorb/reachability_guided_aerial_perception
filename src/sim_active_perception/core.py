"""Compose frozen A1-A4 and recover exact, observed-ground-supported candidates."""

from collections.abc import Mapping
from dataclasses import asdict, dataclass
from numbers import Integral

import numpy as np

from environment_belief import BeliefConfig, EnvironmentBeliefMapper, EnvironmentState
from reachability_guided_nbv import NBVConfig, generate_candidates, rank_viewpoints
from task_relevant_uncertainty import build_task_uncertainty, reconstruct_winner_anchors
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
    support_anchor: str = 'cell_center'

    def __post_init__(self):
        if self.support_anchor not in ('cell_center', 'exact_winner'):
            raise ValueError('support_anchor must be cell_center or exact_winner')
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
    if not isinstance(raw, Mapping):
        raise ValueError('raw must contain original evaluated_candidates')
    return [dict(candidate_id=anchor.candidate_id, x=anchor.x, y=anchor.y, yaw=anchor.yaw,
                 relevance=anchor.relevance, source_id=anchor.source_id, evaluation_index=anchor.evaluation_index)
            for anchor in reconstruct_winner_anchors(field, raw.get('evaluated_candidates'))]


def build_support_task(field, raw, belief, config, operational=None):
    """Build the same A3 input for decisions and saved/rendered worker products."""
    if config.support_anchor == 'exact_winner':
        if not isinstance(raw, Mapping) or raw.get('evaluated_candidates') is None:
            raise ValueError('exact support requires original evaluated_candidates')
        return build_task_uncertainty(field, belief, operational=operational,
                                      evaluated_candidates=raw['evaluated_candidates'])
    return build_task_uncertainty(field, belief, operational=operational)


def _check_exact_catalog(task, catalog):
    """Reject mixed pose identities instead of restoring a cell-center veto."""
    anchors = task.winner_anchors
    if len(catalog) != len(anchors) or len(task.poses) != len(anchors):
        raise ValueError('exact task and catalog must contain the same winner anchors')
    poses = {pose.source_id: pose for pose in task.poses}
    if len(poses) != len(anchors):
        raise ValueError('exact task must have one support pose per winner anchor')
    for anchor, candidate in zip(anchors, catalog):
        if any(candidate.get(name) != value for name, value in asdict(anchor).items()):
            raise ValueError('exact task and catalog winner identity/pose mismatch')
        pose = poses.get(anchor.source_id)
        if (pose is None or pose.xy != (anchor.x, anchor.y) or pose.yaw != anchor.yaw
                or not np.isclose(pose.relevance, anchor.relevance, rtol=0, atol=1e-12)):
            raise ValueError('exact task support pose disagrees with its winner anchor')


def assess_candidates(field, belief, catalog, *, task=None, operational=None, evaluated_candidates=None):
    """Confirm measured ground support, never full navigation/clearance.

    v1 uses raw A2 FREE; v1.1 uses shared blocking plus real ground votes.
    With no cached task, supplying original evaluations selects exact anchors.
    """
    if task is None:
        task = build_task_uncertainty(field, belief, operational=operational,
                                      evaluated_candidates=evaluated_candidates)
    elif evaluated_candidates is not None:
        if (task.anchor_semantics != 'exact-validated-winner-v1.2'
                or task.winner_anchors != reconstruct_winner_anchors(field, evaluated_candidates)):
            raise ValueError('provided task must match the requested exact winner anchors')
    if task.anchor_semantics == 'exact-validated-winner-v1.2':
        _check_exact_catalog(task, catalog)
    expected = 'v1' if operational is None else operational.operational_semantics
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
        if operational is not None and operational.ambiguous_endpoints is not None:
            from operational_gating import ambiguous_footprint_diagnostics
            record['ambiguous_subcell'] = asdict(ambiguous_footprint_diagnostics(
                operational, (candidate['x'], candidate['y']), candidate['yaw'], task.footprint))
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
    task = build_support_task(field, raw, belief, config, operational=operational)
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
    if task.anchor_semantics == 'exact-validated-winner-v1.2':
        choice['anchor_semantics'] = task.anchor_semantics
    return choice, ranking
