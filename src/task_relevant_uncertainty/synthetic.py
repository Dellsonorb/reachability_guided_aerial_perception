"""Exactly four small A3 fixtures, NOT real RM4D/IK or MID360 validation."""

from dataclasses import dataclass

import numpy as np

from environment_belief import EnvironmentBeliefGrid, EnvironmentBeliefMapper, EnvironmentGridSpec, PointCloudObservation
from reachability_guided_aerial_perception import GraspTCP, GridSpec, ManipulationInterestField, build_field_from_result


@dataclass(frozen=True)
class SyntheticScenario:
    name: str
    description: str
    a1: ManipulationInterestField
    a2: EnvironmentBeliefGrid
    grasp: GraspTCP
    a1_result: dict
    after_two: EnvironmentBeliefGrid | None = None


def _interest(poses):
    grasp = GraspTCP('a3-synthetic-grasp', 'map', (0, 0, 0.4), (0, 0, 0, 1))
    candidates = []
    for i, (x, y, yaw, relevance) in enumerate(poses):
        candidates.append({
            'candidate_id': f'synthetic-{i}', 'bunker_x': x, 'bunker_y': y, 'bunker_yaw': yaw,
            'rm4d_reachable': True, 'ik_valid': True, 'collision_free': True,
            'footprint_collision': False, 'valid': True, 'joint_margin_rad': relevance * 0.5,
            'fk_position_residual_m': 0.0, 'fk_orientation_residual_rad': 0.0,
            'rejection_reason': None,
        })
    raw = {
        'schema_version': 1, 'grasp_id': grasp.grasp_id, 'frame_id': 'map',
        'summary': {'inverse_reachable': len(candidates), 'deduplicated': len(candidates),
                    'validation_limit': 256, 'evaluated': len(candidates), 'valid': len(candidates),
                    'rejected_by_reason': {}},
        'evaluated_candidates': candidates, 'candidates': [],
    }
    a1 = build_field_from_result(grasp, raw, grid=GridSpec.centered((0, 0), 3, 3, 0.1))
    return a1, grasp, raw


def _observe(points_map, stamp):
    pose = np.eye(4)
    pose[:3, 3] = [(-1) ** stamp * 0.6, 0, 2]
    return PointCloudObservation(np.asarray(points_map) - pose[:3, 3], 'lidar', float(stamp), pose)


def make_scenarios():
    grid = EnvironmentGridSpec((-1.5, -1.5), 30, 30)
    high, grasp, high_result = _interest([(0.01, 0.01, 0, 0.9)])
    unknown = EnvironmentBeliefMapper(grid).snapshot()
    clear_mapper = EnvironmentBeliefMapper(grid)
    rows, cols = np.indices(grid.shape)
    ground = np.column_stack((-1.5 + (cols.ravel() + 0.5) * 0.1,
                              -1.5 + (rows.ravel() + 0.5) * 0.1, np.zeros(900)))
    after_two = None
    for stamp in range(8):
        clear_mapper.update(_observe(ground, stamp))
        if stamp == 1:
            after_two = clear_mapper.snapshot()
    low, _, low_result = _interest([(0.01, 0.01, 0, 0.2)])
    overlap, _, overlap_result = _interest([(0.01, 0.01, 0, 0.9), (0.65, 0.05, 0, 0.2)])
    occupied_mapper = EnvironmentBeliefMapper(grid)
    occupied_mapper.update(_observe([[-0.45, 0.05, 0.1]], 0))
    return (
        SyntheticScenario('high_unknown', 'R=.9, no environment observations', high, unknown, grasp, high_result),
        SyntheticScenario('high_free', 'R=.9, ground support N=2 then N=8', high,
                          clear_mapper.snapshot(), grasp, high_result, after_two),
        SyntheticScenario('low_unknown', 'R=.2, no environment observations', low, unknown, grasp, low_result),
        SyntheticScenario('occupied_overlap', 'High pose blocked at footprint edge; overlapping low pose survives',
                          overlap, occupied_mapper.snapshot(), grasp, overlap_result),
    )
