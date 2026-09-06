"""Three controlled demonstrations, not real IK/LiDAR or benchmark validation.

Only this fixture module constructs A1 inputs. The A4 algorithm reads A3/A2.
Endpoint histories are synthetic; no hidden obstacle truth enters the scorer.
"""

from dataclasses import dataclass, replace

import numpy as np

from environment_belief import EnvironmentBeliefGrid, EnvironmentBeliefMapper, EnvironmentGridSpec, PointCloudObservation
from reachability_guided_aerial_perception import GraspTCP, GridSpec, build_field_from_result
from task_relevant_uncertainty import TaskRelevantUncertaintyField, build_task_uncertainty
from .geometry import ground_targets
from .model import NBVConfig, Viewpoint, generate_candidates


@dataclass(frozen=True)
class SyntheticCase:
    name: str
    task: TaskRelevantUncertaintyField
    belief: EnvironmentBeliefGrid
    current: Viewpoint
    candidates: tuple[Viewpoint, ...]
    config: NBVConfig


@dataclass(frozen=True)
class SyntheticScenario:
    name: str
    description: str
    cases: tuple[SyntheticCase, ...]


def _interest(grid, poses):
    grasp = GraspTCP('a4-synthetic', 'map', (0, 0, .4), (0, 0, 0, 1))
    candidates = [dict(candidate_id=f'synthetic-{i}', bunker_x=x, bunker_y=y, bunker_yaw=0,
                       rm4d_reachable=True, ik_valid=True, collision_free=True, footprint_collision=False,
                       valid=True, joint_margin_rad=r * .5, fk_position_residual_m=0,
                       fk_orientation_residual_rad=0, rejection_reason=None)
                  for i, (x, y, r) in enumerate(poses)]
    count = len(candidates)
    raw = dict(schema_version=1, grasp_id=grasp.grasp_id, frame_id='map', candidates=[],
               summary=dict(inverse_reachable=count, deduplicated=count, validation_limit=256,
                            evaluated=count, valid=count, rejected_by_reason={}), evaluated_candidates=candidates)
    x0, x1, y0, y1 = grid.extent
    spec = GridSpec.centered(((x0 + x1) / 2, (y0 + y1) / 2), x1 - x0, y1 - y0, .1)
    return build_field_from_result(grasp, raw, grid=spec)


def _observation(points, stamp):
    transform = np.eye(4)
    transform[:3, 3] = [(-1) ** stamp * .2, 0, 2]
    return PointCloudObservation(np.asarray(points) - transform[:3, 3], 'lidar', float(stamp), transform)


def _environment(grid, unknown_rectangles):
    mapper = EnvironmentBeliefMapper(grid)
    points = ground_targets(mapper.snapshot())
    unknown = np.zeros(len(points), dtype=bool)
    for x0, x1, y0, y1 in unknown_rectangles:
        unknown |= ((points[:, 0] >= x0) & (points[:, 0] < x1)
                    & (points[:, 1] >= y0) & (points[:, 1] < y1))
    for stamp in range(8):
        mapper.update(_observation(points[~unknown], stamp))
    return mapper


def make_scenarios():
    config = NBVConfig()
    # 1: Eight headings at one location isolate task weighting from translation.
    grid = EnvironmentGridSpec((-8, -3), 160, 60)
    a1 = _interest(grid, [(4.01, .01, .9), (-4.1, -.5, .12), (-4.1, .5, .12),
                          (-5.2, -.5, .12), (-5.2, .5, .12)])
    belief = _environment(grid, [(3.4, 4.7, -.5, .6), (-6, -3, -1.5, 1.5)]).snapshot()
    current = Viewpoint((.05, .05, 1.5), 0)
    headings = generate_candidates(current, config=replace(config, xy_offsets_m=(0,)))
    task_case = SyntheticCase('heading_choice', build_task_uncertainty(a1, belief), belief,
                              current, headings, config)

    # 2: A barrier outside the task footprint changes visibility, not A3 support.
    grid = EnvironmentGridSpec((-2, -3), 120, 60)
    a1 = _interest(grid, [(4.01, .01, .9)])
    mapper = _environment(grid, [(3.4, 4.7, -.5, .6)])
    clear = mapper.snapshot()
    wall = np.column_stack((np.full(21, 3.05), np.arange(-1, 1.01, .1) + .05, np.full(21, .2)))
    mapper.update(_observation(wall, 8))
    blocked = mapper.snapshot()
    current = Viewpoint((-.45, .05, 1.5), 0)
    pair = (current, Viewpoint((8.55, .05, 1.5), np.pi))
    occlusion_cases = tuple(SyntheticCase(name, build_task_uncertainty(a1, b), b, current, pair, config)
                            for name, b in (('clear', clear), ('known_barrier', blocked)))

    # 3: Slightly larger relevance (.91 vs .90), but a ten-metre reposition.
    grid = EnvironmentGridSpec((-16, -3), 220, 60)
    a1 = _interest(grid, [(4.01, .01, .90), (-14.09, .01, .91)])
    belief = _environment(grid, [(3.4, 4.7, -.5, .6), (-14.7, -13.4, -.5, .6)]).snapshot()
    task = build_task_uncertainty(a1, belief)
    current = Viewpoint((.05, .05, 1.5), 0)
    pair = (current, Viewpoint((-10.05, .05, 1.5), np.pi))
    cost_cases = tuple(SyntheticCase(name, task, belief, current, pair, c)
                       for name, c in (('zero_cost', replace(config, flight_weight=0)), ('with_cost', config)))
    return (
        SyntheticScenario('task_vs_generic', 'Large low-R unknown west versus small high-R unknown east', (task_case,)),
        SyntheticScenario('occlusion', 'Known 1 m surrogate wall outside the manipulation footprint', occlusion_cases),
        SyntheticScenario('flight_cost', 'Small additional predicted gain versus 10.1 m reposition plus yaw', cost_cases),
    )
