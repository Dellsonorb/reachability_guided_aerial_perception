"""A6 contracts on real A1/A2/A3/A4 inputs and the existing file boundary."""

from dataclasses import asdict, replace
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from a6_pilot.policy import decide_policy, rm4d_top_one
from environment_belief import BeliefConfig, EnvironmentBeliefMapper, EnvironmentGridSpec, EnvironmentState
from reachability_guided_aerial_perception import GraspTCP, GridSpec, build_field_from_result
from reachability_guided_nbv import Viewpoint, predict_visibility
from reachability_guided_nbv.geometry import sensor_transform
from sim_active_perception.core import A5Config, candidate_catalog, decide, replay_observations
from sim_active_perception.worker import save_initial
from task_relevant_uncertainty import build_task_uncertainty
from task_relevant_uncertainty.geometry import footprint_cells
from tests.test_a5_core import ground_points, inputs, scan
from tests.test_field import candidate, result


METHODS = ('ours', 'generic', 'fixed', 'no_occlusion', 'no_cost')


def selected_visibility(choice):
    payload = choice['policy_visibility']
    mask = np.zeros(payload['shape'], dtype=bool)
    mask.ravel()[payload['cell_ids']] = True
    return mask


class A6PolicyTests(unittest.TestCase):
    def setUp(self):
        self.grid = EnvironmentGridSpec((-1.5, -1.5), 30, 30)
        self.current = Viewpoint((-4, 0, 1.5), 0)
        self.field, self.raw = inputs([candidate(candidate_id='first', x=.011, y=.019, margin=.3)])
        self.belief = EnvironmentBeliefMapper(self.grid).snapshot()
        self.config = A5Config()

    def policy(self, method, *, field=None, raw=None, belief=None, current=None, round_count=1, config=None):
        return decide_policy(self.field if field is None else field, self.raw if raw is None else raw,
                             self.belief if belief is None else belief,
                             self.current if current is None else current, method=method,
                             round_count=round_count, config=self.config if config is None else config)

    def assert_shared(self, actual, expected):
        self.assertEqual(actual.candidates, expected.candidates)
        self.assertEqual(actual.task_order, expected.task_order)
        self.assertEqual(actual.generic_order, expected.generic_order)
        self.assertEqual(actual.status, expected.status)
        self.assertEqual(actual.config, expected.config)
        for name in ('visibility', 'delta_unknown', 'marginal_task_gain', 'support_state',
                     'best_operational_source', 'a1_cell_state'):
            np.testing.assert_array_equal(getattr(actual, name), getattr(expected, name))

    def test_ours_exactly_equals_frozen_choice_and_ranking_on_real_snapshots(self):
        free = replay_observations(self.grid, [scan(ground_points(self.grid), stamp) for stamp in (1, 2)])
        empty_field, empty_raw = inputs([])
        for field, raw, belief, current, count, config in (
                (self.field, self.raw, self.belief, self.current, 1, self.config),
                (self.field, self.raw, free, self.current, 3, self.config),
                (empty_field, empty_raw, self.belief, self.current, 1, self.config),
                (self.field, self.raw, self.belief, Viewpoint((4, 0, 1.5), 0), 3,
                 replace(self.config, flight_weight=100000))):
            with self.subTest(round=count, candidates=len(raw['evaluated_candidates']), pose=current):
                expected, reference = decide(field, raw, belief, current, round_count=count, config=config)
                choice, ranking = self.policy('ours', field=field, raw=raw, belief=belief, current=current,
                                              round_count=count, config=config)
                self.assertEqual(choice, expected)
                self.assert_shared(ranking, reference)

    def test_generic_reuses_candidate_order_visibility_cost_and_gain_identity(self):
        ours, reference = self.policy('ours')
        choice, ranking = self.policy('generic')
        self.assert_shared(ranking, reference)
        best = ranking.best_generic
        self.assertEqual(choice['next_viewpoint'], [*best.viewpoint.position_xyz, best.viewpoint.yaw_rad])
        self.assertEqual(choice['policy_order'], list(ranking.generic_order))
        self.assertEqual(choice['policy_best_candidate_id'], best.candidate_id)
        self.assertEqual(choice['policy_best_gain'], best.generic_gain)
        self.assertEqual(choice['policy_best_score'], best.generic_score)
        task = build_task_uncertainty(self.field, self.belief)
        alpha = -np.expm1(-1 / self.belief.config.unknown_scale)
        for row, visible in zip(ranking.candidates, ranking.visibility):
            self.assertAlmostEqual(row.generic_gain, alpha * np.sum(visible * self.belief.unknown_score))
            self.assertAlmostEqual(row.task_gain, alpha * np.nansum(visible * task.task_relevant_uncertainty))
        self.assertEqual(choice['selected_candidate'], ours['selected_candidate'])

    def test_generic_positive_gain_survives_no_task_support(self):
        field, raw = inputs([])
        choice, ranking = self.policy('generic', field=field, raw=raw)
        self.assertEqual(ranking.status, 'NO_PREDICTED_TASK_GAIN')
        self.assertGreater(choice['policy_best_gain'], 0)
        self.assertGreater(choice['policy_best_score'], 0)
        self.assertIsNone(choice['stop_reason'])
        self.assertIsNotNone(choice['next_viewpoint'])
        self.assertIsNone(choice['selected_candidate'])

    def test_gain_then_score_then_budget_precedence(self):
        short = EnvironmentBeliefMapper(self.grid, BeliefConfig(max_range_m=.3)).snapshot()
        for method in ('ours', 'generic', 'no_occlusion', 'no_cost'):
            with self.subTest(method=method):
                choice, _ = self.policy(method, belief=short, round_count=3)
                self.assertEqual(choice['stop_reason'], 'NO_PREDICTED_GENERIC_GAIN' if method == 'generic'
                                 else 'NO_PREDICTED_TASK_GAIN')
                choice, _ = self.policy(method, round_count=3)
                self.assertEqual(choice['stop_reason'], 'VIEW_BUDGET_REACHED')
                self.assertIsNone(choice['next_viewpoint'])
        for method in ('ours', 'generic', 'no_occlusion'):
            with self.subTest(nonpositive=method):
                choice, _ = self.policy(method, current=Viewpoint((4, 0, 1.5), 0), round_count=3,
                                         config=replace(self.config, flight_weight=100000))
                self.assertEqual(choice['stop_reason'], 'NONPOSITIVE_SCORE')

    def test_no_valid_sensor_precedes_budget_for_adaptive_methods(self):
        config = replace(self.config, flight_bounds=(-4, 4, -3, 3, -2, 3))
        for method in ('ours', 'generic', 'no_occlusion', 'no_cost'):
            with self.subTest(method=method):
                choice, _ = self.policy(method, current=Viewpoint((-4, 0, -1), 0),
                                         config=config, round_count=3)
                self.assertEqual(choice['stop_reason'], 'NO_VALID_CANDIDATE')
                self.assertIsNone(choice['next_viewpoint'])

    def test_fixed_stays_at_measured_pose_for_exactly_three_windows_without_gain(self):
        field, raw = inputs([])
        for count in (1, 2, 3):
            current = Viewpoint((-.13 + .01 * count, .02, 1.48), .12)
            choice, _ = self.policy('fixed', field=field, raw=raw, current=current, round_count=count)
            self.assertEqual(choice['stop_reason'], 'VIEW_BUDGET_REACHED' if count == 3 else None)
            self.assertEqual(choice['next_viewpoint'], None if count == 3 else
                             [*current.position_xyz, current.yaw_rad])

    def test_confirmed_ground_candidate_does_not_stop_discovery_early(self):
        belief = replay_observations(self.grid, [scan(ground_points(self.grid), stamp) for stamp in (1, 2)])
        for method in METHODS:
            with self.subTest(method=method):
                choice, _ = self.policy(method, belief=belief, round_count=2)
                self.assertEqual(choice['selected_candidate']['candidate_id'], 'first')
                self.assertIsNone(choice['stop_reason'])
                self.assertIsNotNone(choice['next_viewpoint'])

    def test_every_method_keeps_exact_catalog_winner_and_first_tie_across_cells(self):
        field, raw = inputs([
            candidate(candidate_id='weak', x=.298, y=.219, yaw=.2, margin=.1),
            candidate(candidate_id='winner-first', x=.299, y=.219, yaw=.2, margin=.3),
            candidate(candidate_id='same-cell-tie', x=.298, y=.22, yaw=.2, margin=.3),
            candidate(candidate_id='earlier-cell-second', x=-.399, y=-.419, margin=.3)])
        belief = replay_observations(self.grid, [scan(ground_points(self.grid), stamp) for stamp in (1, 2)])
        expected, _ = decide(field, raw, belief, self.current, round_count=3)
        for method in METHODS:
            with self.subTest(method=method):
                choice, _ = self.policy(method, field=field, raw=raw, belief=belief, round_count=3)
                self.assertEqual(choice['selected_candidate'], expected['selected_candidate'])
                self.assertEqual(choice['selected_candidate']['candidate_id'], 'winner-first')
                self.assertEqual(choice['selected_candidate']['x'], .299)

    def test_both_representative_and_exact_footprint_and_clipping_remain_gates(self):
        for x, y, yaw, extra_kind in ((.099, .019, 0, 'exact'), (.011, .019, .2, 'representative'),
                                      (1.49, 0, 0, 'clipped')):
            field, raw = inputs([candidate(candidate_id=extra_kind, x=x, y=y, yaw=yaw, margin=.3)])
            frames = [scan(ground_points(self.grid), stamp) for stamp in (1, 2)]
            if extra_kind != 'clipped':
                exact, _ = footprint_cells(self.grid, (x, y), yaw)
                representative, _ = footprint_cells(self.grid, (.05, .05), yaw)
                extra = np.setdiff1d(exact, representative) if extra_kind == 'exact' else np.setdiff1d(representative, exact)
                self.assertGreater(len(extra), 0)
                point = ground_points(self.grid)[extra[0]].copy()
                point[2] = .2
                frames.append(scan([point], 3))
            belief = replay_observations(self.grid, frames)
            for method in METHODS:
                with self.subTest(method=method, gate=extra_kind):
                    choice, _ = self.policy(method, field=field, raw=raw, belief=belief, round_count=3)
                    self.assertIsNone(choice['selected_candidate'])

    def test_no_cost_changes_only_penalty_and_retains_frozen_tie_break(self):
        reference, base = self.policy('ours', config=replace(self.config, flight_weight=17))
        choice, shared = self.policy('no_cost', config=replace(self.config, flight_weight=17))
        expected, zero = decide(self.field, self.raw, self.belief, self.current, round_count=1,
                                config=replace(self.config, flight_weight=0))
        self.assert_shared(shared, base)
        self.assertEqual(choice['next_viewpoint'], expected['next_viewpoint'])
        self.assertEqual(choice['policy_order'], list(zero.task_order))
        self.assertEqual(choice['policy_candidates'], [asdict(row) for row in zero.candidates])
        np.testing.assert_array_equal(selected_visibility(choice), base.visibility[choice['policy_best_candidate_id']])
        self.assertEqual(choice['best_task_score'], reference['best_task_score'])
        for original, changed in zip(base.candidates, choice['policy_candidates']):
            self.assertEqual(changed['task_gain'], original.task_gain)
            self.assertEqual(changed['generic_gain'], original.generic_gain)
            self.assertEqual(changed['flight_cost'], original.flight_cost)
            if original.status == 'VALID':
                self.assertEqual(changed['task_score'], original.task_gain)

    def occlusion_inputs(self, points=()):
        grid = EnvironmentGridSpec((-1, -1), 70, 20)
        raw = result([candidate(candidate_id='target', x=4.05, y=.05, margin=.3)])
        grasp = GraspTCP('occlusion', 'map', (2.5, 0, .4), (0, 0, 0, 1))
        raw['grasp_id'] = grasp.grasp_id
        field = build_field_from_result(grasp, raw, grid=GridSpec.centered((2.5, 0), 7, 2, .1))
        mapper = EnvironmentBeliefMapper(grid, sensor_frame='uav1/lidar_link')
        if len(points):
            mapper.update(scan(points, 1))
        return field, raw, mapper.snapshot()

    def test_no_occlusion_above_and_through_prisms_use_range_fov_only(self):
        current = Viewpoint((0, 0, 1.5), 0)
        config = replace(self.config, xy_offsets_m=(0,))
        for obstacle, occluded in ((1.05, False), (3.05, True)):
            field, raw, belief = self.occlusion_inputs([(obstacle, .05, .2)])
            choice, base = self.policy('no_occlusion', field=field, raw=raw, belief=belief,
                                       current=current, config=config)
            prediction = predict_visibility(belief, current, sensor=base.sensor, config=base.config)
            with self.subTest(obstacle=obstacle):
                self.assertEqual(bool(prediction.occluded[10, 50]), occluded)
                self.assertTrue(selected_visibility(choice)[10, 50])
                self.assertEqual(bool(base.visibility[0, 10, 50]), not occluded)
                np.testing.assert_array_equal(selected_visibility(choice), prediction.range_fov)
                self.assertEqual(choice['policy_candidates'][0]['flight_cost'], base.candidates[0].flight_cost)
                self.assertAlmostEqual(choice['policy_candidates'][0]['task_gain'],
                                       np.nansum(base.marginal_task_gain * prediction.range_fov))
                self.assertAlmostEqual(choice['policy_candidates'][0]['generic_gain'],
                                       np.sum(base.delta_unknown * prediction.range_fov))

    def test_no_occlusion_keeps_occupied_targets_a3_blockers_range_and_invalid_origin(self):
        current = Viewpoint((0, 0, 1.5), 0)
        config = replace(self.config, xy_offsets_m=(0,))
        field, raw, belief = self.occlusion_inputs([(4.05, .05, .2)])
        choice, base = self.policy('no_occlusion', field=field, raw=raw, belief=belief,
                                   current=current, config=config)
        self.assertFalse(selected_visibility(choice)[10, 50])
        self.assertEqual(choice['policy_candidates'][0]['task_gain'], 0)
        self.assertEqual(choice['stop_reason'], 'NO_PREDICTED_TASK_GAIN')
        field, raw, belief = self.occlusion_inputs()
        belief = replace(belief, config=replace(belief.config, max_range_m=3))
        choice, _ = self.policy('no_occlusion', field=field, raw=raw, belief=belief,
                                current=current, config=config)
        self.assertFalse(selected_visibility(choice)[10, 50])
        current = Viewpoint((0, 0, .5), 0)
        origin = sensor_transform(current)[:3, 3]
        field, raw, belief = self.occlusion_inputs([(origin[0], origin[1], .2)])
        choice, base = self.policy('no_occlusion', field=field, raw=raw, belief=belief,
                                   current=current, config=config)
        self.assertEqual(base.candidates[0].status, 'SENSOR_INSIDE_ASSUMED_PRISM')
        self.assertEqual(choice['policy_candidates'][0]['status'], 'SENSOR_INSIDE_ASSUMED_PRISM')
        self.assertIsNone(choice['policy_visibility'])
        self.assertEqual(choice['stop_reason'], 'NO_VALID_CANDIDATE')

    def test_unsupported_method_rejected(self):
        with self.assertRaisesRegex(ValueError, 'method'):
            self.policy('rm4d_only')

    def test_unsupported_method_is_rejected_before_frozen_evaluation(self):
        with self.assertRaisesRegex(ValueError, 'method'):
            decide_policy(None, None, None, None, method='unsupported', round_count=1)

    def test_all_methods_reject_nonpilot_budget(self):
        for method in METHODS:
            for budget in (2, 4):
                with self.subTest(method=method, budget=budget):
                    with self.assertRaisesRegex(ValueError, 'max_viewpoints.*3'):
                        self.policy(method, config=replace(self.config, max_viewpoints=budget))

    def test_rm4d_top_one_preserves_raw_ranking_and_never_uses_catalog(self):
        first = candidate(candidate_id='raw-first', x=.0123, y=.0213, yaw=.17, margin=.1,
                          final_score=-17.4, joint_solution=[.1, .2])
        second = candidate(candidate_id='catalog-winner', x=.019, y=.025, margin=.4, final_score=200)
        field, raw = inputs([first, second])
        raw['candidates'] = [first, second]
        self.assertEqual(candidate_catalog(field, raw)[0]['candidate_id'], 'catalog-winner')
        selected = rm4d_top_one(raw)
        for key, value in first.items():
            self.assertEqual(selected[key], value)
        self.assertEqual([selected[key] for key in ('x', 'y', 'yaw')], [.0123, .0213, .17])
        self.assertIsNone(rm4d_top_one(dict(raw, candidates=[])))


class A6WorkerTests(unittest.TestCase):
    def run_worker(self, directory, request):
        request_path, response_path = directory / 'request.json', directory / 'response.json'
        request_path.write_text(json.dumps(request))
        process = subprocess.run([sys.executable, 'scripts/a6_core_worker.py', '--request', str(request_path),
                                  '--response', str(response_path)], capture_output=True, text=True)
        self.assertTrue(response_path.exists(), process.stderr)
        return process, json.loads(response_path.read_text())

    def test_worker_ordered_history_matches_direct_decision_and_saves_shared_outputs(self):
        grid = EnvironmentGridSpec((-1.5, -1.5), 30, 30)
        field, raw = inputs([candidate(candidate_id='first', x=.011, y=.019, margin=.3)])
        grasp = GraspTCP('a5-test', 'map', (0, 0, .4), (0, 0, 0, 1))
        config = A5Config(grid_width_m=3, grid_height_m=3)
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            initial = save_initial(grasp, raw, config, directory)
            paths, observations = [], []
            for stamp in (1, 2):
                observation = scan(ground_points(grid), stamp)
                observations.append(observation)
                path = directory / f'cloud-{stamp}.npz'
                np.savez_compressed(path, points_xyz=observation.points_xyz,
                                    T_map_sensor=observation.T_map_sensor,
                                    frame_id=observation.frame_id, stamp_s=observation.stamp_s)
                paths.append(str(path))
            for method in ('ours', 'generic', 'fixed', 'no_occlusion', 'no_cost'):
                with self.subTest(method=method):
                    output = directory / method
                    request = dict(op='observe', method=method, initial_file=initial['initial_file'],
                                   observations=paths, uav_pose=[-4, 0, 1.5, 0], output_dir=str(output))
                    process, response = self.run_worker(directory, request)
                    self.assertEqual(process.returncode, 0, process.stderr)
                    expected, _ = decide_policy(field, raw, replay_observations(grid, observations),
                                                Viewpoint((-4, 0, 1.5), 0), method=method,
                                                round_count=2, config=config)
                    timing = response['computation_timing']
                    self.assertGreaterEqual(timing['worker_observe_wall_s'], timing['policy_wall_s'])
                    self.assertGreaterEqual(timing['policy_wall_s'], 0)
                    self.assertEqual({k:v for k,v in response.items() if k!='computation_timing'},
                                     json.loads(json.dumps(expected)))
                    self.assertEqual(response['total_observation_votes'], 1800)
                    for name in ('decision.json', 'ranking.json', 'fields.npz', 'nbv.png'):
                        self.assertTrue((output / name).exists(), name)
                    self.assertTrue((output / 'a2').is_dir())
            request['observations'] = list(reversed(paths))
            process, response = self.run_worker(directory, request)
            self.assertNotEqual(process.returncode, 0)
            self.assertIn('increasing', response['error'])

    def test_worker_rm4d_select_requires_no_cloud_or_field_and_preserves_top_one(self):
        first = candidate(candidate_id='raw-first', x=.011, y=.019, yaw=.2, final_score=-7)
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            initial = directory / 'initial.json'
            for candidates in ([first], []):
                initial.write_text(json.dumps({'config': {'max_viewpoints': 3},
                                               'result': {'candidates': candidates}}))
                process, response = self.run_worker(directory, dict(op='rm4d_select', initial_file=str(initial)))
                self.assertEqual(process.returncode, 0, process.stderr)
                self.assertTrue(response['ok'])
                self.assertEqual(response['selected_candidate'], rm4d_top_one({'candidates': candidates}))

    def test_worker_rejects_bad_budget_before_init_field_cloud_or_selection(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            initial = directory / 'initial.json'
            for budget in (2, 4):
                config = {'max_viewpoints': budget}
                initial.write_text(json.dumps({'config': config, 'result': {'candidates': []}}))
                for op in ('init', 'observe', 'rm4d_select'):
                    with self.subTest(budget=budget, op=op):
                        request = (dict(op=op, config=config) if op == 'init' else
                                   dict(op=op, initial_file=str(initial)))
                        process, response = self.run_worker(directory, request)
                        self.assertNotEqual(process.returncode, 0)
                        self.assertFalse(response['ok'])
                        self.assertRegex(response['error'], 'max_viewpoints.*3')

    def test_worker_valid_init_preserves_frozen_request_and_return_exactly(self):
        from a6_pilot import worker
        from sim_active_perception import worker as frozen
        # Isolate only the external RM4D initialization boundary; the A6 guard
        # and pass-through are real, with the complete frozen response shape.
        expected = dict(ok=True, initial_file='/tmp/initial.json', candidate_count=1,
                        a1_status='OK', evaluated=1, rm4d_valid=1, baseline_valid=1)
        for values in ({}, {'config': {'max_viewpoints': 3}}):
            with self.subTest(values=values):
                request = dict(op='init', **values)
                with patch.object(frozen, 'initialize', return_value=expected) as initialize:
                    self.assertIs(worker.initialize(request), expected)
                    initialize.assert_called_once_with(request)

    def test_worker_errors_are_structured(self):
        with tempfile.TemporaryDirectory() as tmp:
            process, response = self.run_worker(Path(tmp), {'op': 'missing'})
            self.assertNotEqual(process.returncode, 0)
            self.assertFalse(response['ok'])
            self.assertIn('unsupported', response['error'])


if __name__ == '__main__':
    unittest.main()
