"""Exact original winners supply A3 support and A5 confirmation in v1.2."""

import copy
from dataclasses import FrozenInstanceError, asdict, fields, replace
import importlib.util
import inspect
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from a6_pilot.policy import METHODS, decide_policy
from environment_belief import EnvironmentGridSpec
from operational_gating import OccupiedClass, PerceivedTarget, assess_footprint, derive_operational_evidence
from operational_gating.io import build_operational_context, record_initial_context
from reachability_guided_aerial_perception import CellState, GraspTCP
from sim_active_perception import core as a5
import task_relevant_uncertainty as a3
from task_relevant_uncertainty.core import FIELD_ARRAY_NAMES
from task_relevant_uncertainty.geometry import footprint_cells
from task_relevant_uncertainty.outputs import field_summary
from tests.test_a5_core import ground_points, inputs, scan
from tests.test_field import candidate


class ExactSupportTests(unittest.TestCase):
    def setUp(self):
        self.grid = EnvironmentGridSpec((-1.5, -1.5), 30, 30)
        self.field, self.raw = inputs([
            candidate(candidate_id='z-first', x=.011, y=.019, margin=.3),
            candidate(candidate_id='a-tie', x=.012, y=.02, margin=.3)])
        # Exact xmax=.531, center xmax=.57, target xmin=.56.
        self.target = PerceivedTarget((.68, .019, .0575), 0)
        self.observations = [scan(ground_points(self.grid), stamp) for stamp in (1., 2.)]
        self.belief = a5.replay_observations(self.grid, self.observations)
        self.view = derive_operational_evidence(self.grid, self.observations, self.target)

    def reconstruct(self, field=None, evaluated=None):
        helper = getattr(a3, 'reconstruct_winner_anchors', None)
        self.assertTrue(callable(helper), 'exact winner reconstruction must be public')
        return helper(self.field if field is None else field,
                      self.raw['evaluated_candidates'] if evaluated is None else evaluated)

    def exact_task(self, *, field=None, raw=None, belief=None, view=None):
        self.assertIn('evaluated_candidates', inspect.signature(a3.build_task_uncertainty).parameters)
        return a3.build_task_uncertainty(
            self.field if field is None else field, self.belief if belief is None else belief,
            evaluated_candidates=(self.raw if raw is None else raw)['evaluated_candidates'],
            operational=self.view if view is None else view)

    def config(self):
        self.assertIn('support_anchor', {f.name for f in fields(a5.A5Config)})
        return a5.A5Config(grid_width_m=3, grid_height_m=3, support_anchor='exact_winner')

    def test_original_first_tie_and_saturated_tie_ignore_ids_and_raw_margin(self):
        for margins in ((.3, .3), (.5, 2.)):
            with self.subTest(margins=margins):
                field, raw = inputs([
                    candidate(candidate_id='z-first', x=.011, y=.019, yaw=.2, margin=margins[0]),
                    candidate(candidate_id='a-later', x=.012, y=.02, yaw=.8, margin=margins[1])])
                anchors = self.reconstruct(field, raw['evaluated_candidates'])
                self.assertIsInstance(anchors, tuple)
                self.assertEqual(len(anchors), 1)
                winner = anchors[0]
                self.assertEqual((winner.candidate_id, winner.evaluation_index), ('z-first', 0))
                self.assertEqual((winner.x, winner.y, winner.yaw), (.011, .019, .2))
                with self.assertRaises(FrozenInstanceError):
                    winner.x = .05
                self.assertEqual(a5.candidate_catalog(field, raw), [dict(
                    candidate_id='z-first', x=.011, y=.019, yaw=.2,
                    relevance=min(1., margins[0] / .5), source_id=465, evaluation_index=0)])

    def test_higher_later_winner_replaces_and_return_order_is_original_index(self):
        field, raw = inputs([
            candidate(candidate_id='weak', x=.011, y=.019, margin=.1),
            candidate(candidate_id='other', x=.299, y=.219, margin=.3),
            candidate(candidate_id='strong', x=.012, y=.02, margin=.4)])
        anchors = self.reconstruct(field, raw['evaluated_candidates'])
        self.assertEqual([a.candidate_id for a in anchors], ['other', 'strong'])
        self.assertEqual([a.evaluation_index for a in anchors], [1, 2])

    def test_missing_mismatched_and_nonfinite_originals_raise_value_error(self):
        self.reconstruct()
        for name, value in [('relevance', .7), ('best_yaw', .8)]:
            array = getattr(self.field, name).copy()
            array[15, 15] = value
            with self.subTest(field=name), self.assertRaises(ValueError):
                self.reconstruct(replace(self.field, **{name: array}))
        for change in ({'bunker_x': .111}, {'bunker_yaw': np.nan}, {'bunker_x': True},
                       {'bunker_y': float('inf')}, {'bunker_x': 10 ** 400}, {'candidate_id': ''}):
            raw = copy.deepcopy(self.raw)
            raw['evaluated_candidates'][0].update(change)
            with self.subTest(candidate=change), self.assertRaises(ValueError):
                self.reconstruct(evaluated=raw['evaluated_candidates'])
        for values in ([], self.raw['evaluated_candidates'][:1], {}, [None, None]):
            with self.subTest(originals=values), self.assertRaises(ValueError):
                self.reconstruct(evaluated=values)
        states = self.field.cell_state.copy()
        states[15, 15] = CellState.UNASSESSED
        with self.assertRaises(ValueError):
            self.reconstruct(replace(self.field, cell_state=states))
        for raw in ({}, None, []):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                a5.candidate_catalog(self.field, raw)

    def test_exact_clear_center_collision_changes_support_without_mutating_inputs(self):
        before_a1 = {f.name: getattr(self.field, f.name).copy() for f in fields(self.field)
                     if isinstance(getattr(self.field, f.name), np.ndarray)}
        before_a2 = {f.name: getattr(self.belief, f.name).copy() for f in fields(self.belief)
                     if isinstance(getattr(self.belief, f.name), np.ndarray)}
        originals = copy.deepcopy(self.raw)
        legacy = a3.build_task_uncertainty(self.field, self.belief, operational=self.view)
        task = self.exact_task()
        self.assertTrue(legacy.poses[0].blocked)
        self.assertFalse(task.poses[0].blocked)
        self.assertEqual(task.poses[0].xy, (.011, .019))
        self.assertEqual(task.poses[0].yaw, 0)
        direct = assess_footprint(self.view, task.poses[0].xy, task.poses[0].yaw)
        self.assertEqual(task.poses[0].covered_environment_cells, direct.covered_cells)
        self.assertGreater(np.nansum(task.task_relevant_uncertainty), 0)
        self.assertEqual(np.nansum(legacy.task_relevant_uncertainty), 0)
        cells = list(direct.covered_cells)
        np.testing.assert_array_equal(task.nominal_task_relevance.ravel()[cells], .6)
        np.testing.assert_array_equal(task.task_relevance_at_environment_cell.ravel()[cells], .6)
        np.testing.assert_array_equal(task.task_relevant_uncertainty,
                                      task.task_relevance_at_environment_cell * self.belief.unknown_score)
        for name, value in before_a1.items():
            np.testing.assert_array_equal(getattr(self.field, name), value)
        for name, value in before_a2.items():
            np.testing.assert_array_equal(getattr(self.belief, name), value)
        self.assertEqual(self.raw, originals)

    def test_task_aggregate_ties_still_follow_row_major_source_order(self):
        field, raw = inputs([
            candidate(candidate_id='later-cell-first', x=.111, y=.019, margin=.3),
            candidate(candidate_id='earlier-cell-second', x=.011, y=.019, margin=.3)])
        view = derive_operational_evidence(self.grid, [], PerceivedTarget((2, 2, 1), 0))
        task = self.exact_task(field=field, raw=raw, view=view)
        self.assertEqual([a.evaluation_index for a in task.winner_anchors], [0, 1])
        self.assertEqual([p.source_id for p in task.poses], [465, 466])
        overlap = np.intersect1d(*[p.covered_environment_cells for p in task.poses])
        self.assertGreater(len(overlap), 0)
        for name in ('best_nominal_source', 'best_operational_source'):
            np.testing.assert_array_equal(getattr(task, name).ravel()[overlap], 465)

    def test_exact_confirmation_has_no_extra_center_veto_and_supports_task_none(self):
        task = self.exact_task()
        catalog = a5.candidate_catalog(self.field, self.raw)
        cached = a5.assess_candidates(self.field, self.belief, catalog, task=task, operational=self.view)
        self.assertTrue(cached[0]['confirmed'])
        self.assertFalse(cached[0]['representative_blocked'])
        direct = a5.assess_candidates(self.field, self.belief, catalog, operational=self.view,
                                      evaluated_candidates=self.raw['evaluated_candidates'])
        self.assertEqual(cached, direct)
        self.assertFalse(a5.assess_candidates(self.field, self.belief, catalog,
                                            operational=self.view)[0]['confirmed'])

    def test_exact_task_catalog_and_pose_mismatch_are_rejected(self):
        task = self.exact_task()
        catalog = a5.candidate_catalog(self.field, self.raw)
        for change in ({'x': .05}, {'yaw': .2}, {'candidate_id': 'other'},
                       {'evaluation_index': 1}, {'relevance': .7}, {'source_id': 466}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                a5.assess_candidates(self.field, self.belief, [dict(catalog[0], **change)],
                                     task=task, operational=self.view)
        for invalid in ([], catalog * 2):
            with self.assertRaises(ValueError):
                a5.assess_candidates(self.field, self.belief, invalid, task=task, operational=self.view)
        altered = replace(task, poses=(replace(task.poses[0], xy=(.05, .05)),))
        with self.assertRaises(ValueError):
            a5.assess_candidates(self.field, self.belief, catalog, task=altered, operational=self.view)

    def test_one_missing_ground_vote_and_unknown_never_confirm(self):
        exact_cells, _ = footprint_cells(self.grid, (.011, .019), 0)
        for missing in (int(exact_cells[0]), None):
            points = np.delete(ground_points(self.grid), missing, axis=0) if missing is not None else []
            frames = [scan(points, t) for t in (1., 2.)]
            belief = a5.replay_observations(self.grid, frames)
            view = derive_operational_evidence(self.grid, frames, self.target)
            task = self.exact_task(belief=belief, view=view)
            self.assertFalse(task.poses[0].blocked)
            self.assertEqual(task.poses[0].environment_state, a3.PoseEnvironmentState.UNCONFIRMED)
            assessment = a5.assess_candidates(self.field, belief, a5.candidate_catalog(self.field, self.raw),
                                              task=task, operational=view)[0]
            self.assertFalse(assessment['confirmed'])
            self.assertEqual(assessment['operational']['ground_missing_cells'],
                             1 if missing is not None else len(exact_cells))
            self.assertGreater(np.nansum(task.task_relevant_uncertainty), 0)

    def test_true_collision_environment_and_ambiguous_remain_blocking(self):
        for kind in ('collision', OccupiedClass.ENVIRONMENT, OccupiedClass.AMBIGUOUS):
            points = (ground_points(self.grid) if kind == 'collision' else
                      np.vstack((ground_points(self.grid), [.05, .05, .2])))
            frames = [scan(points, t) for t in (1., 2.)]
            label = OccupiedClass.ENVIRONMENT if kind == 'collision' else kind
            labels = [np.full(len(points), label, dtype=np.int8)] * 2
            target = PerceivedTarget((.2, .019, .0575), 0) if kind == 'collision' else self.target
            view = derive_operational_evidence(self.grid, frames, target, labels=labels)
            belief = a5.replay_observations(self.grid, frames)
            task = self.exact_task(belief=belief, view=view)
            direct = assess_footprint(view, (.011, .019), 0)
            with self.subTest(kind=kind):
                self.assertTrue(task.poses[0].blocked)
                self.assertTrue(direct.blocked)
                self.assertEqual(np.nansum(task.task_relevant_uncertainty), 0)
                assessment = a5.assess_candidates(self.field, belief, a5.candidate_catalog(self.field, self.raw),
                                                  task=task, operational=view)[0]
                self.assertFalse(assessment['confirmed'])
                self.assertEqual(assessment['operational'], asdict(direct))
                if kind == 'collision':
                    self.assertTrue(direct.target_collision)

    def test_exact_clipped_footprint_still_cannot_confirm(self):
        field, raw = inputs([candidate(candidate_id='edge', x=1.49, y=0, margin=.3)])
        view = derive_operational_evidence(self.grid, self.observations, PerceivedTarget((-2, -2, 1), 0))
        task = self.exact_task(field=field, raw=raw, view=view)
        self.assertTrue(task.poses[0].footprint_clipped)
        assessment = a5.assess_candidates(field, self.belief, a5.candidate_catalog(field, raw),
                                          task=task, operational=view)[0]
        self.assertFalse(assessment['confirmed'])

    def test_empty_exact_and_unassessed_keep_nan_support_without_fallback(self):
        for candidates in ([], [candidate(valid=False)], [candidate(x=5, y=5)]):
            field, raw = inputs(candidates)
            task = self.exact_task(field=field, raw=raw)
            self.assertEqual(task.anchor_semantics, 'exact-validated-winner-v1.2')
            self.assertEqual(task.winner_anchors, ())
            self.assertEqual(task.poses, ())
            self.assertTrue(np.isnan(task.task_relevant_uncertainty).all())
            np.testing.assert_array_equal(task.support_state, a3.SupportState.NO_VALIDATED_SUPPORT)
        with self.assertRaises(ValueError):
            a3.build_task_uncertainty(self.field, self.belief, evaluated_candidates=[])

    def test_exact_metadata_keeps_pose_record_shape_and_legacy_output(self):
        legacy = a3.build_task_uncertainty(self.field, self.belief)
        task = self.exact_task()
        self.assertEqual(legacy.anchor_semantics, 'cell-center')
        self.assertEqual(legacy.winner_anchors, ())
        self.assertEqual(set(asdict(legacy.poses[0])), set(asdict(task.poses[0])))
        old = field_summary(legacy)
        summary = field_summary(task)
        self.assertNotIn('anchor_semantics', old)
        self.assertNotIn('winner_anchors', old)
        self.assertFalse(old['representative_pose_ik_validated'])
        self.assertEqual(summary['anchor_semantics'], 'exact-validated-winner-v1.2')
        self.assertEqual(summary['winner_anchors'], [asdict(a) for a in self.reconstruct()])
        self.assertIn('original_evaluation_index', summary['anchor_tie_rule'])
        self.assertTrue(summary['representative_pose_ik_validated'])
        self.assertIn('original', summary['representative_pose_validation'])

    def test_all_policy_routes_use_same_exact_anchors_and_generic_ours_share_a4(self):
        config = self.config()
        from reachability_guided_nbv import Viewpoint
        current = Viewpoint((-1, 0, 1.5), 0)
        task = self.exact_task()
        reference = None
        for method in METHODS:
            choice, ranking = decide_policy(self.field, self.raw, self.belief, current, method=method,
                                             round_count=2, config=config, operational=self.view)
            self.assertEqual(choice['selected_candidate']['candidate_id'], 'z-first')
            self.assertEqual(choice['anchor_semantics'], task.anchor_semantics)
            np.testing.assert_array_equal(ranking.marginal_task_gain,
                                          -np.expm1(-1 / self.belief.config.unknown_scale)
                                          * task.task_relevant_uncertainty)
            if reference is None:
                reference = choice, ranking
            else:
                self.assertEqual(choice['assessments'], reference[0]['assessments'])
                self.assertEqual(ranking.candidates, reference[1].candidates)
                np.testing.assert_array_equal(ranking.visibility, reference[1].visibility)
                np.testing.assert_array_equal(ranking.delta_unknown, reference[1].delta_unknown)

    def test_support_config_helper_and_cli_are_explicit_legacy_defaults(self):
        config = self.config()
        self.assertEqual(a5.A5Config().support_anchor, 'cell_center')
        with self.assertRaises(ValueError):
            a5.A5Config(support_anchor='nearest_clear')
        helper = getattr(a5, 'build_support_task', None)
        self.assertTrue(callable(helper))
        exact = helper(self.field, self.raw, self.belief, config, operational=self.view)
        self.assertEqual(exact.winner_anchors, self.reconstruct())
        for raw in ({}, None, {'evaluated_candidates': None}):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                helper(self.field, raw, self.belief, config, operational=self.view)
        legacy = helper(self.field, self.raw, self.belief, a5.A5Config(), operational=self.view)
        self.assertEqual(legacy.anchor_semantics, 'cell-center')
        spec = importlib.util.spec_from_file_location('exact_support_cli', 'scripts/run_a5_sim.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        args = sum(([name, 'unused'] for name in ('--output-dir', '--core-python', '--rm4d-root',
                                                 '--rm4d-config', '--rm4d-map')), [])
        self.assertEqual(module.build_parser().parse_args(args).support_anchor, 'cell_center')
        options = module.build_parser().parse_args(args + ['--operational-gating', 'v1.1',
                                                         '--support-anchor', 'exact_winner'])
        self.assertEqual(options.support_anchor, 'exact_winner')
        self.assertEqual(options.operational_gating, 'v1.1')

    def test_actual_a5_a6_workers_save_exact_task_arrays_used_by_ranking(self):
        from a6_pilot import worker as a6_worker
        from reachability_guided_nbv import Viewpoint
        from sim_active_perception import worker as a5_worker
        config = self.config()
        grasp = GraspTCP('a5-test', 'map', (0, 0, .4), (0, 0, 0, 1))
        for worker in (a5_worker, a6_worker):
            with self.subTest(worker=worker.__name__), tempfile.TemporaryDirectory() as directory:
                response = a5_worker.save_initial(grasp, self.raw, config, directory)
                record_initial_context(response['initial_file'], dict(
                    operational_gating='v1.1', perceived_target=asdict(self.target)))
                initial_path = Path(response['initial_file'])
                initial_bytes = initial_path.read_bytes()
                initial = json.loads(initial_bytes)
                paths = []
                for i, observation in enumerate(self.observations):
                    path = Path(directory) / f'observation-{i}.npz'
                    np.savez_compressed(path, points_xyz=observation.points_xyz, frame_id=observation.frame_id,
                                        stamp_s=observation.stamp_s, T_map_sensor=observation.T_map_sensor)
                    paths.append(str(path))
                output = Path(directory) / 'round'
                choice = worker.observe(dict(initial_file=str(initial_path), observations=paths,
                                             uav_pose=[-1, 0, 1.5, 0], output_dir=str(output), method='ours'))
                self.assertEqual(initial_path.read_bytes(), initial_bytes)
                self.assertEqual(initial['config']['support_anchor'], 'exact_winner')
                self.assertEqual(choice['selected_candidate']['candidate_id'], 'z-first')
                view, _ = build_operational_context(initial, self.grid, self.observations, self.belief.config)
                task = self.exact_task(view=view)
                _, expected = a5.decide(self.field, self.raw, self.belief, Viewpoint((-1, 0, 1.5), 0),
                                        round_count=2, config=config, operational=view)
                ranking = json.loads((output / 'ranking.json').read_text())
                self.assertEqual(ranking['a3_summary']['anchor_semantics'], task.anchor_semantics)
                self.assertEqual(ranking['a3_summary']['winner_anchors'], [asdict(a) for a in task.winner_anchors])
                self.assertEqual(ranking['a3_poses'][0]['xy'], [.011, .019])
                self.assertEqual(ranking['candidates'], json.loads(json.dumps(
                    [asdict(c) for c in expected.candidates])))
                with np.load(output / 'fields.npz') as arrays:
                    for name in FIELD_ARRAY_NAMES:
                        np.testing.assert_array_equal(arrays[f'a3_{name}'], getattr(task, name))
                    for name in ('visibility', 'delta_unknown', 'marginal_task_gain'):
                        np.testing.assert_array_equal(arrays[name], getattr(expected, name))


if __name__ == '__main__':
    unittest.main()
