"""Offline synthetic records: never activate an evaluation scene or simulator."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS))
from summarize_a6_pilot import summarize_pilot

COHORT = 'paper1-eval-finite-scan-v1-final'


def configuration(n=4, auxiliary=False):
    scenes = [dict(id='scene-%02d' % i, seed=100+i,
                   tier='easy' if i < n-1 else 'hard') for i in range(n)]
    slots = [dict(slot=2*i+j+1, scene=s['id'], method=m)
             for i, s in enumerate(scenes) for j, m in enumerate(('ours', 'generic'))]
    if auxiliary:
        for method in ('fixed', 'rm4d_only', 'no_cost', 'no_occlusion'):
            slots.append(dict(slot=len(slots)+1, scene=scenes[-1]['id'], method=method))
    return dict(status='FROZEN_FOR_EVALUATION', cohort=COHORT, scenes=scenes, slots=slots,
                limits=dict(primary_scenes=n, planned_tasks=len(slots), reserve_starts=12,
                            total_starts=len(slots)+12, max_replacements_per_slot=1),
                sensor_mount_T_uav_lidar=np.eye(4).tolist())


def production_shape_fixture():
    config = configuration(96)
    config['evaluation_version'] = dict(sim_commit='fixture-sim', rm4d_baseline_commit='fixture-rm')
    for i, scene in enumerate(config['scenes']):
        scene['tier'] = 'easy' if i < 38 else 'moderate' if i < 66 else 'hard'
    for tier in ('easy', 'moderate', 'hard'):
        scenes = [s for s in config['scenes'] if s['tier'] == tier]
        for scene in scenes[:2]:
            for method in ('fixed', 'rm4d_only'):
                config['slots'].append(dict(slot=len(config['slots'])+1, scene=scene['id'], method=method))
        if tier == 'hard':
            for scene in scenes[:4]:
                for method in ('no_cost', 'no_occlusion'):
                    config['slots'].append(dict(slot=len(config['slots'])+1, scene=scene['id'], method=method))
    config['limits'].update(planned_tasks=212, total_starts=224)
    return config


class AnalysisTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.a = None
        path = SCRIPTS / 'analyze_paper1_eval.py'
        if path.exists():
            spec = importlib.util.spec_from_file_location('paper1_analysis_under_test', path)
            cls.a = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.a)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = configuration()

    def record(self, slot, success=False, status='VALID_TRIAL', suffix='01',
               metrics=None, events=None, completed=True, **extra):
        spec = self.config['slots'][slot-1]
        scene = next(s for s in self.config['scenes'] if s['id'] == spec['scene'])
        path = self.root / ('slot-%03d-%s' % (slot, suffix))
        path.mkdir()
        record = dict(spec, seed=scene['seed'], kind='EVALUATION_ATTEMPT', cohort=COHORT,
                      status=status, retrieval_success=success, activation_wall=float(suffix))
        if 'evaluation_version' in self.config:
            record.update(scene_spec=scene, full_robot_manipulation=True,
                          joint_velocity_feedback='integrated_pose_interval_velocity',
                          execution_clearance='chassis-clearance-v1', ground_dynamics_csv='diagnostic.csv',
                          physics_state_use='diagnosis_only_never_algorithm_input')
            (path.parent/'task.json').write_text(json.dumps(self.config))
        if completed:
            record['finish_wall'] = float(suffix)+.5
        record.update(extra)
        (path / 'attempt.json').write_text(json.dumps(record))
        (path / 'data').mkdir()
        if metrics is not None:
            (path / 'data/metrics.json').write_text(json.dumps(metrics))
        if events is not None:
            (path / 'data/events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
        return path

    def analysis(self):
        self.assertIsNotNone(self.a, 'prospective offline evaluation analysis must exist')
        return self.a.analyze(self.config, self.root)

    def complete(self, outcomes=None):
        for slot in self.config['slots']:
            self.record(slot['slot'], (outcomes or {}).get(slot['slot'], False))

    def test_collector_accepts_only_explicit_evaluation_cohort_and_kind(self):
        self.record(1, True)
        self.record(2, True, cohort='old')
        self.record(3, True, kind='FORMAL_ATTEMPT')
        report = summarize_pilot(self.config, self.root)
        self.assertEqual(report['counts']['valid_completed_slots'], 1)
        self.assertEqual(len(report['excluded_attempts']), 2)
        self.assertTrue(report['slots'][0]['retrieval_success'])

    def test_aggregate_counts_unweighted_effect_and_two_exact_intervals(self):
        self.complete({1: True, 3: True, 8: True})
        result = self.analysis()
        primary = result['primary_inference']
        self.assertEqual(result['status'], 'COMPLETE')
        self.assertEqual((primary['n'], primary['b'], primary['c']), (4, 2, 1))
        self.assertEqual(primary['risk_difference'], .25)
        from analyze_a6_formal import binomial_interval, exact_mcnemar
        b, c = binomial_interval(2, 4, .025), binomial_interval(1, 4, .025)
        self.assertEqual(primary['conservative_exact_rd_interval'], [b[0]-c[1], b[1]-c[0]])
        self.assertEqual(primary['p_value'], exact_mcnemar(2, 1))
        self.assertNotIn('p_value', result['tiers']['hard'])
        self.assertIn('conservative_exact_rd_interval', result['tiers']['hard'])

    def test_concordant_96_boundary_interval_is_nonzero_and_inside_five_points(self):
        self.config = configuration(96)
        self.complete()
        primary = self.analysis()['primary_inference']
        self.assertEqual(primary['p_value'], 1.)
        lo, hi = primary['conservative_exact_rd_interval']
        self.assertTrue(-.05 < lo < -.04 < 0 < .04 < hi < .05)

    def test_partial_keeps_all_scene_denominator_and_sharp_missing_bounds(self):
        self.record(1, True)  # known Ours, unknown Generic: contribution [0,1]
        self.record(3, False)
        self.record(4, True)  # known -1
        r = self.analysis()
        self.assertEqual(r['status'], 'INCOMPLETE')
        self.assertIsNone(r['primary_inference'])
        self.assertIsNone(r['primary']['risk_difference'])
        self.assertEqual(r['primary']['risk_difference_bounds'], [-.75, .5])
        self.assertEqual(r['primary']['n_scheduled'], 4)

    def test_auxiliary_slots_are_required_and_only_descriptive_original_ours_pairs(self):
        self.config = configuration(2, auxiliary=True)
        for slot in range(1, 5):
            self.record(slot, True)
        self.assertIsNone(self.analysis()['primary_inference'])
        for slot in range(5, 9):
            self.record(slot, False)
        r = self.analysis()
        self.assertEqual(r['status'], 'COMPLETE')
        self.assertEqual(set(r['auxiliary']), {'fixed', 'rm4d_only', 'no_cost', 'no_occlusion'})
        for pair in r['auxiliary'].values():
            self.assertEqual(pair['b'], 1)
            self.assertEqual(pair['pairs'][0]['ours']['slot'], 3)
            self.assertNotIn('p_value', pair)

    def test_one_documented_invalid_replacement_and_first_activation_sensitivity(self):
        self.record(1, None, status='INVALID_TRIAL', reason='startup transport failed')
        self.record(1, True, suffix='02')
        for slot in range(2, 9):
            self.record(slot, False)
        r = self.analysis()
        self.assertEqual(r['status'], 'COMPLETE')
        self.assertEqual(r['primary_inference']['b'], 1)
        self.assertEqual(r['first_activation_invalid_as_failure']['b'], 0)
        self.assertEqual(r['counts']['replacement_starts'], 1)

    def test_invalid_original_runtime_defects_are_retained_without_contaminating_valid_cohort(self):
        self.config['evaluation_version'] = dict(sim_commit='sim', rm4d_baseline_commit='rm')
        self.config['window_sim_s'] = 5
        current = dict(agent='prep', sim='sim', rm4d='rm')
        original = self.record(1, None, status='INVALID_TRIAL', reason='wrong loaded configuration',
                               evaluation_version=dict(name='old'), runtime_versions=dict(
                                   agent='old', sim='wrong-sim', rm4d='old-rm'))
        isolated = self.root/'invalid-original'
        isolated.mkdir()
        original.rename(isolated/'attempt')
        (isolated/'task.json').write_text(json.dumps(dict(self.config, window_sim_s=99)))
        self.record(1, True, suffix='02', evaluation_version=self.config['evaluation_version'],
                    runtime_versions=current)
        for slot in range(2, 9):
            self.record(slot, False, evaluation_version=self.config['evaluation_version'],
                        runtime_versions=current)
        r = self.analysis()
        self.assertEqual(r['status'], 'COMPLETE')
        self.assertEqual(r['runtime_versions'], [current])
        self.assertFalse(r['protocol_issues'])
        self.assertEqual(r['primary_inference']['b'], 1)
        self.assertFalse(r['first_activation_invalid_as_failure']['pairs'][0]['ours']['retrieval_success'])
        self.assertIn('saved task configuration', str(r['invalid_attempt_diagnostics']))
        record_path = isolated/'attempt/attempt.json'
        raw = json.loads(record_path.read_text())
        raw.pop('runtime_versions')
        record_path.write_text(json.dumps(raw))
        r = self.analysis()
        self.assertEqual(r['status'], 'COMPLETE')
        self.assertIn('runtime versions missing', str(r['invalid_attempt_diagnostics']))

    def test_unknown_replacement_start_time_remains_incomplete_without_crashing(self):
        for value in (None, 'unknown'):
            with self.subTest(value=value), tempfile.TemporaryDirectory() as directory:
                self.root = Path(directory)
                self.record(1, None, status='INVALID_TRIAL', reason='startup failure')
                self.record(1, True, suffix='02', activation_wall=value)
                result = self.analysis()
                self.assertEqual(result['status'], 'INCOMPLETE')
                self.assertIsNone(result['primary_inference'])
                self.assertIn('missing activation chronology', str(result['protocol_issues']))

    def test_valid_duplicates_unfinished_or_undocumented_invalid_are_not_replacements(self):
        for defect in ('duplicate', 'unfinished', 'undocumented', 'third'):
            with self.subTest(defect=defect), tempfile.TemporaryDirectory() as tmp:
                self.root = Path(tmp)
                if defect == 'duplicate':
                    self.record(1, False)
                else:
                    self.record(1, None, status='INVALID_TRIAL', completed=defect != 'unfinished',
                                **({} if defect == 'undocumented' else dict(reason='startup failure')))
                self.record(1, True, suffix='02')
                if defect == 'third':
                    self.record(1, False, suffix='03')
                for slot in range(2, 9): self.record(slot)
                r = self.analysis()
                self.assertEqual(r['status'], 'INCOMPLETE')
                self.assertIsNone(r['primary_inference'])
                self.assertTrue(r['protocol_issues'])

    def test_reserve_limit_is_enforced_without_replacing_valid_failures(self):
        self.config['limits'].update(reserve_starts=0, total_starts=8)
        self.record(1, None, status='INVALID_TRIAL', reason='startup')
        self.record(1, True, suffix='02')
        for slot in range(2, 9): self.record(slot)
        r = self.analysis()
        self.assertEqual(r['status'], 'INCOMPLETE')
        self.assertIn('reserve', ' '.join(r['protocol_issues']))

    def test_wrong_configuration_and_declared_count_mismatch_are_rejected(self):
        self.assertIsNotNone(self.a, 'prospective offline evaluation analysis must exist')
        for change in (dict(status='FROZEN_FOR_FORMAL'), dict(cohort='old')):
            with self.assertRaises(ValueError):
                self.a.analyze(dict(self.config, **change), self.root)
        self.config['limits']['primary_scenes'] = 96
        with self.assertRaises(ValueError): self.analysis()

    def test_ground_span_stops_at_terminal_and_missing_handoff_is_null(self):
        events = [dict(state='A6_STAGE_START', stage='ground_navigation', ros_time=10),
                  dict(state='LIFT', ros_time=30), dict(state='FAILED', ros_time=100)]
        self.record(1, False, events=events, metrics=dict(terminal_status='LIFT'))
        self.record(2, True, events=[dict(state='LIFT', ros_time=30)])
        r = self.analysis()
        self.assertFalse(r['slots'][0]['retrieval_success'])
        self.assertEqual(r['slots'][0]['T_ground_sim'], 20)
        self.assertIsNone(r['slots'][1]['T_ground_sim'])

    def test_clock_reset_and_incomplete_event_line_leave_secondary_duration_missing(self):
        path = self.record(1, True, events=[
            dict(state='A6_STAGE_START', stage='ground_navigation', ros_time=10),
            dict(state='OTHER', ros_time=8), dict(state='LIFT', ros_time=30)])
        self.assertIsNone(self.analysis()['slots'][0]['T_ground_sim'])
        with (path/'data/events.jsonl').open('a') as stream: stream.write('{')
        row = self.analysis()['slots'][0]
        self.assertTrue(row['retrieval_success'])
        self.assertTrue(row['analysis_missing'])

    def test_packet_pose_composition_thresholds_yaw_wrap_and_missing_not_zero(self):
        mount = np.eye(4); mount[:3, 3] = [.4, 0, .3]
        self.config['sensor_mount_T_uav_lidar'] = mount.tolist()
        poses = [(0, math.pi-.05), (.1, -math.pi+.25), (.31, -math.pi+.25)]
        events = [dict(state='A5_OBSERVATION', round=i+1, ros_time=i+1,
                       capture_anchor_map=[999, 999, 999, 1], uav_pose_map=[999, 999, 999, 1])
                  for i in range(3)]
        path = self.record(1, True, events=events, metrics=dict(counts=dict(completed_windows=3)))
        for i, (x, yaw) in enumerate(poses):
            base = np.eye(4)
            base[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
            base[0, 3] = x
            np.savez(path/'data'/('observation_%02d.npz' % (i+1)),
                     chunk_T_map_sensor=np.stack([base@mount, np.eye(4)]))
        loc = self.analysis()['slots'][0]['observation_locations']
        self.assertEqual(loc['resolved_translation_changes'], 1)
        self.assertEqual(loc['resolved_yaw_only_changes'], 1)
        self.assertEqual(loc['resolved_location_changes'], 2)
        self.assertAlmostEqual(loc['transitions'][0]['yaw_change_rad'], .3)
        self.assertAlmostEqual(loc['transitions'][1]['displacement_m'], .21)
        (path/'data/observation_02.npz').unlink()
        loc = self.analysis()['slots'][0]['observation_locations']
        self.assertIsNone(loc['resolved_location_changes'])
        self.assertEqual(loc['missing_transitions'], 2)

    def test_missing_mount_does_not_guess_identity_or_infer_no_motion(self):
        self.config.pop('sensor_mount_T_uav_lidar')
        self.record(1, True, events=[dict(state='A5_OBSERVATION', round=1, ros_time=1),
                                     dict(state='A5_OBSERVATION', round=2, ros_time=2)])
        row = self.analysis()['slots'][0]
        self.assertIsNone(row['observation_locations']['resolved_location_changes'])

    def test_saved_ranking_mount_precedes_manifest_and_explicit_fallback(self):
        saved = np.eye(4); saved[0, 3] = 1
        path = self.record(1, True, events=[dict(state='A5_OBSERVATION', round=1, ros_time=1)],
                           metrics=dict(counts=dict(completed_windows=1)))
        ranking = path/'data/rounds/round-01/ranking.json'
        ranking.parent.mkdir(parents=True)
        ranking.write_text(json.dumps(dict(sensor=dict(T_uav_lidar=saved.tolist()))))
        np.savez(path/'data/observation_01.npz', chunk_T_map_sensor=[saved])
        pose = self.analysis()['slots'][0]['observation_locations']['poses_map_xyz_yaw'][0]
        self.assertEqual(pose, [0, 0, 0, 0])

    def test_absent_observation_events_and_nonconsecutive_rounds_leave_changes_missing(self):
        path = self.record(1, True, events=[dict(state='A5_OBSERVATION', round=1, ros_time=1),
                                          dict(state='A5_OBSERVATION', round=3, ros_time=3)],
                           metrics=dict(counts=dict(completed_windows=3)))
        for n in (1, 3):
            np.savez(path/'data'/('observation_%02d.npz' % n), chunk_T_map_sensor=[np.eye(4)])
        loc = self.analysis()['slots'][0]['observation_locations']
        self.assertIsNone(loc['resolved_location_changes'])
        self.assertEqual(loc['missing_transitions'], 2)
        self.assertFalse(any(t['from_window'] == 1 and t['to_window'] == 2 and
                             t['category'] is not None for t in loc['transitions']))

    def test_first_activation_is_chronological_even_when_directory_names_reverse(self):
        self.record(1, None, status='INVALID_TRIAL', reason='startup', suffix='90',
                    activation_wall=1, finish_wall=2)
        self.record(1, True, suffix='01', activation_wall=3, finish_wall=4)
        for slot in range(2, 9): self.record(slot, False)
        result = self.analysis()
        self.assertEqual(result['status'], 'COMPLETE')
        first = result['first_activation_invalid_as_failure']['pairs'][0]['ours']
        self.assertFalse(first['retrieval_success'])
        self.assertTrue(first['attempt_dir'].endswith('-90'))
        history = result['slots'][0]['attempts']
        self.assertIsNone(history[0]['rerun_of'])
        self.assertEqual(history[1]['rerun_of'], history[0]['attempt_dir'])

    def test_declared_evaluation_metadata_requires_recorded_nonmixed_runtime_versions(self):
        self.config['evaluation_version'] = {'name': 'frozen-runtime-v1'}
        self.complete()
        for path in self.root.glob('*/attempt.json'):
            record = json.loads(path.read_text())
            record['evaluation_version'] = self.config['evaluation_version']
            path.write_text(json.dumps(record))
        r = self.analysis()
        self.assertEqual(r['status'], 'INCOMPLETE')
        self.assertIn('runtime versions missing', ' '.join(r['protocol_issues']))

    def test_malformed_runtime_version_record_blocks_inference_without_crashing(self):
        self.config['evaluation_version'] = dict(sim_commit='sim', rm4d_baseline_commit='rm')
        self.record(1, False, evaluation_version=self.config['evaluation_version'], runtime_versions=['wrong'])
        r = self.analysis()
        self.assertEqual(r['status'], 'INCOMPLETE')
        self.assertTrue(r['protocol_issues'])

    def test_uniform_wrong_sim_or_rm4d_is_rejected_but_new_agent_preparation_head_is_recorded(self):
        self.config['evaluation_version'] = dict(sim_commit='expected-sim', rm4d_baseline_commit='expected-rm')
        for slot in range(1, 9):
            self.record(slot, False, evaluation_version=self.config['evaluation_version'],
                        runtime_versions=dict(agent='preparation-head', sim='wrong-sim', rm4d='expected-rm'))
        r = self.analysis()
        self.assertEqual(r['status'], 'INCOMPLETE')
        self.assertIn('pinned sim', ' '.join(r['protocol_issues']))
        for path in self.root.glob('*/attempt.json'):
            record = json.loads(path.read_text())
            record['runtime_versions']['sim'] = 'expected-sim'
            path.write_text(json.dumps(record))
        r = self.analysis()
        self.assertEqual(r['status'], 'COMPLETE')
        self.assertEqual(r['runtime_versions'][0]['agent'], 'preparation-head')

    def test_actual_scene_execution_flags_and_saved_runtime_config_must_match_freeze(self):
        self.config['evaluation_version'] = dict(sim_commit='sim', rm4d_baseline_commit='rm')
        self.config['window_sim_s'] = 5
        for slot in range(1, 9):
            self.record(slot, False, evaluation_version=self.config['evaluation_version'],
                        runtime_versions=dict(agent='prep', sim='sim', rm4d='rm'))
        self.assertEqual(self.analysis()['status'], 'COMPLETE')
        record_path = self.root/'slot-001-01/attempt.json'
        original = json.loads(record_path.read_text())
        for key, value in (('scene_spec', {}), ('full_robot_manipulation', False),
                           ('joint_velocity_feedback', 'stock_native'), ('ground_dynamics_csv', None),
                           ('execution_clearance', None)):
            with self.subTest(key=key):
                record_path.write_text(json.dumps(dict(original, **{key: value})))
                r = self.analysis()
                self.assertEqual(r['status'], 'INCOMPLETE')
                self.assertIn('loaded execution', ' '.join(r['protocol_issues']))
        record_path.write_text(json.dumps(original))
        snapshot = dict(self.config, window_sim_s=6)
        (self.root/'task.json').write_text(json.dumps(snapshot))
        r = self.analysis()
        self.assertEqual(r['status'], 'INCOMPLETE')
        self.assertIn('saved task configuration', ' '.join(r['protocol_issues']))

    def test_nonzero_offset_commands_are_separate_from_observed_location_changes(self):
        events = [dict(state='A5_OBSERVATION', round=1, ros_time=1, uav_pose_map=[0, 0, 1, 0]),
                  dict(state='A5_DECISION', next_viewpoint=[1, 0, 1, 0], ros_time=2),
                  dict(state='A5_VIEWPOINT', view_role='sensing', goal_map=[1, 0, 1, 0], ros_time=3),
                  dict(state='A5_OBSERVATION', round=2, ros_time=4, uav_pose_map=[1, 0, 1, 0]),
                  dict(state='A5_DECISION', next_viewpoint=[1, 0, 1, 0], ros_time=5),
                  dict(state='A5_VIEWPOINT', view_role='sensing', goal_map=[1, 0, 1, 0], ros_time=6)]
        self.record(1, True, events=events)
        row = self.analysis()['slots'][0]
        self.assertEqual(row['nonzero_offset_commands'], 1)
        self.assertIsNone(row['observation_locations']['resolved_location_changes'])

    def test_changed_runtime_version_prevents_final_pooling(self):
        for slot in range(1, 9):
            self.record(slot, False, runtime_versions=dict(agent='same' if slot != 2 else 'changed',
                                                          sim='same', rm4d='same'))
        r = self.analysis()
        self.assertEqual(r['status'], 'INCOMPLETE')
        self.assertIn('runtime versions differ', ' '.join(r['protocol_issues']))

    def test_entry_only_start_is_counted_unresolved_and_not_a_manufactured_failure(self):
        self.complete()
        entry = self.root/'entry-only'
        entry.mkdir()
        (entry/'entry.json').write_text(json.dumps(dict(
            kind='EVALUATION_ENTRY', cohort=COHORT, slot=1, scene='scene-00', seed=100,
            method='ours', start_wall=0, finish_wall=.5, exit_code=2)))
        r = self.analysis()
        self.assertEqual(r['status'], 'INCOMPLETE')
        self.assertEqual(r['counts']['total_activations'], 9)
        self.assertEqual(r['counts']['entry_only_activations'], 1)
        self.assertEqual(r['entry_only_activations'][0]['status'], 'UNRESOLVED_ENTRY_ONLY')
        self.assertIsNone(r['first_activation_invalid_as_failure']['pairs'][0]['ours']['retrieval_success'])
        self.assertIsNone(r['primary_inference'])

    def test_entry_with_its_attempt_is_not_double_counted(self):
        for slot in self.config['slots']:
            path = self.root/('task-%03d' % slot['slot'])
            path.mkdir()
            record_dir = self.record(slot['slot'], False)
            record_dir.rename(path/'attempt')
            scene = next(s for s in self.config['scenes'] if s['id'] == slot['scene'])
            (path/'entry.json').write_text(json.dumps(dict(slot, seed=scene['seed'],
                kind='EVALUATION_ENTRY', cohort=COHORT, start_wall=0)))
        r = self.analysis()
        self.assertEqual(r['status'], 'COMPLETE')
        self.assertEqual(r['counts']['total_activations'], 8)
        self.assertIn('entry_only_activations', r['counts'])
        self.assertEqual(r['counts']['entry_only_activations'], 0)

    def test_known_cohort_start_with_mismatched_slot_metadata_still_counts_toward_cap(self):
        self.complete()
        self.record(1, False, suffix='02', seed=-999)
        r = self.analysis()
        self.assertEqual(r['status'], 'INCOMPLETE')
        self.assertEqual(r['counts']['total_activations'], 9)

    def test_unassignable_known_cohort_entries_consume_total_and_reserve_with_metadata_diagnosis(self):
        self.complete()
        entry = self.root/'unassignable-entry'
        entry.mkdir()
        original = dict(kind='EVALUATION_ENTRY', cohort=COHORT, slot=1, scene='scene-00',
                        seed=100, method='ours', start_wall=0)
        for field, value in (('scene', 'wrong-scene'), ('slot', 999), ('method', 'generic'), ('method', None)):
            with self.subTest(field=field, value=value):
                (entry/'entry.json').write_text(json.dumps(dict(original, **{field: value})))
                r = self.analysis()
                self.assertEqual(r['counts']['total_activations'], 9)
                self.assertEqual(r['counts']['replacement_starts'], 1)
                self.assertEqual(r['counts']['entry_only_activations'], 1)
                self.assertEqual(r['status'], 'INCOMPLETE')
                self.assertIn('entry metadata mismatch', ' '.join(r['protocol_issues']))
                self.assertIsNone(r['entry_only_activations'][0]['retrieval_success'])

    def test_mismatched_entry_with_counted_attempt_is_not_double_counted_or_silently_accepted(self):
        self.complete()
        task = self.root/'misnamed-entry'
        task.mkdir()
        (self.root/'slot-001-01').rename(task/'attempt')
        # Internally valid slot2 metadata disagrees with this directory's slot1 attempt.
        (task/'entry.json').write_text(json.dumps(dict(kind='EVALUATION_ENTRY', cohort=COHORT,
            slot=2, scene='scene-00', seed=100, method='generic', start_wall=0)))
        r = self.analysis()
        self.assertEqual(r['counts']['total_activations'], 8)
        self.assertEqual(r['counts']['entry_only_activations'], 0)
        self.assertEqual(r['status'], 'INCOMPLETE')
        self.assertIn('entry metadata mismatch', ' '.join(r['protocol_issues']))

    def test_entry_with_unreadable_attempt_still_consumes_one_start(self):
        self.complete()
        task = self.root/'broken-attempt'
        (task/'attempt').mkdir(parents=True)
        (task/'attempt/attempt.json').write_text('{')
        (task/'entry.json').write_text(json.dumps(dict(kind='EVALUATION_ENTRY', cohort=COHORT,
            slot=1, scene='scene-00', seed=100, method='ours', start_wall=0)))
        r = self.analysis()
        self.assertEqual(r['counts']['total_activations'], 9)
        self.assertEqual(r['status'], 'INCOMPLETE')

    def test_supplied_mount_fallback_and_invalid_rotation(self):
        self.config.pop('sensor_mount_T_uav_lidar')
        path = self.record(1, True, events=[dict(state='A5_OBSERVATION', round=1, ros_time=1)],
                           metrics=dict(counts=dict(completed_windows=1)))
        np.savez(path/'data/observation_01.npz', chunk_T_map_sensor=[np.eye(4)])
        self.assertIsNotNone(self.a, 'prospective offline evaluation analysis must exist')
        r = self.a.analyze(self.config, self.root, np.eye(4).tolist())
        self.assertEqual(r['slots'][0]['observation_locations']['resolved_location_changes'], 0)
        invalid = np.eye(4); invalid[0, 0] = 2
        r = self.a.analyze(self.config, self.root, invalid.tolist())
        self.assertIsNone(r['slots'][0]['observation_locations']['resolved_location_changes'])

    def test_failure_is_not_fast_event_and_missing_time_keeps_all_n_curve_bounds(self):
        self.record(1, True, metrics=dict(T_active_sim=10, T_task_sim=20,
                                         counts=dict(completed_windows=2)))
        self.record(3, True, metrics=dict(counts=dict(completed_windows=3)))
        for slot in (2, 4, 5, 6, 7, 8):
            self.record(slot, False, metrics=dict(T_active_sim=0, T_task_sim=0,
                                                 counts=dict(completed_windows=1)))
        curves = self.analysis()['resource_curves']['ours']
        at10 = next(p for p in curves['T_active_sim'] if p['threshold'] == 10)
        self.assertEqual((at10['lower'], at10['upper'], at10['denominator']), (.25, .5, 4))
        self.assertEqual([r['lower'] for r in curves['windows']], [0, .25, .5])

    def test_joint_success_costs_are_conditional_paired_and_bootstrapped_by_scene(self):
        for slot, t in ((1, 10), (2, 20), (3, 30), (4, 25)):
            self.record(slot, True, metrics=dict(T_active_sim=t))
        self.record(5, True, metrics=dict(T_active_sim=100))
        for slot in (6, 7, 8): self.record(slot, False, metrics=dict(T_active_sim=0))
        r = self.analysis()['conditional_efficiency']
        self.assertEqual(r['joint_success_scenes'], 2)
        self.assertEqual(r['joint_success_fraction'], .5)
        costs = r['metrics']['T_active_sim']
        self.assertEqual(costs['mean_difference_ours_minus_generic'], -2.5)
        self.assertEqual(costs['median_difference_ours_minus_generic'], -2.5)
        self.assertEqual(costs['bootstrap_resamples'], 10000)
        self.assertEqual(costs['bootstrap_seed'], 2026091024)
        self.assertEqual(costs['mean_difference_interval'], [-10, 5])
        self.assertEqual(costs, self.analysis()['conditional_efficiency']['metrics']['T_active_sim'])

    def test_cli_consumes_attempts_without_existing_precomputed_summary(self):
        self.assertIsNotNone(self.a, 'prospective offline evaluation analysis must exist')
        self.config = production_shape_fixture()
        self.record(1, False)
        config_path, output = self.root/'config.json', self.root/'report.json'
        config_path.write_text(json.dumps(self.config))
        run = subprocess.run([sys.executable, str(SCRIPTS/'analyze_paper1_eval.py'),
                              '--config', str(config_path), '--results-dir', str(self.root),
                              '--output', str(output)], capture_output=True, text=True)
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertEqual(json.loads(output.read_text())['status'], 'INCOMPLETE')

    def test_production_cli_rejects_truncated_same_cohort_manifest(self):
        self.assertIsNotNone(self.a, 'prospective offline evaluation analysis must exist')
        config_path, output = self.root/'config.json', self.root/'report.json'
        config_path.write_text(json.dumps(self.config))
        run = subprocess.run([sys.executable, str(SCRIPTS/'analyze_paper1_eval.py'),
                              '--config', str(config_path), '--results-dir', str(self.root),
                              '--output', str(output)], capture_output=True, text=True)
        self.assertNotEqual(run.returncode, 0)
        self.assertIn('96 scenes', run.stderr)
        self.assertFalse(output.exists())

    def test_production_validation_checks_declared_auxiliary_scene_membership(self):
        self.assertIsNotNone(self.a, 'prospective offline evaluation analysis must exist')
        self.assertTrue(hasattr(self.a, 'validate_production_manifest'))
        config = production_shape_fixture()
        self.a.validate_production_manifest(config)
        next(s for s in config['slots'] if s['method'] == 'fixed')['scene'] = config['scenes'][3]['id']
        with self.assertRaises(ValueError): self.a.validate_production_manifest(config)
        config = production_shape_fixture()
        config.pop('evaluation_version')
        with self.assertRaises(ValueError): self.a.validate_production_manifest(config)


if __name__ == '__main__': unittest.main()
