"""Recorded development endpoints, not new simulated observations or outcomes."""
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('v14_replay', ROOT / 'scripts/replay_v13_operational.py')
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


class GroundPresenceReplayTests(unittest.TestCase):
    def test_a3_v14_output_labels_match_object_aware_computation(self):
        from test_v13_runtime import moderate
        from operational_gating.io import build_operational_context
        from sim_active_perception.core import build_support_task
        from task_relevant_uncertainty.outputs import field_summary
        initial, _, config, field, grid, observation, belief = moderate()
        op, _ = build_operational_context(dict(initial, operational_gating='v1.4'),
                                           grid, [observation], belief.config)
        task = build_support_task(field, initial['result'], belief, config, operational=op)
        summary = field_summary(task)
        self.assertEqual(summary.get('operational_semantics'), 'object-aware-v1.4')
        self.assertIn('operational_blocked', summary['pose_states'])
        self.assertIn('object-aware-unblocked', summary['operational_formula'])

    def test_real_ground_is_counted_without_changing_blocking_or_nbv(self):
        data = ROOT / 'outputs/development/operational-batch/launch-02-moderate-regression/data'
        if not (data / 'observation_03.npz').exists():
            self.skipTest('recorded development observation not present')
        r = replay.replay_round(data, 3, include_v14=True)
        old, new = r['v13'], r['v14']
        self.assertTrue(r['all_common_checks_pass'])
        self.assertEqual(old['blocked'], new['blocked'])
        self.assertEqual(old['task_uncertainty_mass'], new['task_uncertainty_mass'])
        self.assertEqual(old['best_task_score'], new['best_task_score'])
        before = {a['source_id']: a for a in old['assessments']}
        after = {a['source_id']: a for a in new['assessments']}
        for source in (705, 623):
            self.assertFalse(before[source]['confirmed'])
            self.assertGreater(before[source]['operational']['ground_missing_cells'], 0)
            self.assertTrue(after[source]['confirmed'])
        for source in before:
            self.assertEqual(before[source]['operational']['blocked'], after[source]['operational']['blocked'])

    def test_single_window_remains_insufficient_in_recorded_hard(self):
        data = ROOT / 'outputs/development/operational-batch/launch-03-hard002-regression/data'
        if not (data / 'observation_01.npz').exists():
            self.skipTest('recorded development observation not present')
        r = replay.replay_round(data, 1, include_v14=True)
        self.assertTrue(r['all_common_checks_pass'])
        self.assertEqual(r['v14']['confirmed'], 0)
