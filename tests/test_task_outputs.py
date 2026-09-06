import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from task_relevant_uncertainty import build_task_uncertainty, SupportState
from task_relevant_uncertainty.core import FIELD_ARRAY_NAMES
from task_relevant_uncertainty.outputs import field_summary, save_task_field, render_task_field
from task_relevant_uncertainty.synthetic import make_scenarios
from tests.test_task_uncertainty import interest, GRID
from environment_belief import EnvironmentBeliefMapper


class TaskOutputTests(unittest.TestCase):
    def test_four_synthetic_scenarios_have_approved_scores_and_blocking(self):
        scenes = {s.name: s for s in make_scenarios()}
        self.assertEqual(set(scenes), {'high_unknown', 'high_free', 'low_unknown', 'occupied_overlap'})
        fields = {name: build_task_uncertainty(scene.a1, scene.a2) for name, scene in scenes.items()}
        self.assertAlmostEqual(fields['high_unknown'].task_relevant_uncertainty[15, 15], 0.9)
        self.assertAlmostEqual(fields['low_unknown'].task_relevant_uncertainty[15, 15], 0.2)
        self.assertAlmostEqual(fields['high_free'].task_relevant_uncertainty[15, 15], 0.9 * np.exp(-4))
        early = build_task_uncertainty(scenes['high_free'].a1, scenes['high_free'].after_two)
        self.assertAlmostEqual(early.task_relevant_uncertainty[15, 15], 0.9 * np.exp(-1))
        self.assertTrue(fields['occupied_overlap'].poses[0].blocked)
        self.assertFalse(fields['occupied_overlap'].poses[1].blocked)
        self.assertAlmostEqual(fields['occupied_overlap'].task_relevant_uncertainty[15, 18], 0.2)

    def test_npz_and_json_roundtrip_preserve_states_counts_sources_and_semantics(self):
        scene = make_scenarios()[-1]
        field = build_task_uncertainty(scene.a1, scene.a2)
        with tempfile.TemporaryDirectory() as directory:
            files = save_task_field(field, directory)
            self.assertEqual(set(files), {'field', 'summary', 'supports'})
            with np.load(files['field'], allow_pickle=False) as data:
                for name in FIELD_ARRAY_NAMES:
                    np.testing.assert_array_equal(data[name], getattr(field, name))
                self.assertEqual(data['frame_id'].item(), 'map')
                self.assertEqual(data['resolution_m'].item(), 0.1)
            summary = json.loads(files['summary'].read_text())
            self.assertEqual(summary, field_summary(field))
            self.assertEqual(summary['blocked_semantics'], 'A2-occupied-blocked_not_navigation_infeasible')
            self.assertFalse(summary['representative_pose_ik_validated'])
            self.assertFalse(summary['free_forces_zero_score'])
            self.assertEqual(summary['cells']['blocked_only'], 54)
            supports = json.loads(files['supports'].read_text())['poses']
            self.assertEqual(supports[0]['source_id'], 465)
            self.assertEqual(supports[0]['environment_state'], 'A2_OCCUPIED_BLOCKED')
            self.assertEqual(supports[0]['covered_environment_cells'], list(field.poses[0].covered_environment_cells))
            json.dumps(summary, allow_nan=False)

    def test_empty_support_has_json_null_statistics_and_renders_without_warnings(self):
        field = build_task_uncertainty(interest([]), EnvironmentBeliefMapper(GRID).snapshot())
        self.assertIsNone(field_summary(field)['uncertainty_max'])
        with tempfile.TemporaryDirectory() as directory:
            files = save_task_field(field, directory)
            def reject_constant(value):
                self.fail(f'non-JSON numeric constant: {value}')

            json.loads(files['summary'].read_text(), parse_constant=reject_constant)
            path = render_task_field(field, Path(directory) / 'empty.png', title='No inverse support')
            self.assertGreater(path.stat().st_size, 10000)

    def test_render_uses_raw_cells_fixed_score_scales_and_preserves_data(self):
        from matplotlib.axes import Axes
        scene = make_scenarios()[-1]
        field = build_task_uncertainty(scene.a1, scene.a2)
        before = field.task_relevant_uncertainty.copy()
        original, calls = Axes.imshow, []

        def record(axis, *args, **kwargs):
            calls.append(kwargs)
            return original(axis, *args, **kwargs)

        with tempfile.TemporaryDirectory() as directory, patch.object(Axes, 'imshow', record):
            path = render_task_field(field, Path(directory) / 'field.png', title='Synthetic overlap')
            self.assertGreater(path.stat().st_size, 10000)
        self.assertEqual(len(calls), 8)
        self.assertTrue(all(c['interpolation'] == 'none' and c['origin'] == 'lower' for c in calls))
        scores = [c for c in calls if 'vmin' in c]
        self.assertEqual(len(scores), 5)
        self.assertTrue(all(c['vmin'] == 0 and c['vmax'] == 1 for c in scores))
        np.testing.assert_array_equal(field.task_relevant_uncertainty, before)

    def test_import_does_not_switch_backend_or_load_ros_or_live_rm4d(self):
        completed = subprocess.run([sys.executable, '-W', 'error', '-c',
            "import sys,matplotlib; matplotlib.use('svg'); import task_relevant_uncertainty.outputs; "
            "assert matplotlib.get_backend().lower() == 'svg'; "
            "assert 'rospy' not in sys.modules; assert 'rm4d' not in sys.modules; "
            "assert 'pybullet' not in sys.modules"], capture_output=True, text=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_offline_cli_writes_four_scenes_and_known_free_intermediate(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            env = os.environ.copy()
            env['PYTHONPATH'] = str(root / 'src')
            run = subprocess.run([sys.executable, '-W', 'error', str(root / 'scripts/run_a3_offline_validation.py'),
                                  '--output-root', directory], env=env, capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)
            lines = [json.loads(line) for line in run.stdout.splitlines()]
            self.assertEqual(len(lines), 4)
            self.assertEqual({p.name for p in Path(directory).iterdir()},
                             {'high_unknown', 'high_free', 'low_unknown', 'occupied_overlap'})
            for name in ('high_unknown', 'high_free', 'low_unknown', 'occupied_overlap'):
                scene = Path(directory) / name
                self.assertTrue((scene / 'field.png').exists())
                self.assertTrue((scene / 'inputs.npz').exists())
                self.assertEqual(json.loads((scene / 'inputs.json').read_text())['input_kind'],
                                 'synthetic_A1_candidates_and_A2_endpoints_not_real_IK_or_sensor_validation')
            self.assertTrue((Path(directory) / 'high_free/after_two/field.npz').exists())


if __name__ == '__main__':
    unittest.main()
