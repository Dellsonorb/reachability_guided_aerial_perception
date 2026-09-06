import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from environment_belief import EnvironmentBeliefMapper, EnvironmentGridSpec
from environment_belief.outputs import belief_summary, render_belief, save_belief
from tests.test_environment_belief import observation


class EnvironmentOutputTests(unittest.TestCase):
    def make_belief(self):
        mapper = EnvironmentBeliefMapper(EnvironmentGridSpec((0, 0), 4, 4))
        mapper.update(observation([[0.25, 0.35, 0], [0.15, 0.15, 0.1]]))
        mapper.update(observation([[0.25, 0.35, 0]]))
        return mapper.snapshot()

    def test_npz_json_roundtrip_retains_score_counts_and_map_geometry(self):
        belief = self.make_belief()
        with tempfile.TemporaryDirectory() as directory:
            files = save_belief(belief, directory)
            self.assertEqual(set(files), {'belief', 'summary'})
            self.assertEqual({p.name for p in Path(directory).iterdir()}, {'belief.npz', 'summary.json'})
            with np.load(files['belief'], allow_pickle=False) as data:
                for name in ('state', 'occupied_evidence', 'free_evidence', 'observation_count', 'unknown_score'):
                    np.testing.assert_array_equal(data[name], getattr(belief, name))
                np.testing.assert_array_equal(data['origin_xy'], [0, 0])
                self.assertEqual(data['frame_id'].item(), 'map')
                self.assertEqual(data['resolution_m'].item(), 0.1)
            summary = json.loads(files['summary'].read_text())
            self.assertEqual(summary, belief_summary(belief))
            self.assertEqual(summary['cells'], {'unknown': 14, 'free': 1, 'occupied': 1})
            self.assertEqual(summary['observed_cells'], 2)
            self.assertEqual(summary['score_semantics'], 'observation_deficit_heuristic_not_probability')
            self.assertEqual(summary['config']['unknown_scale'], 2)
            self.assertNotIn('p_unknown', json.dumps(summary))
            json.dumps(summary, allow_nan=False)

    def test_render_is_headless_unsmoothed_and_preserves_snapshot(self):
        from matplotlib.axes import Axes
        belief = self.make_belief()
        before = belief.state.copy()
        original = Axes.imshow
        calls = []

        def record(axis, *args, **kwargs):
            calls.append(kwargs)
            return original(axis, *args, **kwargs)

        with tempfile.TemporaryDirectory() as directory, patch.object(Axes, 'imshow', record):
            path = render_belief(belief, Path(directory) / 'nested' / 'belief.png')
            self.assertGreater(path.stat().st_size, 10000)
        self.assertEqual(len(calls), 3)
        self.assertTrue(all(c['interpolation'] == 'none' and c['origin'] == 'lower' for c in calls))
        self.assertEqual((calls[1]['vmin'], calls[1]['vmax']), (0, 1))
        np.testing.assert_array_equal(before, belief.state)

    def test_empty_belief_outputs_remain_unknown(self):
        belief = EnvironmentBeliefMapper(EnvironmentGridSpec((-1.5, -1.5), 30, 30)).snapshot()
        self.assertEqual(belief_summary(belief)['observed_cells'], 0)
        with tempfile.TemporaryDirectory() as directory:
            render_belief(belief, Path(directory) / 'empty.png')
            save_belief(belief, directory)

    def test_package_is_independent_and_does_not_switch_host_backend(self):
        environment = os.environ.copy()
        environment['PYTHONPATH'] = str(Path(__file__).resolve().parents[1] / 'src')
        code = (
            "import sys, matplotlib; matplotlib.use('svg'); "
            "import environment_belief; import environment_belief.outputs; "
            "assert matplotlib.get_backend().lower() == 'svg'; "
            "assert 'reachability_guided_aerial_perception' not in sys.modules; "
            "assert 'rm4d' not in sys.modules; assert 'rospy' not in sys.modules"
        )
        completed = subprocess.run([sys.executable, '-W', 'error', '-c', code],
                                   env=environment, capture_output=True, text=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)


if __name__ == '__main__':
    unittest.main()
