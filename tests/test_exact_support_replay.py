"""Read-only v1.2 diagnostics never promote a less-blocked non-winner."""

import importlib.util
import json
from pathlib import Path
import shutil
import tempfile
import unittest

import numpy as np

from environment_belief import BeliefConfig, EnvironmentGridSpec
from operational_gating import OperationalEvidenceView, PerceivedTarget
from sim_active_perception.core import candidate_catalog
from tests.test_a5_core import inputs
from tests.test_field import candidate


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/replay_exact_support.py'
HARD15 = SCRIPT.parents[1] / 'outputs/a6/formal/slot-015-hard-002-generic-01'


class ExactSupportReplayTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(SCRIPT.is_file(), 'exact-support replay is not implemented')
        spec = importlib.util.spec_from_file_location('exact_support_replay', SCRIPT)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.assertTrue(callable(getattr(self.module, 'nonwinner_diagnostics', None)))
        self.grid = EnvironmentGridSpec((-1.5, -1.5), 30, 30)
        zeros = np.zeros(self.grid.shape, dtype=np.int64)
        self.view = OperationalEvidenceView(
            self.grid, BeliefConfig(), PerceivedTarget((.67, 0, .0575), 0),
            zeros, zeros, zeros, zeros)

    def test_blocked_winner_clear_nonwinner_is_counted_without_reselection(self):
        field, raw = inputs([
            candidate(candidate_id='winner', x=.09, y=.001, margin=.5),
            candidate(candidate_id='alternative', x=.001, y=.001, margin=.3),
        ])
        before = candidate_catalog(field, raw)
        report = self.module.nonwinner_diagnostics(field, raw, self.view)
        self.assertEqual(report['winner_blocked_nonwinner_unblocked_cells'], 1)
        self.assertEqual(report['unblocked_nonwinner_count'], 1)
        self.assertEqual(report['cells'][0]['winner']['candidate_id'], 'winner')
        self.assertEqual(report['cells'][0]['unblocked_nonwinners'][0]['candidate_id'], 'alternative')
        self.assertTrue(report['cells'][0]['winner']['gate']['target_collision'])
        self.assertFalse(report['cells'][0]['unblocked_nonwinners'][0]['gate']['ground_supported'])
        self.assertEqual(candidate_catalog(field, raw), before)
        self.assertEqual(before[0]['candidate_id'], 'winner')

    def test_tied_clear_nonwinner_does_not_replace_first_blocked_winner(self):
        field, raw = inputs([
            candidate(candidate_id='first', x=.09, y=.001, margin=.5),
            candidate(candidate_id='second', x=.001, y=.001, margin=.6),
        ])
        report = self.module.nonwinner_diagnostics(field, raw, self.view)
        self.assertEqual(report['winner_blocked_nonwinner_unblocked_cells'], 1)
        self.assertEqual(candidate_catalog(field, raw)[0]['candidate_id'], 'first')

    def test_clear_winner_or_invalid_nonwinner_is_not_reported_as_rescue(self):
        for records in (
            [candidate(candidate_id='winner', x=.001, y=.001, margin=.5),
             candidate(candidate_id='other', x=.09, y=.001, margin=.3)],
            [candidate(candidate_id='winner', x=.09, y=.001, margin=.5),
             candidate(candidate_id='invalid', x=.001, y=.001, margin=.3, valid=False)],
        ):
            field, raw = inputs(records)
            report = self.module.nonwinner_diagnostics(field, raw, self.view)
            self.assertEqual(report['winner_blocked_nonwinner_unblocked_cells'], 0)
            self.assertEqual(report['unblocked_nonwinner_count'], 0)

    def test_hard002_replay_unlocks_geometry_but_does_not_invent_ground_votes(self):
        self.assertTrue(callable(getattr(self.module, 'replay_attempt', None)), 'recorded replay missing')
        protected = [HARD15 / 'attempt.json', HARD15 / 'data/initial.json',
                     HARD15 / 'data/metrics.json', HARD15 / 'data/rounds/round-03/a2/belief.npz']
        before = [p.read_bytes() for p in protected]
        report = self.module.replay_attempt(HARD15)
        self.assertEqual(len(report['rounds']), 3)
        for row in report['rounds']:
            self.assertTrue(all(row['legacy_replay_checks'].values()))
            self.assertTrue(all(row['a4_input_checks'].values()))
            self.assertEqual(row['v11_operational_cells'], 0)
            self.assertGreater(row['v12_operational_cells'], 0)
            self.assertGreater(row['v12_uncertainty_mass'], 0)
            selected = next(c for c in row['candidates'] if c['source_id'] == 624)
            self.assertTrue(selected['v11']['representative_blocked'])
            self.assertFalse(selected['v12']['representative_blocked'])
            self.assertFalse(selected['v12']['operational']['target_collision'])
            self.assertFalse(selected['v12']['confirmed'])
        self.assertEqual(selected['v12']['operational']['ground_supported_cells'], 87)
        self.assertEqual(selected['v12']['operational']['ground_missing_cells'], 1)
        self.assertEqual([p.read_bytes() for p in protected], before)

    def test_relocated_attempt_uses_its_bundled_reference_not_stale_absolute_path(self):
        original = (HARD15 / 'data/initial.json').read_bytes()
        with tempfile.TemporaryDirectory() as temporary:
            relocated = Path(temporary) / 'relocated-attempt'
            shutil.copytree(HARD15 / 'data', relocated / 'data')
            path = relocated / 'data/initial.json'
            initial = json.loads(path.read_text())
            initial['target_reference_file'] = str(Path(temporary) / 'missing-old-checkout/reference.npz')
            path.write_text(json.dumps(initial))
            before = path.read_bytes()
            report = self.module.replay_attempt(relocated)
            self.assertEqual(len(report['rounds']), 3)
            self.assertTrue(all(all(r['legacy_replay_checks'].values()) for r in report['rounds']))
            self.assertEqual(path.read_bytes(), before)
        self.assertEqual((HARD15 / 'data/initial.json').read_bytes(), original)


if __name__ == '__main__':
    unittest.main()
