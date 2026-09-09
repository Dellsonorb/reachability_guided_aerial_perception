import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts' / 'a6_exact_support_diagnostics.py'
RECORDED = ROOT / 'outputs' / 'a6' / 'v12-integration' / 'natural-attempt-02'


def load_module():
    spec = importlib.util.spec_from_file_location('a6_exact_support_diagnostics', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(SCRIPT.parent))
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path.pop(0)
    return module


def load_snapshot(directory=RECORDED):
    documents = [json.loads((directory / name).read_text()) for name in
                 ('initial.json', 'ranking.json', 'decision.json', 'operational_summary.json')]
    arrays = {}
    for filename in ('fields.npz', 'operational_evidence.npz'):
        with np.load(directory / filename, allow_pickle=False) as saved:
            arrays.update({name: saved[name].copy() for name in saved.files})
    return (*documents[:3], arrays, documents[3], arrays)


class ExactSupportDiagnosticsTest(unittest.TestCase):
    def test_recorded_exact_snapshot_is_reconstructed_without_mutating_inputs(self):
        module = load_module()
        before = {path: path.read_bytes()
                  for path in RECORDED.iterdir() if path.is_file()}
        report = module.check_snapshot(*load_snapshot())
        after = {path: path.read_bytes()
                 for path in RECORDED.iterdir() if path.is_file()}

        self.assertTrue(report['all_checks_pass'], report['checks'])
        self.assertEqual(len(report['winner_anchors']), 37)
        self.assertEqual(report['nonwinner_diagnostics']['winner_blocked_nonwinner_unblocked_cells'], 2)
        self.assertEqual(report['nonwinner_diagnostics']['unblocked_nonwinner_count'], 4)
        self.assertTrue(report['diagnostic_only'])
        self.assertFalse(report['selection_changed'])
        self.assertEqual(before, after)
        for cell in report['nonwinner_diagnostics']['cells']:
            self.assertIn('ground_missing_cells', cell['winner']['gate'])
            self.assertTrue(cell['winner']['gate']['blocked'])
            self.assertTrue(all(not row['gate']['blocked'] for row in cell['unblocked_nonwinners']))

    def test_identity_assessment_array_and_config_mismatches_are_reported(self):
        module = load_module()
        cases = []
        for kind in ('identity', 'assessment', 'array', 'config', 'missing'):
            values = list(load_snapshot())
            values = [copy.deepcopy(value) if isinstance(value, dict) else value for value in values]
            if kind == 'identity':
                values[1]['a3_summary']['winner_anchors'][0]['candidate_id'] = 'altered'
            elif kind == 'assessment':
                values[2]['assessments'][0]['confirmed'] = not values[2]['assessments'][0]['confirmed']
            elif kind == 'array':
                values[3] = dict(values[3]); values[3]['a3_unknown_score'] = values[3]['a3_unknown_score'].copy()
                values[3]['a3_unknown_score'].flat[0] += 1
            elif kind == 'config':
                values[0]['config']['support_anchor'] = 'cell_center'
            else:
                values[3] = dict(values[3]); values[3].pop('a2_state')
            cases.append((kind, module.check_snapshot(*values)))
        for kind, report in cases:
            with self.subTest(kind=kind):
                self.assertFalse(report['all_checks_pass'])
                self.assertTrue(any(not value for value in report['checks'].values()))
                if kind == 'missing':
                    self.assertIsNone(report['nonwinner_diagnostics'])

    def test_operational_semantics_metadata_mismatches_are_reported(self):
        module = load_module()
        for index, path in ((0, ('operational_gating',)),
                            (1, ('a3_summary', 'operational_semantics')),
                            (2, ('operational_semantics',)),
                            (4, ('operational_semantics',))):
            values = list(load_snapshot())
            values = [copy.deepcopy(value) if isinstance(value, dict) else value for value in values]
            target = values[index]
            for key in path[:-1]:
                target = target[key]
            target[path[-1]] = 'v1'
            with self.subTest(document=index):
                self.assertFalse(module.check_snapshot(*values)['all_checks_pass'])

    def test_cli_discovers_nested_round_and_writes_only_report(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            snapshot = root / 'run-01' / 'data' / 'rounds' / 'round-03'
            snapshot.mkdir(parents=True)
            for name in ('ranking.json', 'decision.json', 'fields.npz',
                         'operational_summary.json', 'operational_evidence.npz'):
                (snapshot / name).write_bytes((RECORDED / name).read_bytes())
            (snapshot.parents[1] / 'initial.json').write_bytes((RECORDED / 'initial.json').read_bytes())
            inputs = {path: path.read_bytes() for path in root.rglob('*') if path.is_file()}
            output = root / 'reports' / 'report.json'
            result = subprocess.run([sys.executable, str(SCRIPT), '--results-dir', str(root), '--output', str(output)],
                                    cwd=ROOT, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            report = json.loads(output.read_text())
            self.assertEqual(report['snapshot_count'], 1)
            self.assertTrue(report['all_checks_pass'])
            self.assertEqual(inputs, {path: path.read_bytes() for path in inputs})

    def test_cli_fails_for_no_snapshots_and_for_a_mismatch(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / 'empty.json'
            empty = subprocess.run([sys.executable, str(SCRIPT), '--results-dir', str(root),
                                    '--output', str(output)], cwd=ROOT, text=True, capture_output=True)
            self.assertNotEqual(empty.returncode, 0)
            self.assertFalse(output.exists())

            snapshot = root / 'run-01' / 'data' / 'rounds' / 'round-03'
            snapshot.mkdir(parents=True)
            for name in ('ranking.json', 'fields.npz', 'operational_summary.json',
                         'operational_evidence.npz'):
                (snapshot / name).write_bytes((RECORDED / name).read_bytes())
            (snapshot.parents[1] / 'initial.json').write_bytes((RECORDED / 'initial.json').read_bytes())
            decision = json.loads((RECORDED / 'decision.json').read_text())
            decision['assessments'][0]['confirmed'] = not decision['assessments'][0]['confirmed']
            (snapshot / 'decision.json').write_text(json.dumps(decision))
            mismatch_output = root / 'mismatch.json'
            mismatch = subprocess.run([sys.executable, str(SCRIPT), '--results-dir', str(root),
                                       '--output', str(mismatch_output)], cwd=ROOT,
                                      text=True, capture_output=True)
            self.assertNotEqual(mismatch.returncode, 0)
            self.assertFalse(json.loads(mismatch_output.read_text())['all_checks_pass'])

    def test_cli_rejects_output_inside_source_attempt_without_overwriting_it(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            snapshot = root / 'run-01' / 'data' / 'rounds' / 'round-03'
            snapshot.mkdir(parents=True)
            for name in ('ranking.json', 'decision.json', 'fields.npz',
                         'operational_summary.json', 'operational_evidence.npz'):
                (snapshot / name).write_bytes((RECORDED / name).read_bytes())
            (snapshot.parents[1] / 'initial.json').write_bytes((RECORDED / 'initial.json').read_bytes())
            output = snapshot / 'decision.json'
            before = output.read_bytes()
            result = subprocess.run([sys.executable, str(SCRIPT), '--results-dir', str(root),
                                     '--output', str(output)], cwd=ROOT, text=True, capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(output.read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
