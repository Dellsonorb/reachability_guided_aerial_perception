import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]


class V13ReplayTests(unittest.TestCase):
    def module(self):
        path = ROOT / 'scripts/replay_v13_operational.py'
        self.assertTrue(path.exists(), 'development-only replay is not implemented')
        spec = importlib.util.spec_from_file_location('v13_replay', path)
        module = importlib.util.module_from_spec(spec)
        with patch.object(sys, 'path', [str(ROOT / 'scripts'), *sys.path]):
            spec.loader.exec_module(module)
        return module

    def test_recorded_development_comparison_is_not_a_new_trial(self):
        module = self.module()
        data = ROOT / 'outputs/a6/v12-fresh-validation/slot-003-moderate-ours-01/data'
        result = module.replay_round(data, 1)
        self.assertTrue(result['development_replay_not_trial'])
        self.assertTrue(result['all_common_checks_pass'])
        self.assertEqual(result['v12']['blocked'], 34)
        self.assertEqual(result['v13']['blocked'], 29)
        self.assertEqual(result['v13']['confirmed'], 0)
        self.assertEqual(result['v13']['target_collision'], 29)
        self.assertTrue(result['v13']['nonwinner_diagnostics']['diagnostic_only'])
        self.assertFalse(result['v13']['nonwinner_diagnostics']['selection_changed'])

    def test_callable_rejects_historical_output_before_any_writer(self):
        module = self.module()
        data = ROOT / 'outputs/a6/v12-fresh-validation/slot-003-moderate-ours-01/data'
        # Stop before any potential write even while the guard is still missing.
        with patch.object(module, 'save_operational', side_effect=RuntimeError('would write source')) as writer:
            with self.assertRaisesRegex(ValueError, 'output'):
                module.replay_round(data, 1, data / 'rounds/round-01')
            writer.assert_not_called()

    def test_relocated_record_uses_bundled_runtime_reference(self):
        module = self.module()
        data = ROOT / 'outputs/a6/v12-fresh-validation/slot-003-moderate-ours-01/data'
        read_text = Path.read_text

        def stale_path(path, *args, **kwargs):
            text = read_text(path, *args, **kwargs)
            if path == data / 'initial.json':
                saved = module.json.loads(text)
                saved['target_reference_file'] = '/missing-original-checkout/target_reference.npz'
                return module.json.dumps(saved)
            return text

        with patch.object(Path, 'read_text', stale_path):
            result = module.replay_round(data, 1)
        self.assertTrue(result['all_common_checks_pass'])


if __name__ == '__main__':
    unittest.main()
