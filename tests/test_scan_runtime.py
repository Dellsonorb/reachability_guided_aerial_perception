"""ROS-independent argument routing for the explicit SIM scan capability."""

import json
from pathlib import Path
from types import SimpleNamespace
import unittest

import run_a5_sim as adapter
from run_a6_attempt import adapter_args, ROOT


class ScanRuntimeRoutingTests(unittest.TestCase):
    def test_finite_profile_resolves_program_from_selected_sim_not_scene(self):
        profile = json.loads((ROOT / 'configs/current_sim_task.json').read_text())
        profile['acquisition_model'] = 'finite_scan_v1'
        values = adapter_args(profile, Path('/out'), Path('/selected/sim'), Path('/rm'))
        self.assertIn('--scan-pattern-path', values)
        self.assertEqual(values[values.index('--scan-pattern-path') + 1],
                         '/selected/sim/install/p450-clean/share/sim_platform_assets/models/MID360/scan_mode/mid360.csv')
        self.assertEqual(values[values.index('--scan-publisher-sdf-path') + 1],
                         '/selected/sim/install/p450-clean/share/sim_platform_assets/models/MID360/MID360.sdf')

    def test_legacy_profile_keeps_no_scan_override(self):
        profile = json.loads((ROOT / 'configs/current_sim_task.json').read_text())
        profile.pop('acquisition_model', None)
        values = adapter_args(profile, Path('/out'), Path('/sim'), Path('/rm'))
        self.assertNotIn('--scan-pattern-path', values)

    def test_unknown_model_is_not_silently_idealized(self):
        profile = json.loads((ROOT / 'configs/current_sim_task.json').read_text())
        profile['acquisition_model'] = 'finite_typo'
        with self.assertRaisesRegex(ValueError, 'acquisition_model'):
            adapter_args(profile, Path('/out'), Path('/sim'), Path('/rm'))

    def test_worker_window_comes_from_actual_capture_option(self):
        options = SimpleNamespace(scan_pattern_path=Path('/scan.csv'),
                                  scan_publisher_sdf_path=Path('/sensor.sdf'), cloud_window_s=5.)
        self.assertTrue(hasattr(adapter, 'acquisition_options'))
        self.assertEqual(adapter.acquisition_options(options),
                         dict(scan_pattern_path='/scan.csv', scan_publisher_sdf_path='/sensor.sdf', scan_window_s=5.))


if __name__ == '__main__':
    unittest.main()
