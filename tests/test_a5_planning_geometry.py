import importlib.util
from pathlib import Path
import unittest

import numpy as np


class PlanningGeometryTests(unittest.TestCase):
    def test_local_grasp_matches_after_bridge_and_exposes_missing_height(self):
        path = Path(__file__).resolve().parents[1] / 'scripts/check_a5_frame_bridge.py'
        self.assertTrue(path.exists(), 'integration geometry diagnostic is missing')
        spec = importlib.util.spec_from_file_location('a5_planning_check', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        mount = np.eye(4); mount[:3, 3] = [.15, 0, .122]
        base = np.eye(4); base[:3, 3] = [2.5, .3, .36]
        base[:2, :2] = [[0, -1], [1, 0]]
        ref_base = base.copy(); ref_base[2, 3] = 0
        tcp = np.eye(4); tcp[:3, 3] = [2, 0, .08]
        ref_tcp = tcp.copy(); ref_tcp[2, 3] -= .36
        local = np.linalg.inv(ref_base @ mount) @ ref_tcp
        candidate = {'T_map_bunker': base.tolist(), 'T_aubo_flange': local.tolist()}
        report = module.compare_local_geometry(candidate, tcp, tcp, mount, np.eye(4))
        self.assertLess(report['query_local_max_abs_error'], 1e-12)
        np.testing.assert_allclose(report['T_aubo_exact_grasp'], local, atol=1e-12)
        candidate['T_map_bunker'] = ref_base.tolist()
        report = module.compare_local_geometry(candidate, tcp, tcp, mount, np.eye(4))
        self.assertAlmostEqual(report['query_local_max_abs_error'], .36)


if __name__ == '__main__':
    unittest.main()
