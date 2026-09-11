"""Publication-only reduction: no simulation, no source-result writes."""
import copy
import importlib.util
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))


class PublicationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = ROOT / 'scripts/build_paper1_evaluation_package.py'
        cls.module = None
        if cls.path.exists():
            spec = importlib.util.spec_from_file_location('paper_assets', cls.path)
            cls.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.module)

    def require_module(self):
        self.assertIsNotNone(self.module, 'publication exporter does not exist')
        return self.module

    def test_paired_binary_counts_and_prespecified_effect_interval(self):
        m = self.require_module()
        pairs = [dict(scene=str(i), ours=dict(retrieval_success=o),
                      generic=dict(retrieval_success=g))
                 for i, (o,g) in enumerate([(True,True)]*53+[(True,False)]*27+
                                           [(False,True)]*3+[(False,False)]*13)]
        original = copy.deepcopy(pairs)
        result = m.paired_statistics(pairs)
        self.assertEqual([result[k] for k in ('a','b','c','d')], [53,27,3,13])
        self.assertEqual(result['ours_successes'], 80)
        self.assertEqual(result['generic_successes'], 56)
        self.assertEqual(result['risk_difference'], .25)
        self.assertAlmostEqual(result['p_value'], 8.430331945419312e-6)
        self.assertAlmostEqual(result['conservative_exact_rd_interval'][0], .0856133655338313)
        self.assertEqual(pairs, original)

    def test_missing_outcome_not_silently_failure_or_excluded(self):
        m = self.require_module()
        with self.assertRaises(ValueError):
            m.paired_statistics([dict(scene='s',ours=dict(retrieval_success=None),
                                      generic=dict(retrieval_success=False))])

    def test_stage_mapping_preserves_budget_vs_sensing_and_success(self):
        m = self.require_module()
        make = lambda stage, reason: dict(retrieval_success=False, failure_stage=stage, failure_reason=reason)
        self.assertEqual(m.failure_display_stage(make('active','A5 stopped (VIEW_BUDGET_REACHED) without a confirmed exact candidate')), 'confirmed_candidate')
        self.assertEqual(m.failure_display_stage(make('active','A5 fresh MID360 PointCloud2 window capture timed out')), 'uav_observation')
        self.assertEqual(m.failure_display_stage(make('aerial_observe','A5 Ground map TF is stale')), 'uav_observation')
        self.assertEqual(m.failure_display_stage(make('execution_screen','no confirmed exact candidate passing bounded execution screen')), 'screened_candidate')
        self.assertEqual(m.failure_display_stage(make('refined_pregrasp','rejected')), 'd_exec')
        self.assertEqual(m.failure_display_stage(make('descend','collision')), 'grasp')
        self.assertEqual(m.failure_display_stage(make('retention','lost confirmation')), 'lift_retention')
        self.assertIsNone(m.failure_display_stage(dict(retrieval_success=True)))
        with self.assertRaises(ValueError):
            m.failure_display_stage(make('unrecognized_stage', 'unknown'))

    def test_completed_fixed_records_have_exactly_16_and_40_terminal_failures(self):
        m = self.require_module()
        import json
        a = json.loads((ROOT / 'outputs/paper1-final-eval-v1/analysis-latest.json').read_text())
        rows = m.failure_table(a['slots'])
        self.assertEqual(sum(r['ours'] for r in rows), 16)
        self.assertEqual(sum(r['generic'] for r in rows), 40)
        self.assertEqual(next(r for r in rows if r['stage']=='confirmed_candidate')['generic'],29)
        self.assertEqual(next(r for r in rows if r['stage']=='bunker_navigation')['ours'],0)

    def test_latex_escapes_underscores_and_exports_effect_not_pvalue_ci(self):
        m = self.require_module()
        text=m.latex_table(['metric','value'],[['risk_difference',.25]])
        self.assertIn(r'risk\_difference',text)
        self.assertIn(r'\begin{tabular}',text)

    def test_missing_sim_checkout_does_not_block_offline_tables(self):
        m=self.require_module()
        import tempfile
        import subprocess
        with tempfile.TemporaryDirectory() as directory:
            try:
                value=m.git_ref(Path(directory)/'absent', 'HEAD')
            except subprocess.CalledProcessError:
                self.fail('Optional local version probe must report unavailable, not prevent table export')
            self.assertIsNone(value)


if __name__ == '__main__': unittest.main()
