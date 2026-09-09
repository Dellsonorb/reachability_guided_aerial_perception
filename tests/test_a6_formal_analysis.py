"""Prospective paired inference, not simulated robot outcomes."""
import importlib.util
import copy
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FormalAnalysisTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = ROOT/'scripts/analyze_a6_formal.py'
        cls.module = None
        if path.exists():
            spec = importlib.util.spec_from_file_location('formal_analysis_test', path)
            cls.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.module)

    def setUp(self):
        self.assertIsNotNone(self.module, 'prospective paired analysis must exist')
        self.a = self.module

    def test_exact_mcnemar_zero_and_known_discordance(self):
        self.assertEqual(self.a.exact_mcnemar(0, 0), 1.)
        self.assertAlmostEqual(self.a.exact_mcnemar(6, 0), .03125)
        self.assertEqual(self.a.exact_mcnemar(2, 9), self.a.exact_mcnemar(9, 2))

    def test_invalid_binomial_counts_are_rejected(self):
        for b, c in ((-1, 2), (.5, 2), (True, 3)):
            with self.assertRaises(ValueError): self.a.exact_mcnemar(b, c)

    def test_boundary_interval_is_not_zero_width(self):
        r = self.a.paired_statistics({t: dict(n=40, b=0, c=0)
                                      for t in ('easy', 'moderate', 'hard')})
        self.assertEqual(r['risk_difference'], 0.)
        self.assertEqual(r['p_value'], 1.)
        lo, hi = r['conservative_exact_rd_interval']
        self.assertLess(lo, -.12)
        self.assertGreater(hi, .12)
        self.assertIsNone(r['approximate_rd_interval'])

    def test_stratified_mean_and_paired_variance(self):
        r = self.a.paired_statistics({t: dict(n=40, b=14, c=6)
                                      for t in ('easy', 'moderate', 'hard')})
        self.assertAlmostEqual(r['risk_difference'], .2)
        expected = ((20-40*.2**2)/39/40/3)**.5
        self.assertAlmostEqual(r['standard_error'], expected)
        self.assertLess(r['conservative_exact_rd_interval'][0], 0.)
        self.assertGreater(r['approximate_rd_interval'][0], 0.)
        self.assertIn('within every tier', r['exact_null'])

    def test_equal_tier_weighting_is_not_accidentally_size_weighted(self):
        r = self.a.paired_statistics(dict(a=dict(n=10,b=8,c=0),
                                          b=dict(n=40,b=0,c=8)))
        self.assertAlmostEqual(r['risk_difference'], .3)

    def test_power_is_planning_not_fitted_to_pilot(self):
        self.assertAlmostEqual(self.a.planning_power(120, .2, .5), .86223, places=4)
        self.assertAlmostEqual(self.a.planning_power(120, .2, .6), .78792, places=4)

    def test_holm_two_secondary_tests(self):
        self.assertEqual(self.a.holm([.01,.04]), [.02,.04])
        self.assertEqual(self.a.holm([.04,.01]), [.04,.02])

    def test_partial_study_does_not_receive_final_inference(self):
        cfg = dict(status='FROZEN_FOR_FORMAL', scenes=[dict(id='easy-001',tier='easy',seed=1)],
                   slots=[dict(slot=1,scene='easy-001',method='ours'),
                          dict(slot=2,scene='easy-001',method='generic')])
        summary = dict(config_status='FROZEN_FOR_FORMAL', slots=[], primary_comparison=dict(pairs=[],incomplete_pairs=[{}]),
                       first_activation_invalid_as_failure=dict(pairs=[],incomplete_pairs=[{}]))
        report = self.a.analyze(cfg, summary)
        self.assertEqual(report['status'], 'INCOMPLETE')
        self.assertIsNone(report['primary_inference'])

    def test_pilot_cannot_be_pooled_as_formal(self):
        with self.assertRaisesRegex(ValueError, 'formal'):
            self.a.analyze(dict(status='FROZEN_FOR_PILOT'), {})

    def completed_fixture(self):
        from test_a6_report import ReportTests
        fixture = ReportTests()
        fixture.setUpClass()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        fixture.config['status'] = 'FROZEN_FOR_FORMAL'
        for scene in fixture.config['scenes']: scene['tier'] = scene['id']
        for slot in fixture.config['slots']:
            if slot['method'] == 'ours':
                fixture.activation(slot['slot'],None,status='INVALID_TRIAL',suffix='01',kind='FORMAL_ATTEMPT')
            fixture.activation(slot['slot'],slot['method']=='ours',suffix='02',kind='FORMAL_ATTEMPT')
        return fixture, fixture.config, fixture.summarize()

    def test_complete_collector_primary_sensitivity_ablations_missing_resources(self):
        _,cfg,summary = self.completed_fixture()
        result = self.a.analyze(cfg,summary)
        self.assertEqual(result['status'],'COMPLETE')
        self.assertEqual(result['primary_inference']['b'],3)
        self.assertEqual(result['sensitivity_inference']['b'],0)
        self.assertEqual(result['sensitivity_inference']['p_value'],1.)
        self.assertEqual(len(result['hard_ablations']),2)
        self.assertTrue(all(a['secondary_holm_p_value']==1. for a in result['hard_ablations']))
        self.assertTrue(all(r['paths'] is None for r in summary['slots']))
        self.assertEqual(result['planning_assumptions']['delta'],.2)

    def test_mismatched_or_duplicate_scene_and_seed_cannot_supply_complete_analysis(self):
        _,cfg,summary = self.completed_fixture()
        for kind in ('duplicate','seed','category','pilot_summary','slot','binary_disagrees_with_slot'):
            with self.subTest(kind=kind):
                bad = copy.deepcopy(summary)
                if kind == 'duplicate':
                    bad['primary_comparison']['pairs'][1] = copy.deepcopy(bad['primary_comparison']['pairs'][0])
                elif kind == 'seed': bad['primary_comparison']['pairs'][0]['seed'] = -1
                elif kind == 'category': bad['primary_comparison']['pairs'][0]['category'] = 'neither_success'
                elif kind == 'pilot_summary': bad['config_status'] = 'FROZEN_FOR_PILOT'
                elif kind == 'binary_disagrees_with_slot':
                    bad['primary_comparison']['pairs'][0]['ours']['retrieval_success'] = False
                    bad['primary_comparison']['pairs'][0]['category'] = 'neither_success'
                else: bad['slots'][0]['seed'] = -1
                with self.assertRaises(ValueError): self.a.analyze(cfg,bad)

    def test_unfinished_first_activation_keeps_sensitivity_unavailable(self):
        fixture,cfg,summary = self.completed_fixture()
        record_path = next(fixture.results.glob('slot-02-ours-01/attempt.json'))
        record = json.loads(record_path.read_text()); record.pop('finish_wall')
        record_path.write_text(json.dumps(record))
        result = self.a.analyze(cfg,fixture.summarize())
        self.assertEqual(result['status'],'COMPLETE')
        self.assertIsNone(result['sensitivity_inference'])


if __name__ == '__main__': unittest.main()
