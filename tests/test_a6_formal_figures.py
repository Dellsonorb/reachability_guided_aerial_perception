"""Synthetic final-only figures: literal counts, missingness, and all-trial resources."""

import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/plot_a6_formal.py'
TIERS = ('easy', 'moderate', 'hard')
CORE = ('rm4d_only', 'fixed', 'generic', 'ours')


def synthetic_final():
    pairs = {'easy': [(True, True), (True, False)],
             'moderate': [(False, True), (False, False)],
             'hard': [(True, False), (False, True)]}
    rows, counts = [], {}
    for tier in TIERS:
        counts[tier] = dict(n=2, b=0, c=0, both_success=0, neither_success=0)
        for index, (ours, generic) in enumerate(pairs[tier], 1):
            category = 'both_success' if ours and generic else 'b' if ours else 'c' if generic else 'neither_success'
            counts[tier][category] += 1
            for method in CORE + (('no_occlusion', 'no_cost') if tier == 'hard' else ()):
                slot = len(rows) + 1
                success = ours if method == 'ours' else generic if method == 'generic' else method != 'fixed'
                rows.append(dict(slot=slot, scene=f'{tier}-{index:03d}', tier=tier,
                                 seed=100 * TIERS.index(tier) + index, method=method,
                                 status='VALID_TRIAL', retrieval_success=success,
                                 D_env_applicable=method != 'rm4d_only',
                                 D_env=None if method == 'rm4d_only' else True, D_exec=True,
                                 uav_active_distance_m=0. if method == 'rm4d_only' else float(slot),
                                 uav_active_complete=True, ground_total_distance_m=float(slot) * 2,
                                 ground_total_complete=True, T_task_sim=float(slot) * 10))
    fixed = [row for row in rows if row['tier'] == 'easy' and row['method'] == 'fixed']
    fixed[0].update(uav_active_distance_m=None, uav_active_complete=False,
                    uav_active_observed_distance_lower_bound_m=99., T_task_sim=100.)
    fixed[1].update(uav_active_distance_m=8., T_task_sim=4.)
    next(row for row in rows if row['tier'] == 'hard' and row['method'] == 'generic')['D_env'] = None
    for row in rows:
        if row['method'] == 'no_cost':
            row.update(ground_total_distance_m=None, ground_total_complete=False,
                       ground_total_observed_distance_lower_bound_m=55.)
    analysis = dict(status='COMPLETE', expected_primary_pairs=6, complete_primary_pairs=6,
                    primary_inference=dict(n=6, tiers=counts, b=2, c=2))
    return analysis, dict(resource_source='original_online_metrics', rows=rows)


class FormalFigureTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(SCRIPT.is_file(), 'bounded final-only Matplotlib exporter must exist')
        spec = importlib.util.spec_from_file_location('plot_a6_formal', SCRIPT)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.analysis, self.tables = synthetic_final()
        self.addCleanup(lambda: plt.close('all'))

    def test_incomplete_analysis_is_rejected_before_any_figure_exists(self):
        for change in (dict(status='INCOMPLETE'), dict(primary_inference=None),
                       dict(complete_primary_pairs=5)):
            with self.subTest(change=change):
                before = plt.get_fignums()
                with self.assertRaises(ValueError):
                    self.module.build_figures(dict(self.analysis, **change), self.tables)
                self.assertEqual(plt.get_fignums(), before)

    def test_final_gate_rejects_missing_or_unfinished_slots_and_mismatched_pairs(self):
        variants = []
        missing = copy.deepcopy(self.tables)
        missing['rows'].pop()
        variants.append(missing)
        for field, value in [('status', 'RUNNING'), ('retrieval_success', None)]:
            invalid = copy.deepcopy(self.tables)
            invalid['rows'][0][field] = value
            variants.append(invalid)
        mismatch = copy.deepcopy(self.tables)
        mismatch['rows'][3]['retrieval_success'] = False
        variants.append(mismatch)
        duplicate = copy.deepcopy(self.tables)
        duplicate['rows'].append(duplicate['rows'][0])
        variants.append(duplicate)
        for tables in variants:
            with self.subTest(tables=tables), self.assertRaises(ValueError):
                self.module.build_figures(self.analysis, tables)
        self.assertEqual(plt.get_fignums(), [])

    def test_secondary_bag_metrics_cannot_silently_replace_primary_resources(self):
        self.tables['resource_source'] = 'secondary_bag_public_tf'
        with self.assertRaises(ValueError):
            self.module.build_figures(self.analysis, self.tables)

    def test_paired_bars_are_the_final_per_tier_category_counts(self):
        figures = self.module.build_figures(self.analysis, self.tables)
        self.assertEqual(set(figures), {'outcomes', 'resources'})
        axis = figures['outcomes'].axes[0]
        for container, key in zip(axis.containers, ('both_success', 'b', 'c', 'neither_success')):
            self.assertEqual([patch.get_width() for patch in container],
                             [self.analysis['primary_inference']['tiers'][tier][key] for tier in TIERS])
        self.assertEqual(len(axis.containers), 4)

    def test_confirmation_na_and_missing_endpoint_have_literal_available_denominators(self):
        axis = self.module.build_figures(self.analysis, self.tables)['outcomes'].axes[1]
        texts = {text.get_position(): text.get_text() for text in axis.texts}
        self.assertEqual(texts[0, 0], 'N/A')  # Easy/RM4D confirmation, not 0/2.
        self.assertEqual(texts[0, 10], '1/1')  # Hard/Generic has one unavailable D_env.
        self.assertEqual(texts[2, 1], '0/2')  # Easy/Fixed: two observed valid failures.
        self.assertTrue(np.ma.is_masked(axis.images[0].get_array()[0, 0]))

    def test_resource_values_include_failures_without_zero_or_lower_bound_imputation(self):
        figure = self.module.build_figures(self.analysis, self.tables)['resources']
        active, ground, runtime = figure.axes
        self.assertEqual(list(active.lines[1].get_xdata()), [8.])
        self.assertEqual(list(active.lines[0].get_xdata()), [0., 0.])
        self.assertEqual(list(runtime.lines[1].get_xdata()), [100., 4.])
        self.assertEqual(list(ground.lines[13].get_xdata()), [])
        self.assertIn('1/2', [text.get_text() for text in active.texts])
        self.assertIn('0/2', [text.get_text() for text in ground.texts])
        self.assertIn('not time-to-success', ' '.join(text.get_text() for text in figure.texts))
        self.assertIn('sim s', runtime.get_title())

    def test_input_objects_remain_unchanged(self):
        before = copy.deepcopy((self.analysis, self.tables))
        self.module.build_figures(self.analysis, self.tables)
        self.assertEqual((self.analysis, self.tables), before)

    def test_outcome_tier_labels_fit_inside_the_exported_page(self):
        figure = self.module.build_figures(self.analysis, self.tables)['outcomes']
        figure.canvas.draw()
        renderer = figure.canvas.get_renderer()
        for label in figure.axes[0].get_yticklabels():
            self.assertGreaterEqual(label.get_window_extent(renderer).x0, 0., label.get_text())

    def test_cli_writes_only_two_final_pdfs_and_refuses_interim_output(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            analysis, tables, output = root / 'analysis.json', root / 'tables.json', root / 'figures'
            analysis.write_text(json.dumps(dict(self.analysis, status='INCOMPLETE')))
            tables.write_text(json.dumps(self.tables))
            args = ['--analysis', str(analysis), '--tables', str(tables), '--output-dir', str(output)]
            with self.assertRaises(ValueError):
                self.module.main(args)
            self.assertFalse(output.exists())
            analysis.write_text(json.dumps(self.analysis))
            before = (analysis.read_bytes(), tables.read_bytes())
            self.assertEqual(self.module.main(args), 0)
            self.assertEqual(sorted(path.name for path in output.iterdir()),
                             ['formal-outcomes.pdf', 'formal-resources.pdf'])
            for path in output.iterdir():
                self.assertTrue(path.read_bytes().startswith(b'%PDF'))
            self.assertEqual((analysis.read_bytes(), tables.read_bytes()), before)


if __name__ == '__main__':
    unittest.main()
