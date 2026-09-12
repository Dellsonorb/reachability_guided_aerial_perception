"""Small presentation-only checks; no robot code or simulation is executed."""
import csv
import importlib.util
from pathlib import Path
import unittest

import numpy as np

HERE = Path(__file__).resolve().parent


class PresentationChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = HERE / 'render_figures.py'
        cls.renderer_path = path
        if path.exists():
            spec = importlib.util.spec_from_file_location('paper_renderer', path)
            cls.renderer = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.renderer)

    def test_renderer_exists(self):
        self.assertTrue(self.renderer_path.exists(), 'manuscript-only renderer is missing')

    def test_counts_are_read_from_original_tables(self):
        if not self.renderer_path.exists():
            self.skipTest('renderer not yet implemented')
        data = self.renderer.load_tables()
        self.assertEqual([r['Scenes'] for r in data['paired']], ['53', '27', '3', '13'])
        self.assertEqual([r['Retrieval success'] for r in data['overall']], ['80', '56'])
        self.assertEqual(sum(int(r['N']) for r in data['difficulty']), 96)
        self.assertEqual(len(data['failures']), 8)
        self.assertEqual(sum(int(r['Generic failures']) for r in data['failures']), 40)
        self.assertEqual(sum(int(r['Ours failures']) for r in data['failures']), 16)

    def test_exact_anchor_is_not_rounded_to_grid(self):
        if not self.renderer_path.exists():
            self.skipTest('renderer not yet implemented')
        q = dict(x=2.3963625278486025, y=-.4547808304258378, yaw=.31)
        vertices = self.renderer.footprint_vertices(q, dict(half_length_m=.52, half_width_m=.39))
        np.testing.assert_allclose(vertices.mean(axis=0), [q['x'], q['y']], atol=1e-14)
        self.assertAlmostEqual(np.linalg.norm(vertices[1]-vertices[0]), 1.04)

    def test_translation_preserves_all_numeric_table_cells(self):
        if not self.renderer_path.exists():
            self.skipTest('renderer not yet implemented')
        for path in (HERE.parent / 'paper1_evaluation').glob('table*.csv'):
            with path.open(newline='') as handle:
                rows = list(csv.reader(handle))
            tex = self.renderer.chinese_table(rows)
            for row in rows[1:]:
                for cell in row:
                    try:
                        float(cell)
                    except ValueError:
                        continue
                    self.assertIn(cell, tex)


if __name__ == '__main__':
    unittest.main()
