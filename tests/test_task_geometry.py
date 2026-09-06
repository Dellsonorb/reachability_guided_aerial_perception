import unittest

import numpy as np

from environment_belief import EnvironmentGridSpec
from task_relevant_uncertainty.geometry import FootprintSpec, footprint_cells, footprint_vertices


class FootprintGeometryTests(unittest.TestCase):
    def setUp(self):
        self.grid = EnvironmentGridSpec((-1.5, -1.5), 30, 30)

    def cells(self, xy=(0.05, 0.05), yaw=0, footprint=FootprintSpec()):
        indices, clipped = footprint_cells(self.grid, xy, yaw, footprint)
        return set(map(tuple, np.column_stack(np.unravel_index(indices, self.grid.shape)))), clipped

    def test_default_is_frozen_rm4d_padded_rectangle(self):
        corners = footprint_vertices((0, 0), 0)
        np.testing.assert_allclose(corners.min(axis=0), [-0.52, -0.39])
        np.testing.assert_allclose(corners.max(axis=0), [0.52, 0.39])

    def test_yaw_rotation_translation_and_axis_aligned_cell_coverage(self):
        xy = (0.05, 0.05)
        corners = footprint_vertices(xy, np.pi / 2)
        np.testing.assert_allclose(corners.min(axis=0), [-0.34, -0.47])
        np.testing.assert_allclose(corners.max(axis=0), [0.44, 0.57])
        cells, clipped = self.cells(xy)
        self.assertEqual(cells, {(r, c) for r in range(11, 20) for c in range(10, 21)})
        self.assertFalse(clipped)
        rotated, _ = self.cells(xy, np.pi / 2)
        self.assertEqual(rotated, {(c, r) for r, c in cells})
        reversed_cells, _ = self.cells(xy, np.pi)
        self.assertEqual(reversed_cells, cells)

    def test_overlapping_cell_need_not_have_center_in_footprint(self):
        cells, _ = self.cells()
        # Row center y=.45 lies outside footprint upper y=.44, but overlaps it.
        self.assertIn((19, 15), cells)

    def test_edge_and_corner_contact_count_but_separation_does_not(self):
        grid = EnvironmentGridSpec((0, 0), 6, 6)
        small = FootprintSpec(0.1, 0.1)
        cells, _ = footprint_cells(grid, (0.2, 0.2), 0, small)
        self.assertIn(3 * 6 + 3, cells)  # corner-only contact at (.3,.3)
        self.assertIn(2 * 6 + 3, cells)  # edge-only contact
        separated, _ = footprint_cells(grid, (0.2 - 1e-6, 0.2), 0, small)
        self.assertNotIn(3 * 6 + 3, separated)

    def test_rotated_rectangle_does_not_fill_its_axis_aligned_bounding_box(self):
        cells, _ = self.cells((0.05, 0.05), np.pi / 4)
        self.assertIn((15, 15), cells)
        self.assertNotIn((21, 21), cells)
        self.assertNotIn((9, 21), cells)

    def test_grid_clipping_is_reported_without_wrapping_indices(self):
        cells, clipped = self.cells((-1.45, -1.45))
        self.assertTrue(clipped)
        self.assertIn((0, 0), cells)
        self.assertTrue(all(0 <= r < 30 and 0 <= c < 30 for r, c in cells))
        # Exact contact with grid extent is not missing coverage.
        _, clipped = self.cells((-0.98, -1.11))
        self.assertFalse(clipped)

    def test_invalid_geometry_is_rejected(self):
        for args in ((0, 0.4), (-1, 1), (np.inf, 1), (1, np.nan)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                FootprintSpec(*args)
        for xy, yaw in (((np.nan, 0), 0), ((0,), 0), ((0, 0), np.inf)):
            with self.subTest(xy=xy, yaw=yaw), self.assertRaises(ValueError):
                footprint_cells(self.grid, xy, yaw)


if __name__ == '__main__':
    unittest.main()
