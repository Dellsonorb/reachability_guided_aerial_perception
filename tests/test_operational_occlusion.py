"""Prediction geometry must not restore the gate's removed coarse-cell aliasing."""

from dataclasses import replace
import unittest

import numpy as np

from environment_belief import BeliefConfig, EnvironmentBeliefMapper, EnvironmentGridSpec, EnvironmentState
from operational_gating import PerceivedTarget
from operational_gating.core import OperationalEvidenceView
from operational_gating.subcell import AmbiguousEndpointEvidence


class OperationalOcclusionTests(unittest.TestCase):
    def scene(self, *, environment=0, ambiguous=1, complete=True):
        grid = EnvironmentGridSpec((0, 0), 1, 1)
        config = BeliefConfig()
        belief = EnvironmentBeliefMapper(grid, config).snapshot()
        belief = replace(belief, state=np.full(grid.shape, EnvironmentState.OCCUPIED),
                         occupied_evidence=np.ones(grid.shape, dtype=np.uint32))
        z = np.zeros(grid.shape, dtype=int)
        # Target and ambiguity occupy only the left edge; the right part can
        # receive real ground endpoints even though raw A2 cell is OCCUPIED.
        target = PerceivedTarget((.01, .05, .05), 0, (.02, .02, .1))
        sidecar = AmbiguousEndpointEvidence(
            points_xy=np.array([[.01, .05]]), observation_indices=np.array([0]),
            row_indices=np.array([0]), cell_ids=np.array([0]),
            complete_vote_counts=np.array([[1 if complete else 0]])) if ambiguous else None
        context = OperationalEvidenceView(grid, config, target,
            environment_occupied_votes=z + environment, ambiguous_occupied_votes=z + ambiguous,
            target_occupied_votes=z + 1, ground_votes=z, ambiguous_endpoints=sidecar,
            ground_presence_votes=z + 1)
        return belief, context

    def geometry(self, belief, context):
        from reachability_guided_nbv.operational_occlusion import build_operational_occlusion
        return build_operational_occlusion(belief, context, assumed_height_m=1.)

    def test_local_ambiguity_does_not_occlude_other_side_of_shared_cell(self):
        belief, context = self.scene()
        geometry = self.geometry(belief, context)
        self.assertFalse(geometry.blocked(np.array([.08, .05, 2.]), np.array([[.08, .05, 0.]]))[0])
        self.assertEqual(belief.state[0, 0], EnvironmentState.OCCUPIED)
        self.assertEqual(context.ground_presence_votes[0, 0], 1)

    def test_true_ambiguity_intersection_blocks_and_overflight_does_not(self):
        belief, context = self.scene()
        geometry = self.geometry(belief, context)
        self.assertTrue(geometry.blocked(np.array([.01, .05, 2.]), np.array([[.01, .05, 0.]]))[0])
        self.assertFalse(geometry.blocked(np.array([-.1, .05, 2.]), np.array([[.1, .05, 1.5]]))[0])

    def test_disk_corners_are_not_square_or_whole_cell_blockers(self):
        belief, context = self.scene()
        geometry = self.geometry(belief, context)
        # Outside radius .033 but inside its AABB, also outside the short target
        # in Z: only the actual ambiguity cylinder is relevant.
        xy = [.0364, .0764]
        self.assertFalse(geometry.blocked(np.array([*xy, .8]), np.array([[*xy, .2]]))[0])
        self.assertTrue(geometry.blocked(np.array([.043, .05, .8]), np.array([[.043, .05, .2]]))[0])

    def test_environment_and_missing_ambiguity_history_keep_full_cell_prism(self):
        for values in ({'environment': 1}, {'complete': False}):
            belief, context = self.scene(**values)
            geometry = self.geometry(belief, context)
            self.assertTrue(geometry.blocked(np.array([.08, .05, 2.]), np.array([[.08, .05, 0.]]))[0])

    def test_unaccounted_raw_history_remains_a_full_cell_blocker(self):
        belief, context = self.scene()
        belief = replace(belief, occupied_evidence=np.array([[3]]))
        geometry = self.geometry(belief, context)
        self.assertTrue(geometry.blocked(np.array([.08, .05, 2.]), np.array([[.08, .05, 0.]]))[0])

    def test_target_uses_its_full_oriented_box_not_one_meter_grid_height(self):
        belief, context = self.scene(ambiguous=0)
        context = replace(context, ambiguous_endpoints=None)
        geometry = self.geometry(belief, context)
        self.assertFalse(geometry.blocked(np.array([-.1, .05, .5]), np.array([[.1, .05, .5]]))[0])
        self.assertTrue(geometry.blocked(np.array([-.1, .05, .05]), np.array([[.1, .05, .05]]))[0])
        self.assertTrue(geometry.contains(np.array([.01, .05, .05])))

    def test_unknown_grid_remains_transparent_and_alignment_is_required(self):
        from reachability_guided_nbv.operational_occlusion import build_operational_occlusion
        belief, context = self.scene()
        other = replace(belief, grid=replace(belief.grid, origin_xy=(1., 1.)))
        with self.assertRaisesRegex(ValueError, 'aligned'):
            build_operational_occlusion(other, context)

    def test_finite_scan_uses_local_occluders_without_raw_occupied_cell_veto(self):
        from reachability_guided_nbv.finite_scan import FiniteScan
        from reachability_guided_nbv import SensorModel, Viewpoint
        belief, context = self.scene()
        scan = FiniteScan(np.array([[0., 0., -1.]]), packet_rows=1,
                          sensor=SensorModel(T_uav_lidar=np.eye(4)))
        pose = Viewpoint((.08, .05, 2.), 0)
        self.assertEqual(scan.predict(belief, pose).opportunity[0, 0], 0)
        result = scan.predict(belief, pose, occlusion=self.geometry(belief, context))
        self.assertEqual(result.opportunity[0, 0], 1)
        blocked = scan.predict(belief, Viewpoint((.01, .05, 2.), 0),
                               occlusion=self.geometry(belief, context))
        self.assertEqual(blocked.opportunity[0, 0], 0)
        self.assertEqual(blocked.unoccluded_opportunity[0, 0], 1)


if __name__ == '__main__':
    unittest.main()
