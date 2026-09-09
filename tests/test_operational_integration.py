import unittest

import numpy as np

from a6_pilot.policy import decide_policy
from environment_belief import EnvironmentGridSpec
from operational_gating import OccupiedClass, PerceivedTarget, derive_operational_evidence, assess_footprint
from reachability_guided_nbv import Viewpoint
from sim_active_perception.core import assess_candidates, candidate_catalog, replay_observations
from task_relevant_uncertainty import build_task_uncertainty
from task_relevant_uncertainty.outputs import field_summary
from tests.test_a5_core import ground_points, inputs, scan
from tests.test_field import candidate


class OperationalIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.grid = EnvironmentGridSpec((-1.5, -1.5), 30, 30)
        self.field, self.raw = inputs([candidate(candidate_id='exact', x=.011, y=.019, margin=.3)])
        self.target = PerceivedTarget((.71, .019, .0575), 0)
        ground = ground_points(self.grid)
        points = np.vstack((ground, [.592, .019, .1]))
        self.observations = [scan(points, t) for t in (1., 2.)]
        self.labels = [np.full(len(points), OccupiedClass.TARGET, dtype=np.int8) for _ in (1, 2)]
        self.belief = replay_observations(self.grid, self.observations)
        self.view = derive_operational_evidence(self.grid, self.observations, self.target, labels=self.labels)

    def test_exact_alias_is_unblocked_without_mutating_raw_a2_or_nominal_relevance(self):
        before = self.belief.state.copy()
        legacy = build_task_uncertainty(self.field, self.belief)
        task = build_task_uncertainty(self.field, self.belief, operational=self.view)
        catalog = candidate_catalog(self.field, self.raw)
        old = assess_candidates(self.field, self.belief, catalog)[0]
        new = assess_candidates(self.field, self.belief, catalog, task=task, operational=self.view)[0]
        self.assertFalse(old['confirmed'])
        self.assertTrue(new['confirmed'])
        self.assertGreater(new['occupied_cells'], 0)
        self.assertFalse(new['operational']['blocked'])
        np.testing.assert_array_equal(before, self.belief.state)
        np.testing.assert_array_equal(legacy.nominal_task_relevance, task.nominal_task_relevance)
        self.assertEqual(task.operational_semantics, 'object-aware-v1.1')

    def test_true_collision_uses_same_function_for_representative_and_exact(self):
        target = PerceivedTarget((.2, 0, .0575), .3)
        view = derive_operational_evidence(self.grid, self.observations, target, labels=self.labels)
        task = build_task_uncertainty(self.field, self.belief, operational=view)
        checks = assess_candidates(self.field, self.belief, candidate_catalog(self.field, self.raw),
                                   task=task, operational=view)
        for pose in task.poses:
            direct = assess_footprint(view, pose.xy, pose.yaw)
            self.assertEqual(pose.blocked, direct.blocked)
            self.assertTrue(direct.target_collision)
        self.assertTrue(checks[0]['operational']['target_collision'])
        self.assertFalse(checks[0]['confirmed'])

    def test_unknown_does_not_block_a3_but_does_not_confirm_a5(self):
        view = derive_operational_evidence(self.grid, [], self.target)
        belief = replay_observations(self.grid, [])
        task = build_task_uncertainty(self.field, belief, operational=view)
        self.assertFalse(task.poses[0].blocked)
        self.assertFalse(assess_candidates(self.field, belief, candidate_catalog(self.field, self.raw),
                                          task=task, operational=view)[0]['confirmed'])

    def test_cached_task_cannot_supply_stale_representative_blocking(self):
        cached = build_task_uncertainty(self.field, self.belief, operational=self.view)
        target = PerceivedTarget((.68, .019, .0575), 0)
        current = derive_operational_evidence(self.grid, self.observations, target, labels=self.labels)
        assessment = assess_candidates(self.field, self.belief, candidate_catalog(self.field, self.raw),
                                       task=cached, operational=current)[0]
        self.assertFalse(assessment['operational']['blocked'])
        self.assertTrue(assessment['representative_blocked'])
        self.assertFalse(assessment['confirmed'])

    def test_generic_ours_share_operational_candidates_visibility_and_cost(self):
        current = Viewpoint((-1, 0, 1.5), 0)
        ours, ro = decide_policy(self.field, self.raw, self.belief, current,
                                 method='ours', round_count=2, operational=self.view)
        generic, rg = decide_policy(self.field, self.raw, self.belief, current,
                                    method='generic', round_count=2, operational=self.view)
        self.assertEqual(ours['assessments'], generic['assessments'])
        self.assertEqual(ours['selected_candidate'], generic['selected_candidate'])
        np.testing.assert_array_equal(ro.visibility, rg.visibility)
        self.assertEqual([c.flight_cost for c in ro.candidates], [c.flight_cost for c in rg.candidates])

    def test_mismatched_view_grid_is_rejected(self):
        view = derive_operational_evidence(EnvironmentGridSpec((0, 0), 30, 30), [], self.target)
        with self.assertRaisesRegex(ValueError, 'aligned'):
            build_task_uncertainty(self.field, self.belief, operational=view)

    def test_output_names_expose_revision_without_changing_legacy_metadata(self):
        legacy = field_summary(build_task_uncertainty(self.field, self.belief))
        summary = field_summary(build_task_uncertainty(self.field, self.belief, operational=self.view))
        self.assertNotIn('operational_semantics', legacy)
        self.assertNotIn('operational_blocked', legacy['pose_states'])
        self.assertEqual(summary.get('operational_semantics'), 'object-aware-v1.1')
        self.assertIn('target_geometry', summary['blocked_semantics'])


if __name__ == '__main__':
    unittest.main()
