import copy
import importlib.util
import unittest
from pathlib import Path

import numpy as np


class ConfirmationPolicyTests(unittest.TestCase):
    def api(self):
        path = Path(__file__).resolve().parents[1]/'src/a6_pilot/confirmation.py'
        self.assertTrue(path.is_file(), 'independent confirmation policy is missing')
        from a6_pilot.confirmation import plan_confirmation
        return plan_confirmation

    def plan(self, h, supports, w, remaining=2, **kwargs):
        w = np.asarray(w, float)
        defaults = dict(poses=np.array([[i,0,1,0] for i in range(len(w))]),
            valid=np.ones(len(w), bool), first_cost=np.arange(len(w), dtype=float),
            remaining=remaining, offsets=(-2,-1,0,1,2), yaw_samples=8,
            yaw_cost=.25, facade_tolerance=.05)
        return self.api()(np.array(h), supports, w, **(defaults|kwargs))

    def test_complementary_views_preserve_candidate_and_condition(self):
        p=self.plan([1,1], [[0,1]], np.eye(2))
        self.assertEqual(p['status'], 'ALL_PHASE_NOMINAL_PLAN')
        self.assertEqual(p['view_ids'], [0,1])
        self.assertEqual(p['supported_candidate_indices'], [0])

    def test_zero_support_exceeds_one_remaining_window(self):
        p=self.plan([0], [[0]], [[1]], remaining=1)
        self.assertEqual(p['status'], 'SUPPORT_BUDGET_IMPOSSIBLE')
        self.assertEqual(p['view_ids'], [])

    def test_nominal_no_plan_is_not_true_budget_impossibility(self):
        p=self.plan([1], [[0]], [[0]], remaining=1)
        self.assertEqual(p['status'], 'NO_NOMINAL_PLAN')
        self.assertEqual(p['vote_budget_possible_candidates'], 1)

    def test_candidate_or_not_union_and_shorter_sufficient_plan(self):
        p=self.plan([1,1], [[0],[1]], np.eye(2))
        self.assertEqual(p['view_ids'], [0])
        self.assertEqual(p['supported_candidate_indices'], [0])

    def test_repeat_window_does_not_add_multiple_votes_per_window(self):
        p=self.plan([0], [[0]], [[1]])
        self.assertEqual(p['view_ids'], [0,0])

    def test_nominal_guarantee_precedes_optimistic_shortcut(self):
        p=self.plan([1,1], [[0,1]], [[.5,.5],[1,0],[0,1]])
        self.assertEqual(p['view_ids'], [1,2])

    def test_unknown_phase_is_marked_optimistic(self):
        p=self.plan([1], [[0]], [[.5]])
        self.assertEqual(p['status'], 'OPTIMISTIC_NOMINAL_PLAN')

    def test_no_yaw_only_second_leg(self):
        p=self.plan([1,1], [[0,1]], np.eye(2), poses=np.array([[0,0,1,0],[0,0,1,np.pi/4]]))
        self.assertEqual(p['status'], 'NO_NOMINAL_PLAN')

    def test_no_support_and_confirmed_are_not_sensing_plans(self):
        self.assertEqual(self.plan([0], [], [[1]])['status'], 'NO_CURRENT_SUPPORT')
        self.assertEqual(self.plan([2], [[0]], [[1]])['status'], 'ALREADY_CONFIRMED')
        self.assertEqual(self.plan([1], [[0]], [[1]], remaining=0)['status'], 'VIEW_BUDGET_REACHED')

    def test_inputs_stay_unchanged_and_invalid_values_rejected(self):
        w=np.eye(2); before=w.copy()
        self.plan([1,1], [[0,1]], w)
        np.testing.assert_array_equal(w,before)
        with self.assertRaises(ValueError): self.plan([-1], [[0]], [[1]])
        with self.assertRaises(ValueError): self.plan([1], [[0]], [[np.nan]])

    def test_historical_offline_bounds_agree_for_two_window_alternatives(self):
        # A seed is only for this synthetic unit test, not a robot input.
        rng=np.random.default_rng(141)
        for _ in range(25):
            h=rng.integers(0,2,4); w=rng.integers(0,2,(3,4)).astype(float)
            supports=[[0,1],[2,3]]
            possible=any(any(np.all((h+w[i]+w[j])[s]>=2) for s in supports) for i in range(3) for j in range(3))
            p=self.plan(h,supports,w)
            self.assertEqual(bool(p['view_ids']), possible)

    def test_explicit_policy_route_requires_actual_operational_votes(self):
        from a6_pilot.policy import decide_policy
        from sim_active_perception.core import A5Config
        from tests.test_a5_core import inputs
        from tests.test_field import candidate
        from environment_belief import EnvironmentGridSpec,EnvironmentBeliefMapper
        from reachability_guided_nbv import Viewpoint
        field,raw=inputs([candidate(candidate_id='one',x=.011,y=.019,margin=.3)])
        belief=EnvironmentBeliefMapper(EnvironmentGridSpec((-1.5,-1.5),30,30)).snapshot()
        with self.assertRaisesRegex(ValueError,'real operational ground-presence votes'):
            decide_policy(field,raw,belief,Viewpoint((-4,0,1.5),0),method='confirmation',round_count=1,config=A5Config())

    def test_no_plan_delegates_to_ours_and_records_it_without_mutation(self):
        self.api()
        from a6_pilot.confirmation import apply_confirmation
        from types import SimpleNamespace as NS
        from reachability_guided_nbv import Viewpoint
        from reachability_guided_nbv.model import NBVConfig
        from sim_active_perception.core import A5Config
        choice=dict(anchor_semantics='exact-validated-winner-v1.2',assessments=[dict(footprint_clipped=False,source_id=123,
            operational=dict(blocked=False,covered_cells=[0]))],stop_reason=None,
            next_viewpoint=[1,0,1,0],confirmed_candidate_count=0)
        before=copy.deepcopy(choice)
        ranking=NS(candidates=[NS(viewpoint=Viewpoint((0,0,1),0),status='VALID',flight_cost=0)],
            observation_opportunity=np.zeros((1,1,1)),config=NBVConfig(),acquisition_metadata=dict(model='finite_scan_v1'))
        op=NS(ground_presence_votes=np.array([[1]]),config=NS(free_observations=2))
        result=apply_confirmation(choice,ranking,op,A5Config(),1)
        self.assertEqual(result['completion_plan']['status'],'NO_NOMINAL_PLAN')
        self.assertTrue(result['completion_plan']['fallback_to_ours'])
        self.assertEqual(result['next_viewpoint'],choice['next_viewpoint'])
        self.assertEqual(choice,before)
        op.ground_presence_votes[:]=0
        result=apply_confirmation(choice,ranking,op,A5Config(),2)
        self.assertEqual(result['stop_reason'],'SUPPORT_BUDGET_IMPOSSIBLE')
        self.assertIsNone(result['next_viewpoint'])
        ranking.acquisition_metadata={'model':'idealized'}
        with self.assertRaisesRegex(ValueError,'exact winners, finite scan and two ground votes'):
            apply_confirmation(choice,ranking,op,A5Config(),1)
