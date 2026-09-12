import copy
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np


class DeficitTests(unittest.TestCase):
    def plan(self,h,s,w,remaining=2,cost=None,valid=None):
        self.assertTrue((Path(__file__).resolve().parents[1]/'src/a6_pilot/deficit.py').exists(),
                        'independent single-step control not implemented')
        from a6_pilot.deficit import plan_deficit
        return plan_deficit(h,s,w,remaining=remaining,
            first_cost=np.zeros(len(w)) if cost is None else cost,
            valid=np.ones(len(w),bool) if valid is None else valid)

    def test_candidate_ownership_not_pooled_mass(self):
        p=self.plan([1,1,1],[[0,1],[2]],[[1,0,0],[0,0,.75]])
        self.assertEqual(p['view_ids'],[1])
        self.assertEqual(p['maximizing_candidate_indices'],[1])
        self.assertEqual(p['progress'],.75)

    def test_one_vote_per_window_and_real_remaining_budget(self):
        self.assertEqual(self.plan([0],[[0]],[[1]])['progress'],.5)
        self.assertEqual(self.plan([0],[[0]],[[1]],remaining=1)['status'],'SUPPORT_BUDGET_IMPOSSIBLE')
        self.assertEqual(self.plan([1],[[0]],[[0]])['status'],'NO_NOMINAL_PROGRESS')
        self.assertEqual(self.plan([1],[[0]],[[1]],remaining=0)['status'],'VIEW_BUDGET_REACHED')

    def test_does_not_chase_budget_impossible_candidate(self):
        p=self.plan([0,1],[[0],[1]],[[1,0],[0,.1]],remaining=1)
        self.assertEqual(p['view_ids'],[1])
        self.assertEqual(p['vote_budget_possible_candidates'],1)

    def test_cost_stable_tie_and_validity(self):
        self.assertEqual(self.plan([1],[[0]],[[1],[1],[1]],cost=[2,1,1])['view_ids'],[1])
        self.assertEqual(self.plan([1],[[0]],[[1],[1]],valid=[False,True])['view_ids'],[1])
        self.assertEqual(self.plan([1],[[0]],[[1]],valid=[False])['status'],'NO_NOMINAL_PROGRESS')

    def test_empty_and_confirmed(self):
        self.assertEqual(self.plan([0],[],[[1]])['status'],'NO_CURRENT_SUPPORT')
        self.assertEqual(self.plan([2],[[0]],[[1]],remaining=0)['status'],'ALREADY_CONFIRMED')

    def test_unsigned_counts_above_support_threshold(self):
        p=self.plan(np.array([3,1],dtype=np.uint16),[[0,1]],[[0,1]],remaining=1)
        self.assertEqual(p['view_ids'],[0])

    def test_immutable_and_invalid(self):
        h=np.array([1]);w=np.array([[.5]])
        self.plan(h,[[0]],w)
        np.testing.assert_array_equal(h,[1]);np.testing.assert_array_equal(w,[[.5]])
        for h,w in [([-1],[[1]]),([1],[[float('nan')]]),([.5],[[1]])]:
            with self.assertRaises(ValueError):self.plan(h,[[0]],w)

    def test_adapter_fallback_and_input_guard(self):
        self.plan([1],[[0]],[[0]])
        from a6_pilot.deficit import apply_deficit
        from reachability_guided_nbv import Viewpoint
        from sim_active_perception.core import A5Config
        choice=dict(anchor_semantics='exact-validated-winner-v1.2',assessments=[dict(footprint_clipped=False,
            source_id=17,operational=dict(blocked=False,covered_cells=[0]))],next_viewpoint=[1,0,1,0],stop_reason=None)
        before=copy.deepcopy(choice)
        rank=NS(candidates=[NS(viewpoint=Viewpoint((0,0,1),0),status='VALID',flight_cost=0)],
                observation_opportunity=np.zeros((1,1,1)),acquisition_metadata=dict(model='finite_scan_v1'))
        op=NS(ground_presence_votes=np.array([[1]]),config=NS(free_observations=2))
        new=apply_deficit(choice,rank,op,A5Config(),1)
        self.assertTrue(new['deficit_plan']['fallback_to_ours']);self.assertEqual(new['next_viewpoint'],choice['next_viewpoint'])
        rank.observation_opportunity[:]=1
        new=apply_deficit(choice,rank,op,A5Config(),1)
        self.assertEqual(new['deficit_plan']['maximizing_source_ids'],[17])
        self.assertEqual(new['next_viewpoint'],[0,0,1,0]);self.assertEqual(choice,before)
        with self.assertRaises(ValueError):apply_deficit(choice,rank,None,A5Config(),1)

    def test_explicit_development_entry_only(self):
        import run_retrieval
        p={'status':'DEVELOPMENT_BATCH','max_viewpoints':3}
        d=run_retrieval.task_config(p,{'id':'x'},'deficit')
        self.assertEqual(d['slots'][0]['method'],'deficit');self.assertEqual(p,{'status':'DEVELOPMENT_BATCH','max_viewpoints':3})
        from a6_pilot.policy import DEVELOPMENT_METHODS,METHODS
        self.assertIn('deficit',DEVELOPMENT_METHODS);self.assertNotIn('deficit',METHODS)
