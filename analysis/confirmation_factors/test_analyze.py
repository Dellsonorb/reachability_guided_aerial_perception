import importlib.util
import unittest
from pathlib import Path
import numpy as np


class FactorTests(unittest.TestCase):
    def test_fallback_not_counted_as_an_action_change(self):
        spec=importlib.util.spec_from_file_location('factors',Path(__file__).with_name('analyze.py'))
        m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
        self.assertTrue(hasattr(m,'effective_action'),'fallback-aware comparison missing')
        self.assertEqual(m.effective_action({'status':'NO_NOMINAL_PLAN','view_ids':[]},41),41)
        self.assertEqual(m.effective_action({'status':'NOMINAL_DEFICIT_PROGRESS','view_ids':[41]},41),41)
        self.assertIsNone(m.effective_action({'status':'SUPPORT_BUDGET_IMPOSSIBLE','view_ids':[]},41))

    def test_windows_first_changes_preference_not_model(self):
        path=Path(__file__).with_name('analyze.py')
        self.assertTrue(path.exists(),'offline factor analyzer missing')
        spec=importlib.util.spec_from_file_location('factors',path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
        args=dict(poses=np.array([[i,0,1,0]for i in range(3)]),valid=np.ones(3,bool),first_cost=np.arange(3),
            remaining=2,offsets=(-2,-1,0,1,2),yaw_samples=8,yaw_cost=.25,facade_tolerance=.05)
        h=np.array([1,1]);w=np.array([[.5,.5],[1,0],[0,1]])
        from a6_pilot.confirmation import plan_confirmation
        self.assertEqual(plan_confirmation(h,[[0,1]],w,**args)['view_ids'],[1,2])
        p=m.windows_first(h,[[0,1]],w,args)
        self.assertEqual(p['view_ids'],[0]);self.assertEqual(p['nominal_tier'],1)
        args['remaining']=1
        self.assertEqual(m.windows_first(np.array([0,0]),[[0,1]],w,args)['status'],'SUPPORT_BUDGET_IMPOSSIBLE')
