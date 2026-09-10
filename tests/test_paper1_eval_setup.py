import importlib.util
import unittest
import math


class SetupDescriptionTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('paper1_eval_setup'))
        import paper1_eval_setup
        self.setup = paper1_eval_setup

    def test_continuous_rectangles_not_coarse_grid_alias(self):
        rectangle = dict(center_xy=[0,0], size_xyz=[.1,.1,1], yaw=0)
        separated = dict(rectangle,center_xy=[.1001,0])
        touching = dict(rectangle,center_xy=[.1,0])
        self.assertFalse(self.setup.rectangles_overlap(rectangle,separated))
        self.assertTrue(self.setup.rectangles_overlap(rectangle,touching))

    def test_setup_separates_nominal_fov_from_depth_and_never_filters_scene(self):
        contract = dict(optical_xyz=[0,0,0],optical_rpy=[0,0,0],
                        target_size=[.1,.1,.1],min_depth=.2,max_depth=4.)
        scene = dict(id='synthetic',seed=7,target_xy=[0,0],target_z=2,
                     target_yaw=0,bunker_xy=[3,-2],bunker_yaw=0,boxes=[])
        row = self.setup.describe_scene(scene,[0,0,0,0],contract,math.pi/2,1.)
        self.assertTrue(row['nominal_target_inside_depth_and_fov'])
        self.assertEqual(row['scene_id'],'synthetic')
        scene['target_xy']=[3,0]
        row = self.setup.describe_scene(scene,[0,0,0,0],contract,math.pi/2,1.)
        self.assertFalse(row['nominal_target_inside_depth_and_fov'])
        self.assertTrue(row['depth_overlap'])


if __name__ == '__main__':
    unittest.main()
