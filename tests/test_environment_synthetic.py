import unittest

import numpy as np

from environment_belief import EnvironmentBeliefMapper, EnvironmentState
from environment_belief.synthetic import Box, cast_observation, make_scenarios, sensor_pose


def map_endpoints(cloud):
    return cloud.points_xyz @ cloud.T_map_sensor[:3, :3].T + cloud.T_map_sensor[:3, 3]


class SyntheticEnvironmentTests(unittest.TestCase):
    def test_nearest_box_occludes_ground_and_upward_no_hit_is_invalid(self):
        box = Box((-0.2, -0.2, 0), (0.2, 0.2, 0.5))
        pose = sensor_pose((0, 0, 2), yaw=0.4, pitch=0.25)
        cloud = cast_observation(np.array([[0, 0, 0], [1, 0, 0], [0, 0, 3]]), pose, (box,))
        np.testing.assert_allclose(map_endpoints(cloud)[:2], [[0, 0, 0.5], [1, 0, 0]], atol=1e-12)
        np.testing.assert_array_equal(cloud.valid_return, [True, True, False])
        np.testing.assert_array_equal(cloud.points_xyz[2], [0, 0, 0])

    def test_nearest_box_wins_independent_of_box_order(self):
        near = Box((-0.2, -0.2, 0.8), (0.2, 0.2, 1.0))
        far = Box((-0.2, -0.2, 0), (0.2, 0.2, 0.5))
        pose = sensor_pose((0, 0, 2))
        for boxes in ((near, far), (far, near)):
            cloud = cast_observation(np.array([[0, 0, 0]]), pose, boxes)
            np.testing.assert_allclose(map_endpoints(cloud), [[0, 0, 1]], atol=1e-12)

    def run_scene(self, name):
        scene = {s.name: s for s in make_scenarios()}[name]
        mapper = EnvironmentBeliefMapper(scene.grid, sensor_frame='lidar')
        history = []
        for cloud in scene.observations:
            mapper.update(cloud)
            history.append(mapper.snapshot())
        return scene, history

    def test_clear_has_repeated_ground_support_and_unsampled_unknown_cells(self):
        scene, history = self.run_scene('clear')
        self.assertEqual(len(history), 2)
        self.assertEqual(np.count_nonzero(history[0].state == EnvironmentState.FREE), 0)
        self.assertEqual(np.count_nonzero(history[1].state == EnvironmentState.FREE), 576)
        self.assertEqual(np.count_nonzero(history[1].state == EnvironmentState.UNKNOWN), 324)
        self.assertEqual(history[1].occupied_evidence.sum(), 0)

    def test_overflight_keeps_box_projection_unknown_then_hits_win_over_ground(self):
        scene, history = self.run_scene('overflight_obstacle')
        row, col = scene.probe_cells['box_edge']
        for belief in history[:2]:
            self.assertEqual(belief.state[row, col], EnvironmentState.UNKNOWN)
            self.assertEqual(belief.unknown_score[row, col], 1)
        for belief in history[2:]:
            self.assertEqual(belief.state[row, col], EnvironmentState.OCCUPIED)
        self.assertEqual(history[-1].free_evidence[row, col], 2)
        self.assertEqual(history[-1].occupied_evidence[row, col], 1)
        self.assertEqual(history[-1].observation_count[row, col], 3)
        np.testing.assert_allclose(history[-1].unknown_score[row, col], np.exp(-1.5))
        # Both high observations really pass over the entire box before ground hit.
        for cloud in scene.observations[:2]:
            origin = cloud.T_map_sensor[:3, 3]
            hits = map_endpoints(cloud)[cloud.valid_return]
            for x in (scene.boxes[0].lower[0], scene.boxes[0].upper[0]):
                t = (x - origin[0]) / (hits[:, 0] - origin[0])
                z = origin[2] + t * (hits[:, 2] - origin[2])
                self.assertTrue(np.all(z > scene.boxes[0].upper[2]))

    def test_occlusion_patch_becomes_free_only_after_two_new_viewpoints(self):
        scene, history = self.run_scene('occlusion_multiview')
        row, col = scene.probe_cells['hidden_ground']
        self.assertEqual(history[0].state[row, col], EnvironmentState.UNKNOWN)
        self.assertEqual(history[0].observation_count[row, col], 0)
        self.assertEqual(history[0].unknown_score[row, col], 1)
        self.assertEqual(history[1].state[row, col], EnvironmentState.UNKNOWN)
        self.assertEqual(history[1].free_evidence[row, col], 1)
        self.assertEqual(history[2].state[row, col], EnvironmentState.FREE)
        self.assertEqual(history[2].free_evidence[row, col], 2)
        self.assertLess(history[2].unknown_score[row, col], history[1].unknown_score[row, col])
        occupied = history[0].state == EnvironmentState.OCCUPIED
        self.assertTrue(np.any(occupied))
        self.assertTrue(np.all(history[-1].state[occupied] == EnvironmentState.OCCUPIED))
        self.assertEqual(np.count_nonzero(history[-1].state == EnvironmentState.FREE), 72)


if __name__ == '__main__':
    unittest.main()
