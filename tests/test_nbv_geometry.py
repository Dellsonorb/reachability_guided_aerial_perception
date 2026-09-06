import unittest
from dataclasses import replace

import numpy as np

from environment_belief import EnvironmentBeliefMapper, EnvironmentGridSpec, PointCloudObservation
from reachability_guided_nbv import (
    NBVConfig, SensorModel, Viewpoint, generate_candidates, predict_visibility, sensor_transform,
)
from reachability_guided_nbv.geometry import ground_targets, segments_intersect_box


def belief_with_obstacles(points=()):
    grid = EnvironmentGridSpec((-1, -1), 70, 20)
    mapper = EnvironmentBeliefMapper(grid)
    if len(points):
        transform = np.eye(4)
        transform[2, 3] = 2
        mapper.update(PointCloudObservation(np.asarray(points) - [0, 0, 2], 'lidar', 0, transform))
    return mapper.snapshot()


class GeometryTests(unittest.TestCase):
    def test_composite_mount_and_yaw_change_vertical_fov(self):
        model = SensorModel()
        pitch = .35
        rotation = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0],
                             [-np.sin(pitch), 0, np.cos(pitch)]])
        np.testing.assert_allclose(model.T_uav_lidar[:3, 3], [.13, 0, .23] + rotation @ [0, 0, .05])
        np.testing.assert_allclose(model.T_uav_lidar[:3, :3], rotation)
        belief = belief_with_obstacles()
        forward = predict_visibility(belief, Viewpoint((0, 0, 1.5), 0))
        backward = predict_visibility(belief, Viewpoint((0, 0, 1.5), np.pi))
        self.assertTrue(forward.visible[10, 50])  # Ground center (4.05,.05,0).
        self.assertFalse(backward.visible[10, 50])

    def test_actual_mount_keeps_eight_yaws_and_current_pose(self):
        current = Viewpoint((0, 0, 1.5), .3)
        candidates = generate_candidates(current)
        self.assertEqual(len(candidates), 200)
        self.assertEqual(candidates[0], current)
        self.assertTrue(all(v.position_xyz[2] == 1.5 for v in candidates))
        self.assertEqual(len(set(candidates)), len(candidates))

    def test_upright_centered_mount_collapses_equivalent_yaws(self):
        model = SensorModel(T_uav_lidar=np.eye(4))
        self.assertEqual(len(generate_candidates(Viewpoint((0, 0, 2), .2), sensor=model)), 25)
        offset = np.eye(4)
        offset[0, 3] = .1
        self.assertEqual(len(generate_candidates(Viewpoint((0, 0, 2), 0),
                                                  sensor=SensorModel(T_uav_lidar=offset))), 200)

    def test_unknown_does_not_occlude_and_occupied_target_is_excluded(self):
        pose = Viewpoint((0, 0, 1.5), 0)
        clear = predict_visibility(belief_with_obstacles(), pose)
        occupied = predict_visibility(belief_with_obstacles([(4.05, .05, .2)]), pose)
        self.assertTrue(clear.visible[10, 50])
        self.assertFalse(occupied.visible[10, 50])

    def test_ray_through_prism_is_blocked_but_overflight_is_not(self):
        pose = Viewpoint((0, 0, 1.5), 0)
        above = predict_visibility(belief_with_obstacles([(1.05, .05, .2)]), pose)
        through = predict_visibility(belief_with_obstacles([(3.05, .05, .2)]), pose)
        self.assertTrue(above.visible[10, 50])
        self.assertFalse(above.occluded[10, 50])
        self.assertTrue(through.range_fov[10, 50])
        self.assertTrue(through.occluded[10, 50])
        self.assertFalse(through.visible[10, 50])

    def test_slab_parallel_and_closed_boundary(self):
        end = np.array([[4, 0, 0], [4, 0, 2], [4, 2, 0], [4, .5, 0]])
        np.testing.assert_array_equal(
            segments_intersect_box(np.array([0, 0, 2]), end, [2, -.5, 0], [3, .5, 1]),
            [True, False, False, True],
        )

    def test_sensor_inside_prism_or_below_ground_is_rejected(self):
        model = SensorModel(T_uav_lidar=np.eye(4))
        belief = belief_with_obstacles([(.05, .05, .2)])
        prediction = predict_visibility(belief, Viewpoint((.05, .05, .5), 0), sensor=model)
        self.assertEqual(prediction.status, 'SENSOR_INSIDE_ASSUMED_PRISM')
        self.assertFalse(prediction.visible.any())
        self.assertEqual(predict_visibility(belief, Viewpoint((0, 0, 0), 0), sensor=model).status,
                         'SENSOR_AT_OR_BELOW_GROUND')

    def test_range_is_a2_strict_endpoint_range(self):
        belief = belief_with_obstacles()
        pose = Viewpoint((0, 0, 1.5), 0)
        t = sensor_transform(pose, SensorModel())
        distance = np.linalg.norm(np.array([4.05, .05, 0]) - t[:3, 3])
        belief = replace(belief, config=replace(belief.config, max_range_m=distance - 1e-9))
        self.assertFalse(predict_visibility(belief, pose).visible[10, 50])

    def test_exact_range_boundaries_are_excluded(self):
        belief = belief_with_obstacles()
        pose = Viewpoint((0, 0, 1.5), 0)
        t = sensor_transform(pose, SensorModel())
        sensor_points = (ground_targets(belief) - t[:3, 3]) @ t[:3, :3]
        distance = np.linalg.norm(sensor_points, axis=1).reshape(belief.grid.shape)[10, 50]
        for name in ('min_range_m', 'max_range_m'):
            with self.subTest(bound=name):
                exact = replace(belief, config=replace(belief.config, **{name: distance}))
                self.assertFalse(predict_visibility(exact, pose).visible[10, 50])
                interior = distance + (-1e-9 if name == 'min_range_m' else 1e-9)
                inside = replace(belief, config=replace(belief.config, **{name: interior}))
                self.assertTrue(predict_visibility(inside, pose).visible[10, 50])

    def test_invalid_pose_transform_and_config(self):
        for xyz in ((0, 0, np.nan), (1, 2)):
            with self.assertRaises(ValueError):
                Viewpoint(xyz, 0)
        with self.assertRaises(ValueError):
            Viewpoint((0, 0, 2), 0, frame_id='odom')
        with self.assertRaises(ValueError):
            SensorModel(T_uav_lidar=np.zeros((4, 4)))
        for kwargs in ({'assumed_height_m': 0}, {'flight_weight': -1}, {'yaw_cost_m_per_rad': np.nan}):
            with self.assertRaises(ValueError):
                NBVConfig(**kwargs)


if __name__ == '__main__':
    unittest.main()
