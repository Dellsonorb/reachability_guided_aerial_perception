import unittest

import numpy as np

from environment_belief import (
    BeliefConfig, EnvironmentBeliefMapper, EnvironmentGridSpec,
    EnvironmentState, PointCloudObservation,
)


def observation(points_map, stamp=0.0, transform=None):
    transform = np.eye(4) if transform is None else np.array(transform, copy=True)
    if np.array_equal(transform, np.eye(4)):
        transform[2, 3] = 2.0
    points = np.asarray(points_map, dtype=float).reshape(-1, 3)
    sensor_points = (points - transform[:3, 3]) @ transform[:3, :3]
    return PointCloudObservation(sensor_points, 'lidar', stamp, transform)


class EnvironmentBeliefTests(unittest.TestCase):
    def setUp(self):
        self.grid = EnvironmentGridSpec((0.0, 0.0), 30, 20)
        self.mapper = EnvironmentBeliefMapper(self.grid, sensor_frame='lidar')
        self.cell = (3, 2)

    def test_ground_support_requires_two_observations_not_two_points(self):
        cloud = observation([[0.25, 0.35, 0.0]] * 1000)
        update = self.mapper.update(cloud)
        first = self.mapper.snapshot()
        self.assertEqual(update.ground_points, 1000)
        self.assertEqual(update.free_cells, 1)
        self.assertEqual(first.state[self.cell], EnvironmentState.UNKNOWN)
        self.assertEqual(first.free_evidence[self.cell], 1)
        self.assertAlmostEqual(first.unknown_score[self.cell], np.exp(-0.5))
        self.mapper.update(observation([[0.25, 0.35, 0.0]], stamp=1.0))
        second = self.mapper.snapshot()
        self.assertEqual(second.state[self.cell], EnvironmentState.FREE)
        self.assertEqual(second.observation_count[self.cell], 2)
        self.assertAlmostEqual(second.unknown_score[self.cell], np.exp(-1.0))
        self.assertEqual(second.unknown_score[0, 0], 1.0)

    def test_occupied_wins_within_observation_and_across_observations(self):
        ground = observation([[0.25, 0.35, 0.0]])
        mixed = observation([[0.25, 0.35, 0.0], [0.25, 0.35, 0.1]] * 5)
        self.mapper.update(mixed)
        snapshot = self.mapper.snapshot()
        self.assertEqual(snapshot.free_evidence[self.cell], 0)
        self.assertEqual(snapshot.occupied_evidence[self.cell], 1)
        for _ in range(3):
            self.mapper.update(ground)
        snapshot = self.mapper.snapshot()
        self.assertEqual(snapshot.state[self.cell], EnvironmentState.OCCUPIED)
        self.assertEqual(snapshot.free_evidence[self.cell], 3)
        self.assertEqual(snapshot.observation_count[self.cell], 4)

    def test_observation_order_does_not_change_static_belief(self):
        clouds = [observation([[0.25, 0.35, z]]) for z in (0, 0.1, 0, 0)]
        other = EnvironmentBeliefMapper(self.grid, sensor_frame='lidar')
        for cloud in clouds:
            self.mapper.update(cloud)
        for cloud in reversed(clouds):
            other.update(cloud)
        for name in ('state', 'free_evidence', 'occupied_evidence', 'observation_count', 'unknown_score'):
            np.testing.assert_array_equal(getattr(self.mapper.snapshot(), name), getattr(other.snapshot(), name))

    def test_ground_and_obstacle_height_boundaries_and_ambiguous_gap(self):
        # Use a sensor at z=0 to keep threshold tests independent of subtraction error.
        heights = [-0.021, -0.02, 0.02, 0.03, 0.049, 0.05, 3.0]
        points = [[0.25 + i * 0.1, 0.35, z] for i, z in enumerate(heights)]
        self.mapper.update(PointCloudObservation(np.array(points), 'lidar', 0, np.eye(4)))
        state = self.mapper.snapshot()
        np.testing.assert_array_equal(state.free_evidence[3, 2:9], [0, 1, 1, 0, 0, 0, 0])
        np.testing.assert_array_equal(state.occupied_evidence[3, 2:9], [0, 0, 0, 0, 0, 1, 1])

    def test_known_elevated_horizontal_ground_plane(self):
        mapper = EnvironmentBeliefMapper(self.grid, BeliefConfig(ground_z_m=0.5), 'lidar')
        mapper.update(observation([[0.25, 0.35, 0.5], [0.45, 0.35, 0.6]]))
        snapshot = mapper.snapshot()
        self.assertEqual(snapshot.free_evidence[3, 2], 1)
        self.assertEqual(snapshot.occupied_evidence[3, 4], 1)

    def test_explicit_uav_rotation_and_mount_composition(self):
        uav = np.eye(4)
        uav[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        uav[:3, 3] = [1.05, 1.05, 2.0]
        mount = np.eye(4)
        mount[:3, :3] = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
        mount[:3, 3] = [0.1, 0.0, 0.2]
        cloud = PointCloudObservation(np.array([[2.2, 0.0, 0.0]]), 'lidar', 2, uav @ mount)
        self.mapper.update(cloud)
        snapshot = self.mapper.snapshot()
        self.assertEqual(snapshot.free_evidence[11, 10], 1)
        self.assertEqual(snapshot.observation_count.sum(), 1)

    def test_overhead_ray_projection_does_not_observe_crossed_cells(self):
        pose = np.eye(4)
        pose[:3, 3] = [0.25, 0.35, 2]
        self.mapper.update(observation([[2.25, 0.35, 0]], transform=pose))
        snapshot = self.mapper.snapshot()
        self.assertEqual(snapshot.free_evidence[3, 22], 1)
        self.assertTrue(np.all(snapshot.state[3, 2:22] == EnvironmentState.UNKNOWN))
        self.assertTrue(np.all(snapshot.unknown_score[3, 2:22] == 1))

    def test_invalid_returns_are_filtered_before_transform_with_partitioned_counts(self):
        pose = np.eye(4)
        pose[:3, 3] = [0.25, 0.35, 2]
        points = np.array([[0, 0, 0], [0, 0, -0.2], [0, 0, -40],
                           [np.nan, 0, 0], [0, 0, -2], [0, 0, -1],
                           [4, 0, -2], [0, 0, -1.97]])
        valid = np.array([True, True, True, True, True, False, True, True])
        stats = self.mapper.update(PointCloudObservation(points, 'lidar', 0, pose, valid))
        self.assertEqual(stats.input_points, 8)
        self.assertEqual(stats.masked_invalid_points, 1)
        self.assertEqual(stats.nonfinite_points, 1)
        self.assertEqual(stats.out_of_range_points, 3)
        self.assertEqual(stats.out_of_grid_points, 1)
        self.assertEqual(stats.ambiguous_points, 1)
        self.assertEqual(stats.ground_points, 1)
        self.assertEqual(stats.obstacle_points, 0)
        self.assertEqual(self.mapper.snapshot().occupied_evidence.sum(), 0)

    def test_empty_and_ambiguous_scans_do_not_add_evidence(self):
        self.mapper.update(observation([]))
        self.mapper.update(observation([[0.25, 0.35, 0.03], [0.35, 0.35, -0.1]]))
        snapshot = self.mapper.snapshot()
        self.assertEqual(snapshot.observation_count.sum(), 0)
        self.assertTrue(np.all(snapshot.state == EnvironmentState.UNKNOWN))
        self.assertTrue(np.all(snapshot.unknown_score == 1))

    def test_half_open_grid_and_fixed_origin(self):
        self.mapper.update(observation([[0, 0, 0], [3, 0.05, 0], [0.05, 2, 0], [-0.01, 0.05, 0]]))
        self.assertEqual(self.mapper.snapshot().free_evidence[0, 0], 1)
        self.assertEqual(self.mapper.snapshot().observation_count.sum(), 1)
        self.assertEqual(self.mapper.snapshot().origin_xy, (0, 0))
        self.assertEqual(self.mapper.snapshot().resolution_m, 0.1)

    def test_bad_frames_and_transforms_rejected_without_update(self):
        cloud = observation([[0.25, 0.35, 0]])
        wrong = PointCloudObservation(cloud.points_xyz, 'wrong', 0, cloud.T_map_sensor)
        with self.assertRaises(ValueError):
            self.mapper.update(wrong)
        for bad in (np.zeros((4, 4)), np.diag([-1, 1, 1, 1]), np.diag([2, 1, 1, 1]), np.full((4, 4), np.nan)):
            with self.subTest(transform=bad), self.assertRaises(ValueError):
                PointCloudObservation(cloud.points_xyz, 'lidar', 0, bad)
        self.assertEqual(self.mapper.snapshot().observation_count.sum(), 0)

    def test_internal_edges_and_adjacent_floats_use_half_open_cells(self):
        grid = EnvironmentGridSpec((-1.5, -1.5), 30, 30)
        for axis in (0, 1):
            for index in (2, 4, 15, 28):
                edge = grid.origin_xy[axis] + index * grid.resolution_m
                for coordinate, expected in ((np.nextafter(edge, -np.inf), index - 1),
                                             (edge, index),
                                             (np.nextafter(edge, np.inf), index)):
                    with self.subTest(axis=axis, index=index, coordinate=coordinate):
                        point = [-1.45, -1.45, 0.0]
                        point[axis] = coordinate
                        mapper = EnvironmentBeliefMapper(grid)
                        mapper.update(observation([point]))
                        expected_cell = (0, expected) if axis == 0 else (expected, 0)
                        self.assertEqual(mapper.snapshot().free_evidence[expected_cell], 1)
                        self.assertEqual(mapper.snapshot().observation_count.sum(), 1)

    def test_configuration_and_shapes_are_checked(self):
        for changes in ({'obstacle_min_height_m': 0.01}, {'ground_tolerance_m': -1},
                        {'unknown_scale': 0}, {'free_observations': 1.5},
                        {'min_range_m': 40}, {'ground_z_m': np.inf}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                BeliefConfig(**changes)
        for args in (((0, 0), 0, 20), ((0, np.nan), 30, 20)):
            with self.assertRaises(ValueError):
                EnvironmentGridSpec(*args)
        with self.assertRaises(ValueError):
            EnvironmentGridSpec((0, 0), 30, 20, frame_id='world')
        with self.assertRaises(ValueError):
            PointCloudObservation(np.zeros((2, 2)), 'lidar', 0, np.eye(4))
        with self.assertRaises(ValueError):
            PointCloudObservation(np.zeros((2, 3)), 'lidar', 0, np.eye(4), np.array([1, 0]))

    def test_snapshots_and_observations_do_not_alias_caller_arrays(self):
        cloud = observation([[0.25, 0.35, 0]])
        before = cloud.points_xyz.copy()
        self.mapper.update(cloud)
        first = self.mapper.snapshot()
        self.mapper.update(cloud)
        self.assertEqual(first.free_evidence[self.cell], 1)
        np.testing.assert_array_equal(cloud.points_xyz, before)
        # Even changing a detached snapshot cannot affect mapper state.
        first.free_evidence.setflags(write=True)
        first.free_evidence[self.cell] = 100
        self.assertEqual(self.mapper.snapshot().free_evidence[self.cell], 2)


if __name__ == '__main__':
    unittest.main()
