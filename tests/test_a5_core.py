import unittest
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np

from environment_belief import EnvironmentBeliefMapper, EnvironmentGridSpec, PointCloudObservation
from reachability_guided_aerial_perception import GraspTCP, GridSpec, build_field_from_result
from reachability_guided_nbv import Viewpoint
from task_relevant_uncertainty import build_task_uncertainty
from task_relevant_uncertainty.geometry import footprint_cells
from tests.test_field import candidate, result
from sim_active_perception.core import A5Config, candidate_catalog, assess_candidates, decide, replay_observations
from sim_active_perception.worker import save_initial


def inputs(candidates):
    grasp = GraspTCP('a5-test', 'map', (0, 0, .4), (0, 0, 0, 1))
    raw = result(candidates)
    raw['grasp_id'] = grasp.grasp_id
    return build_field_from_result(grasp, raw, grid=GridSpec.centered((0, 0), 3, 3, .1)), raw


def scan(points, stamp):
    transform = np.eye(4)
    transform[2, 3] = 2
    return PointCloudObservation(np.asarray(points, dtype=float).reshape(-1, 3) - [0, 0, 2],
                                 'uav1/lidar_link', stamp, transform)


def ground_points(grid):
    rows, cols = np.indices(grid.shape)
    return np.column_stack((grid.origin_xy[0] + (cols.ravel() + .5) * .1,
                            grid.origin_xy[1] + (rows.ravel() + .5) * .1,
                            np.zeros(rows.size)))


class A5CoreTests(unittest.TestCase):
    def setUp(self):
        self.grid = EnvironmentGridSpec((-1.5, -1.5), 30, 30)

    def test_catalog_recovers_exact_winner_not_cell_center_and_preserves_ties(self):
        first = candidate(candidate_id='first', x=.011, y=.019, margin=.3)
        tie = candidate(candidate_id='tie', x=.012, y=.02, margin=.3)
        field, raw = inputs([first, tie])
        catalog = candidate_catalog(field, raw)
        self.assertEqual(len(catalog), 1)
        self.assertEqual(catalog[0]['candidate_id'], 'first')
        self.assertEqual(catalog[0]['x'], .011)
        self.assertNotEqual(catalog[0]['x'], .05)
        self.assertEqual(catalog[0]['relevance'], .6)

    def test_exact_footprint_must_be_free_and_can_have_positive_unknown(self):
        field, raw = inputs([candidate(candidate_id='first', x=.011, y=.019, margin=.3)])
        mapper = EnvironmentBeliefMapper(self.grid, sensor_frame='uav1/lidar_link')
        empty = assess_candidates(field, mapper.snapshot(), candidate_catalog(field, raw))
        self.assertFalse(empty[0]['confirmed'])
        for stamp in (1, 2):
            mapper.update(scan(ground_points(self.grid), stamp))
        assessments = assess_candidates(field, mapper.snapshot(), candidate_catalog(field, raw))
        self.assertTrue(assessments[0]['confirmed'])
        self.assertGreater(assessments[0]['mean_unknown_score'], 0)

    def test_exact_extra_cell_occupied_blocks_even_when_representative_is_clear(self):
        field, raw = inputs([candidate(candidate_id='first', x=.099, y=.019, margin=.3)])
        exact, _ = footprint_cells(self.grid, (.099, .019), 0)
        representative, _ = footprint_cells(self.grid, (.05, .05), 0)
        extra = np.setdiff1d(exact, representative)
        self.assertGreater(len(extra), 0)
        mapper = EnvironmentBeliefMapper(self.grid, sensor_frame='uav1/lidar_link')
        for stamp in (1, 2):
            mapper.update(scan(ground_points(self.grid), stamp))
        point = ground_points(self.grid)[extra[0]].copy()
        point[2] = .2
        mapper.update(scan([point], 3))
        task = build_task_uncertainty(field, mapper.snapshot())
        self.assertFalse(task.poses[0].blocked)
        check = assess_candidates(field, mapper.snapshot(), candidate_catalog(field, raw))[0]
        self.assertFalse(check['confirmed'])
        self.assertGreater(check['occupied_cells'], 0)

    def test_replay_is_accumulation_without_double_counting(self):
        frames = [scan(ground_points(self.grid), stamp) for stamp in (1, 2)]
        belief = replay_observations(self.grid, frames)
        np.testing.assert_array_equal(belief.observation_count, 2)
        np.testing.assert_allclose(belief.unknown_score, np.exp(-1))
        with self.assertRaisesRegex(ValueError, 'increasing'):
            replay_observations(self.grid, [frames[0], frames[0]])

    def test_hover_window_chunks_are_one_frozen_a2_observation_not_extra_votes(self):
        spec = importlib.util.spec_from_file_location('a5_support', 'scripts/a5_ros_support.py')
        support = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(support)
        world_points = ground_points(self.grid)
        chunks = []
        for stamp, offset, pitch in [(1., -.03, -.02), (2., .04, .03), (3., .01, 0.)]:
            transform = support.rigid_transform([offset, 0, 2],
                                                [0, np.sin(pitch / 2), 0, np.cos(pitch / 2)])
            chunks.append(dict(points_xyz=(world_points - transform[:3, 3]) @ transform[:3, :3],
                               T_map_sensor=transform, stamp_s=stamp, frame_id='uav1/lidar_link'))
        merged = support.merge_cloud_chunks(chunks)
        observation = PointCloudObservation(merged['points_xyz'], str(merged['frame_id']),
                                            float(merged['stamp_s']), merged['T_map_sensor'])
        once = replay_observations(self.grid, [observation])
        np.testing.assert_array_equal(once.observation_count, 1)
        np.testing.assert_array_equal(once.free_evidence, 1)
        np.testing.assert_array_equal(once.occupied_evidence, 0)
        np.testing.assert_allclose(once.unknown_score, np.exp(-.5))
        second = PointCloudObservation(observation.points_xyz, observation.frame_id,
                                       6., observation.T_map_sensor)
        twice = replay_observations(self.grid, [observation, second])
        np.testing.assert_array_equal(twice.observation_count, 2)
        np.testing.assert_array_equal(twice.free_evidence, 2)

    def test_max_round_stops_even_with_positive_score_and_does_not_force_success(self):
        field, raw = inputs([candidate(candidate_id='first', x=.011, y=.019, margin=.3)])
        belief = EnvironmentBeliefMapper(self.grid).snapshot()
        choice, ranking = decide(field, raw, belief, Viewpoint((-4, 0, 1.5), 0),
                                  round_count=3, config=A5Config())
        self.assertEqual(choice['stop_reason'], 'VIEW_BUDGET_REACHED')
        self.assertIsNone(choice['selected_candidate'])
        self.assertIsNone(choice['next_viewpoint'])
        self.assertGreater(ranking.best_task.task_gain, 0)

    def test_no_task_support_stops_without_generic_fallback(self):
        field, raw = inputs([])
        belief = EnvironmentBeliefMapper(self.grid).snapshot()
        choice, _ = decide(field, raw, belief, Viewpoint((-4, 0, 1.5), 0), round_count=1)
        self.assertEqual(choice['stop_reason'], 'NO_PREDICTED_TASK_GAIN')
        self.assertIsNone(choice['next_viewpoint'])

    def test_nonpositive_score_stops_despite_positive_gain_alternatives(self):
        field, raw = inputs([candidate(candidate_id='first', x=.011, y=.019, margin=.3)])
        belief = EnvironmentBeliefMapper(self.grid).snapshot()
        choice, ranking = decide(field, raw, belief, Viewpoint((4, 0, 1.5), 0),
                                  round_count=1, config=A5Config(flight_weight=100000))
        self.assertEqual(ranking.status, 'RANKED')
        self.assertTrue(any(e.task_gain > 0 for e in ranking.candidates))
        self.assertEqual(choice['best_task_score'], 0)
        self.assertEqual(choice['stop_reason'], 'NONPOSITIVE_SCORE')
        self.assertIsNone(choice['next_viewpoint'])

    def test_equal_relevance_across_cells_preserves_raw_order(self):
        field, raw = inputs([
            candidate(candidate_id='later-cell-first', x=.299, y=.219, margin=.3),
            candidate(candidate_id='earlier-cell-second', x=-.399, y=-.419, margin=.3)])
        belief = replay_observations(self.grid, [scan(ground_points(self.grid), stamp) for stamp in (1, 2)])
        choice, _ = decide(field, raw, belief, Viewpoint((-4, 0, 1.5), 0), round_count=3)
        self.assertEqual(choice['selected_candidate']['candidate_id'], 'later-cell-first')

    def test_representative_only_occupancy_blocks_exact_free_candidate(self):
        field, raw = inputs([candidate(candidate_id='exact-clear', x=.011, y=.019, yaw=.2, margin=.3)])
        exact, _ = footprint_cells(self.grid, (.011, .019), .2)
        representative, _ = footprint_cells(self.grid, (.05, .05), .2)
        extra = np.setdiff1d(representative, exact)
        self.assertGreater(len(extra), 0)
        mapper = EnvironmentBeliefMapper(self.grid, sensor_frame='uav1/lidar_link')
        for stamp in (1, 2):
            mapper.update(scan(ground_points(self.grid), stamp))
        point = ground_points(self.grid)[extra[0]].copy()
        point[2] = .2
        mapper.update(scan([point], 3))
        assessment = assess_candidates(field, mapper.snapshot(), candidate_catalog(field, raw))[0]
        self.assertEqual(assessment['occupied_cells'], 0)
        self.assertEqual(assessment['free_cells'], len(exact))
        self.assertTrue(assessment['representative_blocked'])
        self.assertFalse(assessment['confirmed'])

    def test_yaw_only_goals_filtered_current_rescan_preserved(self):
        field, raw = inputs([candidate(candidate_id='first', x=.011, y=.019, margin=.3)])
        belief = EnvironmentBeliefMapper(self.grid).snapshot()
        current = Viewpoint((-4, 0, 1.5), 0)
        choice, ranking = decide(field, raw, belief, current, round_count=1)
        same_position = [e for e in ranking.candidates if e.viewpoint.position_xyz == current.position_xyz]
        self.assertEqual(len(same_position), 1)
        self.assertEqual(same_position[0].viewpoint, current)
        self.assertIsNone(choice['stop_reason'])

    def test_clipped_exact_footprint_is_not_confirmed(self):
        field, raw = inputs([candidate(candidate_id='edge', x=1.49, y=0, margin=.3)])
        belief = replay_observations(self.grid, [scan(ground_points(self.grid), stamp) for stamp in (1, 2)])
        assessment = assess_candidates(field, belief, candidate_catalog(field, raw))[0]
        self.assertTrue(assessment['footprint_clipped'])
        self.assertFalse(assessment['confirmed'])

    def test_worker_observation_file_boundary_preserves_accumulation_and_exact_pose(self):
        field, raw = inputs([candidate(candidate_id='first', x=.011, y=.019, margin=.3)])
        grasp = GraspTCP('a5-test', 'map', (0, 0, .4), (0, 0, 0, 1))
        config = A5Config(max_viewpoints=2, grid_width_m=3, grid_height_m=3)
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            initial = save_initial(grasp, raw, config, directory)
            paths = []
            for stamp in (1, 2):
                observation = scan(ground_points(self.grid), stamp)
                path = directory / f'cloud-{stamp}.npz'
                np.savez_compressed(path, points_xyz=observation.points_xyz,
                                    T_map_sensor=observation.T_map_sensor,
                                    frame_id=observation.frame_id, stamp_s=observation.stamp_s)
                paths.append(str(path))
            request = dict(op='observe', initial_file=initial['initial_file'], observations=paths,
                           uav_pose=[-4, 0, 1.5, 0], output_dir=str(directory / 'round-2'))
            request_path, response_path = directory / 'request.json', directory / 'response.json'
            request_path.write_text(json.dumps(request))
            process = subprocess.run([sys.executable, 'scripts/a5_core_worker.py', '--request', str(request_path),
                                      '--response', str(response_path)], capture_output=True, text=True)
            self.assertEqual(process.returncode, 0, process.stderr)
            response = json.loads(response_path.read_text())
            self.assertEqual(response['stop_reason'], 'VIEW_BUDGET_REACHED')
            self.assertEqual(response['selected_candidate']['x'], .011)
            self.assertEqual(response['total_observation_votes'], 1800)

    def test_worker_errors_are_nonzero_and_structured(self):
        with tempfile.TemporaryDirectory() as tmp:
            request, response = Path(tmp) / 'request.json', Path(tmp) / 'response.json'
            request.write_text(json.dumps({'op': 'missing'}))
            process = subprocess.run([sys.executable, 'scripts/a5_core_worker.py', '--request', str(request),
                                      '--response', str(response)], capture_output=True, text=True)
            self.assertNotEqual(process.returncode, 0)
            self.assertFalse(json.loads(response.read_text())['ok'])


if __name__ == '__main__':
    unittest.main()
