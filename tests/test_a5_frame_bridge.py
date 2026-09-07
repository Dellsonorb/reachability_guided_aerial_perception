import copy
from contextlib import contextmanager
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import tempfile
from types import ModuleType
import unittest
from unittest.mock import patch

import numpy as np

from sim_active_perception import worker
from tests.test_field import candidate, result


def transform(x=0., y=0., z=0., yaw=0.):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.array([[cosine, -sine, 0., x], [sine, cosine, 0., y],
                     [0., 0., 1., z], [0., 0., 0., 1.]])


def calibration(height=.36):
    return dict(T_map_ground_odom=transform(4., -3., height, .7).tolist(),
                T_ground_odom_bunker=transform(1., 2., 0., -.4).tolist(),
                T_bunker_aubo=transform(.15, 0., .122).tolist())


class FrameBridgeTests(unittest.TestCase):
    def test_map_tcp_008_becomes_reference_minus_028_without_xy_yaw_change(self):
        from sim_active_perception.frame_bridge import FrameBridge

        bridge = FrameBridge(calibration(), transform(.15, 0., .122))
        query = dict(frame_id='map', grasp_id='exact', position_xyz=[.42, -.17, .08],
                     quaternion_xyzw=[0., 5e-7, 0., math.cos(5e-7)],
                     current_bunker_pose=dict(x=4.5, y=-1.2, yaw=.3))
        before = copy.deepcopy(query)
        converted = bridge.query_to_reference(query)

        self.assertAlmostEqual(converted['position_xyz'][2], -.28)
        self.assertEqual(converted['position_xyz'][:2], query['position_xyz'][:2])
        self.assertEqual(converted['frame_id'], 'world')
        self.assertEqual(converted['quaternion_xyzw'], query['quaternion_xyzw'])
        self.assertEqual(converted['current_bunker_pose'], query['current_bunker_pose'])
        self.assertEqual(query, before)

    def test_nondefault_height_and_transform_roundtrip_ignore_spawn_xy_yaw(self):
        from sim_active_perception.frame_bridge import FrameBridge

        values = calibration(.713)
        bridge = FrameBridge(values, transform(.15, 0., .122))
        np.testing.assert_allclose(bridge.T_reference_map, transform(z=-.713), atol=1e-15)
        np.testing.assert_allclose(bridge.T_map_reference, transform(z=.713), atol=1e-15)
        pose = transform(.73, -.29, .08, -.8)
        np.testing.assert_allclose(bridge.T_map_reference @ bridge.T_reference_map @ pose, pose,
                                   atol=1e-15)
        metadata = bridge.as_dict()
        self.assertEqual(metadata['ground_reference_height_m'], .713)
        for key, value in values.items():
            self.assertEqual(metadata[key], value)
        np.testing.assert_allclose(metadata['T_reference_map'], bridge.T_reference_map)
        np.testing.assert_allclose(metadata['T_map_reference'], bridge.T_map_reference)
        values['T_map_ground_odom'][2][3] = 9.
        metadata['T_bunker_aubo'][2][3] = 8.
        self.assertEqual(bridge.as_dict()['T_map_ground_odom'][2][3], .713)
        self.assertEqual(bridge.as_dict()['T_bunker_aubo'][2][3], .122)

    def test_rotated_candidates_preserve_aubo_local_tcp_flange_and_all_evidence(self):
        from sim_active_perception.frame_bridge import FrameBridge

        bridge = FrameBridge(calibration(), transform(.15, 0., .122))
        map_tcp = transform(.42, -.17, .08, .54)
        reference_tcp = transform(z=-.36) @ map_tcp
        flange_tcp = np.array([[0., 1., 0., 0.], [0., 0., 1., 0.],
                               [1., 0., 0., .2], [0., 0., 0., 1.]])
        items = []
        for number, yaw in enumerate((-.8, .35, 2.1)):
            bunker = transform(.1 * number, -.2 * number, 0., yaw)
            aubo = bunker @ transform(.15, 0., .122)
            flange = reference_tcp @ np.linalg.inv(flange_tcp)
            items.append(dict(candidate_id=str(number), bunker_x=bunker[0, 3],
                              bunker_y=bunker[1, 3], bunker_yaw=yaw,
                              T_world_bunker=bunker.tolist(), T_world_aubo=aubo.tolist(),
                              T_world_flange=flange.tolist(),
                              T_aubo_flange=(np.linalg.inv(aubo) @ flange).tolist(),
                              rm4d_reachable=True, ik_valid=True, collision_free=True,
                              footprint_collision=False, valid=True, joint_margin_rad=.25,
                              fk_position_residual_m=.0001, fk_orientation_residual_rad=.0002,
                              rejection_reason=None, transform_roundtrip_error=1e-16,
                              joint_configuration=[.1, -.2, .3, -.4, .5, -.6],
                              ik_attempts=7, ik_seed=20260907, final_score=.7, travel_cost=.9))
        # The frozen planner shares ranked records with evaluated_candidates.
        raw = dict(frame_id='world', grasp_id='exact', schema_version=1, status='ok',
                   summary=dict(evaluated=3, valid=3), provenance=dict(source='frozen'),
                   evaluated_candidates=items, candidates=[items[2], items[0]])
        before = copy.deepcopy(raw)
        converted = bridge.result_to_map(raw)

        self.assertEqual(raw, before)
        self.assertEqual(converted['frame_id'], 'map')
        for key in ('grasp_id', 'schema_version', 'status', 'summary', 'provenance'):
            self.assertEqual(converted[key], before[key])
        for collection in ('candidates', 'evaluated_candidates'):
            for original, adapted in zip(before[collection], converted[collection]):
                for key, value in original.items():
                    if not key.startswith('T_world_'):
                        self.assertEqual(adapted[key], value)
                for name in ('bunker', 'aubo', 'flange'):
                    self.assertNotIn(f'T_world_{name}', adapted)
                    np.testing.assert_allclose(adapted[f'T_map_{name}'],
                                               transform(z=.36) @ original[f'T_world_{name}'],
                                               atol=1e-15)
                map_aubo = np.asarray(adapted['T_map_bunker']) @ transform(.15, 0., .122)
                np.testing.assert_allclose(map_aubo, adapted['T_map_aubo'], atol=1e-15)
                np.testing.assert_allclose(np.linalg.inv(map_aubo) @ map_tcp,
                                           np.linalg.inv(original['T_world_aubo']) @ reference_tcp,
                                           atol=1e-15)
                np.testing.assert_allclose(np.linalg.inv(map_aubo) @ adapted['T_map_flange'],
                                           original['T_aubo_flange'], atol=1e-15)
                np.testing.assert_allclose(np.asarray(adapted['T_map_flange']) @ flange_tcp,
                                           map_tcp, atol=1e-15)
        converted['evaluated_candidates'][0]['joint_configuration'][0] = 99.
        self.assertEqual(raw, before)

    def test_rejects_non_se3_calibration_matrices(self):
        from sim_active_perception.frame_bridge import FrameBridge

        scaled, reflected, bad_bottom, nonfinite = [np.eye(4) for _ in range(4)]
        scaled[0, 0] = 1.1
        reflected[0, 0] = -1.
        bad_bottom[3, 0] = .1
        nonfinite[2, 3] = np.nan
        for key in calibration():
            for invalid in (np.eye(3), scaled, reflected, bad_bottom, nonfinite):
                values = calibration()
                values[key] = invalid.tolist()
                with self.subTest(key=key, invalid=invalid):
                    with self.assertRaisesRegex(ValueError, key):
                        FrameBridge(values, transform(.15, 0., .122))

    def test_rejects_tilted_or_inverted_ground_reference_and_nonzero_local_z(self):
        from sim_active_perception.frame_bridge import FrameBridge

        for key in ('T_map_ground_odom', 'T_ground_odom_bunker'):
            for angle in (.01, math.pi):
                tilted = np.eye(4)
                tilted[1:3, 1:3] = [[math.cos(angle), -math.sin(angle)],
                                     [math.sin(angle), math.cos(angle)]]
                values = calibration()
                values[key] = tilted.tolist()
                with self.subTest(key=key, angle=angle):
                    with self.assertRaisesRegex(ValueError, 'horizontal'):
                        FrameBridge(values, transform(.15, 0., .122))
        values = calibration()
        values['T_ground_odom_bunker'][2][3] = .04
        with self.assertRaisesRegex(ValueError, 'T_ground_odom_bunker.*z=0'):
            FrameBridge(values, transform(.15, 0., .122))

    def test_rejects_missing_calibration_and_mount_disagreement(self):
        from sim_active_perception.frame_bridge import FrameBridge

        for key in calibration():
            values = calibration()
            del values[key]
            with self.subTest(missing=key):
                with self.assertRaisesRegex(ValueError, key):
                    FrameBridge(values, transform(.15, 0., .122))
        for mismatched in (transform(.15, 0., .123), transform(.151, 0., .122),
                           transform(.15, 0., .122, .001)):
            values = calibration()
            values['T_bunker_aubo'] = mismatched.tolist()
            with self.subTest(mismatched=mismatched):
                with self.assertRaisesRegex(ValueError, 'mount.*frozen'):
                    FrameBridge(values, transform(.15, 0., .122))
        with self.assertRaisesRegex(ValueError, 'frozen_T_bunker_aubo'):
            FrameBridge(calibration(), np.eye(3))

    def test_rejects_wrong_query_and_result_frames(self):
        from sim_active_perception.frame_bridge import FrameBridge

        bridge = FrameBridge(calibration(), transform(.15, 0., .122))
        with self.assertRaisesRegex(ValueError, 'map'):
            bridge.query_to_reference(dict(frame_id='odom', position_xyz=[0., 0., .08]))
        with self.assertRaisesRegex(ValueError, 'world'):
            bridge.result_to_map(dict(frame_id='map', candidates=[], evaluated_candidates=[]))


class WorkerFrameBridgeTests(unittest.TestCase):
    def test_initialize_translates_query_and_saves_exact_grasp_baseline_and_calibration(self):
        self.check_initialize(task_asset=False)

    def test_task_asset_uses_independent_validator_and_is_not_labelled_baseline(self):
        self.check_initialize(task_asset=True)

    def check_initialize(self, task_asset):
        values = calibration()
        regularized = [0., math.sin(5e-7), 0., math.cos(5e-7)]
        calls = []
        baseline = result([candidate(x=.011, y=.019, T_world_bunker=transform(.011, .019).tolist(),
                                     T_world_aubo=transform(.161, .019, .122).tolist(),
                                     T_world_flange=transform(.42, -.17, -.48).tolist(),
                                     T_aubo_flange=transform(.259, -.189, -.602).tolist())])
        baseline['grasp_id'] = 'exact'
        baseline['frame_id'] = 'world'

        # These two external boundaries require no installed SIM or live RM4D map.
        @dataclass
        class PoseValues:
            position: tuple
            orientation: tuple

        geometry = ModuleType('rm4d_sim_integration.geometry')
        geometry.PoseValues = PoseValues

        def build_request(frame, pose, current, grasp_id):
            calls.append(dict(frame=frame, pose=pose, current=current, grasp_id=grasp_id))
            return dict(frame_id=frame, grasp_id=grasp_id, position_xyz=list(pose.position),
                        quaternion_xyzw=regularized.copy(),
                        current_bunker_pose=dict(zip(('x', 'y', 'yaw'), current)))

        geometry.build_rm4d_request = build_request

        class FrozenAPI:
            def plan(self, query, top_k):
                calls.append(copy.deepcopy(query))
                returned = copy.deepcopy(baseline)
                returned['frame_id'] = query['frame_id']
                return returned

        @contextmanager
        def open_api(*args):
            yield FrozenAPI()

        task_calls = []

        @contextmanager
        def open_task_api(*args):
            task_calls.append(args)
            yield FrozenAPI()

        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            config_path = directory / 'frozen.json'
            config_path.write_text(json.dumps(dict(transforms=dict(T_bunker_aubo=values['T_bunker_aubo']))))
            request = dict(grasp=dict(grasp_id='exact', frame_id='map',
                                      position_xyz=[.42, -.17, .08], quaternion_xyzw=[0., 0., 0., 1.]),
                           sim_root=str(directory), rm4d_root=str(directory),
                           rm4d_config=str(config_path), rm4d_map=str(directory / 'map.npy'),
                           output_dir=str(directory / 'initial'), current_bunker_pose=[4.5, -1.2, .3],
                           frame_calibration=values)
            if task_asset:
                request['rm4d_task_asset'] = str(directory / 'task_asset')
            before = copy.deepcopy(request)
            with patch.object(worker, 'open_frozen_rm4d_api', open_api), patch.dict(
                    sys.modules, {'rm4d_sim_integration.geometry': geometry}), patch.object(
                        sys, 'path', sys.path.copy()), patch.object(
                            worker, 'open_task_rm4d_api', open_task_api, create=True):
                response = worker.initialize(request)
            initial = json.loads(Path(response['initial_file']).read_text())

            self.assertAlmostEqual(initial['query_request']['position_xyz'][2], -.28)
            self.assertEqual(initial['query_request']['frame_id'], 'world')
            self.assertEqual(initial['query_request']['quaternion_xyzw'], regularized)
            self.assertEqual(initial['query_request']['current_bunker_pose'], dict(x=4.5, y=-1.2, yaw=.3))
            self.assertEqual(calls[0]['frame'], 'map')
            self.assertEqual(list(calls[0]['pose'].position), request['grasp']['position_xyz'])
            self.assertEqual(list(calls[0]['pose'].orientation), request['grasp']['quaternion_xyzw'])
            self.assertEqual(calls[1], initial['query_request'])
            self.assertEqual(initial['grasp'], request['grasp'])
            self.assertEqual(request, before)
            if task_asset:
                self.assertEqual(task_calls, [(request['rm4d_root'], request['rm4d_config'],
                                               request['rm4d_task_asset'], values)])
                self.assertIsNone(initial['baseline_result'])
                self.assertEqual(initial['task_domain_result'], baseline)
                self.assertEqual(initial['rm4d_task_asset'], request['rm4d_task_asset'])
            else:
                self.assertEqual(task_calls, [])
                self.assertEqual(initial['baseline_result'], baseline)
            self.assertEqual(initial['result']['frame_id'], 'map')
            self.assertAlmostEqual(initial['result']['evaluated_candidates'][0]['T_map_bunker'][2][3], .36)
            self.assertEqual(initial['frame_calibration']['ground_reference_height_m'], .36)
            for key, value in values.items():
                self.assertEqual(initial['frame_calibration'][key], value)
            self.assertEqual(initial['config']['ground_z_m'], 0.)
            self.assertEqual(response['candidate_count'], 1)
            self.assertEqual(response.get('rm4d_valid'), 1)
            if task_asset:
                self.assertNotIn('baseline_valid', response)
            else:
                self.assertEqual(response.get('baseline_valid'), 1)

    def test_initialize_requires_calibration_before_external_work(self):
        request = dict(grasp=dict(grasp_id='exact', frame_id='map',
                                  position_xyz=[.42, -.17, .08], quaternion_xyzw=[0., 0., 0., 1.]))
        with self.assertRaisesRegex(ValueError, 'frame_calibration'):
            worker.initialize(request)

    def test_initialize_reads_frozen_mount_and_rejects_disagreement(self):
        with tempfile.TemporaryDirectory() as tmp:
            config_path = Path(tmp) / 'frozen.json'
            config_path.write_text(json.dumps(dict(transforms=dict(T_bunker_aubo=transform(.15, 0., .124).tolist()))))
            request = dict(grasp=dict(grasp_id='exact', frame_id='map',
                                      position_xyz=[.42, -.17, .08], quaternion_xyzw=[0., 0., 0., 1.]),
                           rm4d_config=str(config_path), frame_calibration=calibration())
            with self.assertRaisesRegex(ValueError, 'mount.*frozen'):
                worker.initialize(request)


if __name__ == '__main__':
    unittest.main()
