import json
import os
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

from sim_active_perception.task_map import (
    _copy_positive_overlap,
    build_task_map,
    derive_task_floor_z,
    load_frame_calibration,
    open_task_rm4d_api,
)


FROZEN_ROOT = Path(os.environ.get('RM4D_ROOT', '/tmp/rm4d-aubo-baseline-v1.n9oGee/repo'))
FROZEN_CONFIG = FROZEN_ROOT / 'configs/mr4_offline_base_placement.json'
FROZEN_MAP = Path(
    '/media/lu/P450_PAPER/RM4D_AUBO/runs/formal-10m/data/'
    'rm4d_aubo_i5_joint_42/10000000/rmap.npy'
)


def calibration():
    identity = np.eye(4)
    ground = identity.copy()
    ground[2, 3] = 0.36
    mount = identity.copy()
    mount[:3, 3] = [0.15, 0.0, 0.122]
    return {
        'T_map_ground_odom': ground.tolist(),
        'T_ground_odom_bunker': identity.tolist(),
        'T_bunker_aubo': mount.tolist(),
    }


class TaskMapPureTests(unittest.TestCase):
    def test_floor_is_derived_from_independent_inputs(self):
        self.assertAlmostEqual(
            derive_task_floor_z(
                map_ground_z=0.0,
                ground_reference_height_m=0.36,
                mount_height_m=0.122,
                robot_base_z_m=0.01,
            ),
            -0.472,
        )

    def test_positive_overlap_is_copied_without_reindexing(self):
        source = np.arange(2 * 2 * 2 * 2).reshape(2, 2, 2, 2)
        destination = np.zeros((3, 2, 2, 2), dtype=source.dtype)
        _copy_positive_overlap(
            source,
            destination,
            source_z_limits=(0.0, 0.1),
            destination_z_limits=(-0.05, 0.1),
            voxel_res=0.05,
        )
        np.testing.assert_array_equal(destination[1:], source)
        self.assertFalse(destination[0].any())

    def test_calibration_loader_accepts_saved_request_or_initial(self):
        value = calibration()
        with tempfile.TemporaryDirectory() as directory:
            direct = Path(directory) / 'request.json'
            nested = Path(directory) / 'initial.json'
            direct.write_text(json.dumps({'frame_calibration': value}))
            nested.write_text(json.dumps({'frame_calibration': dict(value, ground_reference_height_m=0.36)}))
            self.assertEqual(load_frame_calibration(direct), value)
            self.assertEqual(load_frame_calibration(nested), value)

    def test_builder_rejects_output_that_would_overwrite_source_map(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'rmap.npy'
            source.write_bytes(b'preserve me')
            config_path = root / 'config.json'
            config_path.write_text('{}')
            calibration_path = root / 'calibration.json'
            calibration_path.write_text('{}')
            with self.assertRaisesRegex(ValueError, 'overwrite source map'):
                build_task_map(root / 'missing-rm4d', config_path, source,
                               calibration_path, root, samples=0)
            self.assertEqual(source.read_bytes(), b'preserve me')


@unittest.skipUnless(FROZEN_CONFIG.is_file() and FROZEN_MAP.is_file(), 'local frozen RM4D unavailable')
class TaskMapFrozenIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.prior_rm4d_modules = {
            name: module for name, module in sys.modules.items()
            if name == 'rm4d' or name.startswith('rm4d.')
        }
        cls.addClassCleanup(cls.restore_rm4d_modules)
        cls.directory = tempfile.TemporaryDirectory()
        cls.calibration_path = Path(cls.directory.name) / 'calibration.json'
        cls.calibration_path.write_text(json.dumps({'frame_calibration': calibration()}))
        cls.asset_dir = Path(cls.directory.name) / 'asset'
        build_task_map(FROZEN_ROOT, FROZEN_CONFIG, FROZEN_MAP,
                       cls.calibration_path, cls.asset_dir, samples=8, seed=42)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    @classmethod
    def restore_rm4d_modules(cls):
        for name in list(sys.modules):
            if name == 'rm4d' or name.startswith('rm4d.'):
                sys.modules.pop(name)
        sys.modules.update(cls.prior_rm4d_modules)

    def test_frozen_map_has_expected_positive_slice_count(self):
        saved = np.load(FROZEN_MAP, allow_pickle=True).item()
        self.assertEqual(tuple(saved['map'].shape), (26, 36, 46, 46))
        self.assertEqual(tuple(saved['z_limits']), (0.0, 1.3))

    def test_built_asset_preserves_positive_map_bit_for_bit(self):
        source = np.load(FROZEN_MAP, allow_pickle=True).item()['map']
        task = np.load(self.asset_dir / 'rmap.npy', allow_pickle=True).item()['map']
        np.testing.assert_array_equal(task[5:], source)
        self.assertGreater(np.count_nonzero(task[:5]), 0)
        metadata = json.loads((self.asset_dir / 'metadata.json').read_text())
        self.assertEqual(metadata['negative_sampling']['negative_fk_samples_stored'], 3)

    def test_runtime_owns_calibrated_plane_and_retains_collision_gates(self):
        with open_task_rm4d_api(FROZEN_ROOT, FROZEN_CONFIG, self.asset_dir,
                                calibration()) as api:
            validator = api.planner.ik_validator
            robot, simulator = validator.robot, validator.simulator
            plane_position, _ = simulator.bullet_client.getBasePositionAndOrientation(
                simulator.plane_id)
            self.assertAlmostEqual(plane_position[2], -0.472)
            robot.reset_joint_pos(robot.home_conf)
            self.assertFalse(robot.in_collision())

            rng = np.random.default_rng(17)
            saw_self_collision = False
            saw_floor_collision = False
            for _ in range(512):
                joints = robot.get_random_joint_config(prevent_collisions=False, rng=rng)
                robot.reset_joint_pos(joints)
                saw_self_collision |= robot.in_self_collision()
                saw_floor_collision |= robot.in_collision_with_plane()
                if saw_self_collision and saw_floor_collision:
                    break
            self.assertTrue(saw_self_collision)
            self.assertTrue(saw_floor_collision)

            accepted = robot.get_random_joint_config(prevent_collisions=True, rng=rng)
            robot.reset_joint_pos(accepted)
            self.assertFalse(robot.in_self_collision())
            self.assertFalse(robot.in_collision_with_plane())

    def test_runtime_rejects_live_floor_calibration_mismatch(self):
        changed = calibration()
        changed['T_map_ground_odom'][2][3] += 0.001
        with self.assertRaisesRegex(ValueError, 'disagrees with task asset floor metadata'):
            with open_task_rm4d_api(FROZEN_ROOT, FROZEN_CONFIG, self.asset_dir, changed):
                self.fail('mismatched calibration opened task API')

    def test_runtime_rejects_missing_workspace_metadata(self):
        metadata_path = self.asset_dir / 'metadata.json'
        original = metadata_path.read_text()
        metadata = json.loads(original)
        metadata.pop('workspace')
        metadata_path.write_text(json.dumps(metadata))
        try:
            with self.assertRaisesRegex(ValueError, 'geometry disagrees with metadata'):
                with open_task_rm4d_api(FROZEN_ROOT, FROZEN_CONFIG,
                                        self.asset_dir, calibration()):
                    self.fail('asset without workspace metadata opened task API')
        finally:
            metadata_path.write_text(original)


if __name__ == '__main__':
    unittest.main()
