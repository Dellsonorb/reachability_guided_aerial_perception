"""Build and open the independently owned A5 task-domain RM4D asset."""

from __future__ import annotations

from contextlib import contextmanager
from copy import deepcopy
import importlib
import json
from pathlib import Path
import sys

import numpy as np

from .frame_bridge import FrameBridge


TASK_XY_LIMITS = (-1.15, 1.15)
TASK_Z_LIMITS = (-0.25, 1.3)
TASK_VOXEL_RES_M = 0.05
TASK_THETA_BINS = 36
ROBOT_BASE_Z_M = 0.01


def derive_task_floor_z(map_ground_z, ground_reference_height_m,
                        mount_height_m, robot_base_z_m):
    """Express the physical map ground in the standalone RM4D robot world."""
    values = np.asarray((map_ground_z, ground_reference_height_m,
                         mount_height_m, robot_base_z_m), dtype=float)
    if not np.isfinite(values).all():
        raise ValueError('task floor inputs must be finite')
    return float(values[0] - values[1] - values[2] + values[3])


def load_frame_calibration(path):
    """Read an A5 request/initial file and return its three measured transforms."""
    try:
        payload = json.loads(Path(path).read_text(encoding='utf-8'))
        value = payload['frame_calibration']
        return {name: value[name] for name in (
            'T_map_ground_odom', 'T_ground_odom_bunker', 'T_bunker_aubo')}
    except (OSError, TypeError, KeyError, json.JSONDecodeError) as error:
        raise ValueError('calibration JSON requires frame_calibration transforms') from error


def _copy_positive_overlap(source, destination, *, source_z_limits,
                           destination_z_limits, voxel_res):
    """Copy identical source bins to their coordinate-equivalent destination bins."""
    offset_float = (float(source_z_limits[0]) - float(destination_z_limits[0])) / float(voxel_res)
    offset = int(round(offset_float))
    if not np.isclose(offset_float, offset, atol=1e-12, rtol=0):
        raise ValueError('positive overlap is not aligned to voxel boundaries')
    stop = offset + source.shape[0]
    if source.shape[1:] != destination.shape[1:] or offset < 0 or stop > destination.shape[0]:
        raise ValueError('source map is incompatible with task workspace')
    expected_stop = float(destination_z_limits[0]) + stop * float(voxel_res)
    if not np.isclose(expected_stop, float(source_z_limits[1]), atol=1e-12, rtol=0):
        raise ValueError('positive overlap has inconsistent z limits')
    destination[offset:stop] = source
    return offset


def _import_rm4d(rm4d_root):
    root = Path(rm4d_root).expanduser().resolve()
    if not root.is_dir():
        raise ValueError('rm4d_root must be an existing directory')
    expected = (root / 'rm4d' / '__init__.py').resolve()
    cached = sys.modules.get('rm4d')
    if cached is not None and Path(cached.__file__).resolve() != expected:
        raise ValueError('rm4d module is not from the requested rm4d_root')
    old_path = list(sys.path)
    sys.path.insert(0, str(root))
    try:
        module = importlib.import_module('rm4d')
    finally:
        sys.path[:] = old_path
    if Path(module.__file__).resolve() != expected:
        raise ValueError('rm4d module is not from the requested rm4d_root')
    return module


def _rm4d_classes(module):
    """Resolve classes from their frozen modules without assuming root exports."""
    return (
        importlib.import_module(f'{module.__name__}.robots').Simulator,
        importlib.import_module(f'{module.__name__}.robots').AuboI5,
        importlib.import_module(f'{module.__name__}.base_placement_ik').AuboIkValidator,
        importlib.import_module(f'{module.__name__}.base_placement').BasePlacementPlanner,
    )


def _calibrated_floor(frame_calibration, frozen_mount, configured_robot_base_z):
    bridge = FrameBridge(frame_calibration, frozen_mount)
    mount_height = float(np.asarray(frozen_mount, dtype=float)[2, 3])
    configured_robot_base_z = float(configured_robot_base_z)
    if not np.isclose(configured_robot_base_z, ROBOT_BASE_Z_M, atol=1e-12, rtol=0):
        raise ValueError('frozen RM4D robot base height must remain unchanged')
    inputs = {
        'map_ground_z': 0.0,
        'ground_reference_height_m': bridge.ground_reference_height_m,
        'mount_height_m': mount_height,
        'robot_base_z_m': configured_robot_base_z,
    }
    return derive_task_floor_z(**inputs), inputs


def _move_plane(simulator, floor_z):
    position, orientation = simulator.bullet_client.getBasePositionAndOrientation(simulator.plane_id)
    simulator.bullet_client.resetBasePositionAndOrientation(
        simulator.plane_id, [position[0], position[1], floor_z], orientation)


def build_task_map(rm4d_root, config_path, map_path, calibration_json,
                   output_dir, samples=100000, seed=42):
    """Create the one-file task map by copying positive bins and sampling negative bins."""
    if int(samples) != samples or int(samples) < 0:
        raise ValueError('samples must be a non-negative integer')
    samples = int(samples)
    config_path, map_path, output = Path(config_path), Path(map_path), Path(output_dir)
    if (output / 'rmap.npy').resolve() == map_path.resolve():
        raise ValueError('output rmap.npy would overwrite source map')
    if not config_path.is_file() or not map_path.is_file():
        raise ValueError('config_path and map_path must be existing files')
    config = json.loads(config_path.read_text(encoding='utf-8'))
    calibration = load_frame_calibration(calibration_json)
    floor_z, floor_inputs = _calibrated_floor(
        calibration, config['transforms']['T_bunker_aubo'],
        config['transforms']['rm4d_robot_base_z_m'])
    rm4d = _import_rm4d(rm4d_root)
    Simulator, AuboI5, _, _ = _rm4d_classes(rm4d)
    source = rm4d.ReachabilityMap4D.from_file(str(map_path))
    if (tuple(source.xy_limits) != TASK_XY_LIMITS
            or tuple(source.z_limits) != (0.0, TASK_Z_LIMITS[1])
            or not np.isclose(source.voxel_res, TASK_VOXEL_RES_M)
            or source.n_bins_theta != TASK_THETA_BINS):
        raise ValueError('frozen source map geometry disagrees with A5 task workspace')
    task = rm4d.ReachabilityMap4D(
        xy_limits=list(TASK_XY_LIMITS), z_limits=list(TASK_Z_LIMITS),
        voxel_res=TASK_VOXEL_RES_M, n_bins_theta=TASK_THETA_BINS)
    positive_offset = _copy_positive_overlap(
        source.map, task.map, source_z_limits=source.z_limits,
        destination_z_limits=task.z_limits, voxel_res=task.voxel_res)

    simulator = Simulator(with_gui=False)
    try:
        _move_plane(simulator, floor_z)
        robot = AuboI5(simulator, base_pos=[0, 0, ROBOT_BASE_Z_M])
        rng = np.random.default_rng(seed=int(seed))
        stored_negative = 0
        for _ in range(samples):
            joints = robot.get_random_joint_config(prevent_collisions=True, rng=rng)
            position, quaternion = robot.forward_kinematics(joints)
            transform = simulator.pos_quat_to_tf(position, quaternion)
            if not TASK_Z_LIMITS[0] <= transform[2, 3] < 0.0:
                continue
            try:
                indices = task.get_indices_for_ee_pose(transform)
            except IndexError:
                continue
            if indices[0] >= positive_offset:
                continue
            task.mark_reachable(indices)
            stored_negative += 1
    finally:
        simulator.disconnect()

    output.mkdir(parents=True, exist_ok=True)
    map_output = output / 'rmap.npy'
    task.to_file(str(map_output))
    metadata = {
        'schema_version': 1,
        'asset_kind': 'a5_task_domain_rm4d',
        'workspace': {
            'xy_limits_m': list(TASK_XY_LIMITS), 'z_limits_m': list(TASK_Z_LIMITS),
            'voxel_resolution_m': TASK_VOXEL_RES_M, 'theta_bins': TASK_THETA_BINS,
        },
        'floor_calibration': {
            'inputs': floor_inputs,
            'task_floor_z': floor_z,
            'validated_T_bunker_aubo': config['transforms']['T_bunker_aubo'],
            'assumption': 'horizontal map ground and BUNKER mount, validated by FrameBridge',
        },
        'positive_overlap': {
            'source_map': str(map_path.resolve()),
            'source_samples': 10000000, 'copied_slices': int(source.map.shape[0]),
            'copied_occupied_bins': int(np.count_nonzero(source.map)),
        },
        'negative_sampling': {
            'samples': samples, 'seed': int(seed), 'negative_fk_samples_stored': stored_negative,
            'proposal': 'uniform_joint_space',
            'rejection': 'AuboI5.get_random_joint_config(prevent_collisions=True)',
        },
        'collision_world': {
            'simulator': 'unchanged rm4d Simulator', 'robot': 'unchanged rm4d AuboI5',
            'robot_base_z_m': ROBOT_BASE_Z_M, 'plane_relocated_only': True,
            'self_collision_enabled': True, 'floor_collision_enabled': True,
        },
    }
    (output / 'metadata.json').write_text(
        json.dumps(metadata, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    return metadata


@contextmanager
def open_task_rm4d_api(rm4d_root, config_path, asset_dir, frame_calibration):
    """Open the task map with its own calibrated simulator and unchanged validators."""
    config_path, asset_dir = Path(config_path), Path(asset_dir)
    map_path, metadata_path = asset_dir / 'rmap.npy', asset_dir / 'metadata.json'
    if not config_path.is_file() or not map_path.is_file() or not metadata_path.is_file():
        raise ValueError('task config and asset files must exist')
    config = json.loads(config_path.read_text(encoding='utf-8'))
    metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
    floor_z, floor_inputs = _calibrated_floor(
        frame_calibration, config['transforms']['T_bunker_aubo'],
        config['transforms']['rm4d_robot_base_z_m'])
    recorded = metadata.get('floor_calibration', {})
    recorded_inputs = recorded.get('inputs', {})
    if (set(recorded_inputs) != set(floor_inputs)
            or not np.allclose([recorded_inputs[name] for name in floor_inputs],
                               list(floor_inputs.values()), atol=1e-8, rtol=0)
            or not np.allclose(recorded.get('validated_T_bunker_aubo', []),
                               config['transforms']['T_bunker_aubo'], atol=1e-8, rtol=0)
            or not np.isclose(recorded.get('task_floor_z', np.nan), floor_z,
                              atol=1e-12, rtol=0)):
        raise ValueError('live frame calibration disagrees with task asset floor metadata')

    rm4d = _import_rm4d(rm4d_root)
    Simulator, AuboI5, AuboIkValidator, BasePlacementPlanner = _rm4d_classes(rm4d)
    simulator = Simulator(with_gui=False)
    try:
        _move_plane(simulator, floor_z)
        robot = AuboI5(simulator, base_pos=[0, 0, ROBOT_BASE_Z_M])
        reachability_map = rm4d.ReachabilityMap4D.from_file(str(map_path))
        workspace = metadata.get('workspace', {})
        workspace_keys = {'xy_limits_m', 'z_limits_m', 'voxel_resolution_m', 'theta_bins'}
        if (set(workspace) != workspace_keys
                or not np.allclose(reachability_map.xy_limits, workspace['xy_limits_m'])
                or not np.allclose(reachability_map.z_limits, workspace['z_limits_m'])
                or not np.isclose(reachability_map.voxel_res,
                                  workspace['voxel_resolution_m'])
                or reachability_map.n_bins_theta != workspace['theta_bins']):
            raise ValueError('task RM4D map geometry disagrees with metadata')
        runtime_config = deepcopy(config)
        runtime_config['provenance'] = {
            'asset_kind': metadata['asset_kind'],
            'map_path': str(map_path.resolve()),
            'metadata_path': str(metadata_path.resolve()),
        }
        validator = AuboIkValidator(
            runtime_config['ik_validation'], simulator=simulator, robot=robot)
        planner = BasePlacementPlanner(
            runtime_config, reachability_map, validator)
        api = rm4d.BasePlacementAPI(planner, owned_validator=validator)
        try:
            yield api
        finally:
            api.close()
    finally:
        simulator.disconnect()
