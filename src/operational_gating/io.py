"""Small existing-worker file boundary; raw A2 artifacts are never rewritten."""

from dataclasses import asdict, replace
import json
from pathlib import Path

import numpy as np

from .association import associate_returns, geometry_allowance, validate_reference
from .core import PerceivedTarget, derive_operational_evidence


def record_initial_context(initial_path, request):
    revision = request.get('operational_gating', 'v1')
    if revision == 'v1':
        return
    if revision != 'v1.1':
        raise ValueError('unsupported operational gating revision')
    target = PerceivedTarget(**request['perceived_target'])
    if target.geometry_allowance_m != 0:
        raise ValueError('runtime geometry allowance is derived from the reference, not supplied')
    path = Path(initial_path)
    initial = json.loads(path.read_text())
    initial.update(operational_gating=revision, perceived_target=asdict(target),
                   target_reference_file=request.get('target_reference_file'),
                   target_reference_status=request.get('target_reference_status', 'UNAVAILABLE'))
    path.write_text(json.dumps(initial, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def _load_reference(path, target):
    with np.load(path, allow_pickle=False) as data:
        reference = {name: data[name].copy() for name in data.files}
    for name in ('frame_id', 'status', 'stamp_s'):
        reference[name] = reference[name].item()
    validate_reference(reference)
    if (np.asarray(reference['target_pose_map_xyzyaw']).shape != (4,) or
            np.asarray(reference['target_size_xyz']).shape != (3,) or
            not np.allclose(reference['target_pose_map_xyzyaw'],
                            [*target.center_xyz, target.yaw_rad], rtol=0, atol=1e-9) or
            not np.allclose(reference['target_size_xyz'], target.size_xyz, rtol=0, atol=1e-9)):
        raise ValueError('RGB-D reference must match the same initial perceived handoff object')
    return reference


def build_operational_context(initial, grid, observations, config):
    revision = initial.get('operational_gating', 'v1')
    if revision == 'v1':
        return None, None
    if revision != 'v1.1':
        raise ValueError('unsupported operational gating revision')
    target = PerceivedTarget(**initial['perceived_target'])
    if target.geometry_allowance_m != 0:
        raise ValueError('initial target must not supply a tuned geometry allowance')
    observations = tuple(observations)
    path = initial.get('target_reference_file')
    reference = _load_reference(path, target) if path else None
    if reference is not None:
        target = replace(target, geometry_allowance_m=geometry_allowance(reference, target))
        if any(o.stamp_s < reference['stamp_s'] for o in observations):
            raise ValueError('MID360 observation cannot precede its initial reference')
    labels = []
    for observation in observations:
        points = np.asarray(observation.points_xyz)
        finite = np.all(np.isfinite(points), axis=1)
        mapped = np.full(points.shape, np.nan)
        t = observation.T_map_sensor
        mapped[finite] = points[finite] @ t[:3, :3].T + t[:3, 3]
        labels.append(associate_returns(mapped, target, reference))
    view = derive_operational_evidence(grid, observations, target, labels=labels, config=config)
    metadata = dict(
        operational_semantics='object-aware-v1.1',
        association_status='AVAILABLE' if reference is not None else 'UNAVAILABLE',
        association_rule='interior_red_mask_AND_registered_depth_AND_expanded_known_object_geometry',
        reference_scope='static_initial_air_phase_only', reference_file=str(path) if path else None,
        target=asdict(target), geometry_allowance_semantics='declared_sensor_budget_not_calibrated_accuracy',
        raw_a2_unchanged=True, ground_semantics='actual_ground_votes_with_environment_or_ambiguous_priority',
        observation_windows=len(observations))
    return view, metadata


def save_operational(directory, view, metadata):
    if view is None:
        return
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    names = ('environment_occupied_votes', 'ambiguous_occupied_votes', 'target_occupied_votes', 'ground_votes')
    np.savez_compressed(directory / 'operational_evidence.npz', **{n: getattr(view, n) for n in names},
                        frame_id='map', origin_xy=view.grid.origin_xy, resolution_m=view.grid.resolution_m,
                        target_vertices_xy=view.target.xy_vertices)
    summary = dict(metadata, votes={n: int(getattr(view, n).sum()) for n in names})
    (directory / 'operational_summary.json').write_text(
        json.dumps(summary, indent=2, allow_nan=False) + '\n', encoding='utf-8')
