"""Positive RGB-D/geometry association for a static, known manipuland.

The metric budget is declared sensor resolution/noise plus pixel footprint, not
an empirically calibrated probability or a tolerance fitted to pilot outcomes.
"""

import numpy as np

from .core import OccupiedClass, PerceivedTarget


DECLARED_LIDAR_RANGE_ALLOWANCE_M = 3 * .01
LIDAR_RANGE_RESOLUTION_M = .002
DEPTH_QUANTIZATION_M = .001


def validate_reference(reference):
    if reference.get('frame_id') != 'map' or reference.get('status') != 'AVAILABLE':
        raise ValueError('available map-frame RGB-D reference required')
    if not np.isfinite(reference['stamp_s']):
        raise ValueError('reference stamp must be finite')
    mask = np.asarray(reference['mask'])
    depth = np.asarray(reference['registered_depth_m'])
    if mask.ndim != 2 or mask.dtype != bool or depth.shape != mask.shape:
        raise ValueError('reference mask/depth must be aligned 2D images')
    for name in ('color_K', 'depth_K'):
        k = np.asarray(reference[name], dtype=float)
        if (k.shape != (3, 3) or not np.all(np.isfinite(k)) or
                k[0, 0] <= 0 or k[1, 1] <= 0 or
                not np.allclose(k[2], [0, 0, 1]) or k[0, 1] != 0 or k[1, 0] != 0):
            raise ValueError('reference intrinsics must be finite pinhole matrices')
    t = np.asarray(reference['T_map_color'], dtype=float)
    if (t.shape != (4, 4) or not np.all(np.isfinite(t)) or
            not np.allclose(t[3], [0, 0, 0, 1]) or
            not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-6) or
            not np.isclose(np.linalg.det(t[:3, :3]), 1, atol=1e-6)):
        raise ValueError('reference map-camera transform must be rigid')


def metric_allowance(reference, optical_depth_m):
    """Three declared LiDAR sigma + resolutions + two half-pixel diagonals."""
    c, d = np.asarray(reference['color_K']), np.asarray(reference['depth_K'])
    pixel = .5 * np.asarray(optical_depth_m) * (
        np.hypot(1 / c[0, 0], 1 / c[1, 1]) + np.hypot(1 / d[0, 0], 1 / d[1, 1]))
    return DECLARED_LIDAR_RANGE_ALLOWANCE_M + LIDAR_RANGE_RESOLUTION_M + DEPTH_QUANTIZATION_M + pixel


def geometry_allowance(reference, target):
    """Apply the same budget to continuous collision geometry, before scoring."""
    validate_reference(reference)
    half = np.asarray(target.size_xyz) / 2
    corners = np.array([[x, y, z] for x in (-half[0], half[0])
                        for y in (-half[1], half[1]) for z in (-half[2], half[2])])
    c, s = np.cos(target.yaw_rad), np.sin(target.yaw_rad)
    rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    points = corners @ rotation.T + target.center_xyz
    t = np.asarray(reference['T_map_color'])
    optical = (points - t[:3, 3]) @ t[:3, :3]
    if np.min(optical[:, 2]) <= 0:
        raise ValueError('target must be in front of the reference camera')
    return float(metric_allowance(reference, np.max(optical[:, 2])))


def _interior_mask(mask):
    # One-pixel erosion excludes a registration boundary, never grows a target.
    inside = np.zeros(mask.shape, dtype=bool)
    if min(mask.shape) >= 3:
        inside[1:-1, 1:-1] = np.logical_and.reduce([
            mask[dy:mask.shape[0] - 2 + dy, dx:mask.shape[1] - 2 + dx]
            for dy in range(3) for dx in range(3)])
    return inside


def associate_returns(points_map, target, reference=None):
    """No target exception without positive segmentation AND depth AND geometry.

    The reference is the initial air-phase RGB-D image. The same static object
    hypothesis is used for all methods. Unobserved faces remain ambiguous.
    """
    points = np.asarray(points_map, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not isinstance(target, PerceivedTarget):
        raise ValueError('points must be Nx3 and target must be PerceivedTarget')
    labels = np.full(len(points), OccupiedClass.ENVIRONMENT, dtype=np.int8)
    finite = np.all(np.isfinite(points), axis=1)
    near = np.zeros(len(points), dtype=bool)
    near[finite] = target.contains(points[finite])
    labels[near | ~finite] = OccupiedClass.AMBIGUOUS
    if reference is None:
        return labels
    validate_reference(reference)
    ids = np.flatnonzero(near)
    if not len(ids):
        return labels
    t, k = np.asarray(reference['T_map_color']), np.asarray(reference['color_K'])
    optical = (points[ids] - t[:3, 3]) @ t[:3, :3]
    front = optical[:, 2] > 0
    ids, optical = ids[front], optical[front]
    pixels = np.rint(optical[:, :2] / optical[:, 2, None] * [k[0, 0], k[1, 1]] +
                     [k[0, 2], k[1, 2]]).astype(np.int64)
    h, w = np.asarray(reference['mask']).shape
    inside = (pixels[:, 0] >= 0) & (pixels[:, 0] < w) & (pixels[:, 1] >= 0) & (pixels[:, 1] < h)
    ids, optical, pixels = ids[inside], optical[inside], pixels[inside]
    u, v = pixels[:, 0], pixels[:, 1]
    measured = np.asarray(reference['registered_depth_m'])[v, u]
    matches = (_interior_mask(np.asarray(reference['mask']))[v, u] &
               np.isfinite(measured) & (measured > 0) &
               (np.abs(measured - optical[:, 2]) <= metric_allowance(reference, optical[:, 2])))
    # Check the actual backprojected segmented surface, not only the LiDAR ray.
    valid_ids = np.flatnonzero(matches)
    z = measured[valid_ids]
    surface = np.column_stack(((u[valid_ids] - k[0, 2]) * z / k[0, 0],
                               (v[valid_ids] - k[1, 2]) * z / k[1, 1], z))
    surface_map = surface @ t[:3, :3].T + t[:3, 3]
    valid_ids = valid_ids[target.contains(surface_map)]
    labels[ids[valid_ids]] = OccupiedClass.TARGET
    return labels
