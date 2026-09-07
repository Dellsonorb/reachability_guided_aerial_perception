"""Translate between public SIM map and the frozen planar RM4D reference."""

from copy import deepcopy

import numpy as np


_TRANSFORM_TOLERANCE = 1e-8


def _se3(value, name):
    try:
        matrix = np.array(value, dtype=float, copy=True)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f'{name} must be a finite SE(3) matrix') from error
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0., 0., 0., 1.],
                               atol=_TRANSFORM_TOLERANCE, rtol=0.)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3),
                               atol=_TRANSFORM_TOLERANCE, rtol=0.)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1.,
                              atol=_TRANSFORM_TOLERANCE, rtol=0.)):
        raise ValueError(f'{name} must be a finite SE(3) matrix')
    return matrix


class FrameBridge:
    """Keep public map XY/yaw while aligning its nominal BUNKER plane to z=0."""

    def __init__(self, frame_calibration, frozen_T_bunker_aubo):
        self._calibration = {}
        for name in ('T_map_ground_odom', 'T_ground_odom_bunker', 'T_bunker_aubo'):
            if not isinstance(frame_calibration, dict) or name not in frame_calibration:
                raise ValueError(f'frame_calibration requires {name}')
            self._calibration[name] = _se3(frame_calibration[name], name)
        for name in ('T_map_ground_odom', 'T_ground_odom_bunker'):
            if not np.allclose(self._calibration[name][:3, 2], [0., 0., 1.],
                               atol=_TRANSFORM_TOLERANCE, rtol=0.):
                raise ValueError(f'{name} must be horizontal with its z axis up')
        if abs(self._calibration['T_ground_odom_bunker'][2, 3]) > _TRANSFORM_TOLERANCE:
            raise ValueError('T_ground_odom_bunker must have local z=0')
        frozen_mount = _se3(frozen_T_bunker_aubo, 'frozen_T_bunker_aubo')
        if not np.allclose(self._calibration['T_bunker_aubo'], frozen_mount,
                           atol=_TRANSFORM_TOLERANCE, rtol=0.):
            raise ValueError('SIM BUNKER-to-AUBO mount disagrees with frozen T_bunker_aubo')

        # Only the public reference-plane height is removed. Spawn XY/yaw stay public.
        self.ground_reference_height_m = float(self._calibration['T_map_ground_odom'][2, 3])
        self.T_reference_map = np.eye(4)
        self.T_reference_map[2, 3] = -self.ground_reference_height_m
        self.T_map_reference = np.eye(4)
        self.T_map_reference[2, 3] = self.ground_reference_height_m

    def as_dict(self):
        """Return the measured and derived calibration for run diagnostics."""
        return dict({name: value.tolist() for name, value in self._calibration.items()},
                    ground_reference_height_m=self.ground_reference_height_m,
                    T_reference_map=self.T_reference_map.tolist(),
                    T_map_reference=self.T_map_reference.tolist())

    def query_to_reference(self, query):
        """Translate an already SIM-regularized map request for frozen world API."""
        if query.get('frame_id') != 'map':
            raise ValueError('query frame_id must be map')
        converted = deepcopy(query)
        converted['position_xyz'] = list(query['position_xyz'])
        converted['position_xyz'][2] -= self.ground_reference_height_m
        converted['frame_id'] = 'world'
        return converted

    def result_to_map(self, result):
        """Translate global transforms only; preserve frozen candidate evidence."""
        if result.get('frame_id') != 'world':
            raise ValueError('baseline result frame_id must be world')
        converted = deepcopy(result)
        converted['frame_id'] = 'map'
        for collection in ('candidates', 'evaluated_candidates'):
            for candidate in converted.get(collection, []):
                for name in ('bunker', 'aubo', 'flange'):
                    old_name, new_name = f'T_world_{name}', f'T_map_{name}'
                    # Ranked records may alias evaluated records: rename once.
                    if old_name in candidate:
                        matrix = _se3(candidate.pop(old_name), old_name)
                        candidate[new_name] = (self.T_map_reference @ matrix).tolist()
        return converted
