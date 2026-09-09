"""Endpoint-local geometry conditional on the public-map TF engineering profile.

The 33 mm disk declares 3 * 10 mm LiDAR range allowance + 2 mm resolution
and a 1 mm numerical rounding reserve. It is neither noise actually applied by
the custom SIM publisher, a calibrated probability, nor a total physical bound.
"""

from dataclasses import dataclass
from numbers import Real

import numpy as np

from task_relevant_uncertainty.geometry import (
    CONTACT_TOLERANCE_M, FootprintSpec, footprint_vertices,
)


AMBIGUOUS_ENDPOINT_PROFILE = 'public-map-tf-conditional-disk33mm-v1'
AMBIGUOUS_ENDPOINT_RADIUS_M = .033


def _readonly(value):
    array = np.array(value, copy=True)
    array.setflags(write=False)
    return array


@dataclass(frozen=True, eq=False)
class AmbiguousEndpointEvidence:
    """Detached accepted map XY endpoints with original observation/row IDs.

    ``complete_vote_counts`` counts completely replayed observation/cell
    groups, never points. Callers retaining only part of a group must not claim
    that group complete. Unrepresented historical votes remain cell blockers.
    The original row index identifies a row in the supplied observation, not
    an inferred firing time or a row in the filtered array.
    """

    points_xy: np.ndarray
    observation_indices: np.ndarray
    row_indices: np.ndarray
    cell_ids: np.ndarray
    complete_vote_counts: np.ndarray
    profile: str | None = AMBIGUOUS_ENDPOINT_PROFILE
    radius_m: float = AMBIGUOUS_ENDPOINT_RADIUS_M

    def __post_init__(self):
        points = np.asarray(self.points_xy, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or not np.all(np.isfinite(points)):
            raise ValueError('points_xy must have finite shape (N, 2)')
        object.__setattr__(self, 'points_xy', _readonly(points))
        for name in ('observation_indices', 'row_indices', 'cell_ids', 'complete_vote_counts'):
            values = np.asarray(getattr(self, name))
            expected_shape = (len(points),) if name != 'complete_vote_counts' else None
            if (values.dtype.kind not in 'iu' or np.any(values < 0)
                    or np.any(values > np.iinfo(np.int64).max)
                    or (expected_shape is not None and values.shape != expected_shape)
                    or (expected_shape is None and (values.ndim != 2 or 0 in values.shape))):
                raise ValueError(f'{name} must contain aligned nonnegative integer counts/indices')
            object.__setattr__(self, name, _readonly(values.astype(np.int64)))
        if np.any(self.cell_ids >= self.complete_vote_counts.size):
            raise ValueError('cell_ids must fit the complete_vote_counts grid')
        # A complete window vote needs a represented observation/cell group.
        groups = np.unique(np.column_stack((self.observation_indices, self.cell_ids)), axis=0)
        represented = np.bincount(groups[:, 1], minlength=self.complete_vote_counts.size)
        if np.any(self.complete_vote_counts.ravel() > represented):
            raise ValueError('complete_vote_counts exceed represented observation/cell groups')
        if self.profile is not None and not isinstance(self.profile, str):
            raise ValueError('profile must be a string or None')
        radius = self.radius_m
        if (isinstance(radius, bool) or not isinstance(radius, Real)
                or not np.isfinite(radius) or radius < 0):
            raise ValueError('radius_m must be finite and nonnegative')
        object.__setattr__(self, 'radius_m', float(radius))

    @property
    def profile_available(self):
        """Only the declared radius/profile can replace a historical cell."""
        return (self.profile == AMBIGUOUS_ENDPOINT_PROFILE
                and self.radius_m == AMBIGUOUS_ENDPOINT_RADIUS_M)


def disk_footprint_intersections(points_xy, xy, yaw, footprint=FootprintSpec(),
                                 *, radius_m=AMBIGUOUS_ENDPOINT_RADIUS_M):
    """Return closed disk intersections with the continuous padded rectangle."""
    # Reuse the existing footprint's finite pose/spec checks.
    footprint_vertices(xy, yaw, footprint)
    points = np.asarray(points_xy, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.all(np.isfinite(points)):
        raise ValueError('points_xy must have finite shape (N, 2)')
    if (isinstance(radius_m, bool) or not isinstance(radius_m, Real)
            or not np.isfinite(radius_m) or radius_m < 0):
        raise ValueError('radius_m must be finite and nonnegative')
    delta = points - np.asarray(xy)
    c, s = np.cos(yaw), np.sin(yaw)
    local = np.column_stack((c * delta[:, 0] + s * delta[:, 1],
                             -s * delta[:, 0] + c * delta[:, 1]))
    outside = np.maximum(np.abs(local) - [footprint.half_length_m, footprint.half_width_m], 0.)
    return np.hypot(outside[:, 0], outside[:, 1]) <= radius_m + CONTACT_TOLERANCE_M


@dataclass(frozen=True)
class AmbiguousFootprintDiagnostics:
    """Separate v1.3 audit fields; legacy FootprintAssessment stays unchanged.

    Endpoint indices index the sidecar; cell IDs are row-major grid IDs.
    Aliased-clear cells have complete history and no intersecting disk from
    that cell. Other cells, including neighboring cells, may still block.
    """

    intersecting_endpoint_indices: tuple[int, ...]
    legacy_fallback_cell_ids: tuple[int, ...]
    coarse_ambiguous_cell_ids: tuple[int, ...]
    aliased_clear_cell_ids: tuple[int, ...]

    @property
    def intersecting_endpoint_count(self):
        return len(self.intersecting_endpoint_indices)

    @property
    def legacy_fallback_cells(self):
        return len(self.legacy_fallback_cell_ids)

    @property
    def coarse_ambiguous_cells(self):
        return len(self.coarse_ambiguous_cell_ids)

    @property
    def aliased_clear_cells(self):
        return len(self.aliased_clear_cell_ids)
