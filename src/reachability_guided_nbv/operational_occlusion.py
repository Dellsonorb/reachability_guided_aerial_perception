"""Prediction-only occluders consistent with existing operational evidence.

Targets retain their perceived 3D box; complete ambiguous XY disks are extruded
to the existing assumed height. Neither geometry is an observation or a vote.
"""

from dataclasses import dataclass

import numpy as np

from environment_belief import EnvironmentState
from operational_gating.core import PerceivedTarget
from .geometry import segments_intersect_box
from .model import readonly, rotation_z


@dataclass(frozen=True, eq=False)
class OperationalOcclusion:
    lower: np.ndarray
    upper: np.ndarray
    target: PerceivedTarget
    ambiguous_xy: np.ndarray
    radius_m: float
    ground_z_m: float
    assumed_height_m: float

    @property
    def metadata(self):
        return dict(model='operational_geometry_v1', full_cell_prisms=len(self.lower),
                    ambiguous_cylinders=len(self.ambiguous_xy), ambiguous_radius_m=self.radius_m,
                    ambiguous_height_semantics='assumed_surrogate_not_measured_height',
                    assumed_height_m=self.assumed_height_m,
                    target_semantics='existing_perceived_expanded_oriented_box',
                    raw_evidence_unchanged=True)

    def contains(self, point):
        return bool(self.blocked(np.asarray(point), np.asarray(point)[None])[0])

    def blocked(self, origin, ends):
        origin, ends = np.asarray(origin, dtype=float), np.asarray(ends, dtype=float)
        if (origin.shape != (3,) or ends.ndim != 2 or ends.shape[1] != 3
                or not np.all(np.isfinite(origin)) or not np.all(np.isfinite(ends))):
            raise ValueError('occlusion segments must be finite XYZ arrays')
        blocked = np.zeros(len(ends), dtype=bool)
        segment_lo, segment_hi = np.minimum(ends, origin), np.maximum(ends, origin)
        for lo, hi in zip(self.lower, self.upper):
            possible = (~blocked & np.all(segment_lo <= hi, axis=1)
                        & np.all(segment_hi >= lo, axis=1))
            ids = np.flatnonzero(possible)
            blocked[ids] = segments_intersect_box(origin, ends[ids], lo, hi)
        center = np.asarray(self.target.center_xyz)
        rotation = rotation_z(self.target.yaw_rad)
        half = np.asarray(self.target.size_xyz) / 2 + self.target.geometry_allowance_m
        blocked |= segments_intersect_box((origin - center) @ rotation,
                                          (ends - center) @ rotation, -half, half)
        # Minimize XY distance only over the segment portion within the vertical
        # height slab. This is exact closed cylinder intersection, not a square.
        direction = ends - origin
        dz = direction[:, 2]
        low, high = np.zeros(len(ends)), np.ones(len(ends))
        moving = np.abs(dz) > 1e-12
        a = (self.ground_z_m - origin[2]) / dz[moving]
        b = (self.ground_z_m + self.assumed_height_m - origin[2]) / dz[moving]
        low[moving], high[moving] = np.maximum(0, np.minimum(a, b)), np.minimum(1, np.maximum(a, b))
        possible_z = (low <= high) & (moving | ((origin[2] >= self.ground_z_m)
                     & (origin[2] <= self.ground_z_m + self.assumed_height_m)))
        length2 = np.sum(direction[:, :2] ** 2, axis=1)
        for xy in self.ambiguous_xy:
            possible = (~blocked & possible_z & np.all(segment_lo[:, :2] <= xy + self.radius_m, axis=1)
                        & np.all(segment_hi[:, :2] >= xy - self.radius_m, axis=1))
            ids = np.flatnonzero(possible)
            if not len(ids):
                continue
            delta = origin[:2] - xy
            closest = np.divide(-direction[ids, :2] @ delta, length2[ids],
                                out=np.zeros(len(ids)), where=length2[ids] > 0)
            closest = np.clip(closest, low[ids], high[ids])
            distance = np.linalg.norm(delta + closest[:, None] * direction[ids, :2], axis=1)
            blocked[ids] = distance <= self.radius_m + 1e-12
        return blocked


def build_operational_occlusion(belief, operational, *, assumed_height_m=1.):
    """Use endpoint locality only where the existing sidecar declares completeness."""
    if operational.grid != belief.grid or operational.config != belief.config:
        raise ValueError('operational occlusion requires aligned belief/context')
    if not np.isfinite(assumed_height_m) or assumed_height_m <= 0:
        raise ValueError('assumed_height_m must be finite and positive')
    sidecar = operational.ambiguous_endpoints
    complete = np.zeros(belief.grid.shape, dtype=int)
    points, radius = np.empty((0, 2)), 0.
    if sidecar is not None and sidecar.profile_available:
        complete = sidecar.complete_vote_counts
        points, radius = np.unique(sidecar.points_xy, axis=0), sidecar.radius_m
    accounted = (operational.environment_occupied_votes + operational.ambiguous_occupied_votes
                 + operational.target_occupied_votes)
    full_cell = ((operational.environment_occupied_votes > 0)
                 | (operational.ambiguous_occupied_votes > complete)
                 | ((belief.state == EnvironmentState.OCCUPIED)
                    & ((accounted == 0) | (belief.occupied_evidence > accounted))))
    rows, cols = np.nonzero(full_cell)
    lower = np.column_stack((belief.origin_xy[0] + cols * belief.resolution_m,
                             belief.origin_xy[1] + rows * belief.resolution_m,
                             np.full(len(rows), belief.config.ground_z_m)))
    return OperationalOcclusion(readonly(lower),
        readonly(lower + [belief.resolution_m, belief.resolution_m, assumed_height_m]),
        operational.target, readonly(points), float(radius), belief.config.ground_z_m,
        float(assumed_height_m))
