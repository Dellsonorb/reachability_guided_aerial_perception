"""Centered BUNKER rectangle and closed-cell overlap, independent of ROS."""

from dataclasses import dataclass
from numbers import Real

import numpy as np

from environment_belief import EnvironmentGridSpec


CONTACT_TOLERANCE_M = 1e-12


@dataclass(frozen=True)
class FootprintSpec:
    """Frozen RM4D's existing padded half-dimensions, in base_link meters."""

    half_length_m: float = 0.52
    half_width_m: float = 0.39

    def __post_init__(self):
        for name in ('half_length_m', 'half_width_m'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
            object.__setattr__(self, name, float(value))


def _pose(xy, yaw):
    center = np.asarray(xy, dtype=float)
    if center.shape != (2,) or not np.all(np.isfinite(center)):
        raise ValueError('xy must contain two finite map coordinates')
    if isinstance(yaw, bool) or not isinstance(yaw, Real) or not np.isfinite(yaw):
        raise ValueError('yaw must be a finite angle in radians')
    c, s = np.cos(yaw), np.sin(yaw)
    return center, np.array([[c, -s], [s, c]])


def footprint_vertices(xy, yaw, footprint=FootprintSpec()):
    """Return four corners in map; xy is the representative base_link origin."""
    if not isinstance(footprint, FootprintSpec):
        raise ValueError('footprint must be FootprintSpec')
    center, rotation = _pose(xy, yaw)
    a, b = footprint.half_length_m, footprint.half_width_m
    return np.array([[-a, -b], [a, -b], [a, b], [-a, b]]) @ rotation.T + center


def footprint_cells(grid, xy, yaw, footprint=FootprintSpec()):
    """Return (row-major flat cell ids, clipped), including edge/corner contact.

    SAT compares rectangle projections on both grid and footprint axes. This
    small offline implementation examines every grid cell; no spatial index,
    dilation, interpolation or center-only containment approximation is used.
    """
    if not isinstance(grid, EnvironmentGridSpec):
        raise ValueError('grid must be EnvironmentGridSpec')
    vertices = footprint_vertices(xy, yaw, footprint)
    center, rotation = _pose(xy, yaw)
    rows, cols = np.indices(grid.shape)
    centers = np.column_stack((grid.origin_xy[0] + (cols.ravel() + 0.5) * grid.resolution_m,
                               grid.origin_xy[1] + (rows.ravel() + 0.5) * grid.resolution_m))
    delta = centers - center
    axes = np.vstack((np.eye(2), rotation.T))
    half_sizes = np.array([footprint.half_length_m, footprint.half_width_m])
    rectangle_radius = np.abs(axes @ rotation) @ half_sizes
    cell_radius = grid.resolution_m * 0.5 * np.abs(axes).sum(axis=1)
    overlap = np.all(np.abs(delta @ axes.T) <= rectangle_radius + cell_radius + CONTACT_TOLERANCE_M,
                     axis=1)
    x0, x1, y0, y1 = grid.extent
    clipped = (np.any(vertices < np.array([x0, y0]) - CONTACT_TOLERANCE_M)
               or np.any(vertices > np.array([x1, y1]) + CONTACT_TOLERANCE_M))
    return np.flatnonzero(overlap), bool(clipped)
