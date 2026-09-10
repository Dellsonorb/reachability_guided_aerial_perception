"""Nominal ground acquisition opportunity under a finite cyclic scan program.

Fractions describe scheduled start phases, not calibrated return probabilities.
Intersecting rays never become measured endpoints, belief updates, or votes.
"""

from copy import deepcopy
from dataclasses import dataclass, field
import hashlib
import math
from numbers import Integral, Real
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from environment_belief import EnvironmentState
from .geometry import occupied_boxes, segments_intersect_box, sensor_transform
from .model import NBVConfig, SensorModel, readonly


def _positive_integer(value, name):
    if isinstance(value, bool) or not isinstance(value, Integral) or value < 1:
        raise ValueError(f'{name} must be a positive integer')
    return int(value)


def _duration(value, name, *, zero=False):
    if (isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value)
            or value < 0 or (value == 0 and not zero)):
        raise ValueError(f'{name} must be finite and {"nonnegative" if zero else "positive"}')
    return float(value)


def _phase_fraction(packet_hits, horizon):
    phases = len(packet_hits)
    if horizon >= phases:
        return packet_hits.any(axis=0).astype(float)
    cumulative = np.concatenate((np.zeros_like(packet_hits[:1], dtype=np.int32),
        np.cumsum(np.concatenate((packet_hits, packet_hits), axis=0), axis=0, dtype=np.int32)))
    starts = np.arange(phases)
    return ((cumulative[starts + horizon] - cumulative[starts]) > 0).mean(axis=0)


def _publisher_mask(values, path, packet_rows, packet_period_s):
    """Mirror the Livox publisher's roundf angular-index and downsample guards.

    The CSV row number stays intact even for an unpublishable ray. Gazebo's
    configured angular bins screen rows; they do not replace the CSV directions.
    """
    root = ET.parse(path).getroot()
    sensors = [sensor for sensor in root.findall('.//sensor')
               if sensor.find('plugin') is not None and
               'livox' in sensor.find('plugin').get('filename', '').lower()]
    if len(sensors) != 1:
        raise ValueError('publisher SDF must contain one Livox sensor plugin')
    sensor = sensors[0]
    plugin = sensor.find('plugin')

    def numeric(element, tag, integer=False):
        text = element.findtext(tag)
        try:
            value = float(text)
        except (TypeError, ValueError) as exc:
            raise ValueError(f'invalid publisher SDF {tag}') from exc
        if not np.isfinite(value) or (integer and value != int(value)):
            raise ValueError(f'invalid publisher SDF {tag}')
        return int(value) if integer else value

    samples = numeric(plugin, 'samples', True)
    downsample = max(1, numeric(plugin, 'downsample', True))
    rate = numeric(sensor, 'update_rate')
    if samples != packet_rows or rate <= 0 or not np.isclose(rate * packet_period_s, 1, rtol=1e-10, atol=1e-12):
        raise ValueError('publisher packet rows/update rate differ from the requested scan schedule')
    emitted = np.arange(len(values)) % packet_rows % downsample == 0
    settings = dict(samples=samples, downsample=downsample, update_rate_hz=rate)
    for axis, radians in (('horizontal', np.radians(values[:, 1])),
                          ('vertical', np.radians(values[:, 2]) - np.pi / 2)):
        element = plugin.find(f'ray/scan/{axis}')
        if element is None:
            raise ValueError(f'publisher SDF missing {axis} scan')
        samples = numeric(element, 'samples', True)
        resolution = numeric(element, 'resolution')
        lower, upper = numeric(element, 'min_angle'), numeric(element, 'max_angle')
        range_count = int(samples * resolution)
        if samples < 1 or resolution <= 0 or range_count < 2 or upper <= lower:
            raise ValueError(f'invalid publisher SDF {axis} angular bins')
        increment = (upper - lower) / (range_count - 1)
        # C++ roundf first converts to float32 and rounds half away from zero.
        ratio = np.asarray((radians - lower) / increment, dtype=np.float32)
        index = np.copysign(np.floor(np.abs(ratio.astype(float)) + .5), ratio)
        emitted &= (index >= 0) & (index < samples)
        settings[axis] = dict(samples=samples, resolution=resolution, min_angle_rad=lower,
                              max_angle_rad=upper, range_count=range_count)
    return emitted, settings


@dataclass(frozen=True, eq=False)
class FiniteScanPrediction:
    status: str
    opportunity: np.ndarray
    unoccluded_opportunity: np.ndarray
    packet_hits: np.ndarray


@dataclass(frozen=True, eq=False)
class FiniteScan:
    """Immutable unit directions in CSV order and a fixed level-hover mount.

    ``window_s`` measures first-to-last packet span, so the nominal packet
    horizon is ceil(window_s / packet_period_s) + 1. Every possible cyclic start
    has equal weight. Pose motion, packet loss and unknown occlusion are absent.
    """

    directions: np.ndarray
    packet_rows: int = 10000
    packet_period_s: float = .1
    window_s: float = 5.
    sensor: SensorModel = field(default_factory=SensorModel)
    emission_mask: np.ndarray | None = None
    _source: dict = field(default_factory=dict, repr=False)
    _slopes: np.ndarray = field(init=False, repr=False)
    _distance_scale: np.ndarray = field(init=False, repr=False)
    _packet_ids: np.ndarray = field(init=False, repr=False)
    _window_packets: int = field(init=False, repr=False)

    def __post_init__(self):
        directions = np.asarray(self.directions, dtype=float)
        if (directions.ndim != 2 or directions.shape[1] != 3 or len(directions) == 0
                or not np.isfinite(directions).all()
                or not np.allclose(np.linalg.norm(directions, axis=1), 1, rtol=0, atol=1e-8)):
            raise ValueError('directions must be a nonempty (N,3) array of finite unit rays')
        rows = _positive_integer(self.packet_rows, 'packet_rows')
        if len(directions) % rows:
            raise ValueError('directions must contain a whole number of packets')
        period = _duration(self.packet_period_s, 'packet_period_s')
        window = _duration(self.window_s, 'window_s', zero=True)
        if not isinstance(self.sensor, SensorModel):
            raise ValueError('sensor must be a SensorModel')
        emitted = np.ones(len(directions), dtype=bool) if self.emission_mask is None else np.asarray(self.emission_mask)
        if emitted.shape != (len(directions),) or emitted.dtype != np.dtype(bool):
            raise ValueError('emission_mask must be a bool array of shape (N,)')
        body_rays = directions @ self.sensor.T_uav_lidar[:3, :3].T
        downward = emitted & (body_rays[:, 2] < -1e-12)
        object.__setattr__(self, 'directions', readonly(directions))
        object.__setattr__(self, 'emission_mask', readonly(emitted))
        object.__setattr__(self, 'packet_rows', rows)
        object.__setattr__(self, 'packet_period_s', period)
        object.__setattr__(self, 'window_s', window)
        object.__setattr__(self, '_window_packets', math.ceil(window / period) + 1)
        object.__setattr__(self, '_packet_ids', readonly(np.flatnonzero(downward) // rows))
        object.__setattr__(self, '_slopes', readonly(-body_rays[downward, :2] / body_rays[downward, 2, None]))
        object.__setattr__(self, '_distance_scale', readonly(-1 / body_rays[downward, 2]))
        object.__setattr__(self, '_source', deepcopy(self._source))

    @classmethod
    def from_csv(cls, path, *, window_s=5., sensor=SensorModel(), packet_rows=10000,
                 packet_period_s=.1, publisher_sdf_path=None):
        """Load azimuth/zenith columns, optionally matching the installed publisher.

        Without an SDF, every row is scheduled for emission. With one, its
        angular index and downsample guards are applied without removing rows.
        """
        path = Path(path).resolve()
        values = np.loadtxt(path, delimiter=',', skiprows=1, ndmin=2)
        if values.ndim != 2 or values.shape[1] != 3 or len(values) == 0 or not np.isfinite(values).all():
            raise ValueError('scan CSV must contain finite time, azimuth and zenith columns')
        rows = _positive_integer(packet_rows, 'packet_rows')
        period = _duration(packet_period_s, 'packet_period_s')
        azimuth, elevation = np.radians(values[:, 1]), np.radians(90 - values[:, 2])
        directions = np.column_stack((np.cos(elevation) * np.cos(azimuth),
                                      np.cos(elevation) * np.sin(azimuth), np.sin(elevation)))
        emitted, publisher = None, None
        if publisher_sdf_path is not None:
            publisher_sdf_path = Path(publisher_sdf_path).resolve()
            emitted, publisher = _publisher_mask(values, publisher_sdf_path, rows, period)
        source = dict(scan_path=str(path), scan_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                      publisher_sdf_path=None if publisher_sdf_path is None else str(publisher_sdf_path),
                      publisher=publisher)
        return cls(directions, packet_rows=rows, packet_period_s=period, window_s=window_s,
                   sensor=sensor, emission_mask=emitted, _source=source)

    @property
    def metadata(self):
        return dict(scan_path=None, scan_sha256=None, publisher_sdf_path=None, publisher=None) | deepcopy(self._source) | dict(
            rows=len(self.directions), packet_rows=self.packet_rows, packet_count=len(self.directions) // self.packet_rows,
            emitted_rows=int(self.emission_mask.sum()), packet_period_s=self.packet_period_s,
            window_s=self.window_s, window_packets=self._window_packets,
            phase_prior='uniform_cyclic_packet_starts', phase_fraction_not_calibrated=True,
            nominal_pose='level_hover_with_fixed_sensor_mount',
            ground_intersections_are_observations=False)

    def predict(self, belief, viewpoint, config=NBVConfig()):
        """Intersect scheduled rays and existing known prisms before cell binning."""
        grid = belief.grid
        shape = grid.shape
        size = grid.width_cells * grid.height_cells
        packets = len(self.directions) // self.packet_rows
        hits = np.zeros((packets, size), dtype=bool)
        unoccluded_hits = np.zeros_like(hits)
        transform = sensor_transform(viewpoint, self.sensor)
        origin = transform[:3, 3]
        lower, upper = occupied_boxes(belief, config.assumed_height_m)
        status = 'VALID'
        if origin[2] <= belief.config.ground_z_m:
            status = 'SENSOR_AT_OR_BELOW_GROUND'
        elif np.any(np.all((origin >= lower) & (origin <= upper), axis=1)):
            status = 'SENSOR_INSIDE_ASSUMED_PRISM'
        if status == 'VALID':
            height = origin[2] - belief.config.ground_z_m
            travel = height * self._distance_scale
            inside_range = (travel > belief.config.min_range_m) & (travel < belief.config.max_range_m)
            slopes = self._slopes[inside_range]
            c, s = np.cos(viewpoint.yaw_rad), np.sin(viewpoint.yaw_rad)
            x = height * (c * slopes[:, 0] - s * slopes[:, 1]) + origin[0]
            y = height * (s * slopes[:, 0] + c * slopes[:, 1]) + origin[1]
            xedges = grid.origin_xy[0] + np.arange(grid.width_cells + 1) * grid.resolution_m
            yedges = grid.origin_xy[1] + np.arange(grid.height_cells + 1) * grid.resolution_m
            cols = np.searchsorted(xedges, x, side='right') - 1
            rows = np.searchsorted(yedges, y, side='right') - 1
            inside = (cols >= 0) & (cols < grid.width_cells) & (rows >= 0) & (rows < grid.height_cells)
            cells = rows[inside] * grid.width_cells + cols[inside]
            packet = self._packet_ids[inside_range][inside]
            ends = np.column_stack((x[inside], y[inside], np.full(inside.sum(), belief.config.ground_z_m)))
            # The existing no-occlusion ablation also excludes occupied targets.
            eligible = belief.state.ravel()[cells] != EnvironmentState.OCCUPIED
            cells, packet, ends = cells[eligible], packet[eligible], ends[eligible]
            unoccluded_hits[packet, cells] = True
            active = np.ones(len(cells), dtype=bool)
            segment_lo, segment_hi = np.minimum(ends, origin), np.maximum(ends, origin)
            for lo, hi in zip(lower, upper):
                possible = active & np.all(segment_lo <= hi, axis=1) & np.all(segment_hi >= lo, axis=1)
                indices = np.flatnonzero(possible)
                active[indices] &= ~segments_intersect_box(origin, ends[indices], lo, hi)
            hits[packet[active], cells[active]] = True
        return FiniteScanPrediction(status,
            readonly(_phase_fraction(hits, self._window_packets).reshape(shape)),
            readonly(_phase_fraction(unoccluded_hits, self._window_packets).reshape(shape)),
            readonly(hits.reshape((packets, *shape))))
