"""Core, ROS-independent types for validated manipulation interest fields."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum
import math
from numbers import Integral
from typing import Any, ClassVar

import numpy as np


def _as_finite_vector(value: Any, size: int, name: str) -> tuple[float, ...]:
    """Return a fixed-size finite vector as an immutable tuple."""
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite vector with shape ({size},)") from exc
    if array.shape != (size,):
        raise ValueError(f"{name} must have shape ({size},)")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")
    return tuple(float(item) for item in array)


class FieldStatus(str, Enum):
    """Assessment status exposed by a manipulation interest field."""

    PARTIALLY_ASSESSED = "partially_assessed"
    NO_INVERSE_REACHABLE = "no_inverse_reachable"


class CellState(IntEnum):
    """Per-cell state, preserving a separate value for unassessed cells."""

    UNASSESSED = -1
    INFEASIBLE = 0
    LOW = 1
    HIGH = 2


@dataclass(frozen=True)
class GraspTCP:
    """A grasp TCP expressed in the fixed map frame."""

    grasp_id: str
    frame_id: str
    position_xyz: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        if not isinstance(self.grasp_id, str) or not self.grasp_id.strip():
            raise ValueError("grasp_id must not be blank")
        if self.frame_id != "map":
            raise ValueError("frame_id must be 'map'")

        position = _as_finite_vector(self.position_xyz, 3, "position_xyz")
        quaternion = _as_finite_vector(self.quaternion_xyzw, 4, "quaternion_xyzw")
        norm = math.sqrt(sum(value * value for value in quaternion))
        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
            raise ValueError("quaternion_xyzw must be a unit quaternion")
        object.__setattr__(self, "position_xyz", position)
        object.__setattr__(self, "quaternion_xyzw", quaternion)

    def as_request(self) -> dict[str, Any]:
        """Return exactly the request fields accepted by the planner boundary."""
        return {
            "grasp_id": self.grasp_id,
            "frame_id": self.frame_id,
            "position_xyz": self.position_xyz,
            "quaternion_xyzw": self.quaternion_xyzw,
        }


@dataclass(frozen=True)
class GridSpec:
    """A finite, axis-aligned XY grid."""

    center_xy: tuple[float, float]
    width_m: float
    height_m: float
    resolution_m: float

    _DIVISIBILITY_TOLERANCE: ClassVar[float] = 1e-9

    def __post_init__(self) -> None:
        center = _as_finite_vector(self.center_xy, 2, "center_xy")
        object.__setattr__(self, "center_xy", center)
        for name in ("width_m", "height_m", "resolution_m"):
            value = getattr(self, name)
            if not isinstance(value, (int, float, np.number)) or not math.isfinite(float(value)):
                raise ValueError(f"{name} must be finite")
            if float(value) <= 0:
                raise ValueError(f"{name} must be positive")
            object.__setattr__(self, name, float(value))
        for name in ("width_m", "height_m"):
            ratio = float(getattr(self, name)) / self.resolution_m
            if not math.isclose(ratio, round(ratio), rel_tol=0.0, abs_tol=self._DIVISIBILITY_TOLERANCE):
                raise ValueError(f"{name} must be divisible by resolution_m")

    @classmethod
    def centered(
        cls,
        center_xy: tuple[float, float],
        width_m: float,
        height_m: float,
        resolution_m: float,
    ) -> "GridSpec":
        return cls(center_xy, width_m, height_m, resolution_m)

    @property
    def origin_xy(self) -> tuple[float, float]:
        return (
            self.center_xy[0] - self.width_m / 2.0,
            self.center_xy[1] - self.height_m / 2.0,
        )

    @property
    def origin_x(self) -> float:
        return self.origin_xy[0]

    @property
    def origin_y(self) -> float:
        return self.origin_xy[1]

    @property
    def width_cells(self) -> int:
        return int(round(self.width_m / self.resolution_m))

    @property
    def height_cells(self) -> int:
        return int(round(self.height_m / self.resolution_m))

    @property
    def shape(self) -> tuple[int, int]:
        return (self.height_cells, self.width_cells)

    @property
    def total_cells(self) -> int:
        return self.width_cells * self.height_cells

    def cell_index(self, x: float, y: float) -> tuple[int, int] | None:
        """Return ``(row, column)`` for a point, or ``None`` outside the grid."""
        try:
            x_value, y_value = float(x), float(y)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(x_value) or not math.isfinite(y_value):
            return None
        origin_x, origin_y = self.origin_xy
        column = math.floor((x_value - origin_x) / self.resolution_m)
        row = math.floor((y_value - origin_y) / self.resolution_m)
        if not (0 <= column < self.width_cells and 0 <= row < self.height_cells):
            return None
        return (row, column)


@dataclass(frozen=True)
class FieldConfig:
    """Numerical thresholds used to map validated grasps to relevance."""

    minimum_joint_margin_rad: float = 0.01
    joint_margin_saturation_rad: float = 0.5
    high_relevance_threshold: float = 0.8

    def __post_init__(self) -> None:
        for name in (
            "minimum_joint_margin_rad",
            "joint_margin_saturation_rad",
            "high_relevance_threshold",
        ):
            value = getattr(self, name)
            if not isinstance(value, (int, float, np.number)) or not math.isfinite(float(value)):
                raise ValueError(f"{name} must be finite")
            object.__setattr__(self, name, float(value))
        if self.minimum_joint_margin_rad < 0:
            raise ValueError("minimum_joint_margin_rad must be nonnegative")
        if self.joint_margin_saturation_rad <= self.minimum_joint_margin_rad:
            raise ValueError("joint_margin_saturation_rad must be greater than minimum_joint_margin_rad")
        if not 0.0 <= self.high_relevance_threshold <= 1.0:
            raise ValueError("high_relevance_threshold must be between 0 and 1")


@dataclass(frozen=True)
class AssessmentCoverage:
    inverse_reachable: bool
    deduplicated_candidates: int
    validation_limit: int
    evaluated_candidates: int
    valid_candidates: int
    evaluated_cells: int
    total_cells: int
    candidate_validation_fraction: float
    assessed_cell_fraction: float
    validation_truncated: bool

    def __post_init__(self) -> None:
        for name in (
            "deduplicated_candidates",
            "validation_limit",
            "evaluated_candidates",
            "valid_candidates",
            "evaluated_cells",
            "total_cells",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Integral) or value < 0:
                raise ValueError(f"{name} must be a nonnegative integer")
            object.__setattr__(self, name, int(value))
        for name in ("candidate_validation_fraction", "assessed_cell_fraction"):
            value = getattr(self, name)
            if not isinstance(value, (int, float, np.number)) or not math.isfinite(float(value)):
                raise ValueError(f"{name} must be finite")
            if not 0.0 <= float(value) <= 1.0:
                raise ValueError(f"{name} must be between 0 and 1")
            object.__setattr__(self, name, float(value))


def _as_grid_array(value: Any, shape: tuple[int, int], name: str) -> np.ndarray:
    array = np.asarray(value)
    if array.shape != shape:
        raise ValueError(f"{name} must have shape {shape}")
    return array


@dataclass(frozen=True)
class ManipulationInterestField:
    grasp_id: str
    frame_id: str
    status: FieldStatus
    grid: GridSpec
    relevance: np.ndarray
    cell_state: np.ndarray
    evaluated_count: int
    feasible_count: int
    best_yaw: np.ndarray
    best_joint_margin_rad: np.ndarray
    best_fk_position_residual_m: np.ndarray
    best_fk_orientation_residual_rad: np.ndarray
    coverage: AssessmentCoverage

    def __post_init__(self) -> None:
        if not isinstance(self.grasp_id, str) or not self.grasp_id.strip():
            raise ValueError("grasp_id must not be blank")
        if self.frame_id != "map":
            raise ValueError("frame_id must be 'map'")
        if not isinstance(self.grid, GridSpec):
            raise ValueError("grid must be a GridSpec")
        if not isinstance(self.status, FieldStatus):
            raise ValueError("status must be a FieldStatus")
        if not isinstance(self.coverage, AssessmentCoverage):
            raise ValueError("coverage must be an AssessmentCoverage")
        for name in (
            "relevance",
            "cell_state",
            "best_yaw",
            "best_joint_margin_rad",
            "best_fk_position_residual_m",
            "best_fk_orientation_residual_rad",
        ):
            object.__setattr__(self, name, _as_grid_array(getattr(self, name), self.grid.shape, name))
        for name in ("evaluated_count", "feasible_count"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Integral) or value < 0:
                raise ValueError(f"{name} must be a nonnegative integer")
            object.__setattr__(self, name, int(value))
        if self.feasible_count > self.evaluated_count:
            raise ValueError("feasible_count cannot exceed evaluated_count")


@dataclass(frozen=True)
class OccupancyGridPayload:
    frame_id: str
    field_status: FieldStatus
    origin_x: float
    origin_y: float
    resolution: float
    width: int
    height: int
    data: np.ndarray

    def __post_init__(self) -> None:
        if self.frame_id != "map":
            raise ValueError("frame_id must be 'map'")
        if not isinstance(self.field_status, FieldStatus):
            raise ValueError("field_status must be a FieldStatus")
        for name in ("origin_x", "origin_y", "resolution"):
            value = getattr(self, name)
            if not isinstance(value, (int, float, np.number)) or not math.isfinite(float(value)):
                raise ValueError(f"{name} must be finite")
            if name == "resolution" and float(value) <= 0:
                raise ValueError("resolution must be positive")
            object.__setattr__(self, name, float(value))
        for name in ("width", "height"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Integral) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
            object.__setattr__(self, name, int(value))
        array = np.asarray(self.data)
        if array.shape not in ((self.height, self.width), (self.height * self.width,)):
            raise ValueError("data must have shape (height, width) or (height * width,)")
        object.__setattr__(self, "data", array)
