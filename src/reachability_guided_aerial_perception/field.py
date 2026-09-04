"""Coverage-aware, ROS-independent manipulation interest-field builders."""

from __future__ import annotations

from collections.abc import Mapping
from numbers import Integral
import math
from typing import Any

import numpy as np

from .model import (
    AssessmentCoverage,
    CellState,
    FieldConfig,
    FieldStatus,
    GraspTCP,
    GridSpec,
    ManipulationInterestField,
)


_POSE_FIELDS = ("bunker_x", "bunker_y", "bunker_yaw")


def _is_gate_valid(candidate: Mapping[str, Any], config: FieldConfig) -> bool:
    """Apply the complete candidate validity gate."""
    if not all(candidate.get(name) is True for name in ("rm4d_reachable", "ik_valid", "collision_free", "valid")):
        return False
    if candidate.get("footprint_collision") is True:
        return False
    margin = _as_scalar(candidate.get("joint_margin_rad"))
    return margin is not None and margin >= config.minimum_joint_margin_rad


def candidate_relevance(candidate: Mapping[str, Any], config: FieldConfig) -> float:
    """Return margin-normalized relevance after all manipulation-validity gates."""
    if not isinstance(candidate, Mapping) or not isinstance(config, FieldConfig):
        return 0.0
    if not _is_gate_valid(candidate, config):
        return 0.0
    margin = _as_scalar(candidate["joint_margin_rad"])
    assert margin is not None
    return float(np.clip(margin / config.joint_margin_saturation_rad, 0.0, 1.0))


def _as_scalar(value: Any) -> float | None:
    """Convert a scalar numeric value without NumPy scalar conversion warnings."""
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None
    if array.shape != ():
        return None
    scalar = float(array)
    return scalar if math.isfinite(scalar) else None


def _require_count(summary: Mapping[str, Any], name: str) -> int:
    value = summary.get(name)
    if isinstance(value, bool) or not isinstance(value, Integral) or value < 0:
        raise ValueError(f"summary.{name} must be a nonnegative integer")
    return int(value)


def _validate_pose(candidate: Mapping[str, Any]) -> tuple[float, float, float]:
    values = []
    for name in _POSE_FIELDS:
        if name not in candidate:
            raise ValueError(f"candidate missing {name}")
        value = _as_scalar(candidate[name])
        if value is None:
            raise ValueError(f"candidate {name} must be finite")
        values.append(value)
    return tuple(values)  # type: ignore[return-value]


def _validate_diagnostic(candidate: Mapping[str, Any], name: str) -> float:
    value = candidate.get(name)
    if value is None:
        return math.nan
    value = _as_scalar(value)
    if value is None:
        raise ValueError(f"candidate {name} must be null or finite nonnegative")
    if value < 0:
        raise ValueError(f"candidate {name} must be null or finite nonnegative")
    return value


def _validate_result(
    grasp: GraspTCP,
    result: Mapping[str, Any],
    config: FieldConfig,
) -> tuple[Mapping[str, Any], dict[str, int]]:
    if not isinstance(result, Mapping):
        raise ValueError("result must be a mapping")
    if result.get("frame_id") != "map":
        raise ValueError("result.frame_id must be 'map'")
    if result.get("grasp_id") != grasp.grasp_id:
        raise ValueError("result.grasp_id does not match grasp")
    summary = result.get("summary")
    if not isinstance(summary, Mapping):
        raise ValueError("result.summary must be a mapping")
    evaluated = result.get("evaluated_candidates")
    if not isinstance(evaluated, list):
        raise ValueError("result.evaluated_candidates must be a list")
    counts = {name: _require_count(summary, name) for name in (
        "inverse_reachable", "deduplicated", "validation_limit", "evaluated", "valid"
    )}
    if counts["evaluated"] != len(evaluated):
        raise ValueError("summary.evaluated must equal evaluated_candidates length")
    if counts["evaluated"] > counts["validation_limit"]:
        raise ValueError("summary.evaluated cannot exceed validation_limit")
    if counts["deduplicated"] > counts["inverse_reachable"]:
        raise ValueError("summary.deduplicated cannot exceed inverse_reachable")
    if counts["inverse_reachable"] == 0 and any(counts[name] for name in ("deduplicated", "evaluated", "valid")):
        raise ValueError("zero inverse-reachable result must have zero candidate counts")
    gate_valid = 0
    for item in evaluated:
        if not isinstance(item, Mapping):
            raise ValueError("each evaluated candidate must be a mapping")
        _validate_pose(item)
        if _is_gate_valid(item, config):
            gate_valid += 1
    if counts["valid"] != gate_valid:
        raise ValueError("summary.valid must equal gate-valid candidate count")
    return result, counts


def build_field_from_result(
    grasp: GraspTCP,
    result: Mapping[str, Any],
    grid: GridSpec | None = None,
    config: FieldConfig = FieldConfig(),
) -> ManipulationInterestField:
    """Build a field from an already-computed RM4D result."""
    if not isinstance(grasp, GraspTCP):
        raise ValueError("grasp must be a GraspTCP")
    if not isinstance(config, FieldConfig):
        raise ValueError("config must be a FieldConfig")
    if grid is None:
        grid = GridSpec.centered((grasp.position_xyz[0], grasp.position_xyz[1]), 3.0, 3.0, 0.1)
    if not isinstance(grid, GridSpec):
        raise ValueError("grid must be a GridSpec")
    _, counts = _validate_result(grasp, result, config)
    evaluated = result["evaluated_candidates"]
    shape = grid.shape
    relevance = np.full(shape, np.nan, dtype=float)
    cell_state = np.full(shape, CellState.UNASSESSED.value, dtype=np.int8)
    evaluated_count = np.zeros(shape, dtype=np.int32)
    feasible_count = np.zeros(shape, dtype=np.int32)
    best_yaw = np.full(shape, np.nan, dtype=float)
    best_margin = np.full(shape, np.nan, dtype=float)
    best_position = np.full(shape, np.nan, dtype=float)
    best_orientation = np.full(shape, np.nan, dtype=float)
    best_score = np.full(shape, np.nan, dtype=float)

    for item in evaluated:
        x, y, yaw = _validate_pose(item)
        cell = grid.cell_index(x, y)
        if cell is None:
            continue
        _validate_diagnostic(item, "fk_position_residual_m")
        _validate_diagnostic(item, "fk_orientation_residual_rad")
        evaluated_count[cell] += 1
        score = candidate_relevance(item, config)
        if score <= 0.0:
            continue
        feasible_count[cell] += 1
        if np.isnan(best_score[cell]) or score > best_score[cell]:
            best_score[cell] = score
            relevance[cell] = score
            best_yaw[cell] = yaw
            best_margin[cell] = float(item["joint_margin_rad"])
            best_position[cell] = _validate_diagnostic(item, "fk_position_residual_m")
            best_orientation[cell] = _validate_diagnostic(item, "fk_orientation_residual_rad")

    assessed = evaluated_count > 0
    infeasible = assessed & (feasible_count == 0)
    cell_state[infeasible] = CellState.INFEASIBLE.value
    feasible = feasible_count > 0
    cell_state[feasible & (relevance < config.high_relevance_threshold)] = CellState.LOW.value
    cell_state[feasible & (relevance >= config.high_relevance_threshold)] = CellState.HIGH.value
    relevance[infeasible] = 0.0
    evaluated_cells = int(np.count_nonzero(assessed))
    coverage = AssessmentCoverage(
        inverse_reachable=counts["inverse_reachable"],
        deduplicated_candidates=counts["deduplicated"],
        validation_limit=counts["validation_limit"],
        evaluated_candidates=counts["evaluated"],
        valid_candidates=counts["valid"],
        evaluated_cells=evaluated_cells,
        total_cells=grid.total_cells,
        candidate_validation_fraction=(counts["evaluated"] / counts["deduplicated"] if counts["deduplicated"] else 0.0),
        assessed_cell_fraction=(evaluated_cells / grid.total_cells if grid.total_cells else 0.0),
        validation_truncated=counts["deduplicated"] > counts["evaluated"],
    )
    status = FieldStatus.NO_INVERSE_REACHABLE if counts["inverse_reachable"] == 0 else FieldStatus.PARTIALLY_ASSESSED
    return ManipulationInterestField(
        grasp_id=grasp.grasp_id,
        frame_id="map",
        status=status,
        grid=grid,
        relevance=relevance,
        cell_state=cell_state,
        evaluated_count=evaluated_count,
        feasible_count=feasible_count,
        best_yaw=best_yaw,
        best_joint_margin_rad=best_margin,
        best_fk_position_residual_m=best_position,
        best_fk_orientation_residual_rad=best_orientation,
        coverage=coverage,
    )


def build_field(
    grasp: GraspTCP,
    rm4d_api: Any,
    grid: GridSpec | None = None,
    config: FieldConfig = FieldConfig(),
) -> ManipulationInterestField:
    """Call the planner once and build a field from its complete evaluation set."""
    result = rm4d_api.plan(grasp.as_request(), top_k=1)
    return build_field_from_result(grasp, result, grid=grid, config=config)
