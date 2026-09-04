"""Portable serialization and visualization payloads for interest fields."""

from __future__ import annotations

import csv
from collections.abc import Mapping
from dataclasses import fields as dataclass_fields
import json
from pathlib import Path
from typing import Any

import numpy as np

from .field import build_field_from_result, candidate_relevance
from .model import (
    CellState,
    FieldConfig,
    FieldStatus,
    GraspTCP,
    ManipulationInterestField,
)


_CANDIDATE_COLUMNS = (
    "candidate_id",
    "bunker_x",
    "bunker_y",
    "bunker_yaw",
    "rm4d_reachable",
    "ik_valid",
    "collision_free",
    "footprint_collision",
    "joint_margin_rad",
    "fk_position_residual_m",
    "fk_orientation_residual_rad",
    "field_relevance",
    "valid",
    "rejection_reason",
)
_RAW_CANDIDATE_COLUMNS = tuple(name for name in _CANDIDATE_COLUMNS if name != "field_relevance")


def _require_field(field: Any) -> ManipulationInterestField:
    if not isinstance(field, ManipulationInterestField):
        raise ValueError("field must be a ManipulationInterestField")
    return field


def _require_config(config: Any) -> FieldConfig:
    if not isinstance(config, FieldConfig):
        raise ValueError("config must be a FieldConfig")
    return config


def _validate_candidates(evaluated_candidates: Any) -> list[Mapping[str, Any]]:
    if not isinstance(evaluated_candidates, list):
        raise ValueError("evaluated_candidates must be a list")
    if not all(isinstance(candidate, Mapping) for candidate in evaluated_candidates):
        raise ValueError("each evaluated candidate must be a mapping")
    for candidate in evaluated_candidates:
        missing = [name for name in _RAW_CANDIDATE_COLUMNS if name not in candidate]
        if missing:
            raise ValueError(f"candidate missing required fields: {', '.join(missing)}")
    return evaluated_candidates


def _candidate_rejection_counts(candidates: list[Mapping[str, Any]]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for candidate in candidates:
        if candidate["valid"] is True:
            continue
        reason = candidate["rejection_reason"]
        if not isinstance(reason, str):
            raise ValueError("invalid candidate must have a string rejection_reason")
        counts[reason] = counts.get(reason, 0) + 1
    return counts


def _validate_field_raster(
    field: ManipulationInterestField,
    candidates: list[Mapping[str, Any]],
    config: FieldConfig,
) -> None:
    coverage = field.coverage
    if len(candidates) != coverage.evaluated_candidates:
        raise ValueError("candidate count does not match field coverage")
    valid_count = sum(candidate["valid"] is True for candidate in candidates)
    if valid_count != coverage.valid_candidates:
        raise ValueError("candidate valid count does not match field coverage")
    if _candidate_rejection_counts(candidates) != dict(coverage.rejected_by_reason):
        raise ValueError("candidate rejection counts do not match field coverage")
    synthetic_grasp = GraspTCP(field.grasp_id, field.frame_id, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    reconstructed = build_field_from_result(
        synthetic_grasp,
        {
            "schema_version": 1,
            "frame_id": field.frame_id,
            "grasp_id": field.grasp_id,
            "summary": {
                "inverse_reachable": coverage.inverse_reachable,
                "deduplicated": coverage.deduplicated_candidates,
                "validation_limit": coverage.validation_limit,
                "evaluated": coverage.evaluated_candidates,
                "valid": coverage.valid_candidates,
                "rejected_by_reason": dict(coverage.rejected_by_reason),
            },
            "evaluated_candidates": candidates,
        },
        grid=field.grid,
        config=config,
    )
    for actual, expected, name in (
        (reconstructed.relevance, field.relevance, "relevance"),
        (reconstructed.cell_state, field.cell_state, "cell_state"),
        (reconstructed.evaluated_count, field.evaluated_count, "evaluated_count"),
        (reconstructed.feasible_count, field.feasible_count, "valid_count"),
    ):
        if not np.array_equal(actual, expected, equal_nan=True):
            raise ValueError(f"provided candidates/config do not reproduce field {name}")


def _csv_value(value: Any) -> Any:
    """Convert NumPy scalar values while retaining CSV's empty-null convention."""
    if value is None:
        return ""
    if isinstance(value, np.generic):
        return value.item()
    return value


def to_occupancy_grid_payload(field: ManipulationInterestField) -> dict[str, Any]:
    """Convert a field into a ROS-independent occupancy-grid wire dictionary."""
    field = _require_field(field)
    data = np.full(field.grid.shape, CellState.UNASSESSED.value, dtype=np.int8)
    if field.status is not FieldStatus.NO_INVERSE_REACHABLE:
        data[field.cell_state == CellState.INFEASIBLE.value] = CellState.INFEASIBLE.value
        for state in (CellState.LOW, CellState.HIGH):
            mask = field.cell_state == state.value
            if not np.any(mask):
                continue
            values = np.asarray(field.relevance[mask], dtype=float)
            finite = np.isfinite(values)
            converted = np.ones(values.shape, dtype=np.int8)
            if np.any(finite):
                converted[finite] = np.clip(np.rint(values[finite] * 100.0), 1, 100).astype(np.int8)
            data[mask] = converted
    return {
        "header": {"frame_id": field.frame_id},
        "info": {
            "resolution": float(field.grid.resolution_m),
            "width": int(field.grid.width_cells),
            "height": int(field.grid.height_cells),
            "origin": {
                "position": {
                    "x": float(field.grid.origin_x),
                    "y": float(field.grid.origin_y),
                    "z": 0.0,
                },
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        },
        "field_status": field.status.value,
        "data": [int(value) for value in data.reshape(-1, order="C")],
    }


def field_summary(field: ManipulationInterestField, config: FieldConfig) -> dict[str, Any]:
    """Return a JSON-safe description of field semantics, geometry, and coverage."""
    field = _require_field(field)
    config = _require_config(config)
    coverage_values = {
        item.name: getattr(field.coverage, item.name)
        for item in dataclass_fields(field.coverage)
    }
    coverage = {
        "inverse_reachable": coverage_values["inverse_reachable"],
        "deduplicated": coverage_values["deduplicated_candidates"],
        "validation_limit": coverage_values["validation_limit"],
        "evaluated": coverage_values["evaluated_candidates"],
        "valid": coverage_values["valid_candidates"],
        "rejected_by_reason": dict(coverage_values["rejected_by_reason"]),
        "assessed_cells": coverage_values["evaluated_cells"],
        "total_cells": coverage_values["total_cells"],
        "coverage_fraction": coverage_values["assessed_cell_fraction"],
    }
    cells = {
        state.name.lower(): int(np.count_nonzero(field.cell_state == state.value))
        for state in CellState
    }
    return {
        "schema_version": 1,
        "field_type": "validated_manipulation_interest",
        "grasp_id": field.grasp_id,
        "frame_id": field.frame_id,
        "status": field.status.value,
        "relevance_semantics": "joint_margin_only",
        "scoring": {
            "minimum_margin_rad": config.minimum_joint_margin_rad,
            "saturation_margin_rad": config.joint_margin_saturation_rad,
            "high_threshold": config.high_relevance_threshold,
            "residual_role": "diagnostic_only",
        },
        "grid": {
            "origin_x_m": field.grid.origin_x,
            "origin_y_m": field.grid.origin_y,
            "resolution_m": field.grid.resolution_m,
            "width": field.grid.width_cells,
            "height": field.grid.height_cells,
            "shape": list(field.grid.shape),
            "convention": "row-major [y, x]; q=(base_link x, base_link y) in map",
        },
        "coverage": coverage,
        "cells": cells,
    }


def save_field_bundle(
    field: ManipulationInterestField,
    evaluated_candidates: list[Mapping[str, Any]],
    output_dir: str | Path,
    config: FieldConfig = FieldConfig(),
) -> dict[str, Path]:
    """Write the field arrays, summary, and candidate diagnostics to a directory."""
    field = _require_field(field)
    config = _require_config(config)
    candidates = _validate_candidates(evaluated_candidates)
    _validate_field_raster(field, candidates, config)
    try:
        output_path = Path(output_dir)
    except (TypeError, ValueError) as exc:
        raise ValueError("output_dir must be path-like") from exc
    output_path.mkdir(parents=True, exist_ok=True)

    field_path = output_path / "field.npz"
    np.savez(
        field_path,
        relevance=np.array(field.relevance, copy=True),
        state=np.array(field.cell_state, copy=True),
        evaluated_count=np.array(field.evaluated_count, copy=True),
        valid_count=np.array(field.feasible_count, copy=True),
        grasp_id=np.array(field.grasp_id),
        frame_id=np.array(field.frame_id),
        status=np.array(field.status.value),
        origin_x_m=np.array(field.grid.origin_x),
        origin_y_m=np.array(field.grid.origin_y),
        resolution_m=np.array(field.grid.resolution_m),
        width=np.array(field.grid.width_cells, dtype=np.int64),
        height=np.array(field.grid.height_cells, dtype=np.int64),
    )

    summary_path = output_path / "summary.json"
    summary_path.write_text(
        json.dumps(field_summary(field, config), allow_nan=False, indent=2, sort_keys=True) + "\n"
    )

    candidates_path = output_path / "candidate_diagnostics.csv"
    with candidates_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=_CANDIDATE_COLUMNS)
        writer.writeheader()
        for candidate in candidates:
            row = {
                "candidate_id": _csv_value(candidate.get("candidate_id")),
                "bunker_x": _csv_value(candidate.get("bunker_x")),
                "bunker_y": _csv_value(candidate.get("bunker_y")),
                "bunker_yaw": _csv_value(candidate.get("bunker_yaw")),
                "rm4d_reachable": _csv_value(candidate.get("rm4d_reachable")),
                "ik_valid": _csv_value(candidate.get("ik_valid")),
                "collision_free": _csv_value(candidate.get("collision_free")),
                "footprint_collision": _csv_value(candidate.get("footprint_collision")),
                "joint_margin_rad": _csv_value(candidate.get("joint_margin_rad")),
                "fk_position_residual_m": _csv_value(candidate.get("fk_position_residual_m")),
                "fk_orientation_residual_rad": _csv_value(candidate.get("fk_orientation_residual_rad")),
                "field_relevance": candidate_relevance(candidate, config),
                "valid": _csv_value(candidate.get("valid")),
                "rejection_reason": _csv_value(candidate.get("rejection_reason")),
            }
            writer.writerow(row)

    return {"field": field_path, "summary": summary_path, "candidates": candidates_path}
