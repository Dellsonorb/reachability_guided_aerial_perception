"""Offline Matplotlib rendering for validated manipulation interest fields."""

from __future__ import annotations

from collections.abc import Mapping
import math
from numbers import Real
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg", force=True)

import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm, ListedColormap, Normalize
from matplotlib.cm import ScalarMappable
import numpy as np

from .field import candidate_relevance
from .model import CellState, FieldConfig, FieldStatus, GraspTCP, ManipulationInterestField


def _validate_candidate_list(evaluated_candidates: Any) -> list[Mapping[str, Any]]:
    if not isinstance(evaluated_candidates, list):
        raise ValueError("evaluated_candidates must be a list")
    candidates: list[Mapping[str, Any]] = []
    for index, candidate in enumerate(evaluated_candidates):
        if not isinstance(candidate, Mapping):
            raise ValueError(f"evaluated_candidates[{index}] must be a mapping")
        for name in ("bunker_x", "bunker_y"):
            value = candidate.get(name)
            if isinstance(value, bool) or not isinstance(value, Real):
                raise ValueError(f"evaluated_candidates[{index}].{name} must be a finite real")
            if not math.isfinite(float(value)):
                raise ValueError(f"evaluated_candidates[{index}].{name} must be a finite real")
        candidates.append(candidate)
    return candidates


def _validate_output_path(output_path: Any) -> Path:
    try:
        path = Path(output_path)
    except (TypeError, ValueError) as exc:
        raise ValueError("output_path must be a filesystem path") from exc
    if not str(path).strip() or path.name in (".", ".."):
        raise ValueError("output_path must name a PNG file")
    if path.suffix.lower() != ".png":
        raise ValueError("output_path must have a .png suffix")
    if path.exists() and path.is_dir():
        raise ValueError("output_path must name a file, not a directory")
    return path


def _set_xy_axes(axis: Any, extent: tuple[float, float, float, float]) -> None:
    axis.set_xlim(extent[0], extent[1])
    axis.set_ylim(extent[2], extent[3])
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")


def render_field(
    field: ManipulationInterestField,
    grasp: GraspTCP,
    evaluated_candidates: list[Mapping[str, Any]],
    output_path: str | Path,
    config: FieldConfig = FieldConfig(),
) -> Path:
    """Render a field and its evaluated candidates to a deterministic PNG.

    The renderer consumes immutable field arrays and candidate mappings without
    modifying either input.  It is deliberately ROS-independent and always uses
    Matplotlib's non-interactive Agg backend.
    """
    if not isinstance(field, ManipulationInterestField):
        raise ValueError("field must be a ManipulationInterestField")
    if not isinstance(grasp, GraspTCP):
        raise ValueError("grasp must be a GraspTCP")
    if field.grasp_id != grasp.grasp_id:
        raise ValueError("field and grasp grasp_id must match")
    if field.frame_id != grasp.frame_id or field.frame_id != "map":
        raise ValueError("field and grasp frame_id must be 'map'")
    if not isinstance(config, FieldConfig):
        raise ValueError("config must be a FieldConfig")
    if config != field.config:
        raise ValueError("config does not match the field's scoring config")
    candidates = _validate_candidate_list(evaluated_candidates)
    path = _validate_output_path(output_path)

    extent = (
        field.grid.origin_x,
        field.grid.origin_x + field.grid.width_m,
        field.grid.origin_y,
        field.grid.origin_y + field.grid.height_m,
    )
    state_values = np.asarray(field.cell_state, dtype=np.int8)
    relevance_values = np.asarray(field.relevance, dtype=float)
    raw_relevance = np.ma.masked_where(
        (state_values == CellState.UNASSESSED.value) | ~np.isfinite(relevance_values),
        relevance_values,
    )

    positive_x: list[float] = []
    positive_y: list[float] = []
    positive_scores: list[float] = []
    invalid_x: list[float] = []
    invalid_y: list[float] = []
    for candidate in candidates:
        x = float(candidate["bunker_x"])
        y = float(candidate["bunker_y"])
        score = candidate_relevance(candidate, config)
        if score > 0.0:
            positive_x.append(x)
            positive_y.append(y)
            positive_scores.append(score)
        else:
            invalid_x.append(x)
            invalid_y.append(y)

    figure, axes = plt.subplots(1, 3, figsize=(15, 5), constrained_layout=False)
    try:
        relevance_image = axes[0].imshow(
            raw_relevance,
            origin="lower",
            interpolation="none",
            extent=extent,
            vmin=0,
            vmax=1,
            cmap="viridis",
            aspect="equal",
        )
        axes[0].set_title("Raw relevance")
        _set_xy_axes(axes[0], extent)
        figure.colorbar(relevance_image, ax=axes[0], label="Relevance (0–1)")

        state_cmap = ListedColormap(("#bdbdbd", "#d95f02", "#1b9e77", "#7570b3"))
        state_norm = BoundaryNorm((-1.5, -0.5, 0.5, 1.5, 2.5), state_cmap.N)
        state_image = axes[1].imshow(
            state_values,
            origin="lower",
            interpolation="none",
            extent=extent,
            cmap=state_cmap,
            norm=state_norm,
            aspect="equal",
        )
        axes[1].set_title("Cell state")
        _set_xy_axes(axes[1], extent)
        state_bar = figure.colorbar(state_image, ax=axes[1], ticks=[state.value for state in CellState])
        state_bar.ax.set_yticklabels([state.name for state in CellState])

        if positive_scores:
            candidate_plot = axes[2].scatter(
                positive_x,
                positive_y,
                c=positive_scores,
                cmap="viridis",
                vmin=0,
                vmax=1,
                marker="o",
                s=45,
                edgecolors="black",
                linewidths=0.4,
                label="positive relevance",
            )
            figure.colorbar(candidate_plot, ax=axes[2], label="Candidate relevance (0–1)")
        else:
            candidate_norm = Normalize(vmin=0, vmax=1)
            candidate_map = ScalarMappable(norm=candidate_norm, cmap="viridis")
            candidate_map.set_array(np.asarray([], dtype=float))
            figure.colorbar(candidate_map, ax=axes[2], label="Candidate relevance (0–1)")
        if invalid_x:
            axes[2].scatter(
                invalid_x,
                invalid_y,
                color="gray",
                marker="x",
                s=45,
                linewidths=1.0,
                label="invalid / zero relevance",
            )
        axes[2].scatter(
            [grasp.position_xyz[0]],
            [grasp.position_xyz[1]],
            color="gold",
            edgecolors="black",
            marker="*",
            s=220,
            linewidths=1.2,
            zorder=5,
            label="Grasp TCP",
        )
        axes[2].set_title("Evaluated candidates")
        _set_xy_axes(axes[2], extent)
        axes[2].legend(loc="best", fontsize="small")
        if field.status is FieldStatus.NO_INVERSE_REACHABLE:
            axes[2].text(
                0.5,
                0.5,
                "NO_INVERSE_REACHABLE",
                transform=axes[2].transAxes,
                ha="center",
                va="center",
                color="black",
                fontweight="bold",
                bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none"},
                zorder=6,
            )

        coverage = field.coverage
        figure.suptitle(
            f"Validated manipulation interest field — {field.status.value} | "
            f"cell coverage {coverage.assessed_cell_fraction:.1%} "
            f"({coverage.evaluated_cells}/{coverage.total_cells})",
            y=0.99,
        )
        figure.tight_layout(rect=(0, 0, 1, 0.94))
        path.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(path, format="png", dpi=150)
    finally:
        plt.close(figure)
    return path
