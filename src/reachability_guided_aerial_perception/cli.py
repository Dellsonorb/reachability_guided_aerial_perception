"""Command-line boundary for the frozen RM4D manipulation-interest workflow."""

from __future__ import annotations

import argparse
from collections.abc import Iterator, Sequence
from contextlib import contextmanager
import importlib
import json
from pathlib import Path
import sys
from typing import Any

from .field import build_field_from_result
from .model import FieldConfig, GraspTCP, GridSpec, ManipulationInterestField
from .outputs import field_summary, save_field_bundle
from .visualization import render_field


_GRASP_KEYS = frozenset(
    {"grasp_id", "frame_id", "position_xyz", "quaternion_xyzw"}
)


def load_grasp(path: str | Path) -> GraspTCP:
    """Load an already map-resolved grasp TCP with no planner-bias inputs."""
    try:
        grasp_path = Path(path)
        payload = json.loads(grasp_path.read_text())
    except (OSError, TypeError, json.JSONDecodeError) as exc:
        raise ValueError("grasp path must contain valid JSON") from exc
    if not isinstance(payload, dict) or set(payload) != _GRASP_KEYS:
        raise ValueError(
            "grasp JSON must contain exactly grasp_id, frame_id, "
            "position_xyz, and quaternion_xyzw"
        )
    return GraspTCP(
        grasp_id=payload["grasp_id"],
        frame_id=payload["frame_id"],
        position_xyz=payload["position_xyz"],
        quaternion_xyzw=payload["quaternion_xyzw"],
    )


def _existing_directory(path: str | Path, name: str) -> Path:
    try:
        resolved = Path(path).expanduser().resolve()
    except (OSError, TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a directory path") from exc
    if not resolved.is_dir():
        raise ValueError(f"{name} must be an existing directory")
    return resolved


def _existing_file(path: str | Path, name: str) -> Path:
    try:
        resolved = Path(path).expanduser().resolve()
    except (OSError, TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a file path") from exc
    if not resolved.is_file():
        raise ValueError(f"{name} must be an existing file")
    return resolved


@contextmanager
def open_frozen_rm4d_api(
    rm4d_root: str | Path,
    config_path: str | Path,
    map_path: str | Path,
) -> Iterator[Any]:
    """Lazily import and open the frozen repository's ``BasePlacementAPI``."""
    root = _existing_directory(rm4d_root, "rm4d_root")
    config = _existing_file(config_path, "config_path")
    reachability_map = _existing_file(map_path, "map_path")
    root_string = str(root)
    inserted = root_string not in sys.path
    if inserted:
        sys.path.insert(0, root_string)
    try:
        module = importlib.import_module("rm4d")
        base_placement_api = module.BasePlacementAPI
        api = base_placement_api.from_files(str(config), str(reachability_map))
    finally:
        if inserted:
            sys.path.remove(root_string)

    if callable(getattr(api, "__enter__", None)) and callable(
        getattr(api, "__exit__", None)
    ):
        with api as active_api:
            yield active_api
        return

    try:
        yield api
    finally:
        close = getattr(api, "close", None)
        if callable(close):
            close()


def run_scenario(
    api: Any,
    grasp: GraspTCP,
    output_dir: str | Path,
    *,
    grid_width_m: float = 3.0,
    grid_height_m: float = 3.0,
    grid_resolution_m: float = 0.1,
    config: FieldConfig = FieldConfig(),
) -> tuple[ManipulationInterestField, dict[str, Any]]:
    """Call RM4D exactly once and persist one validated field."""
    grid = GridSpec.centered(
        grasp.position_xyz[:2],
        grid_width_m,
        grid_height_m,
        grid_resolution_m,
    )
    result = api.plan(grasp.as_request(), top_k=1)
    field = build_field_from_result(grasp, result, grid=grid, config=config)
    evaluated_candidates = result["evaluated_candidates"]
    save_field_bundle(field, evaluated_candidates, output_dir, config=config)
    render_field(
        field,
        grasp,
        evaluated_candidates,
        Path(output_dir) / "field.png",
        config=config,
    )
    return field, field_summary(field, config)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build one validated manipulation interest field from frozen RM4D",
    )
    parser.add_argument("--rm4d-root", required=True)
    parser.add_argument("--rm4d-config", required=True)
    parser.add_argument("--rm4d-map", required=True)
    parser.add_argument("--grasp", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--grid-width-m", type=float, default=3.0)
    parser.add_argument("--grid-height-m", type=float, default=3.0)
    parser.add_argument("--grid-resolution-m", type=float, default=0.1)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    grasp = load_grasp(args.grasp)
    with open_frozen_rm4d_api(
        args.rm4d_root,
        args.rm4d_config,
        args.rm4d_map,
    ) as api:
        _, summary = run_scenario(
            api,
            grasp,
            args.output_dir,
            grid_width_m=args.grid_width_m,
            grid_height_m=args.grid_height_m,
            grid_resolution_m=args.grid_resolution_m,
        )
    print(json.dumps(summary, allow_nan=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
