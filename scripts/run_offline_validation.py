#!/usr/bin/env python3
"""Run the three approved A1 scenarios against one frozen RM4D instance."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import json
from pathlib import Path

import numpy as np

from reachability_guided_aerial_perception import CellState, FieldStatus
from reachability_guided_aerial_perception.cli import (
    load_grasp,
    open_frozen_rm4d_api,
    run_scenario,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SCENARIO_NAMES = ("nominal", "boundary", "no_inverse")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the three A1 frozen-RM4D offline validation scenarios",
    )
    parser.add_argument("--rm4d-root", required=True)
    parser.add_argument("--rm4d-config", required=True)
    parser.add_argument("--rm4d-map", required=True)
    parser.add_argument("--output-root", required=True)
    return parser


def _validate_scenario(name, field) -> None:
    if name == "nominal":
        assert field.status is FieldStatus.PARTIALLY_ASSESSED
        assert np.any(field.cell_state == CellState.HIGH)
    elif name == "boundary":
        assert field.status is FieldStatus.PARTIALLY_ASSESSED
        assert field.coverage.evaluated_cells > 0
    elif name == "no_inverse":
        assert field.status is FieldStatus.NO_INVERSE_REACHABLE
        assert field.coverage.evaluated_candidates == 0


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    output_root = Path(args.output_root)
    with open_frozen_rm4d_api(
        args.rm4d_root,
        args.rm4d_config,
        args.rm4d_map,
    ) as api:
        for name in SCENARIO_NAMES:
            grasp = load_grasp(
                REPOSITORY_ROOT / "examples" / "scenarios" / f"{name}.json"
            )
            field, summary = run_scenario(api, grasp, output_root / name)
            _validate_scenario(name, field)
            print(
                json.dumps(
                    {
                        "scenario": name,
                        "status": summary["status"],
                        "evaluated_candidates": summary["coverage"]["evaluated"],
                        "evaluated_cells": summary["coverage"]["assessed_cells"],
                        "high_cells": summary["cells"]["high"],
                    },
                    allow_nan=False,
                    sort_keys=True,
                )
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
