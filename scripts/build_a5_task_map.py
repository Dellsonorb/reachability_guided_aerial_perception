#!/usr/bin/env python3
"""Build the bounded A5 task-domain RM4D asset."""

import argparse
import json
from pathlib import Path

from sim_active_perception.task_map import build_task_map


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rm4d-root', type=Path, required=True)
    parser.add_argument('--rm4d-config', type=Path, required=True)
    parser.add_argument('--rm4d-map', type=Path, required=True)
    parser.add_argument('--calibration-json', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=100000)
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args(argv)
    metadata = build_task_map(
        args.rm4d_root, args.rm4d_config, args.rm4d_map,
        args.calibration_json, args.output_dir, args.samples, args.seed)
    print(json.dumps(metadata, indent=2, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
