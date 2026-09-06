#!/usr/bin/env python3
"""Four A3 demonstrations using synthetic A1 candidates and A2 endpoints only."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np

from task_relevant_uncertainty import build_task_uncertainty
from task_relevant_uncertainty.outputs import field_summary, render_task_field, save_task_field
from task_relevant_uncertainty.synthetic import make_scenarios


def save_scene(scene, belief, directory, label):
    field = build_task_uncertainty(scene.a1, belief)
    save_task_field(field, directory)
    render_task_field(field, directory / 'field.png', title=f'SYNTHETIC A3 | {scene.name} | {label}')
    inputs = {
        'input_kind': 'synthetic_A1_candidates_and_A2_endpoints_not_real_IK_or_sensor_validation',
        'scene': scene.name, 'description': scene.description, 'grasp': scene.grasp.as_request(),
        'a1_result': scene.a1_result, 'a1_config': asdict(scene.a1.config), 'a2_config': asdict(belief.config),
    }
    (directory / 'inputs.json').write_text(json.dumps(inputs, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    np.savez_compressed(directory / 'inputs.npz', **{name: getattr(belief, name) for name in (
        'state', 'occupied_evidence', 'free_evidence', 'observation_count', 'unknown_score')})
    return field


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', type=Path, default=Path('outputs/a3'))
    args = parser.parse_args(argv)
    for scene in make_scenarios():
        directory = args.output_root / scene.name
        field = save_scene(scene, scene.a2, directory, scene.description)
        if scene.after_two is not None:
            save_scene(scene, scene.after_two, directory / 'after_two', 'FREE at N=2; score remains exp(-1)')
        summary = field_summary(field)
        print(json.dumps({'scene': scene.name, 'cells': summary['cells'],
                          'uncertainty_max': summary['uncertainty_max'],
                          'blocked_poses': sum(p.blocked for p in field.poses)}, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
