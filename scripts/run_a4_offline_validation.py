#!/usr/bin/env python3
"""Exactly three synthetic A4 groups: task weighting, occlusion and flight cost."""

import argparse
import json
from pathlib import Path

from reachability_guided_nbv import rank_viewpoints
from reachability_guided_nbv.outputs import render_occlusion_rays, render_result, save_result
from reachability_guided_nbv.synthetic import make_scenarios


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('outputs/a4'))
    args = parser.parse_args(argv)
    records = []
    for scene in make_scenarios():
        for case in scene.cases:
            result = rank_viewpoints(case.task, case.belief, case.current,
                                     candidates=case.candidates, config=case.config)
            directory = args.output / scene.name / case.name
            save_result(result, case.task, case.belief, directory)
            render_result(result, case.task, case.belief, directory / 'nbv.png',
                           title=f'SYNTHETIC A4 | {scene.name} | {case.name}')
            record = {'scene': scene.name, 'case': case.name, 'description': scene.description,
                      'best_task_id': None if result.best_task is None else result.best_task.candidate_id,
                      'best_generic_id': None if result.best_generic is None else result.best_generic.candidate_id,
                      'candidates': [{'id': c.candidate_id, 'task_gain': c.task_gain,
                                      'generic_gain': c.generic_gain, 'cost': c.flight_cost,
                                      'task_score': c.task_score} for c in result.candidates]}
            records.append(record)
            print(json.dumps(record, allow_nan=False))
    render_occlusion_rays(args.output / 'occlusion' / 'through_vs_above.png')
    (args.output / 'summary.json').write_text(json.dumps(records, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
