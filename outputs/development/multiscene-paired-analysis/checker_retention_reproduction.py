#!/usr/bin/env python3
"""Offline checker-only retention contrast; no installed code or result changes."""
from collections import deque
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

SIM = Path('/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation')
spec = importlib.util.spec_from_file_location('check_demo', SIM / 'scripts/check_air_ground_pick_demo.py')
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


def pose(stamp, frame='map'):
    return SimpleNamespace(header=SimpleNamespace(frame_id=frame,
                                                  stamp=SimpleNamespace(to_sec=lambda: stamp)))


def contrast(lossless=False):
    monitor = checker.DemoMonitor(None, 3, 'pick_target')
    if lossless:
        monitor.air_observations = deque()  # in-memory counterfactual, not a runtime patch
    monitor.air_pose(pose(33.811))
    initial = checker.observation_summary(monitor.air_observations, 34.217, 33.811, 'map', 1.)
    for index in range(6000):
        monitor.air_pose(pose(34.0 + index / 25.))
    try:
        final = checker.observation_summary(monitor.air_observations, 34.217, 33.811, 'map', 1.)
        error = None
    except checker.DemoCheckError as caught:
        final, error = None, str(caught)
    return dict(initial=initial, final=final, error=error, retained_samples=len(monitor.air_observations))


def main():
    original, counterfactual = contrast(), contrast(True)
    assert original['error'] == 'no map observation matched status stamp 33.811000'
    assert counterfactual['final'] == original['initial']
    rejections = {}
    for label, sample, status_time in (
            ('wrong_frame', dict(stamp=33.811, frame='odom'), 34.217),
            ('wrong_stamp', dict(stamp=30., frame='map'), 34.217),
            ('stale_age', dict(stamp=33.811, frame='map'), 36.)):
        try:
            checker.observation_summary([sample], status_time, 33.811, 'map', 1.)
        except checker.DemoCheckError as caught:
            rejections[label] = str(caught)
    assert len(rejections) == 3
    result = dict(diagnostic_only=True, installed_code_changed=False, original_outcomes_changed=False,
                  original=original, in_memory_lossless_counterfactual=counterfactual,
                  unchanged_negative_checks=rejections,
                  limitation='Synthetic callback count/rate, not a reconstruction of slot3. Its checker buffer and aerial topic were not saved; eviction versus original subscription miss cannot be distinguished.')
    Path(__file__).with_name('checker-retention-reproduction.json').write_text(
        json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(exact_error_reproduced=True, negative_checks=3, runtime_fix_applied=False)))


if __name__ == '__main__':
    main()
