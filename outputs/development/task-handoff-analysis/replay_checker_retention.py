#!/usr/bin/env python3
"""Replay real aerial callbacks into old/new bounded-task checker storage."""
from collections import deque
import importlib.util
import json
from pathlib import Path
import rosbag

HERE = Path(__file__).resolve().parent
SIM = Path('/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation')
spec = importlib.util.spec_from_file_location('task_checker', SIM / 'scripts/check_air_ground_pick_demo.py')
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


def main():
    directory = HERE.parent / 'task-handoff/launch-01-moderate-generic/attempt'
    physical = json.loads((directory / 'physical_summary.json').read_text())
    accepted = physical['observations']['aerial']
    old, current = [checker.DemoMonitor(None, 3, 'pick_target') for _ in range(2)]
    old.air_observations = deque(maxlen=5000)
    count, later = 0, 0
    with rosbag.Bag(str(directory / 'diagnostics.bag')) as bag:
        for _, message, _ in bag.read_messages(topics=['/air_observer/target_pose']):
            count += 1
            later += message.header.stamp.to_sec() > accepted['stamp'] + .05
            old.air_pose(message)
            current.air_pose(message)
    args = (accepted['stamp'] + accepted['age_s'], accepted['stamp'], 'map', 1.)
    try:
        checker.observation_summary(old.air_observations, *args)
        raise AssertionError('old bounded deque unexpectedly retained this early handoff')
    except checker.DemoCheckError as error:
        old_error = str(error)
    now = checker.observation_summary(current.air_observations, *args)
    result = dict(diagnostic_only=True, original_results_unchanged=True,
        input='launch-01-moderate-generic/attempt/diagnostics.bag:/air_observer/target_pose',
        status_time_source='accepted physical_summary observation stamp plus recorded age',
        received=count, later_than_handoff_tolerance=later,
        old_retained=len(old.air_observations), old_error=old_error,
        current_retained=len(current.air_observations), current_match=now)
    (HERE / 'checker-replay.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
