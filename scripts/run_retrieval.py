#!/usr/bin/env python3
"""One current-development sensor-driven retrieval task, or its recorded status.

This is a thin entry over the existing SIM runner, not a matrix/mission engine.
Scene geometry initializes simulation only; the algorithm redetects the brick.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import time

from run_a6_attempt import ROOT, SIM, RM, PX4, slot_spec


def load_scene(path, scene_id):
    data = json.loads(path.read_text())
    if 'scenes' not in data:
        if scene_id is not None and scene_id != data['id']:
            raise ValueError('scene ID does not match the scene file')
        return data
    matches = [s for s in data['scenes'] if s['id'] == scene_id]
    if len(matches) != 1:
        raise ValueError('select exactly one explicit --scene-id from this scene collection')
    return matches[0]


def task_config(profile, scene, method):
    if profile['status'] != 'DEVELOPMENT_BATCH' or method not in ('generic', 'ours', 'confirmation', 'deficit'):
        raise ValueError('current entry is development only: Generic, Ours, confirmation or deficit')
    config = copy.deepcopy(profile)
    config.update(scenes=[copy.deepcopy(scene)], slots=[dict(slot=1, scene=scene['id'], method=method)])
    return config


def task_command(sim_root, rm_root, config_path, output, slot=1):
    return [str(sim_root/'scripts/with_p450_env.bash'), '/usr/bin/python3',
            str(ROOT/'scripts/run_a6_attempt.py'), '--config', str(config_path),
            '--slot', str(slot), '--output-dir', str(output), '--sim-root', str(sim_root),
            '--rm4d-root', str(rm_root), '--integrated-joint-velocity',
            '--full-robot-manipulation', '--execution-clearance', '--ground-dynamics']


def read_metadata(path, errors):
    if not path.exists():
        return {}
    for retry in range(3):
        try:
            value = json.loads(path.read_text())
            if not isinstance(value, dict):
                raise ValueError('expected a JSON object')
            return value
        except (OSError, ValueError) as error:
            if retry < 2:
                time.sleep(.01)
            else:
                errors.append('%s: %s' % (path.name, error))
    return {}


def task_status(output):
    attempt_dir = output/'attempt'
    errors = []
    attempt = read_metadata(attempt_dir/'attempt.json', errors)
    entry = read_metadata(output/'entry.json', errors)
    events = []
    path = attempt_dir/'data/events.jsonl'
    if path.exists():
        for line in path.read_text().splitlines():
            try:
                events.append(json.loads(line))
            except ValueError:
                continue  # an active append can have an unfinished last row
    windows = [e for e in events if e.get('state') == 'A6_ENV_RESULT']
    failure = next((e.get('reason') for e in events if e.get('state') == 'FAILED'), None)
    return dict(run_status='RECORD_INCOMPLETE' if errors else
                'FINISHED' if 'finish_wall' in attempt or 'finish_wall' in entry else
                'IN_PROGRESS' if attempt or entry else 'NOT_STARTED',
                trial_status=attempt.get('status'),
                last_state=events[-1].get('state') if events else None,
                completed_windows=windows[-1]['round'] if windows else 0,
                confirmed_candidates=windows[-1]['confirmed_candidate_count'] if windows else None,
                retrieval_success=attempt.get('retrieval_success'),
                first_failure=failure or attempt.get('reason') or attempt.get('classification_reason') or entry.get('error'),
                launcher_exit_code=entry.get('exit_code'),
                record_errors=errors,
                output_dir=str(output))


def wait_for_launcher(process):
    try:
        return process.wait()
    except KeyboardInterrupt:
        # The runner is in its own session. Forward once, then let its existing
        # finally block stop its owned ROS/Gazebo children. subprocess.run would
        # kill this runner while its child-session cleanup is still in progress.
        try:
            process.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass
        while True:
            try:
                process.wait()
                return 130
            except KeyboardInterrupt:
                continue  # don't interrupt the runner's cleanup a second time


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest='action', required=True)
    status = actions.add_parser('status', help='read current/finished state without contacting robots')
    status.add_argument('output_dir', type=Path)
    run = actions.add_parser('run', help='start exactly one fresh SIM task; never retry automatically')
    run.add_argument('--scene-file', type=Path, default=ROOT/'examples/sim/natural.json')
    run.add_argument('--scene-id', help='required for a multi-scene JSON collection')
    run.add_argument('--method', choices=('generic', 'ours', 'confirmation', 'deficit'), default='ours')
    run.add_argument('--output-dir', type=Path, required=True, help='new directory; never overwrites a task')
    run.add_argument('--sim-root', type=Path, default=SIM)
    run.add_argument('--rm4d-root', type=Path, default=RM)
    run.add_argument('--dry-run', action='store_true', help='show common profile/command without writing or starting SIM')
    evaluation = actions.add_parser('evaluation', help='one predeclared evaluation slot; not a matrix or authorization')
    evaluation.add_argument('--config', type=Path, required=True)
    evaluation.add_argument('--slot', type=int, required=True)
    evaluation.add_argument('--output-dir', type=Path, required=True)
    evaluation.add_argument('--sim-root', type=Path, default=SIM)
    evaluation.add_argument('--rm4d-root', type=Path, default=RM)
    evaluation.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    output = args.output_dir.expanduser().resolve()
    if args.action == 'status':
        print(json.dumps(task_status(output), indent=2))
        return 0
    if args.action == 'evaluation':
        config = json.loads(args.config.read_text())
        if (config.get('status') != 'FROZEN_FOR_EVALUATION' or
                config.get('cohort') != 'paper1-eval-finite-scan-v1-final'):
            raise ValueError('requires the named evaluation cohort; old formal slots cannot be promoted')
        slot, scene = slot_spec(config, args.slot)
    else:
        config = task_config(json.loads((ROOT/'configs/current_sim_task.json').read_text()),
                             load_scene(args.scene_file, args.scene_id), args.method)
        slot, scene = config['slots'][0], config['scenes'][0]
    sim_root, rm_root = args.sim_root.resolve(), args.rm4d_root.resolve()
    command = task_command(sim_root, rm_root, output/'task.json', output/'attempt',slot=slot['slot'])
    print('PROFILE', config['profile'], 'METHOD', slot['method'], 'SCENE', scene['id'], flush=True)
    print('COMMAND', shlex.join(command), flush=True)
    if args.dry_run:
        print(json.dumps(config, indent=2))
        return 0
    if output.exists():
        raise FileExistsError('task output already exists: %s' % output)
    # Dedicated existing runtime ports: never attach to or clean another task.
    for port in (11951, 11952):
        with socket.socket() as connection:
            if connection.connect_ex(('127.0.0.1', port)) == 0:
                parser.error('task port %d is in use; stop its owner before launching' % port)
    environment = os.environ.copy()
    environment.setdefault('P450_PX4_ROOT', PX4)
    output.mkdir(parents=True, exist_ok=False)
    entry = dict(start_wall=time.time(), command=command, log=str(output/'launcher.log'))
    if args.action == 'evaluation':
        entry.update(kind='EVALUATION_ENTRY',cohort=config['cohort'],slot=slot['slot'],
                     scene=scene['id'],seed=scene['seed'],method=slot['method'])
    (output/'entry.json').write_text(json.dumps(entry, indent=2)+'\n')
    print('STATUS: python3 scripts/run_retrieval.py status '+shlex.quote(str(output)), flush=True)
    try:
        config['runtime_versions'] = {name: subprocess.check_output(
            ['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()
            for name, path in (('agent', ROOT), ('sim', sim_root), ('rm4d', rm_root))}
        (output/'task.json').write_text(json.dumps(config, indent=2)+'\n')
        with (output/'launcher.log').open('w') as log:
            process = subprocess.Popen(command, env=environment, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            entry['exit_code'] = wait_for_launcher(process)
        if entry['exit_code']:
            entry['error'] = 'platform launcher exited %d; see launcher.log' % entry['exit_code']
    except (OSError, subprocess.SubprocessError) as error:
        entry.update(exit_code=2, error='%s: %s' % (type(error).__name__, error))
    except KeyboardInterrupt:
        entry.update(exit_code=130, error='task launcher interrupted by operator')
    finally:
        entry['finish_wall'] = time.time()
        (output/'entry.json').write_text(json.dumps(entry, indent=2)+'\n')
    summary = task_status(output)
    print(json.dumps(summary, indent=2), flush=True)
    return 0 if summary['retrieval_success'] is True else entry['exit_code'] or 1


if __name__ == '__main__':
    raise SystemExit(main())
