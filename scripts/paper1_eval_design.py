#!/usr/bin/python3
"""Generate the approved independent cohort or print ONE command. Never launch."""

import argparse
import copy
import json
import math
from pathlib import Path
import random
import shlex

ROOT = Path(__file__).resolve().parents[1]
COHORT = 'paper1-eval-finite-scan-v1-final'
STATUS = 'FROZEN_FOR_EVALUATION'
TIERS = ('easy', 'moderate', 'hard')


def build_config(profile, excluded):
    """The approved RNG streams; no outcome, ranking, candidate or sensor input."""
    config = copy.deepcopy(profile)
    config.update(status=STATUS, cohort=COHORT,
                  protocol='docs/PAPER1_FORMAL_EXPERIMENT_PLAN.md',
                  scene_count=96, planned_tasks=212, attempt_limit=224,
                  generation=dict(tier_rng=2026091021, scene_rng=2026091022,
                                  order_rng=2026091023, geometry='six_original_draws_then_wall_xy_yaw_v1'),
                  limits=dict(primary_scenes=96, planned_tasks=212, reserve_starts=12,
                              total_starts=224, max_replacements_per_slot=1,
                              expected_wall_hours=[24,36], wall_clock_hard_hours=120,
                              new_data_gib=500, minimum_free_gib=100),
                  execution_authorization='NOT_GRANTED: preparation only; separate user authorization required',
                  historical_seed_count=len(set(excluded)), scenes=[], slots=[])
    tier_rng, seed_rng, order_rng = (random.Random(k) for k in (2026091021,2026091022,2026091023))
    used = set(excluded)
    tier_counts = dict.fromkeys(TIERS, 0)
    for number in range(1,97):
        tier = tier_rng.choice(TIERS)
        seed = seed_rng.randrange(1,2**31)
        while seed in used:
            seed = seed_rng.randrange(1,2**31)
        used.add(seed)
        rng = random.Random(seed)
        tier_counts[tier] += 1
        scene = dict(id='eval-%s-%03d' % (tier,tier_counts[tier]), tier=tier,
                     generation_index=number, tier_index=tier_counts[tier], seed=seed,
                     target_xy=[2+rng.uniform(-.15,.15),rng.uniform(-.15,.15)],
                     target_yaw=rng.uniform(-math.pi/6,math.pi/6), target_z=.0575,
                     bunker_xy=[3+rng.uniform(-.1,.1),-2.5+rng.uniform(-.1,.1)],
                     bunker_yaw=math.pi+rng.uniform(-math.pi/36,math.pi/36), bunker_z=.36,
                     boxes=[], context_control=tier_counts[tier] <= 2,
                     ablation=tier == 'hard' and tier_counts[tier] <= 4,
                     retain_success_rgbd=tier_counts[tier] <= 2)
        centers = {'easy': [], 'moderate': [[.4,.65]], 'hard': [[.3,.5],[.5,-.4]]}[tier]
        for index, center in enumerate(centers):
            scene['boxes'].append(dict(name='eval_wall_%d' % index,
                center_xy=[center[0]+rng.uniform(-.15,.15),center[1]+rng.uniform(-.15,.15)],
                size_xyz=[.15,1. if tier == 'moderate' else .6,1.],
                yaw=rng.uniform(-math.pi/12,math.pi/12)))
        config['scenes'].append(scene)
    # Fix the order convention explicitly: shuffle scene blocks first; then
    # assign balanced first-method labels in generation order within each tier.
    blocks = config['scenes'][:]
    order_rng.shuffle(blocks)
    first_methods = {}
    for tier in TIERS:
        scenes = [s for s in config['scenes'] if s['tier'] == tier]
        labels = ['generic']*(len(scenes)//2) + ['ours']*(len(scenes)-len(scenes)//2)
        order_rng.shuffle(labels)
        first_methods.update((s['id'],label) for s,label in zip(scenes,labels))
    for scene in blocks:
        first = first_methods[scene['id']]
        methods = [first, 'ours' if first == 'generic' else 'generic']
        auxiliary = (['rm4d_only','fixed'] if scene['context_control'] else [])
        if scene['ablation']:
            auxiliary += ['no_occlusion','no_cost']
        order_rng.shuffle(auxiliary)
        for method in methods+auxiliary:
            role = ('primary' if method in ('generic','ours') else
                    'secondary_baseline' if method in ('rm4d_only','fixed') else 'secondary_ablation')
            config['slots'].append(dict(slot=len(config['slots'])+1,scene=scene['id'],
                                        method=method, comparison_role=role))
    config['tier_counts'] = tier_counts
    return config


def slot_command(config, config_path, slot_number, results_dir):
    """Read-only explicit single-slot command; never a loop or a launch."""
    if config.get('status') != STATUS or config.get('cohort') != COHORT:
        raise ValueError('only the named evaluation cohort is supported')
    slot = next((s for s in config['slots'] if s['slot'] == slot_number), None)
    if slot is None:
        raise ValueError('slot is not in the predeclared task list')
    version = config.get('evaluation_version') or json.loads((ROOT/'configs/evaluation_version.json').read_text())
    output = Path(results_dir)/('slot-%03d-%s-%s' % (slot_number,slot['scene'],slot['method']))
    sim, rm = Path(version['sim_root']), Path(version['rm4d_root'])
    return ['/usr/bin/python3',str(ROOT/'scripts/run_retrieval.py'),'evaluation',
            '--config',str(Path(config_path).resolve()),'--slot',str(slot_number),
            '--output-dir',str(output.resolve()),'--sim-root',str(sim),'--rm4d-root',str(rm)]


def write_config(path, config):
    path = Path(path)
    with path.open('x') as stream:
        stream.write(json.dumps(config,indent=2,allow_nan=False)+'\n')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest='action',required=True)
    generate = actions.add_parser('generate')
    generate.add_argument('--output',type=Path,required=True)
    generate.add_argument('--excluded',type=Path,default=ROOT/'configs/paper1_eval_excluded_seeds.json')
    preview = actions.add_parser('preview')
    preview.add_argument('--config',type=Path,default=ROOT/'configs/paper1_eval.json')
    preview.add_argument('--slot',type=int,required=True)
    preview.add_argument('--results-dir',type=Path,default=ROOT/'outputs/paper1-final-eval-v1')
    args = parser.parse_args(argv)
    if args.action == 'preview':
        config = json.loads(args.config.read_text())
        print('PREVIEW ONLY — requires separate execution authorization')
        print(shlex.join(slot_command(config,args.config,args.slot,args.results_dir)))
    else:
        excluded = json.loads(args.excluded.read_text())['seed_values']
        config = build_config(json.loads((ROOT/'configs/current_sim_task.json').read_text()),excluded)
        config['evaluation_version'] = json.loads((ROOT/'configs/evaluation_version.json').read_text())
        # Reporting metadata only. Runtime decisions still use public TF.
        from a6_scene import rotation_rpy
        import xml.etree.ElementTree as ET
        launch = Path(config['evaluation_version']['sim_root'])/'src/platform/sim_platform_bringup/launch/p450_runtime.launch'
        params = {p.get('name'):p.get('value') for p in ET.parse(launch).iter('param')}
        transform = [[0.,0.,0.,0.] for _ in range(4)]
        rotation = rotation_rpy(*[float(params['Lidar/offset_'+a]) for a in ('roll','pitch','yaw')])
        for i,axis in enumerate(('x','y','z')):
            transform[i][:3] = rotation[i].tolist()
            transform[i][3] = float(params['Lidar/offset_'+axis])
        transform[3][3] = 1.
        config['sensor_mount_T_uav_lidar'] = transform
        config['sensor_mount_use'] = 'offline location accounting only; derived from public SIM launch'
        write_config(args.output,config)
        print('FROZEN MANIFEST:96 scenes,212 tasks; no simulation started')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
