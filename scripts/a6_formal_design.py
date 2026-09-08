"""Pure prospective formal design; no scene filtering, outcomes, or execution."""

import copy
import math
import random


TIERS = ('easy', 'moderate', 'hard')
CORE_METHODS = ('rm4d_only', 'fixed', 'generic', 'ours')
REPLICATES = 40
SEED_DRAW_SEED = 202609083
ORDER_SEED = 202609084


def build_config(pilot1, pilot2):
    """Copy frozen Pilot-2 settings and serialize every independent scene/slot.

    Seed loop: replicate 1..40, then easy/moderate/hard. Order loop: first
    easy/moderate/hard, then ten groups of four replicates; shuffle a fresh
    core base and then four rotation indices. Finally, in replicate order,
    shuffle a fresh tier list and append each complete scene method block.
    """
    config = copy.deepcopy(pilot2)
    config.update(
        protocol='docs/A6_FORMAL_PROTOCOL.md', status='FROZEN_FOR_FORMAL',
        experiment='Formal', attempt_limit=560, scene_count=120,
        diagnostic_image_scope='ground_handoff_to_end',
        replicates_per_tier=REPLICATES, tiers=list(TIERS),
        seed_draw_seed=SEED_DRAW_SEED, order_seed=ORDER_SEED,
        seed_draw_rule='Random(202609083).randrange(1, 2**31), first eligible unique draws; '
                       'skip only Pilot-1/Pilot-2 seeds and already assigned seeds. '
                       'Assign in replicate 1..40 order, then easy/moderate/hard.',
        scene_draw_rule='For each scene Random(seed), in order: '
                        'target x=2+U(-.15,.15), y=U(-.15,.15), yaw=U(-pi/6,pi/6); '
                        'bunker x=3+U(-.1,.1), y=-2.5+U(-.1,.1), yaw=pi+U(-pi/36,pi/36). '
                        'Copy tier boxes and target/bunker z unchanged from Pilot-2.',
        order_rule='Use only Random(202609084). First loop tiers easy/moderate/hard; '
                   'within each tier loop ten consecutive groups of four replicates. '
                   'For each group shuffle fresh [rm4d_only,fixed,generic,ours], then '
                   'shuffle fresh [0,1,2,3]; assign each left rotation once in replicate order. '
                   'After all groups, loop replicates 1..40 and shuffle fresh '
                   '[easy,moderate,hard] for scene-block order. Emit each core block intact. '
                   'Append [no_occlusion,no_cost] to odd Hard replicates and the reverse '
                   'to even Hard replicates, without additional RNG draws.',
        scenes=[], slots=[])
    templates = {scene['id']: scene for scene in pilot2['scenes']}
    used = {scene['seed'] for pilot in (pilot1, pilot2) for scene in pilot['scenes']}
    seed_rng, order_rng = random.Random(SEED_DRAW_SEED), random.Random(ORDER_SEED)
    for replicate in range(1, REPLICATES + 1):
        for tier in TIERS:
            seed = seed_rng.randrange(1, 2**31)
            while seed in used:
                seed = seed_rng.randrange(1, 2**31)
            used.add(seed)
            rng = random.Random(seed)
            scene = copy.deepcopy(templates[tier])
            scene.update(id=f'{tier}-{replicate:03d}', tier=tier, replicate=replicate, seed=seed,
                         target_xy=[2 + rng.uniform(-.15, .15), rng.uniform(-.15, .15)],
                         target_yaw=rng.uniform(-math.pi / 6, math.pi / 6),
                         bunker_xy=[3 + rng.uniform(-.1, .1), -2.5 + rng.uniform(-.1, .1)],
                         bunker_yaw=math.pi + rng.uniform(-math.pi / 36, math.pi / 36))
            config['scenes'].append(scene)
    core_orders = {}
    for tier in TIERS:
        for group in range(REPLICATES // 4):
            base, rotations = list(CORE_METHODS), list(range(4))
            order_rng.shuffle(base)
            order_rng.shuffle(rotations)
            for offset, rotation in enumerate(rotations):
                core_orders[tier, group * 4 + offset + 1] = base[rotation:] + base[:rotation]
    for replicate in range(1, REPLICATES + 1):
        tiers = list(TIERS)
        order_rng.shuffle(tiers)
        for tier in tiers:
            methods = core_orders[tier, replicate][:]
            if tier == 'hard':
                methods += (['no_occlusion', 'no_cost'] if replicate % 2
                            else ['no_cost', 'no_occlusion'])
            for method in methods:
                config['slots'].append(dict(slot=len(config['slots']) + 1,
                                           scene=f'{tier}-{replicate:03d}', method=method))
    return config
