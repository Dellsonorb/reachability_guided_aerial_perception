#!/usr/bin/env python3
"""Offline predeclared paired inference; no robot or experiment execution.

Use the existing core interpreter with SciPy. The exact test concerns symmetry
within every tier. The average-effect normal interval is only approximate;
the simultaneous exact-binomial bound is conservative and boundary-safe.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.stats import beta, binom, norm


def _count(value):
    if type(value) is not int or value < 0:
        raise ValueError('counts must be nonnegative integers')
    return value


def exact_mcnemar(b, c):
    b, c = _count(b), _count(c)
    return 1. if b+c == 0 else min(1., float(2.*binom.cdf(min(b,c), b+c, .5)))


def binomial_interval(k, n, alpha=.05):
    k, n = _count(k), _count(n)
    if n == 0 or k > n or not 0 < alpha < 1:
        raise ValueError('binomial interval requires n>0, k<=n, 0<alpha<1')
    return [0. if k == 0 else float(beta.ppf(alpha/2., k, n-k+1)),
            1. if k == n else float(beta.ppf(1.-alpha/2., k+1, n-k))]


def paired_statistics(tiers, alpha=.05):
    """Equal-tier effect with paired sample variance and exact count bounds."""
    if not tiers or not 0 < alpha < 1:
        raise ValueError('nonempty tiers and alpha in (0,1) required')
    details, warnings = {}, []
    for name, row in tiers.items():
        n, b, c = (_count(row[key]) for key in ('n','b','c'))
        if not n or b+c > n: raise ValueError('invalid paired tier counts')
        delta = (b-c)/n
        variance = ((b+c)-n*delta**2)/(n-1)/n if n > 1 else None
        bounds_b = binomial_interval(b,n,alpha/(2*len(tiers)))
        bounds_c = binomial_interval(c,n,alpha/(2*len(tiers)))
        details[name] = dict(row, risk_difference=delta, mean_variance=variance,
                             simultaneous_b_interval=bounds_b,
                             simultaneous_c_interval=bounds_c)
        if min(b,c) < 5:
            warnings.append(name+': sparse paired outcome category; normal interval may be unreliable')
        if variance == 0:
            warnings.append(name+': zero paired sample variance')
    delta = sum(row['risk_difference'] for row in details.values())/len(tiers)
    lo = sum(row['simultaneous_b_interval'][0]-row['simultaneous_c_interval'][1]
             for row in details.values())/len(tiers)
    hi = sum(row['simultaneous_b_interval'][1]-row['simultaneous_c_interval'][0]
             for row in details.values())/len(tiers)
    variances = [row['mean_variance'] for row in details.values()]
    variance = None if any(v is None for v in variances) else sum(variances)/len(tiers)**2
    se = None if variance is None else math.sqrt(max(0.,variance))
    approximate = None
    if se is not None and se > 0:
        radius = float(norm.ppf(1-alpha/2))*se
        approximate = [max(-1.,delta-radius),min(1.,delta+radius)]
    else:
        warnings.append('zero or unavailable sample variance; do not report a degenerate normal interval')
    b, c = (sum(row[key] for row in details.values()) for key in ('b','c'))
    return dict(n=sum(row['n'] for row in details.values()), tiers=details, b=b,c=c,
                risk_difference=delta, standard_error=se,
                approximate_rd_interval=approximate,
                conservative_exact_rd_interval=[lo,hi], alpha=alpha,
                p_value=exact_mcnemar(b,c),
                exact_null='paired directional symmetry within every tier; not merely average RD=0',
                interval_notes='normal interval is approximate; exact bound uses simultaneous CP with Bonferroni',
                warnings=warnings)


def planning_power(n, delta, discordance, alpha=.05):
    """Exact planning mixture for a homogeneous scenario, never a pilot fit."""
    n = _count(n)
    if not n or not 0 < discordance <= 1 or abs(delta) > discordance or not 0 < alpha < 1:
        raise ValueError('invalid binomial planning scenario')
    probability, power = (discordance+delta)/(2*discordance), 0.
    for d in range(n+1):
        b = np.arange(d+1)
        p = np.minimum(1., 2.*binom.cdf(np.minimum(b,d-b),d,.5))
        conditional = float(binom.pmf(b,d,probability)[p <= alpha].sum())
        power += float(binom.pmf(d,n,discordance))*conditional
    return power


def holm(p_values):
    if any(not math.isfinite(p) or not 0 <= p <= 1 for p in p_values):
        raise ValueError('finite p-values in [0,1] required')
    result, previous = [None]*len(p_values), 0.
    for rank,index in enumerate(sorted(range(len(p_values)),key=p_values.__getitem__)):
        previous = max(previous,min(1.,(len(p_values)-rank)*p_values[index]))
        result[index] = previous
    return result


def _paired_tiers(pairs, scenes):
    result = {s.get('tier',s['id']):dict(n=0,b=0,c=0,both_success=0,neither_success=0)
              for s in scenes.values()}
    seen = set()
    for pair in pairs:
        scene_id = pair['scene']
        if scene_id not in scenes or scene_id in seen or pair['seed'] != scenes[scene_id]['seed']:
            raise ValueError('paired scene/seed coverage does not match the formal configuration')
        seen.add(scene_id)
        ours,generic = (pair[m]['retrieval_success'] for m in ('ours','generic'))
        if type(ours) is not bool or type(generic) is not bool:
            raise ValueError('paired primary outcomes must be literal binary values')
        category = ('both_success' if ours and generic else 'ours_only_b' if ours
                    else 'generic_only_c' if generic else 'neither_success')
        if pair['category'] != category:
            raise ValueError('paired category contradicts its binary outcomes')
        row = result[scenes[pair['scene']].get('tier',pair['scene'])]
        row['n'] += 1
        category = dict(ours_only_b='b',generic_only_c='c').get(category,category)
        row[category] += 1
    return result


def analyze(config, summary):
    """Final inference only for the complete prospectively specified study."""
    if config.get('status') != 'FROZEN_FOR_FORMAL':
        raise ValueError('requires a frozen formal configuration; do not pool pilots')
    if summary.get('config_status') != config['status']:
        raise ValueError('summary must originate from the frozen formal configuration')
    scenes = {s['id']:s for s in config['scenes']}
    expected = {s['slot']:(s['scene'],scenes[s['scene']]['seed'],s['method']) for s in config['slots']}
    seen = set()
    for row in summary['slots']:
        slot = row['slot']
        if slot in seen or expected.get(slot) != (row['scene'],row['seed'],row['method']):
            raise ValueError('summary slot identities do not match the formal configuration')
        seen.add(slot)
    primary = summary['primary_comparison']
    complete = (not primary['incomplete_pairs'] and len(primary['pairs']) == len(scenes)
                and len(summary['slots']) == len(config['slots'])
                and all(row['status'] == 'VALID_TRIAL' for row in summary['slots']))
    result = dict(status='COMPLETE' if complete else 'INCOMPLETE',
                  expected_primary_pairs=len(scenes), complete_primary_pairs=len(primary['pairs']),
                  primary_inference=None, sensitivity_inference=None, hard_ablations=[],
                  planning_assumptions=dict(n=120,per_tier=40,delta=.20,discordance=.50,
                                            ours_only=.35,generic_only=.15,alpha=.05,
                                            exact_test_power=.86223,
                                            scenario='homogeneous planning assumptions, not fitted to pilot outcomes'),
                  note='No final inference or optional efficacy look until every prelisted valid slot completes.')
    if not complete: return result
    by_key = {(r['scene'],r['method']):r for r in summary['slots']}
    for pair in primary['pairs']:
        for method in ('ours','generic'):
            row = by_key.get((pair['scene'],method))
            if (row is None or pair[method]['slot'] != row['slot']
                    or pair[method]['retrieval_success'] is not row['retrieval_success']):
                raise ValueError('primary pair contradicts its selected slot outcome')
    result['primary_inference'] = paired_statistics(_paired_tiers(primary['pairs'], scenes))
    sensitivity = summary['first_activation_invalid_as_failure']
    if not sensitivity['incomplete_pairs'] and len(sensitivity['pairs']) == len(scenes):
        result['sensitivity_inference'] = paired_statistics(_paired_tiers(sensitivity['pairs'],scenes))
    for method in ('no_occlusion','no_cost'):
        counts = dict(n=0,b=0,c=0,both_success=0,neither_success=0)
        for scene in config['scenes']:
            if scene.get('tier',scene['id']) != 'hard': continue
            ours, ablated = (by_key[(scene['id'],m)]['retrieval_success'] for m in ('ours',method))
            if type(ours) is not bool or type(ablated) is not bool:
                raise ValueError('completed ablation must have two binary outcomes')
            counts['n'] += 1
            counts['both_success' if ours and ablated else 'b' if ours else 'c' if ablated else 'neither_success'] += 1
        if counts['n']:
            result['hard_ablations'].append(dict(method=method,**paired_statistics(dict(hard=counts))))
    adjusted = holm([r['p_value'] for r in result['hard_ablations']])
    for row,value in zip(result['hard_ablations'],adjusted): row['secondary_holm_p_value'] = value
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--summary',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args(argv)
    config, summary = (json.loads(p.read_text()) for p in (args.config,args.summary))
    result = analyze(config,summary)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return 0


if __name__ == '__main__': raise SystemExit(main())
