#!/usr/bin/env python3
"""Two final-only figures from completed formal analysis and result-table JSON.

No experiment access or inference. Resources are observed online measurements
across all valid trials, not time-to-success or success-conditioned efficiency.
"""

import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


TIERS = ('easy', 'moderate', 'hard')
METHODS = ('rm4d_only', 'fixed', 'generic', 'ours', 'no_occlusion', 'no_cost')
NAMES = dict(rm4d_only='RM4D', fixed='Fixed', generic='Generic', ours='Ours',
             no_occlusion='No occlusion', no_cost='No cost')
CATEGORIES = ('both_success', 'b', 'c', 'neither_success')
RESOURCES = (('uav_active_distance_m', 'Active UAV path (m)', 'uav_active_complete'),
             ('ground_total_distance_m', 'Ground path (m)', 'ground_total_complete'),
             ('T_task_sim', 'Task runtime (sim s)', None))


def _final_groups(analysis, tables):
    """Reject interim/stale cohorts before creating any Matplotlib figure."""
    primary = analysis.get('primary_inference') or {}
    expected = analysis.get('expected_primary_pairs')
    if (analysis.get('status') != 'COMPLETE' or type(expected) is not int or expected <= 0
            or analysis.get('complete_primary_pairs') != expected or primary.get('n') != expected
            or set(primary.get('tiers', {})) != set(TIERS)):
        raise ValueError('figures require COMPLETE final formal analysis')
    if tables.get('resource_source') != 'original_online_metrics':
        raise ValueError('figures require original online resources, not secondary replacements')
    scenes, slots = {}, set()
    for row in tables.get('rows', []):
        key = (row['tier'], row['scene'])
        members = scenes.setdefault(key, {})
        if (row['status'] != 'VALID_TRIAL' or type(row['retrieval_success']) is not bool
                or row['slot'] in slots or row['method'] in members or row['tier'] not in TIERS):
            raise ValueError('result tables must contain unique completed valid slots')
        slots.add(row['slot'])
        members[row['method']] = row
    if len(scenes) != expected:
        raise ValueError('result-table scene coverage differs from final analysis')
    counts = {tier: dict.fromkeys(('n', *CATEGORIES), 0) for tier in TIERS}
    for (tier, _), members in scenes.items():
        required = set(METHODS if tier == 'hard' else METHODS[:4])
        if set(members) != required or len({row['seed'] for row in members.values()}) != 1:
            raise ValueError('incomplete scene method block or inconsistent paired seed')
        ours, generic = (members[method]['retrieval_success'] for method in ('ours', 'generic'))
        category = 'both_success' if ours and generic else 'b' if ours else 'c' if generic else 'neither_success'
        counts[tier]['n'] += 1
        counts[tier][category] += 1
    if (any(primary['tiers'][tier].get(key) != value for tier in TIERS
            for key, value in counts[tier].items())
            or any(primary.get(key) != sum(counts[tier][key] for tier in TIERS) for key in ('b', 'c'))):
        raise ValueError('paired result-table outcomes differ from final analysis')
    return [((tier, method), [row for row in tables['rows']
                             if row['tier'] == tier and row['method'] == method])
            for tier in TIERS for method in (METHODS if tier == 'hard' else METHODS[:4])]


def build_figures(analysis, tables):
    """Return native figures; all input values remain untouched."""
    groups = _final_groups(analysis, tables)
    labels = [tier.title() + ' / ' + NAMES[method] for (tier, method), _ in groups]
    primary = analysis['primary_inference']
    outcomes, (paired, endpoints) = plt.subplots(1, 2, figsize=(11, 6.5),
                                                gridspec_kw=dict(width_ratios=[1, 1.2]))
    outcomes.subplots_adjust(left=.12, right=.94, bottom=.18, top=.85, wspace=.65)
    left = [0] * len(TIERS)
    colors = ('#478c78', '#2878b5', '#df8f32', '#d2d6da')
    names = ('Both success', 'Ours only (b)', 'Generic only (c)', 'Neither success')
    for key, name, color in zip(CATEGORIES, names, colors):
        values = [primary['tiers'][tier][key] for tier in TIERS]
        paired.barh(range(len(TIERS)), values, left=left, height=.55, color=color, label=name)
        for index, value in enumerate(values):
            if value:
                paired.text(left[index] + value / 2, index, str(value), ha='center', va='center', fontsize=9)
        left = [a + b for a, b in zip(left, values)]
    paired.set_yticks(range(len(TIERS)), [tier.title() + ' (n=%s)' % primary['tiers'][tier]['n']
                                        for tier in TIERS], fontsize=9)
    paired.invert_yaxis()
    paired.set_xlabel('Scene pairs')
    paired.set_title('Paired E2E outcomes\nOurs vs Generic', fontsize=11)
    paired.legend(loc='upper center', bbox_to_anchor=(.5, -.12), ncol=2, fontsize=8, frameon=False)
    values, texts = [], []
    for (_, method), members in groups:
        line, text = [], []
        for endpoint in ('D_env', 'D_exec', 'retrieval_success'):
            available = [row.get(endpoint) for row in members if type(row.get(endpoint)) is bool]
            numerator, denominator = sum(value is True for value in available), len(available)
            na = endpoint == 'D_env' and method == 'rm4d_only'
            line.append(float('nan') if na or not denominator else numerator / denominator)
            text.append('N/A' if na else '%d/%d' % (numerator, denominator))
        values.append(line)
        texts.append(text)
    cmap = plt.get_cmap('Blues').copy()
    cmap.set_bad('#eeeeee')
    picture = endpoints.imshow(values, aspect='auto', vmin=0, vmax=1, cmap=cmap)
    for index, line in enumerate(texts):
        for column, text in enumerate(line):
            endpoints.text(column, index, text, ha='center', va='center', fontsize=8,
                           color='white' if values[index][column] > .6 else '#222222')
    endpoints.set_yticks(range(len(groups)), labels, fontsize=8)
    endpoints.set_xticks(range(3), ['Confirm.\nD_env', 'Execution\nD_exec', 'Retrieval'], fontsize=9)
    endpoints.set_title('Recorded endpoints\npositive / available', fontsize=11)
    outcomes.colorbar(picture, ax=endpoints, fraction=.045, pad=.025, label='Positive fraction')
    outcomes.suptitle('Final study: paired outcomes and recorded endpoints', fontsize=12)
    outcomes.text(.5, .025, 'Each cell retains its available denominator; RM4D confirmation is not applicable.',
                  ha='center', fontsize=9)

    resources, axes = plt.subplots(1, 3, figsize=(11, 6.5), sharey=True)
    resources.subplots_adjust(left=.2, right=.98, bottom=.15, top=.86, wspace=.25)
    for axis, (field, title, complete) in zip(axes, RESOURCES):
        largest = 0.
        for index, (_, members) in enumerate(groups):
            available = [row[field] for row in members
                         if type(row.get(field)) in (int, float) and math.isfinite(row[field])
                         and (complete is None or row.get(complete) is True)]
            axis.plot(available, [index] * len(available), 'o', color='#2878b5',
                      markersize=3, alpha=.5, clip_on=False)
            largest = max([largest, *available])
            axis.text(.98, index, '%d/%d' % (len(available), len(members)),
                      transform=axis.get_yaxis_transform(), ha='right', va='center', fontsize=8)
        axis.set_xlim(0, largest * 1.3 if largest > 0 else 1.)
        axis.set_ylim(len(groups) - .4, -.6)
        axis.set_title(title, fontsize=10)
        axis.set_yticks(range(len(groups)))
        axis.grid(axis='x', color='#dddddd', linewidth=.5)
        axis.set_axisbelow(True)
        axis.text(.98, 1.015, 'n/N', transform=axis.transAxes, ha='right', fontsize=8)
        for boundary in (3.5, 7.5):
            axis.axhline(boundary, color='#dddddd', linewidth=.7)
    axes[0].set_yticklabels(labels, fontsize=8)
    resources.suptitle('Observed resources across all valid trials', fontsize=12)
    resources.text(.5, .035, 'Points are available online measurements (n/N shown); missing paths are omitted, never zero-filled.\n'
                   'Observed runtime is not time-to-success; early failures can be short.', ha='center', fontsize=9)
    return dict(outcomes=outcomes, resources=resources)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--analysis', type=Path, required=True, help='Completed final-analysis JSON')
    parser.add_argument('--tables', type=Path, required=True, help='Result-table JSON from the same complete study')
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args(argv)
    figures = build_figures(json.loads(args.analysis.read_text()), json.loads(args.tables.read_text()))
    try:
        paths = {name: args.output_dir / ('formal-' + name + '.pdf') for name in figures}
        if any(path.exists() for path in paths.values()):
            raise FileExistsError('final figure paths must be new files')
        args.output_dir.mkdir(parents=True, exist_ok=True)
        for name, figure in figures.items():
            figure.savefig(paths[name], format='pdf')
    finally:
        for figure in figures.values():
            plt.close(figure)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
