#!/usr/bin/env python3
"""Plot the already specified full-denominator resource curves, offline only."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parents[1]
report = json.loads((root / 'analysis-latest.json').read_text())
assert report['status'] == 'COMPLETE'
fig, axes = plt.subplots(1, 3, figsize=(11, 3.5), sharey=True)
for ax, key, label in zip(axes, ('windows', 'T_active_sim', 'T_task_sim'),
                         ('Accepted observation windows', 'Active perception (sim s)', 'Whole task (sim s)')):
    for method, color in (('ours', '#1769aa'), ('generic', '#d66b17')):
        rows = report['resource_curves'][method][key]
        x = [r['threshold'] for r in rows]
        low = [r['lower'] for r in rows]
        high = [r['upper'] for r in rows]
        ax.step(x, low, where='post', color=color, label=method.capitalize(), linewidth=2)
        ax.fill_between(x, low, high, step='post', color=color, alpha=.18)
        if key == 'windows':
            ax.scatter(x, low, color=color, s=22)
    ax.set_xlabel(label)
    ax.set_ylim(0, 1)
    ax.grid(alpha=.2)
    if key == 'windows':
        ax.set_xticks([1, 2, 3])
        ax.set_xlim(.95, 3.05)
    else:
        ax.set_xlim(left=0)
axes[0].set_ylabel('P(retrieval success AND resource <= budget)')
axes[0].legend(loc='upper left', frameon=False)
fig.suptitle('Fixed evaluation: all 96 scenes per method; failures are not fast successes', fontsize=11)
fig.tight_layout()
fig.savefig(root / 'resource-curves.svg')
fig.savefig(root / 'resource-curves.png', dpi=160)
plt.close(fig)
