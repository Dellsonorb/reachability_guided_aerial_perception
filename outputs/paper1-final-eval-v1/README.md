# Completed Paper 1 fixed evaluation

See [the final report](../../docs/PAPER1_FIXED_EVALUATION_RESULTS.md).
Collection is complete:212 planned slots,96 main pairs,214 formal activations,
plus1 external bare Gazebo start charged conservatively =215/224 total starts.
No further task/replacement is pending. No method/configuration was changed.

## Package

- `analysis-latest.json` is the complete frozen reducer output, including source
  attempt/metric/physical summaries and all prespecified paired calculations.
- `slots.csv` contains212 selected valid task rows; `pairs.csv` contains96 pairs.
  `activations.csv` preserves214 starts including the two invalid originals.
  Empty CSV values mean unavailable/not applicable, never an implicit zero.
- `descriptive-summary.json`, `mechanism-rounds.csv`, `same-state-scoring.csv`
  and `resource-curves.{png,svg}` are post-collection offline exports.
- `resource-baseline.json`, `resource-summary.json`, `retention-index.json`
  and operator incident notes explain the deadline, limits, invalids and storage.
- Explicit compact original `entry.json`, `task.json`, `attempt.json`, physical
  summaries, metrics, events and retention notes are versioned. Raw bags, large
  native CSV gzip, clouds, fields, runtime/ROS logs and exported review images
  remain locally under each original slot/attempt directory; they are **not**
  uploaded to GitHub. The ignore rule only affects Git, not local retention.

Some compact notes reference local files not included in a clone. Four retained
sample frames cannot reconstruct a removed full RGB-D sequence. The129 authorized
successful-image removals are explicit in retention notes; failure/invalid and
predeclared-retention imagery was not deleted. Original131 partial data and its
startup snapshot are preserved. `operator/progress.jsonl` has one unmodified
280-byte damaged line1196 from the USB interruption; JSONL consumers must report
and skip that malformed line, not rewrite it. Primary slot/attempt data is complete.

Historical operator `serial-execution-recipe.py` and `resume-*.py` are records of
one-time executions, **not restart instructions**. Do not run them: they either
refuse existing directories or contain completed one-time continuation logic.

## Offline reproduction only

From the AGENT root on the current workstation, with the retained local data:

```bash
PYTHONPATH=scripts OPENBLAS_NUM_THREADS=1 \
  /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python \
  scripts/analyze_paper1_eval.py --config configs/paper1_eval.json \
  --results-dir outputs/paper1-final-eval-v1 \
  --output /tmp/paper1-final-recomputed.json

PYTHONPATH=src:scripts OPENBLAS_NUM_THREADS=1 \
  /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python \
  outputs/paper1-final-eval-v1/operator/export-final-tables.py

MPLCONFIGDIR=/tmp/a6-mpl OPENBLAS_NUM_THREADS=1 \
  /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python \
  outputs/paper1-final-eval-v1/operator/plot-final-curves.py
```

These commands do not run Gazebo. The frozen reducer needs retained window NPZ
files for packet-pose location metrics. On a compact Git-only clone, raw binary
outcomes and the paired primary table are independently recomputable from
`pairs.csv`/`attempt.json`/physical summaries, but full location/mechanism replay
needs local raw data. A recomputation lacking those files must report secondary
missingness; do not claim full replay from this compact package alone.

All212 selected executions used AGENT3d13a95, SIMa0ae8e3 and RM4De9d4312 plus the
unchanged task-domain asset. Post-collection reports and export scripts in the
results commit do not retrospectively change those actual runtime references.
Preparation/frozen tags, prior branches and development results remain intact.
This checkpoint does not merge main or authorize a new matrix.
