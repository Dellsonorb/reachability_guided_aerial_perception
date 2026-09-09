# Offline development-result projections

Source: sibling `multiscene-paired/launch-01-*` through `launch-12-*` only.
No old pilots, natural regression or interrupted formal slots are pooled.
All12 runs used AGENT c8b5b1b (runtime unchanged from6dfef58) and SIM e4e4f69
(installed runtime fbb191b). No additional Gazebo starts or runtime revisions.

The following run from the AGENT repository root. These scripts read original
recordings; none controls robots, queries GT for an algorithm, changes gates,
adds observations, selects alternatives or reclassifies the original attempts.
They are small result projections, not an experiment runner/framework.

```bash
PYTHONPATH=src:scripts /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python scripts/a6_scoring_diagnostics.py --results-dir outputs/development/multiscene-paired --output outputs/development/multiscene-paired-analysis/same-state-scoring.json
PYTHONPATH=src:scripts /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python scripts/a6_operational_diagnostics.py --results-dir outputs/development/multiscene-paired --output outputs/development/multiscene-paired-analysis/mechanism.json
PYTHONPATH=src:scripts MPLCONFIGDIR=/tmp/multiscene-analysis-mpl XDG_CACHE_HOME=/tmp/multiscene-analysis-cache /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python outputs/development/multiscene-paired-analysis/diagnose_exact_final.py
python3 outputs/development/multiscene-paired-analysis/assemble_report.py
python3 outputs/development/multiscene-paired-analysis/checker_retention_reproduction.py
```

Existing diagnostics reject overwriting some outputs; when reproducing those,
choose a new `--output` location. Do not alter original attempts. `results.json`
and `tasks.csv` are deterministic projections and may be regenerated from them.
The legacy mechanism report's `kind` name includesv11; its per-round
`operational_semantics` correctly reports the current object-aware-v1.4 rule.

The existing Hard diagnostic has historical directory names in its CLI list.
Reuse the same functions with this batch's four directories, without modifying
the saved implementation or replaying any sensor/robot operation:

```bash
PYTHONPATH=src:scripts MPLCONFIGDIR=/tmp/multiscene-analysis-mpl /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python - <<'PY'
import sys
import diagnose_hard_nbv_ground as d
d.RUNS = ('launch-05-hard01-generic', 'launch-06-hard01-ours',
          'launch-11-hard02-ours', 'launch-12-hard02-generic')
sys.argv = ['diagnose_hard_nbv_ground.py',
            '--source-dir', 'outputs/development/multiscene-paired',
            '--output-dir', 'outputs/development/multiscene-paired-analysis/hard-reproduced']
raise SystemExit(d.main())
PY
```

This uses saved public TF/endpoints and the current assumed-height occlusion
model, not Gazebo GT or new votes. No optional scan-pattern inference was used;
do not attribute every missing return to a particular pattern/occluder.

Slot3's separate native physical-motion diagnostic (large CSV retained locally
under the existing ignore rule) was generated with:

```bash
PYTHONPATH=src /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python outputs/development/ground-manipulation-batch/dynamics-analysis/analyze.py --trace outputs/development/multiscene-paired/launch-03-moderate01-ours/ground-dynamics.csv --events outputs/development/multiscene-paired/launch-03-moderate01-ours/data/events.jsonl --output-dir outputs/development/multiscene-paired-analysis/physical-launch03-reproduced
```

That output does not backfill the failed physical checker or establish its
missing aerial-topic match. `checker_retention_reproduction.py` reproduces a
possible loss mechanism using synthetic callbacks, not a reconstruction of the
actual checker buffer. Its in-memory contrast does not patch installed SIM.

Final relevant regressions: AGENT74 passed (attempt, adapter, metrics, execution
selection, scene admission); SIM127 passed (execution planning/clearance,
manipulation scene/TF/full robot, camera geometry, existing demo). No physics,
controller, collision, protocol, scene or algorithm code was changed by this
batch. Terminal/NOT_REACHED/null distinctions remain in the original files.
