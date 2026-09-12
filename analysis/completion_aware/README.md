# Completion-aware offline analysis

Read [REPORT.md](REPORT.md) for findings, mathematical definitions and limits.
`PLAN.md` records the analysis rules fixed before aggregate results.

## Reproduce (repository root, no ROS/Gazebo)

```bash
python3 -m unittest discover -s analysis/completion_aware -p 'test_*.py'
OPENBLAS_NUM_THREADS=1 python3 analysis/completion_aware/offline.py
PYTHONPATH=src OPENBLAS_NUM_THREADS=1 python3 analysis/completion_aware/phase_check.py
python3 analysis/completion_aware/report_assets.py
```

Dependencies: existing Python, NumPy and Matplotlib. No new package was installed.
The optional-to-core `phase_check.py` requires the original retained MID360 scan
CSV and publisher SDF at the paths recorded in `ranking.json`. It only reuses
the unchanged nominal predictor, not simulator processes or scene truth.

## Inputs and outputs

- Source records: `outputs/paper1-final-eval-v1/slots.csv` and selected attempt
  `data/` snapshots. Original records are read-only and retain their classifications.
- Evaluation reference: `paper1-eval-finite-scan-v1`; AGENT `3d13a95`, SIM
  `a0ae8e3`, RM4D baseline `e9d4312`, as already documented in the evaluation package.
- Analysis starts from documentation commit `e5be749` on the separate
  `analysis/completion-aware-offline` branch. Runtime source/configuration is not
  changed by this analysis. This is post-hoc development evidence, not a new test set.
- Generated derived tables and figures live only in this directory's `results/`.
  Predictions cover available pre-terminal snapshots, including explicitly marked
  already-confirmed states; use unresolved-only counts for sensing decisions.
- `accounting.csv`: all 212 tasks and available/absent observation inputs.
- `trajectories.csv`, `candidates.csv`: current deficit envelope and fixed initial
  anchor, plus all exact candidate histories; no non-winner reselection.
- `predictions.csv`: bounds, restricted sequences, actual command matching,
  same-state Generic/Ours first-action completion tiers, strict vs tie-only changes.
- `actual_transitions.csv`: actual-view-only next-window calibration labels.
- `generic_29_cases.csv`: all 29 confirmation failures, paired Ours outcome and
  proposed views; outcomes are never counterfactual labels for the proposals.
- `paired_trajectories.csv`: all 96 pairs, with missing fields left empty.
- `joint_phase_diagnostics.csv`: all four optimistic-only nominated plans checked
  using joint scan masks, without assuming independent cell/phase probabilities.
- `overprediction_pose_diagnostics.csv`: public packet TF diagnostics for the three
  failed OR-level predictions; no causal root-cause claim.
- `summary.json`, `report_facts.json`, `completion_diagnostics.{pdf,png}`: aggregates.

This code does not write belief votes, call a robot interface, change the task
budget, update the paper's final statistics, or estimate off-policy retrieval.

Verification: 12 standalone analysis unit tests and 78 existing finite-scan,
operational-occlusion/gating, exact-support and A5-core tests passed. Independent
read-only review checked the predictive bounds, runtime transition constraints,
joint phase masks, report denominators and limitations. No substantive issue
remained after correcting the offline facade filter and reporting already-confirmed
states separately. Existing tests ran with Python's unittest (pytest is not installed).
