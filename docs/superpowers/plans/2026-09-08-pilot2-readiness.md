# Pilot-2 and Formal Readiness Implementation Plan

> **For agentic workers:** Use executing-plans for coupled local orchestration;
> dispatch independent diagnostic implementation and spec/quality review with
> the applicable parallel/review skills. Work in the user-designated A6 branch.

**Goal:** Run independently seeded v1.1 Pilot-2, diagnose its mechanism and
execution outcomes, and determine whether the user's formal-readiness criteria
permit the next subproject; stop immediately at a scientific stop condition.

**Architecture:** Reuse the one-attempt launcher, frozen A6 policies and v1.1
gate. Only add a new predeclared configuration and read-only mechanism report,
forward the existing v1.1 option, and record necessary passive diagnostics.
No change to frozen algorithms or Pilot-1 files.

**Tech Stack:** Existing Python3.10 numerical core, Python3.8/ROS Noetic,
Gazebo public interfaces, JSON/NPZ and existing rosbag if needed; no dependency.

## Task 1 — independent configuration and option forwarding (root)

Files: `configs/a6_pilot2.json`, `tests/test_a6_pilot2.py`,
`scripts/run_a6_attempt.py`; protocol `docs/A6_PILOT2_PROTOCOL.md`.

- [x] RED tests for absent config and existing launcher omission:

```python
self.assertTrue((ROOT/'configs/a6_pilot2.json').is_file())
args = attempt.adapter_args(dict(old, operational_gating='v1.1'), out, sim, rm)
self.assertEqual(args[args.index('--operational-gating')+1], 'v1.1')
```

- [x] Create serialized config with root draw 202609082 and exact seeds
  1015873452/1240085801/656391333; preserve all old shared settings, initialization
  and per-tier boxes. Reconstruct each six-draw pose with `random.Random(seed)`
  in a test and assert exact equality; assert old config unchanged, seeds disjoint,
  each method appears once per tier, two Hard ablations only.
- [x] Minimal launcher forwarding (v1 absence remains unchanged):

```python
if 'operational_gating' in config:
    args += ['--operational-gating', config['operational_gating']]
```

- [x] Record config path/revision in new attempt metadata when explicitly supplied;
  no runtime target or allowance injected from setup metadata. RM4D-only remains
  original top-one, not operationally gated.
- [x] Run `PYTHONPATH=src:tests CORE_PY -m unittest test_a6_pilot2 test_a6_attempt
  test_a6_adapter -q`; replace CORE_PY with the existing conda interpreter path.
  Commit seed/protocol freeze before any method execution.

## Task 2 — mechanism report (bounded agent, no runtime mutation)

Files: new `scripts/a6_operational_diagnostics.py`,
`tests/test_a6_operational_diagnostics.py`.

- [x] RED synthetic report fixture: one alias-only retained candidate, one true
  target collision, one ambiguous blocker, one representative-only blocker;
  raw occupied/operational blocked/retained/confirmed counts stay distinct.
- [x] Implement `describe_round(decision, operational_summary, arrays)` as a pure
  function. For each exact assessment compute the predeclared indicator:

```python
alias = (candidate['occupied_cells'] > 0 and gate['target_cells'] > 0
         and not gate['blocked'] and not gate['target_collision'])
```

  Retain ID/pose, clipping, representative, ground support and confirmation
  separately. Include per-class vote sums and occupied cell counts. Missing
  inputs are explicit unavailable; N/A for RM4D-only. CLI reads existing
  `*/data/rounds/round-*` artifacts and joins existing `metrics.json` and
  `attempt.json` for confirmed/D_exec/retrieval without recategorizing outcomes.
- [x] Test missing reference, mixed class votes, no fabricated ground/confirmation,
  input preservation and actual natural-v1.1 saved result (zero alias retention,
  two final confirmed). Use existing summaries; no Gazebo or GT access.
- [x] Obtain spec then quality review; exercise saved scoring diagnostics to
  confirm no effect on candidate generation, V, cost or Ground selection.

## Task 3 — preparation and independent setup (root)

- [x] Read existing topic contracts; add only necessary passive control/RGB-D
  logging using existing tooling, with tests for opt-in command construction.
  No navigation/D435/arm parameter adjustment. Missing optional logs cannot
  reclassify a demonstrated primary failure.
- [x] Run method-independent nominal depth check for all three serialized seeds.
  Execute existing `run_a6_attempt.py --config configs/a6_pilot2.json
  --setup-scene TIER --output-dir NEW_SETUP_DIR` for each tier. No RM4D or scores.
- [x] Inspect setup summaries and public geometry; preserve any activation.
  Freeze successful setup confirmation before starting methods. Baseline focused
  tests, full regression and independent review must be green.

## Task 4 — Pilot-2 execution and analysis (root)

- [ ] Run exactly prelisted slots sequentially with a fresh simulator each time,
  via existing `run_a6_attempt.py --config configs/a6_pilot2.json --slot N
  --output-dir NEW_SLOT_DIR`. Poll logs/results, never run concurrent Gazebo.
- [ ] Classify terminal events before considering any rerun. Retain valid failures,
  rerun only demonstrated INVALID, and stop at a scientific stop condition.
- [ ] Generate old-schema pilot summary, same-state score checks, scene descriptions
  and new mechanism report. Summarize paired b/c outcomes and simulation-time
  resources, all stage results, invalid history and unresolved causal limits.
- [ ] Review explicit structural/fairness/measurement/core-hypothesis readiness,
  write `docs/A6_PILOT2_RESULTS.md`, test/check source boundaries, commit/push.

## Task 5 — conditional next subproject

- [ ] If and only if readiness clearly passes, design and freeze independent
  formal protocol, paired sample size and analysis plan before formal outcomes;
  then implement/run the separate formal-execution plan and manuscript analysis.
  This plan does not silently treat the small pilot as a powered formal study.
- [ ] Otherwise follow the user's scientific stop rule with concrete evidence and
  the smallest research decision needed; no algorithm tuning or scene replacement.

## Observed runtime corrections during Pilot-2 (ordinary engineering)

Slot2's checker missed the initial state burst during duplicate TCPROS
connection turnover. A5 already unregisters its temporary parent publisher.
Do not change A5 or increase delays/thresholds: expose final adapter construction
completion on its existing log, then start the unchanged checker. Keep the
same total wall guard and existing subscriber wait before task start. Test the
ordering and narrowly classify this demonstrated missing-measurement case;
actual FAILED events retain priority. Preserve the first automatic classification
inside the reviewed record and rerun only this INVALID activation.

Raw bags also demonstrate local Python TF-buffer gaps despite available public
transforms. A bounded independent task may add a **separate secondary** offline
bag-derived resource report, not overwrite the original metrics: preserve all
original sample timestamps/order/wall times; use only causally received public
TF with header stamps no later than each sample, coherent TF2 common-time
composition, existing .5 s age and .3 s scheduling-gap checks, and the unchanged
metrics reducer. Process every sample uniformly, not only failed/missing ones.
No GT, smoothing, inserted trajectory samples or outcome changes. Any formal use
must be prospectively declared before formal activation. This is measurement
repair, not a new research method or execution framework.
