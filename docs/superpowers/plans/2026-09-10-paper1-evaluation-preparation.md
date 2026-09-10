# Paper 1 evaluation preparation implementation plan

> **For agentic workers:** Use executing-plans / subagent-driven-development task by task. The user has approved the research design and preparation, not matrix execution.

**Goal:** Freeze all96 independent scenes and212 ordered tasks, adapt only cohort labels and offline accounting, then request one explicit resource/execution authorization.

**Architecture:** Keep the existing single-slot runner and shared robot pipeline. Add a pure seeded manifest generator and a dedicated offline analyzer; extend the old runner/collector with one named evaluation status, leaving old cohorts intact. No matrix engine or online setup runs.

**Tech stack:** Existing Python3.8 runner and Python3.10 NumPy/SciPy analysis; unittest; existing SIM installation and public frame/model files.

## 1. Manifest and setup-only description

Files: create `scripts/paper1_eval_design.py`, `tests/test_paper1_eval_design.py`,
`configs/paper1_eval_excluded_seeds.json`, `configs/paper1_eval.json`.

- [ ] Test first: `build_config(profile, excluded)` deterministically yields96 unique seeds/38 Easy28 Moderate30 Hard,212 tasks, consecutive Generic/Ours pairs and balanced within-tier first methods; no profile mutation.
- [ ] Test exact original six random draws followed by wall perturbations; excluded seed numbers skipped without reading outcomes. Each planned context/ablation subset is based only on generation ID.
- [ ] Run `PYTHONPATH=scripts:tests python3 -m unittest test_paper1_eval_design -v`, observe missing implementation fail, implement pure generator, rerun.
- [ ] Capture only historical numeric seed/scene_seed values from existing configs/outputs into the ordinary exclusion JSON before generating the new manifest. Materialize all scenes/order once, preserve them and do not launch them.
- [ ] Offline check nominal optical depth/FOV and spawn geometry against current public SIM model files; save a setup-only report. No candidate, score, sensor task or success-based admission.

## 2. Named cohort, unchanged execution

Files: modify `scripts/run_a6_attempt.py`, `tests/test_a6_attempt.py`; add explicit single-slot command preparation in `scripts/paper1_eval_design.py` and its tests.

- [ ] Test first: `attempt_kind('FROZEN_FOR_EVALUATION') == 'EVALUATION_ATTEMPT'`; evaluation accepts exactly the current integrated-velocity/full-robot/clearance/dynamics flags. ODE contrast/Ground replay remain development-only.
- [ ] Test normalized `adapter_args`, diagnostic commands and environment values match the development profile; cohort metadata never enters numeric decision requests.
- [ ] Run failing focused tests, extend only label/flag checks and recorded cohort metadata, rerun all existing attempt/entry/metrics tests.
- [ ] Provide read-only command preview for any one listed slot. Never execute the preview in this turn. Preserve evaluation tag and SIM installation.

## 3. Offline collection and analysis

Files: modify `scripts/summarize_a6_pilot.py` minimally for the new kind/cohort;
create `scripts/analyze_paper1_eval.py`, `tests/test_paper1_eval_analysis.py`.

- [ ] Test first: legacy/wrong-cohort attempts excluded, valid failure kept, only explicitly invalid original can have one replacement, partial/missing pair has no final inference, duplicate valid remains ambiguous.
- [ ] Test aggregate IID paired tables/McNemar and two97.5%CP bounds (not historical tier weights), first-activation-invalid sensitivity, auxiliary tables remain descriptive.
- [ ] Test sim-time Ground span, windows and packet-derived measured UAV location changes; missing transforms/metrics stay missing. No path-distance claim.
- [ ] Test conditional jointly successful paired timing/count summaries and scene bootstrap; all-task retrieval-vs-window/time curves keep failed/missing observations in denominators.
- [ ] Run tests red, implement using existing collector/math helpers and stored events/NPZ only, run tests green plus historical collection/statistics regressions.
- [ ] Analyze synthetic fixtures and empty final-results directory only; final inference must remain unavailable for0/212.

## 4. Review, version identity and authorization handoff

Files: create `docs/PAPER1_EVALUATION_READY.md`, update links in current version/protocol docs as an addendum, retain original evaluation tag.

- [ ] Independent spec review, then code-quality review; resolve concrete issues without broad refactoring.
- [ ] Verify manifests regenerate identically, all legacy algorithm/source/profile/asset files untouched, SIM clean, every planned slot's command uses current common flags and named cohort.
- [ ] Record prospective budget:24–36h estimate,120h hard aggregate campaign wall-time including attempts/startup/cleanup/diagnostic work;212 tasks+at most12 invalid replacements;≤500GiB new total campaign footprint with≥100GiB free. No external resources.
- [ ] Commit/push preparation checkpoint without moving old tags or merging main. Final response requests one authorization for precisely those ceilings; no robots start in this preparation.
