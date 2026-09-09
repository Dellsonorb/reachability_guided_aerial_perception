# v1.2 Fresh Paired Validation Execution Plan

> **For agentic workers:** Use subagent-driven-development for bounded launcher
> plumbing and diagnostic tasks, with specification then quality review. Root
> owns the only serial SIM. The user approved the proposal and routine autonomy;
> do not seek another approval except for the stated scientific stop conditions.

**Goal:** Complete six fresh paired validation slots, or stop on demonstrated
structural deadlock, then deliver a review checkpoint without formal execution.

**Architecture:** Reuse existing single-slot launcher, A6 adapter/worker and
diagnostic functions. Add only explicit exact-anchor plumbing and a standalone
frozen six-slot configuration. No research-source or dependency changes.

**Tech Stack:** Existing ROS Noetic/Python3.8, core Python3.10, SIM/PX4, NumPy.

## 1. Freeze the complete setup before any method

- [x] Read approved proposal, existing protocol, launcher and seed metadata;
  retain feature checkout and four unrelated untracked A5 directories.
- [x] Determine first three eligible seeds with draw2026090901, excluded prior
  config union; freeze pair order with shuffle2026090902 and tier alternation.
- [ ] Add `tests/test_a6_v12_validation.py`: assert configuration exists, frozen
  common parameters equal Pilot-2, seeds equal first eligible draws, all scene
  values reproduce original six draws, boxes/z unchanged, six exact slots only.
- [ ] RED launcher test: compare `adapter_args` before/after adding
  `support_anchor='exact_winner'`; require only the two CLI words to differ.
- [ ] Minimal launcher patch in `scripts/run_a6_attempt.py`:
  `if 'support_anchor' in config: args += ['--support-anchor', config['support_anchor']]`.
  Test both explicit values and unchanged legacy omission; no default change.
- [ ] Create `configs/a6_v12_validation.json` by copying common Pilot-2 settings,
  changing only experiment/protocol/count/seeds/order, explicit exact anchor and
  diagnostic image scope `ground_handoff_to_end` already supported by launcher.
- [ ] Run `PYTHONPATH=src:tests CORE -m unittest test_a6_v12_validation
  test_a6_attempt test_a6_pilot2 test_exact_support -q`; independent spec then
  quality review, commit and push seed/protocol/config freeze before any run.

## 2. Prepare diagnostic reuse and execute serially

- [ ] Confirm operational/scoring/metrics commands and non-winner helper reuse.
  Exact A6 snapshots must not use the legacy-cell-center `replay_attempt` path.
  Any small reporting addition first receives a test using recorded data;
  diagnostics never participate in selection or rewrite previous results.
- [ ] For each scene run existing `scripts/run_a6_attempt.py --config
  configs/a6_v12_validation.json --setup-scene TIER --output-dir NEW_SETUP_DIR`
  in SIM's pinned Noetic environment with dedicated11951/11952 ports. No pose override.
- [ ] Confirm all three existing setup checks pass. Keep every activation;
  investigate genuine domain conflict instead of resampling.
- [ ] Run explicit slots1–6 with the same command's `--slot N` instead of
  `--setup-scene`, fresh output directory per activation. Do not call any formal
  generator/matrix or select order from results. Inspect the terminal result
  and structural gate diagnostics before the next slot.
- [ ] Record every valid result and separately every proven INVALID retry;
  resolve ordinary runtime bugs with focused reproduction/TDD only. Stop if
  frozen semantics must change or a new structural deadlock is established.

## 3. Analysis, review and stop

- [ ] Generate existing per-round operational and dual-score reports; verify
  actual saved exact-winner support and non-winner diagnostics. Include raw and
  derived evidence, confirmation, D_exec, retrieval and failure stages.
- [ ] Report all three paired binary outcomes/discordants and simulation-clock
  resources with missingness, not formal inference or outcome-driven tuning.
- [ ] Run full core, native Noetic and actual task-map regressions plus frozen
  source/old-output diff checks; independently review claims and completeness.
- [ ] Update results/readiness report and README, retain old data, commit/push
  checkpoint and update Draft PR#6. Close owned processes. Stop; no formal run.
