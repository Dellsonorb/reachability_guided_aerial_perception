# Exact-pose Support Anchoring Implementation Plan

> **For agentic workers:** Use subagent-driven-development for the bounded core
> implementation and independent reviews; root owns replay diagnostics and SIM.
> The user already authorized this design and routine decisions. Keep the
> existing A6 feature checkout; do not ask again or start slot17.

**Goal:** Implement/verify v1.2 exact-winner support without changing A1/A2/A4
definitions or requiring the recorded Hard-002 state to reach handoff.

**Architecture:** Deterministic candidate-only reconstruction supplies A3 and
A5 with one common exact anchor. Explicit version selection retains historical
cell-center replay. A small offline regression reports non-winner viability
without feeding it to the policy.

**Tech Stack:** Existing NumPy/core Python3.10, Noetic Python3.8 boundary,
Matplotlib and existing SIM/RM4D interfaces; no new dependencies/framework.

## Task1 — core and worker integration (bounded implementation agent)

Files: new `src/task_relevant_uncertainty/anchors.py`; modify A3 core/outputs/
exports, `src/sim_active_perception/core.py` and worker,
`src/a6_pilot/worker.py`, `scripts/run_a5_sim.py`; new
`tests/test_exact_support.py` plus focused existing integration tests if needed.

- [x] RED: reconstruct the existing two-candidate same-cell fixture from
  `tests.test_a5_core.inputs`; assert first tied candidate wins, exact XY is
  not cell center, and a higher later relevance replaces it. Saturated margins
  must not break a relevance tie. Corrupt field relevance/yaw or omit a feasible
  cell's candidate and require ValueError.
- [x] GREEN: implement a frozen WinnerAnchor record and stable loop:

```python
value = candidate_relevance(candidate, field.config)
cell = field.grid.cell_index(candidate['bunker_x'], candidate['bunker_y'])
if cell is not None and value > 0:
    if cell not in winners or value > winners[cell].relevance:
        winners[cell] = WinnerAnchor(
            source_id=cell[0] * field.grid.width_cells + cell[1],
            candidate_id=candidate['candidate_id'], evaluation_index=index,
            x=float(candidate['bunker_x']), y=float(candidate['bunker_y']),
            yaw=float(candidate['bunker_yaw']), relevance=value)
# Return original evaluation-index order, after field/cell/R/yaw checks.
```

- [x] RED: call `build_task_uncertainty(field,belief,
  evaluated_candidates=raw['evaluated_candidates'],operational=view)` on
  an exact-clear/center-colliding synthetic target. Require its pose.xy to
  equal the winner; exact block false, legacy center block true. Real missing
  ground votes must still prevent A5 confirmation; genuine collision/E/A
  blocks remain. Empty field/evaluations must not silently use another pose.
- [x] GREEN: use recovered source-ID anchor lookup only for the A3 xy/yaw
  assignment; retain row-major source/max/tie projection and every evidence
  equation. Add exact-only metadata, without changing legacy PoseSupport JSON.
- [x] RED/GREEN: make `candidate_catalog` use the same reconstruction. Add
  A5Config.support_anchor and a shared task-building helper used in decide,
  A5 worker and A6 worker saving/rendering. Wire `--support-anchor exact_winner`
  into the existing initial config; default remains legacy, frozen JSON unchanged.
- [x] RED/GREEN: test all five policy routes and both numerical workers for
  exact anchor metadata and saved A3 input consistency; Generic/Ours candidate
  geometry, visibility, cost and exact assessments must agree on the same state.
- [x] Run `PYTHONPATH=src:tests CORE -m unittest test_exact_support
  test_a5_core test_task_uncertainty test_operational_gating
  test_operational_worker test_a6_policy -q`.
  Report observed RED failures and GREEN commands, inspect diff, commit core.
- [x] Independent spec review, address findings; then independent code-quality
  review. No method expansion for a failing test.

## Task2 — saved-state regression (root, alongside core work)

Files: `scripts/replay_exact_support.py`, `tests/test_exact_support_replay.py`;
outputs only in a new `outputs/a6/exact-anchor-v12/recorded-replay` tree.

- [x] RED/GREEN: build a pure diagnostic for winner-blocked/non-winner-unblocked
  using original gate-valid candidates and `assess_footprint`. Test a same-cell
  blocked winner/clear non-winner and prove catalog/winner/selection do not change.
- [x] Replay saved observations with existing `make_field`, `replay_observations`
  and `build_operational_context`. Reconstruct legacy and exact tasks from the
  same field/belief/view, compare raw A2 and operational arrays to saved arrays,
  and save separate derived per-round counts/IDs and unsmoothed geometry/field
  figures. Do not edit initial.json, events, scores, attempts or physical outcomes.
- [x] On slot15 Hard-002, assert source624 exact viable and unconfirmed with
  its original87/88 support, no representative displacement collision, positive
  operational support/uncertainty. Replay slot16 too; do not force confirmation.
- [x] Evaluate A4 with identical current/candidates/visibility/cost on old/new
  A3 inputs. Confirm only support-derived task gains can change; no scoring
  formula, generic gain or frozen setting changed. Record non-winner counts.
- [x] Run focused replay tests and the standalone replay command; independently
  review outputs and all five original v1.1 gating regression groups.

## Task3 — natural SIM, review and checkpoint (root)

- [x] Use exact original natural commands in `docs/OBJECT_AWARE_GATING_V11.md`,
  fresh `outputs/a6/exact-anchor-v12/natural-a5-attempt-01`, and only add the
  new exact-anchor flag. Serial runtime/checker/adapter, owned11951/11952 ports.
- [x] Inspect initial config, actual anchors, real observation windows, Ground
  selection, navigation/refine/pregrasp/descend/close/lift and independent physical
  outcome. Failed regression is retained; only demonstrated engineering bugs
  can be repaired without tuning. Shut down owned runtime afterward.
- [x] Run full core suite and Noetic split, actual task-map tests as relevant,
  read-only legacy slot15 reconstruction through the new replay (do not run
  the old script that writes original artifacts), A4 input/scoring checks and
  source-boundary diff.
- [x] Write `docs/EXACT_POSE_SUPPORT_V12.md` with equations/API/tie rules,
  before/after Hard evidence, non-winner diagnostic, natural result and limits.
  Append the interruption designation to formal-results documentation without
  rewriting the16 original outcomes or frozen configuration.
- [x] Independent final review of core, replay, startup repair and honest natural
  failure classification. Handoff instruction: commit/push the reviewed
  checkpoint, keep Draft PR, and stop for user review. No slot17, formal
  restart, new seeds or v1.2 statistics.

## Observed acceptance gap

Core `8aea2de`, synthetic checks and all six Hard-002 saved windows pass.
Natural launches 01/02 fail before activation on the status publisher gap;
the minimal reviewed repair preserves the topic's reference while replacing
the publisher and starts the checker after adapter readiness. Launch 03 is a
genuine activated regression failure on the unchanged first-window hover
stability condition. No MID360 window or A3/A4 action runs. It is not invalidated
or retried for a favorable result. Natural E2E acceptance remains **OPEN/FAIL**;
completed checkboxes above mean the checks were performed, not that this live
acceptance passed. No full v1.2 acceptance or formal readiness is claimed.
