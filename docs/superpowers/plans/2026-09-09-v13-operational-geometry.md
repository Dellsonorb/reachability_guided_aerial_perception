# v1.3 operational geometry implementation plan

> **For agentic workers:** Use subagent-driven-development for the isolated
> core task and independent reviews; the root integrates coupled runtime paths.

**Goal:** Replace AMBIGUOUS whole-cell aliases with endpoint-local declared
engineering geometry, preserving real collision, actual support and historical
results, then verify perception-to-handoff consistency on development inputs.

**Architecture:** Extend the existing operational view with an optional small
endpoint sidecar. Its absence retains v1.1 exactly. A3 and A5 call the same
footprint function. Explicit `--operational-gating v1.3` activates the new view;
legacy config files and old results are not changed. Public-map conditional
disk radius33 mm, named and recorded, independent of association allowance.

**Tech stack:** existing NumPy/dataclasses, unittest, NPZ/JSON, ROS Noetic/SIM
only for the existing natural regression. No dependencies or framework added.

## Boundary and budget

User's latest autonomous-development authorization supersedes per-module
freeze/approval barriers, not historical result preservation, fair shared
components or final-test protocol. This plan uses offline development replay
and the existing natural A5 regression only. Remaining fresh validation and
all formal slots stay paused. No new sample-size/matrix decision here.
Keep old A6 branch at its existing checkpoint; work on
`feature/v13-operational-geometry` in the user-designated checkout. Existing
unrelated untracked A5 outputs are untouched.

## Task 1 — endpoint sidecar and shared continuous gate

Files: create `src/operational_gating/subcell.py`, tests
`tests/test_ambiguous_subcell.py`; modify only
`src/operational_gating/core.py` and package exports.

- [x] Write failing analytic tests: .033 m disk disjoint/tangent/intersecting
  against rotated padded rectangle; multiple endpoints within one vote; an
  adjacent-cell disk reaching the rectangle; missing/partial historical groups
  retaining cell fallback; later ground never deleting ambiguity; raw vote
  arrays and old assessment dictionaries unchanged without opt-in.
- [x] Run `PYTHONPATH=src:tests .../conda-env/bin/python -m unittest
  test_ambiguous_subcell -q` and confirm expected missing-feature failures.
- [x] Implement optional sidecar with `points_xy`, original observation/row
  indices, cell IDs, per-cell complete window-vote counts and named33 mm profile.
  Point counts never replace window votes. Arrays must align, be finite and
  detached; complete counts cannot exceed original ambiguous votes.
- [x] Add optional `retain_ambiguous_endpoints=False` to
  `derive_operational_evidence`; collect every accepted AMBIGUOUS endpoint at
  the same existing filters, with original row IDs. Do not change any counts.
- [x] Keep `FootprintAssessment` legacy fields/schema. Compute blocked from
  shared continuous endpoint test + unrepresented legacy ambiguity cells +
  unchanged environment cells and target collision. Expose a small separate
  subcell diagnostic for endpoint hit/fallback/alias counts.
- [x] Run new tests and existing operational/target tests; spec then quality
  review before integration.

## Task 2 — explicit runtime opt-in, persistence and shared semantics

Files: `src/operational_gating/io.py`, `src/task_relevant_uncertainty/{core,outputs}.py`,
`src/sim_active_perception/core.py`, `scripts/run_a5_sim.py`, focused tests.

- [x] Add failing parser/worker tests for `v1.3`; v1/v1.1 defaults remain exact.
  Require an operational view semantic property rather than hardcoded v1.1 in
  A3/A5; v1.3 assessments must agree on the identical exact pose.
- [x] Accept explicit v1.3 request, reuse initial perception reference and all
  existing observations. Save endpoint arrays, identities, complete vote counts
  and conditional profile alongside existing operational NPZ, plus metadata.
  Missing old sidecars remain legacy, never synthesize cell-center endpoints.
- [x] Attach new per-candidate subcell diagnostics only in v1.3 responses so
  old snapshot schemas replay identically. Update object-aware visualization
  recognition without relabeling A2 raw states.
- [x] Run parser/worker/shared A3/A5 tests and old exact snapshot reporter.

## Task 3 — development replay, perception/active/handoff consistency

Files: read-only `scripts/diagnose_v13_ambiguous_geometry.py`, test, small
development replay report under `outputs/diagnostics/v13-development`.

- [x] Reporter mismatched source/count arrays raise ValueError rather than
  assertion-only errors; test a genuinely modified temporary saved vote array.
- [x] Replay Moderate using source observations with new sidecar, outside the
  historical attempt directory. Verify all old A2/class/ground arrays identical;
 29 target collisions remain; all five exact-separated aliases assessed using
  all109 endpoints; present current support deficits without claiming handoff.
- [x] Rebuild A3 and A4 through normal functions; record shared Generic/Ours
  scoring consistency, task support and whether further sensing is predicted.
  Check raw occupancy's visibility/ground-vote interaction for remaining
  structural inconsistency; diagnose and minimally iterate if demonstrated,
  rather than changing outcomes or treating the initial33 mm as a success knob.
- [x] Replay Hard-002 exact anchor/87-of88 support, v1.1 regression fixtures,
  old natural states, and non-winner diagnostics without reselection.
- [x] Run focused and full core regression once after integration; retain
  old snapshots/outcomes unchanged and separate new diagnostic outputs.

## Task 4 — existing natural A5 regression and delivery

- [x] Read `docs/V12_INTEGRATION_CHECKPOINT.md` and reuse the established natural
  launch/cleanup commands with a fresh development output directory and explicit
  v1.3. Keep observer, hover, actual collision/grasp/lift conditions and budgets.
- [x] Confirm public readiness; run one natural E2E regression. Reproduce and
  minimally fix ordinary orchestration defects if needed, preserving failed
  development attempts. Do not consume any fresh-validation/formal slot.
- [x] Independent spec/code review, necessary regressions, scoped diff review.
- [x] Document geometric/confirmation/active/E2E results and remaining limits.
  Publication follows these completed checks: commit/push the development
  branch; PR #6 stays Draft and main untouched. Report concentrated development
  results, not final-test efficacy.

Task 4 engineering addition: post-core capture XYZ anchor is established during
its unchanged bounded stable dwell, not frozen at an uncommanded transient
sample. Commanded flight arrival stays strict; yaw is not reanchored. Five
focused regressions and independent review pass. Two temporary startup-runner
errors and the activated pre-capture hover failure are retained separately;
the repaired fourth launch completes natural grasp/lift. See the development
checkpoint for numeric results and final verification.
