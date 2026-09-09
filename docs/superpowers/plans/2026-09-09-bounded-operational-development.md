# Bounded operational development plan

Goal: determine whether continuous geometry, operational evidence, sensing
and Ground handoff still contradict each other. Design and run budget are in
`docs/DEV_OPERATIONAL_BATCH.md`; no formal experiments or additional approval
process. Work sequentially against one SIM; independent read-only/code review
may run alongside it.

- [x] Serialize eight slots and fresh development seeds; add minimal explicit
  DEVELOPMENT_ATTEMPT record kind in `scripts/run_a6_attempt.py`, test legacy
  kinds remain unchanged. Check RGB-D setup without selecting on outcomes.
- [x] Run Moderate/Hard known regressions; replay their runtime observations
  using `scripts/replay_v13_operational.py`. Diagnose any new contradiction
  before changing method code; test-first for any necessary repair.
- [x] Complete the three Generic/Ours pairs on a common version. Use no more
  than four reserved launches for documented issues, at most twelve total.
- [x] Correct diagnostic reporting for endpoint-local AMBIGUOUS blockers,
  preserve historical missing-provenance fallback. Review actual saved fields.
- [x] Produce per-attempt/pair results, diagnostic replay and structural issue
  assessment; run relevant regressions, independent review, commit/push the
  development checkpoint. Keep formal matrix stopped and old results intact.

All twelve starts exhausted, including invalid preflight01 and startup11;
slot08 completed in launch12. No online launches remain. Results and review:
`docs/DEV_OPERATIONAL_BATCH_RESULTS.md`. SIM namespace patch is source-only,
offline tested; its planned online check was cancelled for the invalid rerun.
