# Reusable task handoff implementation plan

> **For agentic workers:** Use subagent-driven-development for the independent SIM correction and requesting-code-review before online execution. Main agent owns integration and online runs.

**Goal:** A clear current task entry with reliable observation checking and shared stop-on-screened-candidate behavior.

**Architecture:** Keep existing numerical worker and ROS adapter boundaries; add one explicit stop mode and a thin single-task entry. SIM checker bookkeeping is a separate platform patch. No new framework.

**Tech Stack:** Existing Python 3.8 ROS Noetic adapter, Python 3.10 numerical core, MoveIt, Gazebo/PX4.

## Tasks

Execution note: implementation, reviews and the bounded four starts are complete;
see `docs/TASK_HANDOFF_DEVELOPMENT_RESULTS.md`. The original pre-run checklist
below is retained as the plan, not a claim of outstanding implementation. The
design addendum explicitly reallocates starts 3/4 to the runtime-corrected
Moderate pair. The final `460a810` callback-readiness fix has offline verification
only; broader project completion and online validation of that fix remain open.

- [ ] SIM `scripts/check_air_ground_pick_demo.py`, `tests/test_air_ground_pick_demo.py`: test actual callbacks with an early valid observation plus 6000 later observations; verify current failure; replace only air/ground `deque(maxlen=5000)` with lossless per-invocation retention; rerun original invalid age/frame/stamp tests and demo suite. Separate feature branch/commit.
- [ ] AGENT `scripts/a5_execution_selection.py`, `scripts/run_a5_sim.py`, `scripts/run_a6_sim.py`, tests: add explicit shared stopping helper/flag. Test no-confirmation continues, accepted confirmed preview stops, rejection continues if budget permits, terminal failure is retained, exceptions propagate, legacy behavior unchanged. Use `--handoff-stop screened_candidate` only in new development entry. Emit actual common stop event for metrics before selection; do not call a successful screen twice.
- [ ] AGENT task entry/profile and `scripts/run_a6_attempt.py`: regression-test current flags, method-independent common arguments, read-only command preview, no historical/formal activation, and two missing diagnostic topics. One task per invocation. Preserve legacy CLI defaults and require unused output directory. Use the existing ROS environment wrapper, never duplicate platform startup.
- [ ] Offline Hard first-state masks: enumerate at most two visibility opportunities on saved exact viable footprints, report best possible support deficits and actual chosen sequence. Keep it under development outputs, not in runtime.
- [ ] Run focused tests, independent spec/code review, then commit current runtime before online runs. Execute the four design-specified starts only. Inspect real outcomes, first failure, intervention, windows and control behavior. No automatic rerun.
- [ ] Update README to current task usage, preserve A1/history as linked documentation, record branch commits/config and result limitations. Run relevant regression, review, commit/push checkpoint; no merge/formal execution.
