# Current methods section: scope and source guide

This is an outline for manuscript writing, not a new method or experiment.
The old `docs/PAPER1_SIMULATION_METHODS.md` describes the interrupted development
matrix and must not be used as the current study's methods text.

1. **Robot/task boundary.** Known brick, static horizontal ground, P450 aerial
   RGB-D/MID360, BUNKER+AUBO i5+AG95 with wrist D435. AGENT uses perceived targets,
   public poses/TF and measured control feedback. Explicitly disclose that the SIM
   public map localization uses private Gazebo body state/spawn alignment, while
   grasp confirmation uses simulated contacts. “Sensor-fed decisions” must not
   be expanded into a claim of fully sensor-only localization or tactile estimation.
   The independent physical checker uses simulated object motion only for outcome
   measurement; see `docs/SIM_TO_REAL_READINESS.md` for the exact separation.
2. **Manipulation support.** Unchanged RM4D literature algorithm/robot and separately
   calibrated Ground-task map; record reference-plane bridge. A1 gates and clipped
   joint-margin quality define per-cell relevance; exact validated per-cell winners
   anchor footprints (not cell centers). Unassessed is distinct from infeasible.
   Use current `src/reachability_guided_aerial_perception`, `src/sim_active_perception`
   and exact-support code, not initial A3 representative-pose prose.
3. **Environment and operational geometry.** Real endpoint-window evidence, no
   airborne XY ray carving. Raw A2 state is preserved. Current object-aware-v1.4
   operational evidence independently tracks actual ground presence and blockers;
   target continuous geometry, sub-cell ambiguous endpoints and conservative
   environmental evidence share the exact footprint check. Do not describe the
   old rule “all ambiguous coarse cells permanently block” or raw FREE as the
   sole ground-confirmation criterion. Allowances are declared model/sensor budgets,
   not independently calibrated safety guarantees.
4. **Finite-scan viewpoint policy.** Use the installed MID360 scan pattern and
   publisher angular discretization, finite five-second window opportunity and
   tilted mount; current nominal viewpoint offsets are {-2,-1,0,1,2}m with yaw
   candidates and shared bounds. Unknown does not occlude; modeled heights remain
   surrogates. The common cost and observation opportunity are identical across
   Generic/Ours; only global deficit versus footprint-support weighting differs.
   Do not use the early binary-visibility, one-guaranteed-endpoint description as
   the implemented finite-scan model. See `FINITE_SCAN_DEVELOPMENT.md`, current
   saved rankings and `EVALUATION_VERSION.md`.
5. **Handoff and execution.** Three accepted windows maximum, shared screened-
   candidate stopping after updates, bounded candidate/IK/approach searches and
   actual-arrival revalidation. Mere confirmation is not D_exec or retrieval.
   The common12mm development clearance and SIM feedback correction are shared
   platform/execution settings, not Ours-only contributions. Do not copy the old
   mandatory-budget/no-fallback protocol. Current source/profile and
   `PAPER1_FORMAL_EXPERIMENT_PLAN.md` define the executed behavior.
6. **Evaluation.**96 independent paired scenes,212 planned tasks, two documented
   infrastructure-invalid replacements;214 formal activations plus one external
   bare startup. Frozen seeds/order and realized38/28/30 tiers. Primary endpoint:
   physical retrieval including lift/short retention. Exact aggregate McNemar,
   prespecified conservative paired effect interval, first-activation sensitivity;
   conditional cost estimates and all-N success-resource curves. Auxiliary
   controls/ablations are descriptive. No old560 slots, pilot pooling, stratified
   normal primary interval, post-hoc sample expansion, or extra ablation p-values.

Remaining author work: combine this outline with a concise algorithm description,
equations, related work and the final figures/tables; verify all named constants
against the executed configuration, explain surrogate assumptions, and adapt the
layout to the venue. No new robot runs or new method implementation are required
to correct the manuscript's historical-version mismatch.
