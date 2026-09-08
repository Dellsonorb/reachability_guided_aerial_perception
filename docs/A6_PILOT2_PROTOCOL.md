# Pilot-2 — independent v1.1 validation and conditional formal readiness

2026-09-08. User authorization: autonomous simulation research. This document
supersedes the *stop-after-Pilot-1* authority only; Pilot-1 protocol/results and
six execution-failure classifications stay untouched. Pilot-2 is descriptive,
not pooled into future formal data. No formal sample size is selected yet.

## Method and initialization freeze

The research implementation is `56ece4a`: A1/A2/A4 and RM4D/task-domain asset
unchanged; approved A3/A5 `object-aware-v1.1` explicitly enabled for every
environment-gated method. `operational_gating` code is not tuned or modified.
Generic/Ours share generator/order, geometry/visibility, cost, 3-window budget,
A2 updater, operational gate, exact Ground selection and execution pipeline.
Only generic unknown vs task-relevant gain weighting differs.

Freeze the implemented sensor/pixel allowance rule and all its constants in
`src/operational_gating/association.py`. The prior natural-scene allowance
.0439527451 m is one measured-geometry output, not a replacement constant.
Different accepted image depths/intrinsics use the same deterministic rule;
record every derived value, never fit it to a method outcome or rescue count.
Perceived known-size object collision uses the same expanded geometry.

Common sensing pose remains map (-1.4,0,1.2), yaw=0; common UAV launch remains
(-.5,0,.15), yaw=0. Setup qualification reuses the existing RGB-D acceptance,
5 s raw MID360 ground-return check (at least 100 distinct .10 m cells), and
public hover/TF readiness. It never runs RM4D, scores, candidate selection or
retrieval. No new pose search or seed replacement is planned.

All unchanged rules are inherited from `A6_PILOT_PROTOCOL.md` sections 1–5:
3 total completed 5 s windows, 20 s capture guard, common method-specific-gain
stopping, no first-confirmed early stop, one Ground handoff, same execution
timeouts/thresholds, INVALID-versus-method-failure policy and simulation-time
resource metrics. RM4D-only retains original top-one selection and zero MID360
windows; environment discovery/mechanism is N/A, not zero. Its physical outcome
and D_exec are measured normally. Raw RGB-D may be recorded passively.

The sole deliberate Ground-gate replacement is v1.1: real ground-support votes
plus shared operational blocking, not all raw cells FREE. ENVIRONMENT and
AMBIGUOUS block; true continuous perceived-target intersection blocks; TARGET
cell aliasing alone does not. Preserve the representative guard, catalog
winner/tie ordering and exact map pose. Confirmation still is not D_exec.

## Independent seeds and order — fixed before any method outcome

The root seed draw is `random.Random(202609082).sample(range(1,2**31),3)`.
Its first three values, in Easy/Moderate/Hard order, are:

| Tier | New seed |
|---|---:|
| Easy | 1015873452 |
| Moderate | 1240085801 |
| Hard | 656391333 |

No alternative draw is evaluated. These are disjoint from Pilot-1 seeds
2026090801/02/03 and reserved from future formal sampling. Each scene uses
`random.Random(scene_seed)` and the original six draws, in order: target dx/dy
uniform ±.15 m, target yaw uniform ±30 degrees, BUNKER dx/dy uniform ±.10 m,
BUNKER yaw uniform ±5 degrees about pi. Centers remain target (2,0) and BUNKER
(3,-2.5); all z, brick/model/dynamics and tier-specific boxes are copied exactly
from Pilot-1 configuration. Serialized configuration is the runtime input.

Shuffle `[rm4d_only,fixed,generic,ours]` once with `Random(202609082)`, yielding
that same order; rotate left by tier index for each subsequent block. Four
methods therefore have position counts differing by at most one across tiers.
The two Hard secondary ablations run last in `no_occlusion,no_cost` order.
There are 14 planned slots; w/o task weighting is Generic, not another slot.

Difficulty criteria and descriptive checks are unchanged. No score gap,
candidate count, confirmation, result or method win selects/replaces a scene.
Setup failures are diagnosed as setup, not hidden method outcomes. If nominal
geometry is outside the frozen admitted domain, document it before method
execution; do not quietly resample. After execution begins, valid method
failures stay failures. Physics and planner residual randomness remain
uncontrolled where the platform does not expose a seed.

## v1.1 mechanism observations (offline, no policy feedback)

For every completed environment window retain existing raw observation, target
reference/status, raw A2 snapshot, derived operational arrays, exact candidate
assessments, A3 summary and common ranking. The new read-only report records:

- Per-class occupied vote sums **and** occupied cell counts for TARGET,
  ENVIRONMENT and AMBIGUOUS; mixed classes are not exclusive cell counts.
- Exact raw-grid-blocked candidate IDs/count (`occupied_cells>0`) and separate
  representative blocked diagnostics.
- Exact object-aware blocked IDs/count and causes (environment, ambiguous,
  continuous expanded-target collision); combined representative/exact gating.
- `target_alias_retained_exact`: raw occupied overlap, positive TARGET overlap,
  no operational exact blocker and no continuous target intersection.
  Separately report whether its representative is blocked, footprint clipped,
  actual ground votes complete, and final confirmation. Retained is not rescued
  retrieval, nor necessarily an executable candidate.
- Confirmed IDs/count and first discovery; existing metrics supply
  confirmed → D_exec → physical retrieval. D_exec still requires actual Ground
  arrival, D435 refinement and successfully executed refined pregrasp.

Missing reference/round is explicit missing, never guessed TARGET or zero.
RM4D-only is N/A. Same-state score identities remain checked by the existing
`a6_scoring_diagnostics.py`; no controller uses this report or scene GT.
There is no requirement for a natural seed to demonstrate an alias rescue;
positive-association synthetic regressions already test that mechanism.

## Readiness and continuation

After each activation inspect terminal status, outcome measurement and any
new failure. Only independently demonstrated INVALID activations are rerun in
a new directory with the identical slot/seed/method. Do not overwrite any
activation or change a valid failure. Ordinary logging/orchestration/platform
bugs may be fixed, tested and their affected runs identified; do not mix
different platform versions in a primary pair without a documented rerun.

After Pilot-2, formal readiness requires evidence that the gate behaves as
defined, no sensing-irreparable structural handoff deadlock remains, sharing
and primary outcome measurement are intact, failures are classifiable, and no
core method hypothesis is contradicted. Merely reaching 14 slots or obtaining
one success is insufficient. Low power or an unexciting method comparison does
not itself authorize tuning or scene redesign.

Distinguish genuine physical/modelled collisions and insufficient finite-budget
ground coverage from a structural gate problem. Investigate persistent blocking
of continuously separated perceived geometries, including ambiguous target
boundary evidence and representative/exact disagreement. A proven structural
method deadlock triggers the user's scientific stop rule immediately; do not
launch formal runs to average it away.

If all conditions clearly hold, write/freeze a separate formal protocol and
independent seeds before formal activation. Preplan sample size and paired
binary inference for the sole primary comparison (Ours vs Generic E2E), report
discordant pairs, secondary resources and prespecified ablations. Pilot-2 is
not used as an interim formal efficacy look. Formal execution and manuscript-
ready analysis are authorized only under these readiness conditions.

Stop for a scientific decision if: v1.1 structural method deadlock; required
core A1–A4 definition change; irreparable Generic/Ours unfairness; required
result-driven formal seed/budget/endpoint/success change; required runtime GT;
or evidence systematically contradicting the Paper-1 core hypothesis. All
ordinary engineering decisions remain autonomous. No artifact locks, SHA
audits, security/lifecycle/verifier framework or new method is part of this work.
