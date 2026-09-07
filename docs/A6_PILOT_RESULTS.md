# A6 pilot results — in progress

This is the bounded 14-slot pilot, not the formal matrix. A1–A5, the task-domain
RM4D asset, seeds, three-window budget, gates, Ground selection and success
criteria are unchanged. No valid method failure has been rerun or replaced.

## Frozen initialization

All methods/seeds use map sensing destination `(-1.4,0,1.2), yaw=0` and public
launch `(-.5,0,.15), yaw=0`. RGB-D retains its 4 m gate. Sensor-only setup passed
all three original seeds before pilot. See [initialization qualification](A6_INITIALIZATION.md)
and [operational protocol](A6_PILOT_PROTOCOL.md). There is no runtime GT initializer.

## Completed Easy block

At this checkpoint: **4/14 completed, all valid; 0 pilot INVALID activations**.
The nine earlier setup activations are separate sensor/geometry checks, never
pilot primary denominators. Remaining slots continue in their serialized order.

| Seed / method | Final confirmed | D_exec | Retrieval | Earliest failure |
|---|---:|---:|---:|---|
| Easy 2026090801 / Fixed-view | 1 | 1 | 1 | — |
| Easy 2026090801 / Ours | 0 | 0 | 0 | NO_GROUND_HANDOFF: three-window budget |
| Easy 2026090801 / RM4D-only | N/A | 0 | 0 | GROUND_NAVIGATION: original top-1 action timeout |
| Easy 2026090801 / Generic | 2 | 1 | 0 | DESCEND: wrist_3 joint-speed check, 1.1306 |

The sole primary Easy Ours/Generic pair is **neither succeeds**. Generic reached
confirmed support and actual collision-aware refined pregrasp; Ours did not.
This does not turn their equal binary outcome into equivalent stage performance.
Fixed-view physical brick lift was .148615 m and TCP lift .149759 m; the external
checker passed and runtime teardown completed. Controller LIFT alone is not used
as a physical-success substitute.

| Method | Voted windows | Actual NBV move commands | First confirmation (s from active start) | Task simulation time (s) | UAV observed path lower bound (m) |
|---|---:|---:|---:|---:|---:|
| Fixed-view | 3 | 0 | 33.872 | 174.972 | 6.483 |
| Ours | 3 | 2 | not reached | 92.964 | 7.383 |
| RM4D-only | 0 | 0 | N/A | 203.085 | 5.946 |
| Generic | 3 | 1 | 50.649 | 250.393 | 11.566 |

Distances shown here are explicitly observed-segment lower bounds, not complete
paths: missing public TF samples remain recorded. No efficiency advantage is
inferred from faster failure. A policy stay target can still require a physical
small reposition after hover drift; actual move/rescan commands and raw paths
are measured, not inferred from nominal candidate displacement.

## Initial observations for review, not tuning

- Easy Ours' best otherwise-clear exact footprint retained one UNKNOWN cell at
  its last decision (minimum unknown cells across eligible candidates: 103, 10,
  1). Fixed-view's corresponding minima were 104, 5, 0. The full-footprint FREE
  gate was not relaxed; the failure is retained. This is a useful pilot boundary
  case, not evidence that the gate or method should be changed mid-run.
- All nine saved Easy MID360 decision snapshots pass the offline identities
  `U_task=u*M_operational`, shared-mask weighted/unweighted gains and the same
  flight penalty. Seven snapshots have different reconstructed Ours/Generic
  argmax IDs. These are same-state diagnostics, not extra closed-loop trials.
- RM4D-only incurred actual bootstrap/query/return/landing costs despite zero
  MID360 windows; no A2 confirmation or alternate Ground candidate was added.
- Generic's physical descent failure is retained after successful D_exec.
  No joint-speed threshold, planning parameter or success criterion was tuned.

## Engineering work before execution

The setup clock race and Hard origin-spawn collision were resolved before pilot;
all final common initialization checks were repeated without querying RM4D.
The A6 adapter and recorder were reviewed for exception-before-cleanup timing,
actual Ground/refined-pregrasp stage entry and missing measurement handling.
Before first pilot: 324 full unit tests passed; 52 system-Python ROS-boundary
tests passed; the complete ROS adapter import check passed. Source comparison
showed no frozen A1–A5, RM4D asset or SIM changes.

The final report will replace this in-progress checkpoint after all 14 slots,
including all paired outcomes, Hard ablations, descriptive scene checks and
formal-design recommendations. No formal experiment is authorized or launched.
