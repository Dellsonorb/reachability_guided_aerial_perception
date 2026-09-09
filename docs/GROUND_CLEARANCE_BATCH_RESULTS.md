# Ground clearance development results

Status: **in progress, 1/6 Gazebo starts used**. No formal experiments. Earlier
Ground manipulation results remain preserved without reclassification.

## Natural latest-code reference

`launch-01-natural-latest`: AGENT `7320b9e`, SIM `625b84c`, Easy/Ours,
integrated-pose SIM feedback and full-robot manipulation explicitly enabled.
No replay, candidate override, or manual station. Original result:
**VALID_TRIAL / retrieval failed**.

- Normal P450 perception and three MID360 windows produced confirmed counts
  **0 → 1 → 5**. Two NBV moves occurred. Active flight distance3.25856m and
  active interval43.507sim seconds; total UAV distance11.0870m.
- Program selected exact winner source585 / candidate-000006 at
  `[2.430140806, -0.621187969, 2.268928028]`.
- BUNKER navigation and parking completed in35.077sim seconds. Actual arrival
  `[2.417332605, -0.677964202, 2.199209587]` passed the unchanged arrival checks.
- Refine failed before camera movement: `target planning-frame transform is
  stale`. Therefore **confirmed=yes → D_exec=no → retrieval=no**; the latest
  contact-height and payload checks were configured but not reached. This run
  cannot demonstrate latest integrated grasp/lift completion.

The recorded public TF chain remained fresh: age17ms at failure, dynamic gaps
at most21ms. Native replay plus installed startup/TF behavior support delayed
subscriber processing during MoveIt construction, rather than missing platform
localization. SIM `157f296` adds bounded freshness waiting, preserving the500ms
freshness threshold and original500ms wait budget. The exact stale cached stamp
inside the failed process was not logged. Review added a delayed-wake deadline
test; fresh data arriving after the budget is still rejected. Online verification
of this correction will use a local run, not an immediate full-chain retry.

## Independent clearance basis

Prior launches05–08 contain26,683 paired desired/actual arm samples. Maximum
corresponding collision-box point displacement due to arm tracking is11.490625mm;
the common planning-only chassis allowance is **12mm**, rounded up at1mm.
Maximum TCP error11.266851mm. These are development observations, not a
calibrated bound for every branch/load or sensing error.

Native MoveIt reproduces source624's old planned0.404698mm finger/chassis gap
and its measured model collision. A copy of the authoritative chassis box
expanded12mm rejects that old plan before motion. Checked home and observation
endpoints remain valid. Only root-rigid structural links are exempt from this
planning copy; arm-mounted camera/pads and prospective payload are not.
Physical robot geometry, true collision checks and actual grasp constraints
remain unchanged. Other collision pairs do not acquire a claimed positive margin.

AG95 arm common-mode motion cancels internally; a global24mm self-clearance
requirement is not justified. Real loaded master articulation reached0.521260rad,
so the new prospective closure/load check extends to0.522rad without changing
the0.70rad force goal or replacing real grasp confirmation. Ground perception
repeatability is not a calibrated pose covariance and is not used to invent a
general target inflation.

Diagnostics and reproducible scripts are under
`outputs/development/ground-clearance-batch/offline-{errors,moveit,tf}`. Native
bags and physics CSV remain local diagnostics only, never algorithm input.
Remaining local and final natural-run results will be appended here.
