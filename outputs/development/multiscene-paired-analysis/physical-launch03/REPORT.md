# Ground dynamics: offline native-rate / pose comparison

Diagnostic only. Original task outcomes and native feedback are unchanged. Missing/nonfinite values are JSON null with explicit counts; they are not zeros.

| Interval / phase | Rows | Wrist q change, rad | Native dq trapezoid, rad | Relative-quaternion change, rad | Median native dq, rad/s | Median pose rate, rad/s |
|---|---:|---:|---:|---:|---:|---:|
| lift / before_base_set | 13007 | 0.0001362157 | -1.812407 | 0.0001362165 | -0.1424388 | 0.0006153113 |
| lift / after_base_set | 13007 | 0.0001362157 | -1.812407 | 0.0001362165 | -0.1424388 | 0.0006153113 |
| lift / update_end | 13007 | 0.0001665651 | -1.812513 | 0.0001665659 | -0.1424388 | 0.0006153113 |
| lift_tail_2s_proxy / before_base_set | 2001 | 0.0002046685 | -0.2485978 | 0.0002046685 | -0.113332 | 0.0008930993 |
| lift_tail_2s_proxy / after_base_set | 2001 | 0.0002046685 | -0.2485978 | 0.0002046685 | -0.113332 | 0.0008930993 |
| lift_tail_2s_proxy / update_end | 2001 | 0.0001867464 | -0.2485537 | 0.0001867463 | -0.113332 | 0.000885944 |
| retention / before_base_set | 662 | -3.065519e-05 | -0.07652853 | -3.06552e-05 | -0.1124476 | 0.0008266341 |
| retention / after_base_set | 662 | -3.065519e-05 | -0.07652853 | -3.06552e-05 | -0.1124476 | 0.0008266341 |
| retention / update_end | 662 | -1.510645e-05 | -0.07651104 | -1.510646e-05 | -0.1124196 | 0.0008422229 |

## Before/after base setters

| Interval | Paired rows | Max abs wrist dq change, rad/s | Max abs wrist q change, rad | Max base angular-velocity change norm, rad/s |
|---|---:|---:|---:|---:|
| lift | 13007 | 0 | 0 | 0.0007485225 |
| lift_tail_2s_proxy | 2001 | 0 | 0 | 6.823337e-05 |
| retention | 662 | 0 | 0 | 3.186621e-05 |

## Native-rate cadence at update end

| Interval | Mean even / odd iteration dq, rad/s | Negative / positive samples | Largest nonzero FFT bin, Hz |
|---|---|---:|---:|
| lift | -0.1367215 / -0.1419887 | 12657 / 350 | 0.1537634 |
| lift_tail_2s_proxy | -0.1232551 / -0.1253145 | 2001 / 0 | 0.4997501 |
| retention | -0.1147847 / -0.1166987 | 662 / 0 | 193.3535 |

## Target diagnostics at update end

- lift: target XYZ change [0.0012527233240859914, -0.0007709893423821501, 0.14758575675677163] m; target displacement in wrist-child coordinates max 0.003174806 m.
- lift_tail_2s_proxy: target XYZ change [0.00014297055451528706, 0.0034209468781349808, 0.05938001440540783] m; target displacement in wrist-child coordinates max 0.0009337405 m.
- retention: target XYZ change [-6.495561942987393e-06, 1.011045884709641e-06, 2.7820028553893206e-06] m; target displacement in wrist-child coordinates max 1.18577e-05 m.

## Limits

- No native feedback substitution, actuation change, synthetic vote, or new task success is produced.
- Lift tail is the final 2 s before the stage outcome, not an exact commanded trajectory-end/settle-start timestamp.
- Only positive-time consecutive iteration pairs are integrated; gaps and missing values are excluded and counted.
- Left/right/trapezoid dq integrals are diagnostic quadrature; none is asserted to be the physics integration rule.
- Quaternion increments use parent-frame child orientation and project its shortest-arc rotation onto the local hinge axis.
- GetForce is native joint effort, not a measured gripping/contact reaction wrench.
- Other update-begin callbacks may run after this plugin; after_base_set to update_end includes those callbacks and physics.
- Target Model getters and link poses/twists are diagnostic truth only. Relative target movement is not a retention acceptance test.
- Closed/open labels describe commanded gripper states, not proven loaded/unloaded force conditions.
- A success with instrumentation alone does not establish that instrumentation fixed a prior failure.
