# Ground dynamics: offline native-rate / pose comparison

Diagnostic only. Original task outcomes and native feedback are unchanged. Missing/nonfinite values are JSON null with explicit counts; they are not zeros.

| Interval / phase | Rows | Wrist q change, rad | Native dq trapezoid, rad | Relative-quaternion change, rad | Median native dq, rad/s | Median pose rate, rad/s |
|---|---:|---:|---:|---:|---:|---:|
| lift / before_base_set | 4667 | 0.0004145421 | -0.5801177 | 0.000414542 | -0.1322476 | 0.0001904201 |
| lift / after_base_set | 4667 | 0.0004145421 | -0.5801177 | 0.000414542 | -0.1322476 | 0.0001904201 |
| lift / update_end | 4667 | 0.0003788118 | -0.5802924 | 0.0003788117 | -0.1322811 | 0.0001904201 |
| lift_tail_2s_proxy / before_base_set | 2001 | 7.401922e-05 | -0.2618038 | 7.401916e-05 | -0.1313796 | -0.0004249195 |
| lift_tail_2s_proxy / after_base_set | 2001 | 7.401922e-05 | -0.2618038 | 7.401916e-05 | -0.1313796 | -0.0004249195 |
| lift_tail_2s_proxy / update_end | 2001 | 3.182978e-05 | -0.2618527 | 3.182971e-05 | -0.1314323 | -0.0005059831 |
| retention / before_base_set | 798 | 2.50626e-05 | -0.1098219 | 2.506258e-05 | -0.1390077 | -0.001552163 |
| retention / after_base_set | 798 | 2.50626e-05 | -0.1098219 | 2.506258e-05 | -0.1390077 | -0.001552163 |
| retention / update_end | 798 | 9.848841e-05 | -0.109791 | 9.848839e-05 | -0.138852 | -0.001545269 |

## Before/after base setters

| Interval | Paired rows | Max abs wrist dq change, rad/s | Max abs wrist q change, rad | Max base angular-velocity change norm, rad/s |
|---|---:|---:|---:|---:|
| lift | 4667 | 0 | 0 | 0.02097888 |
| lift_tail_2s_proxy | 2001 | 0 | 0 | 0.02097888 |
| retention | 798 | 0 | 0 | 2.859586e-05 |

## Native-rate cadence at update end

| Interval | Mean even / odd iteration dq, rad/s | Negative / positive samples | Largest nonzero FFT bin, Hz |
|---|---|---:|---:|
| lift | -0.1250807 / -0.1236356 | 4597 / 70 | 0.2142704 |
| lift_tail_2s_proxy | -0.1318902 / -0.1299855 | 2001 / 0 | 32.48376 |
| retention | -0.13814 / -0.1373964 | 798 / 0 | 30.07519 |

## Target diagnostics at update end

- lift: target XYZ change [-0.0004380158052414984, 0.0011620316597361346, 0.14889604166353787] m; target displacement in wrist-child coordinates max 0.001466165 m.
- lift_tail_2s_proxy: target XYZ change [-0.0018055781869827925, 0.0011137046147272067, 0.06472295838526151] m; target displacement in wrist-child coordinates max 0.0002374677 m.
- retention: target XYZ change [-1.8339472576123228e-06, 0.000135993815746277, -2.1667565745786543e-06] m; target displacement in wrist-child coordinates max 0.0001446026 m.

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
