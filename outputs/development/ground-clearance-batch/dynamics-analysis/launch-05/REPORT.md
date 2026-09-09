# Ground dynamics: offline native-rate / pose comparison

Diagnostic only. Original task outcomes and native feedback are unchanged. Missing/nonfinite values are JSON null with explicit counts; they are not zeros.

| Interval / phase | Rows | Wrist q change, rad | Native dq trapezoid, rad | Relative-quaternion change, rad | Median native dq, rad/s | Median pose rate, rad/s |
|---|---:|---:|---:|---:|---:|---:|
| lift / before_base_set | 16368 | 0.0007988171 | -1.918234 | 0.0007988169 | -0.1080001 | 0.0005020342 |
| lift / after_base_set | 16368 | 0.0007988171 | -1.918234 | 0.0007988169 | -0.1080001 | 0.0005020342 |
| lift / update_end | 16368 | 0.0008130767 | -1.918316 | 0.0008130765 | -0.1080001 | 0.0005020342 |
| lift_tail_2s_proxy / before_base_set | 2001 | 0.0002860746 | -0.1939311 | 0.0002860746 | -0.09332164 | 0.0007734492 |
| lift_tail_2s_proxy / after_base_set | 2001 | 0.0002860746 | -0.1939311 | 0.0002860746 | -0.09332164 | 0.0007734492 |
| lift_tail_2s_proxy / update_end | 2001 | 0.0002889782 | -0.1939296 | 0.0002889782 | -0.09332164 | 0.0007734492 |
| retention / before_base_set | 830 | -4.306093e-05 | -0.08146728 | -4.306093e-05 | -0.09471638 | 0.0005598874 |
| retention / after_base_set | 830 | -4.306093e-05 | -0.08146728 | -4.306093e-05 | -0.09471638 | 0.0005598874 |
| retention / update_end | 830 | -4.703107e-05 | -0.08146745 | -4.703107e-05 | -0.09471638 | 0.0005598874 |

## Before/after base setters

| Interval | Paired rows | Max abs wrist dq change, rad/s | Max abs wrist q change, rad | Max base angular-velocity change norm, rad/s |
|---|---:|---:|---:|---:|
| lift | 16368 | 0 | 0 | 0.01849818 |
| lift_tail_2s_proxy | 2001 | 0 | 0 | 0.01849818 |
| retention | 830 | 0 | 0 | 2.934414e-05 |

## Native-rate cadence at update end

| Interval | Mean even / odd iteration dq, rad/s | Negative / positive samples | Largest nonzero FFT bin, Hz |
|---|---|---:|---:|
| lift | -0.1150571 / -0.1193484 | 16304 / 64 | 0.06109482 |
| lift_tail_2s_proxy | -0.09548617 / -0.0984482 | 2001 / 0 | 86.95652 |
| retention | -0.09700856 / -0.09954206 | 830 / 0 | 134.9398 |

## Target diagnostics at update end

- lift: target XYZ change [0.00171921455873969, 0.0004635881565693556, 0.14854652504760235] m; target displacement in wrist-child coordinates max 0.002572406 m.
- lift_tail_2s_proxy: target XYZ change [-0.0017431609903575396, 0.0008303980643533659, 0.0544333135371364] m; target displacement in wrist-child coordinates max 1.584332e-05 m.
- retention: target XYZ change [-1.5556373764713527e-05, 5.011799681431395e-06, 1.3320293550750373e-05] m; target displacement in wrist-child coordinates max 9.645988e-06 m.

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
