# Ground dynamics: offline native-rate / pose comparison

Diagnostic only. Original task outcomes and native feedback are unchanged. Missing/nonfinite values are JSON null with explicit counts; they are not zeros.

| Interval / phase | Rows | Wrist q change, rad | Native dq trapezoid, rad | Relative-quaternion change, rad | Median native dq, rad/s | Median pose rate, rad/s |
|---|---:|---:|---:|---:|---:|---:|
| lift / before_base_set | 6630 | 0.000608953 | -0.9034329 | 0.0006089529 | -0.1398637 | -0.0008585998 |
| lift / after_base_set | 6630 | 0.000608953 | -0.9034329 | 0.0006089529 | -0.1398637 | -0.0008585998 |
| lift / update_end | 6630 | 0.0006396261 | -0.9035777 | 0.000639626 | -0.1398845 | -0.0008533137 |
| lift_tail_2s_proxy / before_base_set | 2001 | -5.803749e-05 | -0.3369518 | -5.803752e-05 | -0.1683261 | -0.0001794488 |
| lift_tail_2s_proxy / after_base_set | 2001 | -5.803749e-05 | -0.3369518 | -5.803752e-05 | -0.1683261 | -0.0001794488 |
| lift_tail_2s_proxy / update_end | 2001 | -3.914791e-05 | -0.336966 | -3.914794e-05 | -0.1683261 | -0.0001298796 |
| closed_hold / before_base_set | 2024 | -4.322328e-05 | -0.3344305 | -4.322328e-05 | -0.1631213 | 0.002991099 |
| closed_hold / after_base_set | 2024 | -4.322328e-05 | -0.3344305 | -4.322328e-05 | -0.1631213 | 0.002991099 |
| closed_hold / update_end | 2024 | -2.314198e-05 | -0.3344234 | -2.314197e-05 | -0.1631213 | 0.003054769 |
| release_attempt_failed / before_base_set | 1004 | 7.164066e-05 | -0.02589406 | 7.164072e-05 | -0.001912229 | 0.0001768606 |
| release_attempt_failed / after_base_set | 1004 | 7.164066e-05 | -0.02589406 | 7.164072e-05 | -0.001912229 | 0.0001768606 |
| release_attempt_failed / update_end | 1004 | 5.98973e-05 | -0.02573387 | 5.989736e-05 | -0.001912229 | 0.0001457165 |

## Before/after base setters

| Interval | Paired rows | Max abs wrist dq change, rad/s | Max abs wrist q change, rad | Max base angular-velocity change norm, rad/s |
|---|---:|---:|---:|---:|
| lift | 6630 | 0 | 0 | 0.01487534 |
| lift_tail_2s_proxy | 2001 | 0 | 0 | 0.01432258 |
| closed_hold | 2024 | 0 | 0 | 0.01429775 |
| release_attempt_failed | 1004 | 0 | 0 | 9.334628e-05 |

## Native-rate cadence at update end

| Interval | Mean even / odd iteration dq, rad/s | Negative / positive samples | Largest nonzero FFT bin, Hz |
|---|---|---:|---:|
| lift | -0.1378842 / -0.1347141 | 6580 / 50 | 0.1508296 |
| lift_tail_2s_proxy | -0.1705555 / -0.1663983 | 2001 / 0 | 0.4997501 |
| closed_hold | -0.1706312 / -0.1599894 | 2024 / 0 | 500 |
| release_attempt_failed | -0.024814 / -0.02660545 | 598 / 406 | 0.9960159 |

## Target diagnostics at update end

- lift: target XYZ change [-0.00023500297900835854, 0.001100534413709206, 0.1490227958172452] m; target displacement in wrist-child coordinates max 0.001620153 m.
- lift_tail_2s_proxy: target XYZ change [0.0001631270686786035, -0.00019717940511468357, -9.214230397808221e-05] m; target displacement in wrist-child coordinates max 0.0002612264 m.
- closed_hold: target XYZ change [0.00015561471545511019, -0.00030552195074269595, -4.836729436036302e-05] m; target displacement in wrist-child coordinates max 0.0003540725 m.
- release_attempt_failed: target XYZ change [0.0005252366941017605, -0.0032703832658248777, 0.0004565533001137656] m; target displacement in wrist-child coordinates max 0.003317276 m.

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
