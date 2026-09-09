# Ground dynamics: offline native-rate / pose comparison

Diagnostic only. Original task outcomes and native feedback are unchanged. Missing/nonfinite values are JSON null with explicit counts; they are not zeros.

| Interval / phase | Rows | Wrist q change, rad | Native dq trapezoid, rad | Relative-quaternion change, rad | Median native dq, rad/s | Median pose rate, rad/s |
|---|---:|---:|---:|---:|---:|---:|
| lift / before_base_set | 13212 | 0.0003620874 | -1.605485 | 0.0003620874 | -0.126953 | 0.0004723758 |
| lift / after_base_set | 13212 | 0.0003620874 | -1.605485 | 0.0003620874 | -0.126953 | 0.0004723758 |
| lift / update_end | 13212 | 0.0003705714 | -1.605564 | 0.0003705715 | -0.126953 | 0.0004748663 |
| lift_tail_2s_proxy / before_base_set | 2000 | 0.0001294765 | -0.1745562 | 0.0001294765 | -0.082445 | 0.0001727551 |
| lift_tail_2s_proxy / after_base_set | 2000 | 0.0001294765 | -0.1745562 | 0.0001294765 | -0.082445 | 0.0001727551 |
| lift_tail_2s_proxy / update_end | 2000 | 0.0001374038 | -0.1745545 | 0.0001374038 | -0.082445 | 0.0001758862 |
| retention / before_base_set | 837 | 5.151075e-06 | -0.07347673 | 5.151079e-06 | -0.08412479 | 0.0001869226 |
| retention / after_base_set | 837 | 5.151075e-06 | -0.07347673 | 5.151079e-06 | -0.08412479 | 0.0001869226 |
| retention / update_end | 837 | -2.031559e-06 | -0.07348295 | -2.031555e-06 | -0.08412866 | 0.0001853894 |

## Before/after base setters

| Interval | Paired rows | Max abs wrist dq change, rad/s | Max abs wrist q change, rad | Max base angular-velocity change norm, rad/s |
|---|---:|---:|---:|---:|
| lift | 13212 | 0 | 0 | 0.0003438993 |
| lift_tail_2s_proxy | 2000 | 0 | 0 | 3.466099e-05 |
| retention | 837 | 0 | 0 | 0.02235372 |

## Native-rate cadence at update end

| Interval | Mean even / odd iteration dq, rad/s | Negative / positive samples | Largest nonzero FFT bin, Hz |
|---|---|---:|---:|
| lift | -0.1239293 / -0.1191231 | 13169 / 43 | 0.07568877 |
| lift_tail_2s_proxy | -0.0883273 / -0.08630813 | 2000 / 0 | 209.5 |
| retention | -0.08931668 / -0.08646911 | 837 / 0 | 201.9116 |

## Target diagnostics at update end

- lift: target XYZ change [0.00027840531233280785, 0.0015236653764615776, 0.14851514199063298] m; target displacement in wrist-child coordinates max 0.00292683 m.
- lift_tail_2s_proxy: target XYZ change [-0.0009067143316010551, 0.0019841413847076356, 0.06686581225410929] m; target displacement in wrist-child coordinates max 0.0001665365 m.
- retention: target XYZ change [-3.8062629394985947e-06, 1.7355757189169152e-05, -5.808805703388931e-06] m; target displacement in wrist-child coordinates max 6.635463e-06 m.

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
