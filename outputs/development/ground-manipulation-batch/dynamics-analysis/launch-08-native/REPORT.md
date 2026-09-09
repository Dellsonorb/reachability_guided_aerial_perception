# Ground dynamics: offline native-rate / pose comparison

Diagnostic only. Original task outcomes and native feedback are unchanged. Missing/nonfinite values are JSON null with explicit counts; they are not zeros.

| Interval / phase | Rows | Wrist q change, rad | Native dq trapezoid, rad | Relative-quaternion change, rad | Median native dq, rad/s | Median pose rate, rad/s |
|---|---:|---:|---:|---:|---:|---:|
| lift / before_base_set | 5534 | 0.0001807483 | -0.4578517 | 0.0001807479 | -0.07978142 | -0.0004542947 |
| lift / after_base_set | 5534 | 0.0001807483 | -0.4578517 | 0.0001807479 | -0.07978142 | -0.0004542947 |
| lift / update_end | 5534 | 0.0001666557 | -0.4579626 | 0.0001666554 | -0.07979098 | -0.0004542947 |
| lift_tail_2s_proxy / before_base_set | 2001 | 6.692308e-05 | -0.1507261 | 6.692288e-05 | -0.07278464 | 0.0001869976 |
| lift_tail_2s_proxy / after_base_set | 2001 | 6.692308e-05 | -0.1507261 | 6.692288e-05 | -0.07278464 | 0.0001869976 |
| lift_tail_2s_proxy / update_end | 2001 | 5.252635e-05 | -0.1507641 | 5.252617e-05 | -0.07278464 | 0.0001869976 |
| retention / before_base_set | 582 | -3.271131e-05 | -0.0540698 | -3.271128e-05 | -0.0920691 | 0.0003661168 |
| retention / after_base_set | 582 | -3.271131e-05 | -0.0540698 | -3.271128e-05 | -0.0920691 | 0.0003661168 |
| retention / update_end | 582 | -1.67898e-05 | -0.05404028 | -1.67898e-05 | -0.09199035 | 0.0003661168 |

## Before/after base setters

| Interval | Paired rows | Max abs wrist dq change, rad/s | Max abs wrist q change, rad | Max base angular-velocity change norm, rad/s |
|---|---:|---:|---:|---:|
| lift | 5534 | 0 | 0 | 0.01967203 |
| lift_tail_2s_proxy | 2001 | 0 | 0 | 0.01967203 |
| retention | 582 | 0 | 0 | 3.315286e-05 |

## Native-rate cadence at update end

| Interval | Mean even / odd iteration dq, rad/s | Negative / positive samples | Largest nonzero FFT bin, Hz |
|---|---|---:|---:|
| lift | -0.08197108 / -0.08355648 | 5453 / 81 | 0.1807011 |
| lift_tail_2s_proxy | -0.07473651 / -0.07604098 | 2001 / 0 | 0.4997501 |
| retention | -0.09327504 / -0.09274739 | 582 / 0 | 34.36426 |

## Target diagnostics at update end

- lift: target XYZ change [0.0013070445441405454, 0.0007029849221880796, 0.14821854840866222] m; target displacement in wrist-child coordinates max 0.002163613 m.
- lift_tail_2s_proxy: target XYZ change [-0.0008861565673354832, 0.0015684291562029773, 0.050657232693366944] m; target displacement in wrist-child coordinates max 0.0002598652 m.
- retention: target XYZ change [-4.1794763325775364e-05, 1.7753643222484516e-06, 4.443530470438217e-06] m; target displacement in wrist-child coordinates max 4.042956e-05 m.

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
