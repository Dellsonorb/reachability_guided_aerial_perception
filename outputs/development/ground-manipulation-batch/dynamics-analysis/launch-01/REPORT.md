# Ground dynamics: offline native-rate / pose comparison

Diagnostic only. Original task outcomes and native feedback are unchanged. Missing/nonfinite values are JSON null with explicit counts; they are not zeros.

| Interval / phase | Rows | Wrist q change, rad | Native dq trapezoid, rad | Relative-quaternion change, rad | Median native dq, rad/s | Median pose rate, rad/s |
|---|---:|---:|---:|---:|---:|---:|
| lift / before_base_set | 4660 | 0.0004512702 | -0.5843794 | 0.0004512702 | -0.1322757 | 0.0001313215 |
| lift / after_base_set | 4660 | 0.0004512702 | -0.5843794 | 0.0004512702 | -0.1322757 | 0.0001313215 |
| lift / update_end | 4660 | 0.0004419011 | -0.5845183 | 0.000441901 | -0.1322967 | 0.0001313215 |
| lift_tail_2s_proxy / before_base_set | 2001 | 0.0001844718 | -0.263825 | 0.0001844718 | -0.1320214 | -0.0001297037 |
| lift_tail_2s_proxy / after_base_set | 2001 | 0.0001844718 | -0.263825 | 0.0001844718 | -0.1320214 | -0.0001297037 |
| lift_tail_2s_proxy / update_end | 2001 | 0.0001840524 | -0.2638214 | 0.0001840523 | -0.1320214 | -0.0001297037 |
| retention / before_base_set | 648 | -0.000167632 | -0.08858533 | -0.0001676321 | -0.1370073 | -6.822312e-05 |
| retention / after_base_set | 648 | -0.000167632 | -0.08858533 | -0.0001676321 | -0.1370073 | -6.822312e-05 |
| retention / update_end | 648 | -0.0001040441 | -0.0885723 | -0.0001040441 | -0.1370073 | 4.432787e-05 |

## Before/after base setters

| Interval | Paired rows | Max abs wrist dq change, rad/s | Max abs wrist q change, rad | Max base angular-velocity change norm, rad/s |
|---|---:|---:|---:|---:|
| lift | 4660 | 0 | 0 | 0.02111113 |
| lift_tail_2s_proxy | 2001 | 0 | 0 | 0.02111113 |
| retention | 648 | 0 | 0 | 2.602015e-05 |

## Target diagnostics at update end

- lift: target XYZ change [-0.0004317551815096188, 0.0011697456614832291, 0.14889403744566962] m; target displacement in wrist-child coordinates max 0.0014552 m.
- lift_tail_2s_proxy: target XYZ change [-0.0018403094887680105, 0.001145945574014151, 0.06511997732480027] m; target displacement in wrist-child coordinates max 0.0002402627 m.
- retention: target XYZ change [5.8946716574581615e-06, 8.001873244806168e-05, -5.852874712442002e-06] m; target displacement in wrist-child coordinates max 8.868411e-05 m.

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
