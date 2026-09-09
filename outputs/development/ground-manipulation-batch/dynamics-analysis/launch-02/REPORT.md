# Ground dynamics: offline native-rate / pose comparison

Diagnostic only. Original task outcomes and native feedback are unchanged. Missing/nonfinite values are JSON null with explicit counts; they are not zeros.

| Interval / phase | Rows | Wrist q change, rad | Native dq trapezoid, rad | Relative-quaternion change, rad | Median native dq, rad/s | Median pose rate, rad/s |
|---|---:|---:|---:|---:|---:|---:|
| lift / before_base_set | 6618 | -6.80345e-06 | -0.9670271 | -6.803457e-06 | -0.1476133 | 0.0004600854 |
| lift / after_base_set | 6618 | -6.80345e-06 | -0.9670271 | -6.803457e-06 | -0.1476133 | 0.0004600854 |
| lift / update_end | 6618 | 9.262978e-06 | -0.9671607 | 9.26297e-06 | -0.1476221 | 0.0004600854 |
| lift_tail_2s_proxy / before_base_set | 2001 | -1.632549e-05 | -0.336523 | -1.632548e-05 | -0.1660393 | 0.001245418 |
| lift_tail_2s_proxy / after_base_set | 2001 | -1.632549e-05 | -0.336523 | -1.632548e-05 | -0.1660393 | 0.001245418 |
| lift_tail_2s_proxy / update_end | 2001 | -1.788077e-05 | -0.3365345 | -1.788077e-05 | -0.1660435 | 0.001245418 |
| closed_hold / before_base_set | 2008 | -4.08249e-05 | -0.3188489 | -4.082483e-05 | -0.1615973 | 0.0006020527 |
| closed_hold / after_base_set | 2008 | -4.08249e-05 | -0.3188489 | -4.082483e-05 | -0.1615973 | 0.0006020527 |
| closed_hold / update_end | 2008 | -3.681685e-05 | -0.3187668 | -3.681678e-05 | -0.1615973 | 0.0006106024 |

## Before/after base setters

| Interval | Paired rows | Max abs wrist dq change, rad/s | Max abs wrist q change, rad | Max base angular-velocity change norm, rad/s |
|---|---:|---:|---:|---:|
| lift | 6618 | 0 | 0 | 0.02129947 |
| lift_tail_2s_proxy | 2001 | 0 | 0 | 2.281355e-05 |
| closed_hold | 2008 | 0 | 0 | 0.01441871 |

## Target diagnostics at update end

- lift: target XYZ change [-0.0004175518590205929, 0.0018762909681823425, 0.14930145610270334] m; target displacement in wrist-child coordinates max 0.001556775 m.
- lift_tail_2s_proxy: target XYZ change [9.41387011714756e-05, -6.140480895283629e-06, -7.554415017158034e-05] m; target displacement in wrist-child coordinates max 0.000107564 m.
- closed_hold: target XYZ change [-0.00015066901663285392, 0.0006841166524764186, 3.161234899601295e-05] m; target displacement in wrist-child coordinates max 0.0007194541 m.

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
