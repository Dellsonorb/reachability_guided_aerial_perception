# Launch05: public integrated feedback and physical motion

Runtime integrated-mode log confirmed: True. The raw native CSV is retained separately; no native values are rewritten.

| Window | Wrist unwrapped travel, rad | Public joint-state wrist speed abs p95 / max, rad/s | Max public minus 1 ms before-set pose rate, rad/s | Native wrist dq median, rad/s |
|---|---:|---|---:|---:|
| ground_refine_before_close | 2.4652872 | 0.36080802 / 0.4084121 | 0 | 0.25432999 |
| refined_pregrasp_before_close | -3.196277 | 0.36130572 / 0.37773325 | 0 | -0.35990196 |
| lift | 0.0004145421 | 0.039166872 / 0.31447282 | 0 | -0.13224758 |
| retention | 2.5062604e-05 | 0.046403451 / 0.069024358 | 0 | -0.1390077 |
| post_nominal_lift_endpoint_0 | -4.7269442e-05 | 0.061383153 / 0.069024358 | 0 | -0.13671175 |
| post_controller_result_0 | -0.00020482089 | 0.069024358 / 0.069024358 | 0 | -0.15316353 |

## Physical target and wrist-child motion

- ground_refine_before_close (11.875–24.564 s): target XYZ change [-1.4087619959468611e-11, -1.0354400670209429e-10, 0.0] m; wrist-child XYZ change [-0.30423358079973517, -0.3057804852921709, -0.24175465289131637] m; target-in-hand displacement max 0.8757557571912274 m.
- refined_pregrasp_before_close (24.565–34.321 s): target XYZ change [-1.0831335828243027e-11, -7.961031833758625e-11, 0.0] m; wrist-child XYZ change [0.07423483034894796, 0.008284109596626343, -0.24176698443724448] m; target-in-hand displacement max 0.8269245571068163 m.
- lift (40.345–45.011 s): target XYZ change [-0.0004380158052414984, 0.0011620316597361346, 0.14889604166353787] m; wrist-child XYZ change [-0.0006895979170697952, 3.295139138187464e-05, 0.14937884055749545] m; target-in-hand displacement max 0.001466165403197661 m.
- retention (45.011–45.808 s): target XYZ change [-1.8339472576123228e-06, 0.000135993815746277, -2.1667565745786543e-06] m; wrist-child XYZ change [-8.380040374511566e-06, -5.967623734060012e-07, 2.695067642999316e-06] m; target-in-hand displacement max 0.0001446026452600846 m.
- post_nominal_lift_endpoint_0 (44.961554258–45.011 s): target XYZ change [2.8705027715680842e-05, -8.434638604504219e-07, -2.1856038172951564e-05] m; wrist-child XYZ change [1.2301182326091009e-05, -5.667901390393748e-06, -1.8958566894655693e-05] m; target-in-hand displacement max 1.6635240455928293e-05 m.
- post_controller_result_0 (45.002–45.011 s): target XYZ change [6.578933650080998e-06, 5.071791974065043e-06, -1.394558901579579e-06] m; wrist-child XYZ change [1.093383982464502e-06, 2.403865620614898e-06, -1.1264899758001157e-06] m; target-in-hand displacement max 1.1924598379916829e-05 m.

## Limits

- All whole event-defined observation/pregrasp/lift/retention phases are retained; no quiet subwindow or value selection is used.
- Post-nominal endpoint uses explicit trajectory stamp if nonzero, otherwise goal-id stamp, plus final duration. Public result-to-stage-END is separately reported.
- Expected feedback reference is the before-base-set WorldUpdateBegin pose difference over the preceding consecutive physics step. Same-header after/end references are also reported, not selected for best fit.
- Public messages are matched by their header nanoseconds to rounded CSV simulation time. No interpolation, timestamp shifts or bag-receipt alignment is used.
- All seven public joint position/velocity distributions are reported. The gripper has no raw per-step CSV column, so its 1 ms derivative is not independently validated by this CSV.
- Public coarse-position differences are descriptive averages over the public message interval, not the 1 ms hardware feedback calculation.
- Raw native ODE rate and integrated-pose feedback have different semantics. The native discrepancy remains visible and is not overwritten.
- Wrist relative-quaternion validation is independent of the native joint angle; other arm joints have raw position references but no simultaneous per-joint quaternion columns.
- Physical target/TCP motion and retention evidence are separate from controller action success; no additional acceptance threshold is applied.
