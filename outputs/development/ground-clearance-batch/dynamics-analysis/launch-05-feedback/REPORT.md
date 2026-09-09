# Clearance batch launch05: public integrated feedback and physical motion

Runtime integrated-mode log confirmed: True. The raw native CSV is retained separately; no native values are rewritten.

| Window | Wrist unwrapped travel, rad | Public joint-state wrist speed abs p95 / max, rad/s | Max public minus 1 ms before-set pose rate, rad/s | Native wrist dq median, rad/s |
|---|---:|---|---:|---:|
| ground_refine_before_close | 2.4647653 | 0.36076688 / 0.40615529 | 0 | 0.09951216 |
| refined_pregrasp_before_close | -0.049218743 | 0.027887547 / 0.033606381 | 0 | 3.7905119e-09 |
| descend | 3.0807912e-05 | 0.0001209044 / 0.0020697588 | 0 | 8.7465524e-06 |
| close | -0.00075558363 | 0.0081687801 / 0.021306378 | 0 | -9.9512175e-08 |
| lift | 0.00079881712 | 0.046869896 / 0.33430065 | 0 | -0.10800011 |
| retention | -4.3060934e-05 | 0.011579655 / 0.017051289 | 0 | -0.094716383 |
| post_nominal_lift_endpoint_0 | 9.5040213e-06 | 0.01345624 / 0.014229645 | 0 | -0.094542408 |
| post_controller_result_0 | -5.5822373e-06 | 0.006553245 / 0.006553245 | 0 | -0.096459508 |

## Physical target and wrist-child motion

- ground_refine_before_close (175.975–190.398 s): target XYZ change [-1.6012746684168633e-11, -1.1769368812863945e-10, 0.0] m; wrist-child XYZ change [-0.1999326010539375, -0.2260752650909927, -0.24135894888849374] m; target-in-hand displacement max 0.7563732569407078 m.
- refined_pregrasp_before_close (190.398–210.609 s): target XYZ change [-2.243871755069904e-11, -1.6492457399763794e-10, 0.0] m; wrist-child XYZ change [0.07931033971684909, 0.013282914435804816, -0.22868751475510402] m; target-in-hand displacement max 0.2419184409055339 m.
- descend (210.614–217.668 s): target XYZ change [-7.831513215705854e-12, -5.756162213543803e-11, 0.0] m; wrist-child XYZ change [0.0002519126600015742, -0.00022206717525541375, -0.14962641807278165] m; target-in-hand displacement max 0.14964305908476788 m.
- close (217.668–218.984 s): target XYZ change [0.000190076368312031, -0.0008556853301200373, 0.00039045724596271925] m; wrist-child XYZ change [-1.5884875408156418e-05, 0.00024315229979776642, 0.0002589191034731986] m; target-in-hand displacement max 0.0015222026213364535 m.
- lift (218.985–235.352 s): target XYZ change [0.00171921455873969, 0.0004635881565693556, 0.14854652504760235] m; wrist-child XYZ change [0.0010039128634047323, -0.0006520652267597571, 0.14917990675533627] m; target-in-hand displacement max 0.0025724059554542 m.
- retention (235.352–236.181 s): target XYZ change [-1.5556373764713527e-05, 5.011799681431395e-06, 1.3320293550750373e-05] m; wrist-child XYZ change [-4.233226462035944e-06, 1.6045252591578851e-06, 1.0783181553675192e-05] m; target-in-hand displacement max 9.645988403124268e-06 m.
- post_nominal_lift_endpoint_0 (235.295475872–235.352 s): target XYZ change [2.8173543718645888e-05, -2.1562220855864123e-05, -1.5705311426833513e-05] m; wrist-child XYZ change [1.4861326812054543e-05, -1.1519046687974432e-05, -2.205043811037699e-05] m; target-in-hand displacement max 6.676651505632553e-06 m.
- post_controller_result_0 (235.345–235.352 s): target XYZ change [8.658171561926054e-07, -8.985111920045696e-07, -7.470014967703165e-08] m; wrist-child XYZ change [6.196172814831868e-07, -3.7578288819561934e-07, -7.090177460700176e-07] m; target-in-hand displacement max 7.481069412598914e-07 m.

## Limits

- All whole event-defined observation/pregrasp/descent/close/lift/retention phases are retained; no quiet subwindow or value selection is used.
- Post-nominal endpoint uses explicit trajectory stamp if nonzero, otherwise goal-id stamp, plus final duration. Public result-to-stage-END is separately reported.
- Expected feedback reference is the before-base-set WorldUpdateBegin pose difference over the preceding consecutive physics step. Same-header after/end references are also reported, not selected for best fit.
- Public messages are matched by their header nanoseconds to rounded CSV simulation time. No interpolation, timestamp shifts or bag-receipt alignment is used.
- All seven public joint position/velocity distributions are reported. The gripper has no raw per-step CSV column, so its 1 ms derivative is not independently validated by this CSV.
- Public coarse-position differences are descriptive averages over the public message interval, not the 1 ms hardware feedback calculation.
- Raw native ODE rate and integrated-pose feedback have different semantics. The native discrepancy remains visible and is not overwritten.
- Wrist relative-quaternion validation is independent of the native joint angle; other arm joints have raw position references but no simultaneous per-joint quaternion columns.
- Physical target/TCP motion and retention evidence are separate from controller action success; no additional acceptance threshold is applied.
