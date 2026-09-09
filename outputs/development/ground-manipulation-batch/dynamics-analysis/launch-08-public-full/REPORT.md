# Launch08: public integrated feedback and physical motion

Runtime integrated-mode log confirmed: True. The raw native CSV is retained separately; no native values are rewritten.

| Window | Wrist unwrapped travel, rad | Public joint-state wrist speed abs p95 / max, rad/s | Max public minus 1 ms before-set pose rate, rad/s | Native wrist dq median, rad/s |
|---|---:|---|---:|---:|
| ground_refine_before_close | 2.0592934 | 0.23457822 / 0.24468971 | 0 | 0.22882114 |
| refined_pregrasp_before_close | -0.11517576 | 0.070976203 / 0.072089412 | 0 | -0.0017116368 |
| descend | 2.4639287e-06 | 0.00023610929 / 0.0048353154 | 0 | 1.5151766e-05 |
| close | -0.00018389527 | 0.0051338691 / 0.024717524 | 0 | -3.3058557e-08 |
| lift | 0.00018074827 | 0.038822249 / 0.45788317 | 0 | -0.079781424 |
| retention | -3.2711312e-05 | 0.039140097 / 0.052637705 | 0 | -0.092069098 |
| post_nominal_lift_endpoint_0 | 1.7668713e-07 | 0.02295399 / 0.02790634 | 0 | -0.090274001 |
| post_controller_result_0 | -4.0030748e-06 | 0.02790634 / 0.02790634 | 0 | -0.093217037 |

## Physical target and wrist-child motion

- ground_refine_before_close (11.772–26.176 s): target XYZ change [0.0, -6.776462724289445e-11, 0.0] m; wrist-child XYZ change [0.260133832790592, 0.6438390555770472, -0.24373621186568117] m; target-in-hand displacement max 1.5764643923182848 m.
- refined_pregrasp_before_close (26.176–30.82 s): target XYZ change [0.0, -2.1848023390447224e-11, 0.0] m; wrist-child XYZ change [0.07332424846973251, 1.0285068093709993e-05, -0.22541758026621622] m; target-in-hand displacement max 0.23930615444942285 m.
- descend (30.822–36.21 s): target XYZ change [0.0, -2.5348223520182955e-11, 0.0] m; wrist-child XYZ change [9.065003211672362e-05, -0.00021691478331942182, -0.1496874291941685] m; target-in-hand displacement max 0.14969689893051624 m.
- close (36.21–37.517 s): target XYZ change [8.776535086996162e-05, -0.001506369578877051, 0.000675783023483735] m; wrist-child XYZ change [-2.3873013476283234e-05, 0.0003820012582564092, 0.00030535962475458645] m; target-in-hand displacement max 0.0024681817126466113 m.
- lift (37.517–43.05 s): target XYZ change [0.0013070445441405454, 0.0007029849221880796, 0.14821854840866222] m; wrist-child XYZ change [5.6256251030450954e-05, -0.00024179919802955196, 0.14918892955754542] m; target-in-hand displacement max 0.002163613310688295 m.
- retention (43.05–43.631 s): target XYZ change [-4.1794763325775364e-05, 1.7753643222484516e-06, 4.443530470438217e-06] m; wrist-child XYZ change [-5.90130888600271e-07, 6.5121736495532545e-06, 7.596459939418487e-06] m; target-in-hand displacement max 4.042955917805698e-05 m.
- post_nominal_lift_endpoint_0 (43.001104969–43.05 s): target XYZ change [1.597975491574033e-05, -4.168784303285045e-05, -1.639243274159652e-05] m; wrist-child XYZ change [8.66097903529095e-06, -1.4270889158118916e-05, -1.7819531240637065e-05] m; target-in-hand displacement max 2.1123003964325224e-05 m.
- post_controller_result_0 (43.043–43.05 s): target XYZ change [1.9249525471387585e-06, -3.575096946720202e-06, 3.5665507660265217e-06] m; wrist-child XYZ change [6.109325121350651e-07, -1.1410932423544518e-06, -1.0809345100670775e-06] m; target-in-hand displacement max 1.0079897670535674e-05 m.

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
