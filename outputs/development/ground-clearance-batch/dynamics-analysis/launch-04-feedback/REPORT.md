# Clearance batch launch04: public integrated feedback and physical motion

Runtime integrated-mode log confirmed: True. The raw native CSV is retained separately; no native values are rewritten.

| Window | Wrist unwrapped travel, rad | Public joint-state wrist speed abs p95 / max, rad/s | Max public minus 1 ms before-set pose rate, rad/s | Native wrist dq median, rad/s |
|---|---:|---|---:|---:|
| ground_refine_before_close | -1.0618867 | 0.13122056 / 0.1351388 | 0 | -0.030487104 |
| refined_pregrasp_before_close | -0.13530488 | 0.092152781 / 0.10003537 | 0 | -9.1592894e-06 |
| descend | -1.5923024e-05 | 0.00068346148 / 0.0029426478 | 0 | -4.1804476e-05 |
| close | -0.00037069735 | 0.0052799732 / 0.041976901 | 0 | -3.7796899e-08 |
| lift | 0.00036208736 | 0.03797908 / 0.51713025 | 0 | -0.12695305 |
| retention | 5.1510746e-06 | 0.019593245 / 0.035881347 | 0 | -0.084124788 |
| post_nominal_lift_endpoint_0 | -2.3900165e-05 | 0.020789112 / 0.023913573 | 0 | -0.085503511 |
| post_controller_result_0 | -1.6821334e-05 | 0.003083834 / 0.003083834 | 0 | -0.086552262 |

## Physical target and wrist-child motion

- ground_refine_before_close (11.913–29.197 s): target XYZ change [0.0, -8.131378903541986e-11, 0.0] m; wrist-child XYZ change [0.25635018483003713, 0.6420052702127765, -0.24140214801000792] m; target-in-hand displacement max 2.342453411992998 m.
- refined_pregrasp_before_close (29.197–45.264 s): target XYZ change [0.0, -7.55883272640645e-11, 0.0] m; wrist-child XYZ change [0.0769033651871538, 0.002442461724382311, -0.22816609125065113] m; target-in-hand displacement max 0.2417749446723157 m.
- descend (45.267–51.408 s): target XYZ change [0.0, -2.8890764780520328e-11, 0.0] m; wrist-child XYZ change [0.00034848445535118344, -0.0007326812400633848, -0.1498101523705646] m; target-in-hand displacement max 0.14983099483871962 m.
- close (51.409–52.709 s): target XYZ change [8.09051378332093e-05, -0.0013961292645434525, 0.0006285158169761357] m; wrist-child XYZ change [-2.7651620597701765e-05, 0.0001122474227670811, 0.00032296471908221935] m; target-in-hand displacement max 0.001895959933687993 m.
- lift (52.71–65.921 s): target XYZ change [0.00027840531233280785, 0.0015236653764615776, 0.14851514199063298] m; wrist-child XYZ change [-0.00032149240137879787, 0.0006622688459254344, 0.1493972538077465] m; target-in-hand displacement max 0.0029268304028475457 m.
- retention (65.922–66.758 s): target XYZ change [-3.8062629394985947e-06, 1.7355757189169152e-05, -5.808805703388931e-06] m; wrist-child XYZ change [-4.966081338597661e-06, 1.2835385204384986e-05, -9.271596508930724e-06] m; target-in-hand displacement max 6.635462933595846e-06 m.
- post_nominal_lift_endpoint_0 (65.87838157099999–65.921 s): target XYZ change [1.4244911112015757e-05, -2.4893121522734374e-05, -1.99432348587103e-05] m; wrist-child XYZ change [7.680571295409777e-06, -1.2149194890606618e-05, -1.964475662752374e-05] m; target-in-hand displacement max 8.957298615132588e-06 m.
- post_controller_result_0 (65.912–65.921 s): target XYZ change [2.461914590856651e-06, -5.8813989685518875e-06, -1.8972433187824933e-06] m; wrist-child XYZ change [1.3940706939941094e-06, -1.582889199769344e-06, -1.4798623955702972e-06] m; target-in-hand displacement max 6.8878610064535175e-06 m.

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
