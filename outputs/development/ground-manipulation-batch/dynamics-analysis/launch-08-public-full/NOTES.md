# Launch08: bounded offline dynamics validation

This completed Moderate/Ours trial used integrated pose-interval feedback, full-robot manipulation planning, and the trial contact geometry recorded by the root runner. It was conditioned on archived aerial confirmation and arrival; it is not a fresh navigation or end-to-end perception trial. No simulator starts, controller/physics changes, or original-result edits were performed by this analysis. Target/body states below are diagnosis only, never algorithm geometry inputs.

## Outcome and physical evidence

Lift was reached and completed: event interval 37.517–43.050 s; full retention 43.050–43.631 s. The original physical summary is CHECKS_PASS with grasp confirmed, target lift 148.902299 mm, and TCP lift 149.537324 mm. In event-bounded raw CSV, target rise during lift is 148.218548 mm and wrist-child rise is 149.188930 mm; the difference from the summary uses a different initial anchor. Maximum target-in-wrist displacement during lift is 2.163613 mm. During the complete 0.581 s retention, target z changes +4.443530 µm and target-in-wrist displacement is at most 0.040430 mm. These pose measurements support an actual lifted and retained object, separately from controller SUCCESS.

## Public feedback and missing-data checks

Runtime logs explicitly confirm `integrated_position_interval_average`. All six arm joint-state and controller-state actual position/velocity samples match same-header before-base-set raw positions and preceding consecutive 1 ms pose increments exactly (maximum stored-value error 0). After-base-set references also match exactly. No interpolation, timestamp shift, velocity-based filtering, or quiet-window selection is applied. All seven public velocity streams are finite; the gripper has no raw per-step joint-angle column and is therefore descriptive, not independently validated at 1 ms.

| Whole phase | Event interval, s | Joint-state / controller samples per arm joint | Raw rows per phase | dt/iteration gaps / missing geometry |
|---|---|---:|---:|---:|
| observation | 11.772–26.176 | 1440 / 1440 | 14405 | 0 / 0 |
| pregrasp | 26.176–30.82 | 465 / 465 | 4645 | 0 / 0 |
| descend | 30.822–36.21 | 539 / 539 | 5389 | 0 / 0 |
| close | 36.21–37.517 | 130 / 131 | 1308 | 0 / 0 |
| lift | 37.517–43.05 | 554 / 554 | 5534 | 0 / 0 |
| retention | 43.05–43.631 | 58 / 59 | 582 | 0 / 0 |

JSON also retains the update-end reference, all raw phases, native comparisons, public positions, and exact matched/unmatched counts. Whole-phase joint-state and controller samplings occur at different header times; their observed speed maxima can differ without a matching error.

## Public joint-state speeds: all seven joints

Absolute speed p95 / maximum in rad/s.

| Joint | observation | pregrasp | descend | close | lift | retention |
|---|---|---|---|---|---|---|
| shoulder_pan_joint | 0.177952 / 0.189786 | 0.0674175 / 0.0697537 | 0.000128582 / 0.00142401 | 0.00124122 / 0.00218906 | 0.000992639 / 0.00775243 | 4.99513e-05 / 9.46404e-05 |
| shoulder_lift_joint | 0.110824 / 0.123376 | 0.218774 / 0.225182 | 0.0752349 / 0.0788412 | 0.00195288 / 0.00284197 | 0.075526 / 0.0788706 | 0.000233358 / 0.000388358 |
| elbow_joint | 0.304141 / 0.339004 | 0.133652 / 0.145398 | 0.0415316 / 0.0450298 | 0.00500556 / 0.0107852 | 0.0406553 / 0.0728357 | 0.000631214 / 0.000980553 |
| wrist_1_joint | 0.0652123 / 0.0741118 | 0.360718 / 0.367523 | 0.0574291 / 0.0603147 | 0.00924152 / 0.0182135 | 0.0563948 / 0.11417 | 0.00621956 / 0.0079646 |
| wrist_2_joint | 0.289067 / 0.30651 | 0.0154571 / 0.0170327 | 0.00118663 / 0.00783247 | 0.00602212 / 0.0107079 | 0.010789 / 0.112396 | 0.00452178 / 0.00578837 |
| wrist_3_joint | 0.234578 / 0.24469 | 0.0709762 / 0.0720894 | 0.000236109 / 0.00483532 | 0.00513387 / 0.0247175 | 0.0388222 / 0.457883 | 0.0391401 / 0.0526377 |
| left_outer_knuckle_joint | 0.106221 / 0.466734 | 0.215589 / 0.218734 | 0.0017732 / 0.0137224 | 0.296663 / 1.25153 | 0.142173 / 0.520154 | 0.0444511 / 0.165183 |

## Controller actual speeds: all seven joints

Absolute speed p95 / maximum in rad/s.

| Joint | observation | pregrasp | descend | close | lift | retention |
|---|---|---|---|---|---|---|
| shoulder_pan_joint | 0.177912 / 0.201596 | 0.0674678 / 0.0697618 | 0.000125728 / 0.00138859 | 0.00124041 / 0.00218151 | 0.000960778 / 0.00764798 | 6.48077e-05 / 0.000173172 |
| shoulder_lift_joint | 0.1108 / 0.123419 | 0.218765 / 0.225143 | 0.075245 / 0.0788122 | 0.00197603 / 0.00283387 | 0.0754446 / 0.0788204 | 0.000416609 / 0.000558597 |
| elbow_joint | 0.30406 / 0.339168 | 0.133516 / 0.145414 | 0.0415376 / 0.0450184 | 0.00488967 / 0.0109095 | 0.0407949 / 0.0734719 | 0.000795568 / 0.000884187 |
| wrist_1_joint | 0.0639618 / 0.0730404 | 0.3609 / 0.367592 | 0.0574228 / 0.0603365 | 0.00831124 / 0.0196792 | 0.0558175 / 0.116161 | 0.00373162 / 0.00526697 |
| wrist_2_joint | 0.289227 / 0.316122 | 0.0154999 / 0.0171355 | 0.00116648 / 0.00645397 | 0.00633345 / 0.0108856 | 0.0106231 / 0.111526 | 0.00256773 / 0.00509943 |
| wrist_3_joint | 0.233965 / 0.2449 | 0.0709777 / 0.072136 | 0.000249122 / 0.00405686 | 0.00807844 / 0.0313248 | 0.0397585 / 0.489222 | 0.0279747 / 0.035838 |
| left_outer_knuckle_joint | 0.100144 / 0.464944 | 0.215632 / 0.219053 | 0.00163594 / 0.0125827 | 0.295399 / 0.498441 | 0.166338 / 0.512487 | 0.186076 / 0.242353 |

## Independent wrist motion and native-rate discrepancy

Relative parent/child quaternion increments remove parent motion, are projected on the parent-local wrist axis, and retain the orthogonal residual. This validates a second kinematic representation within the same simulator, not an independent physical sensor. Other arm joints have raw q references, not individual relative-quaternion evidence.

| Whole phase | Signed wrist travel, rad | q-rate minus quaternion-rate p95 / max, rad/s | Off-axis rate max, rad/s | Pose speed abs p95 / max, rad/s |
|---|---:|---|---:|---|
| observation | 2.05929146 | 9.84913e-06 / 0.00451965 | 0.00787409 | 0.234005 / 0.276517 |
| pregrasp | -0.115176086 | 6.98405e-11 / 1.00659e-09 | 0.00087785 | 0.0710275 / 0.072136 |
| descend | 2.88723982e-06 | 3.8849e-11 / 3.4234e-10 | 0.000852914 | 0.000261536 / 0.00987114 |
| close | -0.000185512022 | 3.86761e-10 / 2.04354e-08 | 0.00362115 | 0.0066436 / 0.0421944 |
| lift | 0.000166655717 | 2.33037e-08 / 1.01747e-07 | 0.00819326 | 0.0403611 / 0.489222 |
| retention | -1.67897995e-05 | 2.24627e-08 / 6.59017e-08 | 0.00222 | 0.0330387 / 0.0526377 |

The wrist really rotates +2.059291 rad during whole observation and −0.115176 rad during pregrasp. Target z remains unchanged during both; the accepted observation records measured opening 94.946884 mm. This is nonzero pre-close motion, not stationary-only validation. All quaternion discrepancies, including the 0.00451965 rad/s observation maximum, remain included.

| Whole loaded interval, update end | q change, rad | Native dq integral, rad | Native dq median, rad/s |
|---|---:|---:|---:|
| lift | 0.000166655717 | -0.45796258 | -0.0797909773 |
| retention | -1.67897995e-05 | -0.0540402763 | -0.0919903536 |

The native-rate/pose discrepancy has not disappeared. Retention native wrist dq is entirely negative (−0.141717…−0.074706 rad/s), while physical one-step pose speed changes sign and reaches absolute maximum 0.0526377 rad/s. Raw physics feedback is preserved; the interval-average adapter is not a repair of the solver or an assertion that both velocity semantics are interchangeable.

## Endpoint boundaries and limits

The lift goal has zero trajectory header, goal-id stamp 37.541 s and duration 5.460104969 s. Its nominal endpoint under the explicit goal-stamp start convention is 43.001104969 s. Public result header is 43.043 s (receipt 43.044), and stage END is 43.050 s: only 48.895 ms postnominal and 7 ms postresult before retention. For a zero-stamped trajectory the actual acceptance instant is not directly logged, so the nominal endpoint is not claimed to be a directly observed controller-start time. The native report retains its clearly labelled two-second tail proxy, but that proxy includes ongoing lift motion and is not used here as a settled phase.

No failed-lift contrast ran, and no post-release unloaded hold exists. Retention lasts only 0.581 s simulated time. This single changed-station/configuration success does not establish reproducibility, all-joint quaternion equivalence, gripper 1 ms reference equivalence, long-term holding, isolated causal effects of contact geometry, or aggregate reliability. Planning/collision reasoning is reviewed separately by the camera/planning agent; this report does not infer it from success alone. Budget is exhausted and no further online action is suggested or taken here.

Generated with the existing offline analyzers into new launch08 directories. Only this new report title was corrected from the helper's fixed launch05 heading; no prior result or analysis script was modified. Companion `summary.json` and `REPORT.md` retain whole event phases plus the explicitly nominal/result-bound short intervals.
