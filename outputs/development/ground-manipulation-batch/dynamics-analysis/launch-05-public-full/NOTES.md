# Launch05 validation notes

Scope: completed opt-in integrated-feedback launch05 only; all prior trials and native traces remain unchanged. Runtime confirms `integrated_position_interval_average`. This is evidence for one bounded physical trial, not a reliability claim or automatic adoption.

## Exact public feedback comparison

For every recorded arm sample below, both joint position and velocity match the same-header **before-base-set** CSV reference exactly (maximum error 0 at stored precision). Velocity reference is the preceding consecutive 1 ms position increment, not a difference over the public 10 ms interval. No interpolation or timestamp shift is applied. The after-base-set match is also exact; update-end references are retained separately in JSON. No sample is chosen for its velocity magnitude.

| Whole phase | Time interval, s | Joint-state samples per joint | Arm-controller samples per joint | Raw rows per phase | Missing/gap pairs |
|---|---|---:|---:|---:|---:|
| observation | 11.875–24.564 | 1269 | 1269 | 12690 | 0 |
| pregrasp | 24.565–34.321 | 975 | 975 | 9757 | 0 |
| descend | 34.322–38.841 | 452 | 451 | 4520 | 0 |
| close | 38.842–40.344 | 150 | 151 | 1503 | 0 |
| lift | 40.345–45.011 | 467 | 466 | 4667 | 0 |
| retention | 45.011–45.808 | 80 | 80 | 798 | 0 |

All seven public joint-state and controller-state velocity streams are finite in these phases; all six arm streams have zero unmatched headers. The bag contains controller `/state`, action goals/results/status, but no arm action `/feedback` topic. Controller actual feedback below refers specifically to `/state.actual`.

## Joint-state speed distributions, all seven joints

Each cell is absolute-speed p95 / maximum, rad/s. Different controller and joint-state maxima reflect distinct recorded header times, not a shifted reference.

| Joint | observation | pregrasp | descend | close | lift | retention |
|---|---|---|---|---|---|---|
| shoulder_pan_joint | 0.0883803 / 0.103009 | 0.0081295 / 0.00874929 | 0.000121803 / 0.000606193 | 0.00170787 / 0.00370456 | 0.00330364 / 0.0145443 | 0.000202643 / 0.000227426 |
| shoulder_lift_joint | 0.00483838 / 0.0127137 | 0.0761321 / 0.0809288 | 0.0755729 / 0.0829354 | 0.00190291 / 0.00681423 | 0.075254 / 0.0851159 | 0.000228935 / 0.000293996 |
| elbow_joint | 0.0954357 / 0.11134 | 0.0607085 / 0.0648337 | 0.0585156 / 0.0635409 | 0.00448496 / 0.0155662 | 0.0566316 / 0.129363 | 0.000574199 / 0.000794937 |
| wrist_1_joint | 0.0864885 / 0.096124 | 0.338377 / 0.354583 | 0.0442951 / 0.0526583 | 0.00651777 / 0.0170373 | 0.0461716 / 0.184596 | 0.00597258 / 0.00771866 |
| wrist_2_joint | 0.0840305 / 0.0972876 | 0.357324 / 0.371987 | 0.00169708 / 0.00306852 | 0.0163611 / 0.041569 | 0.011007 / 0.0912988 | 0.00805369 / 0.0103123 |
| wrist_3_joint | 0.360808 / 0.408412 | 0.361306 / 0.377733 | 0.00108278 / 0.0023784 | 0.011164 / 0.0595034 | 0.0391669 / 0.314473 | 0.0464035 / 0.0690244 |
| left_outer_knuckle_joint | 0.0589601 / 0.588991 | 0.142941 / 0.260355 | 0.0713435 / 0.07338 | 0.521278 / 0.867265 | 0.169913 / 0.65893 | 0.195763 / 0.264547 |

## Controller actual speed distributions, all seven joints

Each cell is absolute-speed p95 / maximum, rad/s. Different controller and joint-state maxima reflect distinct recorded header times, not a shifted reference.

| Joint | observation | pregrasp | descend | close | lift | retention |
|---|---|---|---|---|---|---|
| shoulder_pan_joint | 0.0883751 / 0.103032 | 0.00811758 / 0.00876045 | 0.00012129 / 0.000598727 | 0.00164643 / 0.00368046 | 0.00324926 / 0.0146601 | 0.000183993 / 0.000242201 |
| shoulder_lift_joint | 0.00482061 / 0.0255467 | 0.076116 / 0.0809024 | 0.0756012 / 0.0829379 | 0.00183383 / 0.00677 | 0.0752389 / 0.0798093 | 0.000220192 / 0.000232356 |
| elbow_joint | 0.0954356 / 0.111323 | 0.0606972 / 0.064994 | 0.0585277 / 0.0635739 | 0.00426791 / 0.0154738 | 0.0563479 / 0.130785 | 0.000548384 / 0.000934816 |
| wrist_1_joint | 0.0861767 / 0.0958801 | 0.338307 / 0.355971 | 0.0441268 / 0.0521205 | 0.00589122 / 0.0167585 | 0.0460359 / 0.188046 | 0.0041048 / 0.00688662 |
| wrist_2_joint | 0.0839091 / 0.105883 | 0.357466 / 0.371339 | 0.00171391 / 0.00576446 | 0.0173907 / 0.0422474 | 0.0117598 / 0.0808112 | 0.00795177 / 0.00929961 |
| wrist_3_joint | 0.360799 / 0.408493 | 0.36133 / 0.377439 | 0.00108405 / 0.00292441 | 0.0135174 / 0.0388309 | 0.0364696 / 0.366855 | 0.047991 / 0.0760981 |
| left_outer_knuckle_joint | 0.0589434 / 0.412222 | 0.123228 / 0.258663 | 0.0713727 / 0.0730994 | 0.521109 / 0.557345 | 0.163845 / 1.74226 | 0.119662 / 0.183976 |

The gripper column is descriptive, not a raw-CSV 1 ms validation: this trace records six arm joint angles only. Its public position range is −0.000206…0.000526 rad during pregrasp, 0.469216…0.510850 rad during lift, and 0.510486…0.512651 rad during retention. Thus its finite-difference calculation is not independently proven here.

## Independent wrist pose check

Parent motion is removed by relative parent/child quaternion increments; the increment is projected onto the parent-local wrist axis and the orthogonal residual is retained. This is an independent kinematic representation within the same simulator, not an independent physical sensor. All 1 ms pairs in each whole phase are included.

| Whole phase | Signed wrist travel, rad | q-rate minus quaternion-rate p95 / max, rad/s | Off-axis rate max, rad/s | Pose speed abs p95 / max, rad/s |
|---|---:|---|---:|---|
| observation | 2.46528824 | 1.01958e-05 / 0.00388674 | 0.0027295 | 0.360798 / 0.408944 |
| pregrasp | -3.19627767 | 2.17834e-08 / 0.000645355 | 0.00483912 | 0.361359 / 0.400801 |
| descend | -5.73970952e-06 | 1.60225e-10 / 6.56588e-09 | 0.00166562 | 0.00108785 / 0.0203942 |
| close | -0.00052258741 | 3.26689e-10 / 1.45737e-09 | 0.000295206 | 0.0129215 / 0.0689442 |
| lift | 0.000378811803 | 6.32977e-09 / 7.90068e-08 | 0.00354845 | 0.0395863 / 0.441106 |
| retention | 9.84884103e-05 | 4.74244e-09 / 7.70411e-09 | 0.00209524 | 0.0442729 / 0.0839864 |

The observation and pregrasp wrist motions are substantial (+2.4653 and −3.1963 rad). The target remains at the table during both, and the measured pre-close opening is 94.935 mm. The largest nonzero quaternion discrepancy remains reported, not filtered. Loaded retention is also not perfectly static: physical wrist speed reaches 0.0839864 rad/s, while net angle changes only 98.4884 µrad.

## Raw native discrepancy, settling boundary, and object retention

At update end, whole-lift native wrist rate integrates to −0.580292 rad while actual q changes +0.000378812 rad; during retention it integrates to −0.109791 rad while q changes +0.0000984884 rad. Native retention rate stays negative (−0.210267…−0.0666565 rad/s; median −0.138852). The native discrepancy is still present; the adapter has not repaired or redefined the physics engine.

The recorded lift goal has zero trajectory header, goal-id stamp 40.351 s, and final duration 4.610554258 s. Under the stated goal-stamp start convention its nominal endpoint is 44.961554258 s. Public SUCCESS result header is 45.002 s and lift-stage END is 45.011 s. These give only 49.446 ms postnominal and 9 ms postresult before retention, not a two-second settled phase. The whole retention phase is 45.011–45.808 s; it is reported separately and entirely. The controller acceptance instant for a zero-stamped goal is not explicitly recorded, so the nominal endpoint must not be called a directly logged controller-start time.

Physical evidence: recorded acceptance summary has target lift 149.412887 mm and TCP lift 149.754805 mm. Event-bounded CSV lift target rise is 148.896042 mm (different initial anchor), wrist-child rise 149.378841 mm, and maximum target-in-wrist displacement 1.466165 mm. During complete retention, target z changes −2.166757 µm and target-in-wrist displacement is at most 0.144603 mm. Grasp is confirmed. These are object motion/retention diagnostics only, not geometry supplied to the algorithm.

No closed/open failure contrast was triggered on this success, so there is no unloaded post-release hold. This trial does not establish gripper per-step reference equivalence, all-joint independent quaternion agreement, longer-term holding, reproducibility across random IK/timing, or native/integrated velocity interchangeability. No original controller threshold, gain, physics solver, or result label was changed by the analyzer.

Companion files: `summary.json` contains all raw phases, all public seven-joint statistics, exact header-match errors, native comparisons, goals/results and missing-data counts. `REPORT.md` gives the concise overview. The older `launch-05-public` output is preserved; this full-phase output adds descent and closing.


