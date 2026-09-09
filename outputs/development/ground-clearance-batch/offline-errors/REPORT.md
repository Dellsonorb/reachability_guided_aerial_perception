# Recorded development clearance errors

Read-only offline diagnostic for AGENT `c97298ffb9bfc8adec9d0082a36be83eba9b611f` and rendered SIM `625b84ca70670964387b6e0d092d27f546661662`. The inputs are the finalized launches05–08 requested for this audit. No production code, prior records, runtime gates, metrics, or candidate outcomes are changed.

## Development allowance

Use **12 mm for arm-induced model-surface displacement against the fixed chassis**, obtained by rounding the largest observed all-recording collision-box-point bound, **11.490625 mm**, upward to the next whole millimetre. The maximum occurs on launch06's right finger during pregrasp at controller time 27.804 s; maximum TCP error is 11.266851 mm. The number is determined from all four recordings before testing candidate separation, not selected to rescue a candidate.

This is a finite-sample development envelope for the observed trajectories and model, **not a calibrated probability, a joint-error worst-case guarantee at arbitrary IK branches, or continuous-motion certification**. The same allowance should be used to compare candidates. New IK configurations can have different error amplification, and new load/speed regimes can exceed these observations.

Do not apply a universal 24 mm self-clearance requirement. Common ancestor motion cancels when a pair is expressed in either member's local frame. The JSON contains all 210 unordered model-link pairs, with a bound for every event stage and complete recording. For an observed pair/stage, round its maximum relative bound upward at the same 1 mm resolution; retain numerically zero common-mode terms as zero rather than rounding floating-point noise to 1 mm. Treat unobserved configurations as uncalibrated. Pair topology can establish exact common-mode cancellation independently of these measurements.

## Coverage and method

The script reads only the five named public interfaces plus JointState, listed explicitly in `summary.json`. It never reads Gazebo model/link states, raw contacts, physical-checker output, spawn coordinates, or dynamics CSV. It starts no node, service, ROS master, or simulator. All **26,683 arm controller samples** are retained; controller `desired.positions` and `actual.positions` share a message timestamp. The recorded arm `error.positions` agrees with wrapped desired-minus-actual to numerical precision. TCP FK uses the existing rendered URDF. Gripper controller samples match exactly for 3,590/3,593, 2,732/2,734, and 3,370/3,371 arm samples in launches05,06,08. Launch07 has no gripper controller stream. A nearest public JointState supplies only hand articulation for unmatched arm-only FK (largest offset 9 ms overall, 3 ms on launch07); missing desired hand values are not imputed as measured tracking success.

Each link's existing collision geometry is enclosed by its local bounding box. Maximum displacement of its eight corresponding corners under measured versus desired transforms bounds every enclosed collision point at that sample. Arm-only comparisons freeze the same measured hand articulation in both configurations. For pair A/B, both states are transformed into B's frame and displacement of A's box is bounded, then the reverse direction is calculated; the smaller valid directional bound is retained. This cancels shared rigid motion. These are **point-displacement bounds**, not collision distances, penetration estimates, or AABB-only collision decisions.

Event windows retain the first duplicate START and close at the matching END/FAILED; an open failed stage closes at FAILED. The entire recording is also included so no stage or quiet subwindow selection hides peaks. Whole public sample spacing is nominally 10 ms, with maximum 12 ms. Action-goal endpoint versus measured TCP at each public result is reported separately, using the nearest arm feedback (maximum offset 4 ms).

| Run | All arm samples | Complete-record TCP maximum (mm) | Complete-record arm surface bound (mm) | Maximum TCP rotation error (rad) |
|---|---:|---:|---:|---:|
| launch-05 | 3593 | 8.992235 | 9.999989 | 0.017892658 |
| launch-06 | 2734 | 11.266851 | 11.490625 | 0.019184027 |
| launch-07 | 16985 | 9.472751 | 10.310815 | 0.018677386 |
| launch-08 | 3371 | 9.680318 | 10.812056 | 0.019364779 |

## Stage-specific results

Entries below pool all complete event windows for the named stage; they are descriptive maxima, not separately tuned operational tolerances. Launch06's descend consists of two start-check samples and no executed descend; launch07 close consists of eight samples before hypothetical closure rejection and no physical close command. Launch05/08 supply the executed closure, lift, and retention data.

| Stage | Samples | TCP maximum (mm) | Arm surface bound vs chassis (mm) | Largest moving/moving relative bound (mm) |
|---|---:|---:|---:|---:|
| ground_refine | 5123 | 9.680318 | 10.812056 | 9.445918 |
| refined_pregrasp | 3266 | 11.266851 | 11.490625 | 8.201545 |
| descend | 1508 | 7.223869 | 7.546092 | 4.190383 |
| close | 290 | 6.874367 | 7.128131 | 3.410652 |
| lift | 1020 | 7.323071 | 7.656091 | 4.245951 |
| retention | 139 | 6.971341 | 7.327342 | 4.251633 |

Across complete recordings, selected structural pairs give the following relative arm-motion bounds. All pairs, including zero-motion ones, are retained in the JSON; no allowed-collision matrix is changed by this analysis.

| Pair | Maximum relative arm-motion bound (mm) |
|---|---:|
| Forearm / wrist1 | 0.449836 |
| Forearm / left finger | 2.004597 |
| Upper arm / left finger | 7.271091 |
| Wrist1 / left finger | 0.655862 |
| Left finger / right finger | < 0.000000000001 |
| D435 / left finger, fixed hand articulation | < 0.000000000001 |

Every AG95/AG95 arm-only pair is at numerical zero (largest stage value 8.214e-16 m). Hand articulation error is a separate term; arm common-mode cancellation does not remove it.

Absolute joint maxima over all four complete recordings, in shoulder_pan, shoulder_lift, elbow, wrist1, wrist2, wrist3 order, are **[0.003528104, 0.006211231, 0.010043015, 0.006225883, 0.006127899, 0.004787102] rad**. The JSON preserves per-run/per-stage signed extrema, median, p95, p99, maximum and peak timestamp for every joint and TCP axis. These independent joint maxima need not occur simultaneously. Propagating their full box through a new IK branch is a different and more conservative model than the observed simultaneous 12 mm envelope.

## Gripper contact and loaded articulation

Do not interpret the difference between the .70 rad force command and contact-loaded actual q as free-motion tracking uncertainty. Launch05/08 terminate closing by the existing contact/stall branch. Their later nominal desired-minus-actual differences approach .20 rad, and the combined arm+gripper desired/actual model-point bound reaches 16.923651 mm. Those numbers describe a force goal continuing beyond contact. They do not justify expanding finger/target contact exemptions or treating a command as measured physical closure.

| Run | Stage | Full measured master-q range (rad) |
|---|---|---|
| launch-05 | close | [-0.000036669, 0.468688200] |
| launch-05 | lift | [0.469251664, 0.510876354] |
| launch-05 | retention | [0.510549744, 0.512703286] |
| launch-08 | close | [0.291801962, 0.469018114] |
| launch-08 | lift | [0.469559932, 0.520995983] |
| launch-08 | retention | [0.520544456, 0.521259581] |

Launch08 attachment used a recorded measured TCP and hand q≈.46665 at its measurement stamp; the whole close window continues to q=.469018. A nominal geometry sweep ending at q_contact=.457374407 does not cover the subsequently measured q up to .521259581. For development, model the observed closure/loaded articulation explicitly through its full range and validate actual attachment/lift states. The range is observed behavior, not a guaranteed future bound or an instruction to command .53/.70. Exactly the existing four finger/pad links may have intended target contact during close/attached stages; knuckles/body/chassis contacts remain forbidden.

Before close, the largest available measured master error is .035812502 rad on launch06; it made that recorded chassis separation larger, but its sign must not be assumed. Launch05 approached with the hand effectively fully open and has no accepted-preshape event. Launch07 has no desired hand controller evidence. Keep existing measured-aperture checks and the documented .10 rad preshape tolerance distinct from the newly derived arm allowance.

## Perception uncertainty and mesh precision

The public `/ground_observer/target_pose` publishes accepted, fused PoseStamped cuboids without pose covariance or raw fit residuals. All published samples with observation stamps up to the accepted refinement stamp are included below. Post-refinement samples are counted in the JSON but are not labelled sensor noise because the object may subsequently move. The 4-frame fused publications are correlated; repeatability around the final accepted estimate does not measure bias or true pose error.

| Run | Pre-refinement publications | Maximum center deviation from accepted refinement (mm) | Maximum yaw deviation (rad) |
|---|---:|---:|---:|
| launch-05 | 17 | 1.993062 | 0.002256361 |
| launch-06 | 62 | 2.598845 | 0.000509481 |
| launch-07 | 25 | 3.688316 | 0.000712023 |
| launch-08 | 9 | 0.378795 | 0.001097433 |

The existing known .240×.053 m top and 90% measured-span acceptance rule imply conditional one-sided missing-support center terms **12 mm along / 2.65 mm across** the true rectangle (Euclidean combination 12.289121 mm), assuming the accepted top corresponds to the declared surface and axes. This is a geometric truncation allowance, not a bound on depth bias, segmentation error, camera calibration, localization/TF error, target dimension error, yaw bias, or payload slip. The estimator's point covariance selects its principal axis; it is not an estimator covariance.

Current Ground configuration allows 15 mm top-band selection, 15° fitted top tilt, and 4-frame spread limits 50 mm position / .20 rad yaw. These are acceptance parameters with different semantics; none can be relabelled as measured uncertainty. The recorded 0.379–3.688 mm repeatability does not justify replacing the 12 mm missing-support term with a smaller fitted radius. A target/environment margin requires explicit sensing terms in addition to tracking; this diagnostic does **not** establish a single calibrated total target inflation. The target's relative pose is expressed in public frames, so public frame error is also not measured independently here.

Maximum binary-STL float32 coordinate spacing after mesh scaling is **2.980232239e-8 m (0.000029802 mm)**. It is many orders smaller than tracking and is a representation-scale observation, not a bound on CAD/model error. Existing pad-edge calibration rounding (~40.56 micrometres) is deterministic geometry calibration and should remain separate. Neither mesh float spacing nor shallow overlap should be used to tune a clearance radius or exempt a contact.

## Reproduction and limits

From the AGENT repository:

```bash
source /opt/ros/noetic/setup.bash
source /media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation/install/p450-clean/setup.bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 outputs/development/ground-clearance-batch/offline-errors/analyze_errors.py
```

`analyze_errors.py` prints the complete JSON and creates no files. `summary.json` is that output saved via the workspace patch tool. It includes exact goals/results, all stage statistics, published perceived poses, the ordered 210-pair table, model SHA256, timestamp coverage and source topics. Numerical checks establish zero identical-pose displacement and the expected 10 mm pure-translation bound; the controller error identity is checked on every arm sample. Previous launch06/07 direct-public-TF versus JointState audits provide independent FK consistency evidence but their terminal error magnitudes are not used to choose this allowance.

These four development recordings do not estimate rare-event tails, real-hardware model error, continuous-time extrema, post-attachment object slip or unseen approaches. No formal launch, runtime action, external write, planning gate relaxation or new empirical success claim is made.
