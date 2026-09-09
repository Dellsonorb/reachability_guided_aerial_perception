# Launch08 Moderate Ours: completed Ground manipulation audit

## Recorded outcome and scope

At SIM `c12d8ba` / AGENT `6d1dba2`, launch08 completes the recorded Ground sequence with **D_exec=true, retrieval_success=true, adapter result=true**, terminal `LIFT`. Public feedback corroborates successful arm lift and maintained grasp confirmation.

This is **conditioned on archived confirmation and arrival**, not a new aerial/navigation E2E success or a Generic-versus-Ours comparison. Original `data/metrics.json` reports `physical_success=null`; this audit does not replace that field or infer an independent physical target trajectory. All eight starts are consumed; this audit performed no new runtime action.

Candidate is `candidate-000009`, source664, map `[2.5304007767576087,-0.3015636791168017,-2.617993877991494]`. Measured arrival is `[2.530399988684981,-0.30156332114535067,-2.617993731655107]`.

## Exact perceived geometry and stages

Accepted refined target map x,y,z,yaw:
`[2.058472706748215,0.07937483959188794,0.05852908753931587,0.055768414454207216]` (observation 25.830 s).

Refined planning-box position/quaternion x,y,z,w in `ground/base_link`:
`[0.21823842374832814,-0.5658661930006514,-0.30147091246068414,0,0,0.97276628902492,0.2317881509842253]`; dimensions `[.240,.053,.115]` m.

| Stage | Simulation time | Recorded result |
|---|---:|---|
| Accepted aerial cuboid | 12.831 | Scene updated, target contacts disallowed |
| Camera view0 | 16.398 | Plan success after current-view timeout |
| Refine / contact calibration / refined scene | 26.175 / 26.176 / 26.188 | Passed |
| Measured preshape | 28.342 | q=.29093340618238894; conservative opening=.06541842981874901 m > .055 m |
| Grasp IK attempt1 | 28.359 | Code1 |
| Reverse approach / pregrasp / forward approach | 28.367 / 28.381 / 28.397 | Fraction1 / success / fraction1 |
| D_exec | 30.820 | Recorded true; attempts2–6 unnecessary |
| Descend | 30.822–36.210 | Completed |
| Closure geometry | 36.225 | Five full-state checks to q_contact; passed |
| Perceived payload attached | 37.515 | Successful scene apply and attached full-state validation |
| Lift | 37.517–43.050 | Completed |
| Retention / terminal LIFT | 43.050–43.631 | Passed, grasp_confirmed=true |
| Replay end | 43.640 | success=true |

Active calibration: q_contact=`.45737440709566757`, open edge=`.0156` m, contact edge=`.0023094379126535995` m, pad down-shift=`.0132905620873464` m, unchanged overlap=`.020` m, corrected grasp TCP map z=`.09371964962666227` m. Preshape command remains `.29271008403361354` rad, with existing .10 rad tracking allowance.

## Independent public controller/confirmation evidence

All four recorded arm action results are status3/code0: camera 26.111 s, pregrasp 30.811 s, descend 36.200 s, lift 43.043 s.

The gripper's close action is **not** a normal trajectory success. It returns status4/code−4 at 37.495 s, `left_outer_knuckle_joint path error 0.080008`, while driving the unchanged .70-rad force goal. The existing contact-stall branch accepts it only because measured q exceeds the existing minimum and fresh grasp confirmation is true; adapter log records acceptance at q≈.4666. No tolerance was relaxed.

The public `/ground/gripper/grasp_confirmed` signal changes false→true at bag time **37.200 s**, before attachment, and has no later false transition in the recorded tail. There are **122/122 true samples** from attachment to replay end, **111/111** during lift and **12/12** during retention. The last sample is 43.599 s (32 ms before LIFT); largest tail sample gap is 84 ms. These are runtime confirmation observations, not a guarantee derived from a command.

Measured master q is .466649468 at the attachment TF stamp, .468974245 near close completion, and .520551493 at final retention. Joint/action records retain this contact-loaded behavior rather than substituting the .70 goal for feedback.

Terminal event reports **TCP lift .1495373237916204 m**. Adjacent public TF snapshots independently show base-frame TCP z moving from −.2699058171 near descend completion to −.1203804182 near lift completion (approximately .149525399 m); small timestamp differences account for the micrometre-scale difference.

## Planning attachment and whole-model activation

The payload status records measured TCP at **37.489 s** in `ground/base_link`:

`[.21861162137803042,-.5610198290666959,-.2696153416725945,-.6872727628576919,.16032555618533445,.6884538232786002,.16728179423027018]`.

It attaches `perceived_pick_target` to `ground/gripper_tcp_link` with exactly:

`ground/left_finger, ground/right_finger, ground/left_finger_pad, ground/right_finger_pad`.

The accepted cuboid is transformed by `T_tcp_target = inverse(T_base_tcp_measured) × T_base_target`, not by a commanded TCP or GT target. Its expected local center is **[.031805486473513,.004786716673108649,−.001975419894786723] m**; reconstructed world transform agrees to maximum matrix-element error 1.11e−16. Full matrices are in the JSON. This is an expected-transform calculation, not a saved native scene dump.

Deployed code makes the runtime evidence substantive:

- `GROUND_MANIPULATION_SCENE` follows checked ApplyPlanningScene success; target ACM uses all RobotCommander links and blocks all nonfinger/pad target pairs.
- Contact exemptions begin immediately before close, remain restricted to those four links, and do not disable gripper/chassis or other self-collisions.
- `GROUND_GRASP_GEOMETRY` follows five successful `GetStateValidity` requests with empty group name (whole robot).
- `GROUND_PAYLOAD_MODELED` follows fresh real-confirmation checking, measured-TCP attachment, checked scene-apply success, and another whole-robot state check requiring the perceived attachment.
- Lift performs a separate attached whole-state start check. Cartesian planning is collision-aware; its measured RobotState copies current scene attachments and uses `is_diff=true`, avoiding accidental payload clearing.

Thus full-model/attachment hooks were genuinely traversed, not merely enabled in a configuration file. The diagnostic bag did **not** record serialized planning-scene or service-request topics: this conclusion combines gated events, public feedback and deployed source, not an independently archived full scene.

## Remaining limits

No failure remains in this particular recorded Ground attempt. It does not erase launch06/07 failures or prove all candidates/branches reliable. Closure checking is finite, not continuous; the event explicitly reports `continuous_clearance_proven=false`. Full-state validation is at the documented stages, not a recorded assertion about every actual feedback sample. The accepted box is a perception-based payload approximation; no claim of zero slip, perfect target-pose tracking, or physical GT agreement follows from its planning attachment.

No production settings, metrics or old records were altered by this audit. No Gazebo model/link-state GT, raw contact-pair messages or physical-checker records were used here. The public grasp-confirmation signal is intentionally used as the robot execution interface; the separate dynamics audit checks physical target motion.

## Reproduction

```bash
source /opt/ros/noetic/setup.bash
source /media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation/install/p450-clean/setup.bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 outputs/development/ground-manipulation-batch/moderate-diagnostic/analyze_launch08.py
```

`launch08_findings.json` preserves exact events, action results, public confirmation coverage, joint samples and attachment-transform arithmetic. The script reads the finalized bag offline and prints JSON only.
