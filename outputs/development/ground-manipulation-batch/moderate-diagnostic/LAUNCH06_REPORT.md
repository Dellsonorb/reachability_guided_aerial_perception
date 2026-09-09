# Launch06 Moderate: measured pregrasp collision

This is a public-record/model audit of `launch-06-moderate-full-robot`, not a new runtime attempt. The historical baseline in `REPORT.md` remains unchanged.

## Outcome and stage evidence

The original metrics remain **D_exec=true, retrieval_success=false**. The controller completed pregrasp; the subsequent full-state check rejected the actual state before any descend motion. This is not another six-attempt IK failure and not a completed retrieval.

| Recorded stage | Simulation time | Result |
|---|---:|---|
| Accepted aerial cuboid inserted; all target contacts checked | 12.694 | Passed |
| Current camera view | 15.823 | Timed out; raw measured surface used only for camera aim |
| Camera view0 plan / accepted refined cuboid update | 15.907 / 23.857 | Passed |
| Measured target-sized preshape | 26.006 | Passed |
| Grasp IK attempt1 | 26.023 | Code 1 |
| Reverse Cartesian / pregrasp trajectory / forward Cartesian | 26.028 / 26.079 / 26.092 | Code 1, fraction 1 / success / code 1, fraction 1 |
| Attempts2–6 | — | Not run: attempt1 succeeded |
| Pregrasp controller result / D_exec event | 37.404 / 37.413 | Success / recorded true |
| Actual-state check at descend start | 37.424 | Invalid: `ground/left_finger` versus `ground/base_link` |
| Postfailure diagnostic IK | 37.429 | Code 1, collision-free, contacts empty; not executed |
| Close / contact sweep / attachment / lift | — | Not reached |

Candidate source624 is map `[2.5368278045875257,-0.3799571971654724,-pi]`. Accepted refined target is map `[2.058489459985013,0.07944843012284865,0.05857919684434868,0.05584618446422396]` as x,y,z,yaw; box dimensions are `[.240,.053,.115]` m. No surface cue is promoted to a cuboid.

Preshape command q=`0.29271008403361354`; accepted measured q=`0.2906401077395726`; conservative opening=`0.06544845348730398` m > required `0.055` m. These use the existing 0.10 rad tracking allowance and aperture calibration; no fitted change was made.

## Measured versus planned state

Positions below are TCP coordinates in `ground/base_link`; quaternion convention is x,y,z,w. Planned arm positions are the actual action-goal endpoint. The planned-hand mesh comparison uses adjacent preshape feedback q=0.290632955, because the exact planning-service request state is not recorded.

| TCP | Position (m) | Quaternion x,y,z,w |
|---|---|---|
| Planned action endpoint, model FK | (0.478313020, -0.459447870, -0.129515154) | (0.706868213, 0.019672539, -0.706797558, 0.019685876) |
| Public JointState FK, 37.423 s | (0.476766398, -0.457088837, -0.133682095) | (-0.705936153, -0.020815579, 0.707740860, -0.017992660) |
| Direct public link TF, dynamic edges 37.413 s | (0.476766410, -0.457068976, -0.133692711) | (-0.705920631, -0.020829753, 0.707755838, -0.017996051) |

Quaternion sign is immaterial. JointState-FK minus planned TCP is **(-1.546622,+2.359033,-4.166941) mm**, norm **5.031945 mm**, rotation difference **0.004871172 rad (0.2791°)**. Positive base-y moves this finger toward the chassis. Direct TF differs from adjacent feedback FK by 22.52 µm position and 0.000052064 rad orientation, and independently gives the same collision sign.

Failure-time master q is **0.3259806223720947** at 37.423 s; gripper controller at 37.424 s reports actual 0.3259523440283534, desired 0.29271008403361354. The gripper therefore drifted **more closed**, not open. Its failure-time conservative aperture is 61.830801 mm, still above 55 mm.

Arm controller desired-minus-actual errors at 37.424 s, ordered shoulder_pan, shoulder_lift, elbow, wrist1, wrist2, wrist3, are:
`[0.000373513,-0.005399090,0.003998213,0.004695021,-0.000992409,-0.000410257]` rad. Pregrasp action success and the existing 20 mm/0.12 rad TCP acceptance do not guarantee submillimetre chassis clearance.

## Positive mesh evidence and causal counterfactual

Existing rendered chassis bounds in base frame are x=[-0.4951086975,0.5312265215], y=[-0.390015017,0.392729919], z=[-0.357998132,0.03715665] m.

The existing STL collision triangles are transformed by either URDF FK from measured joints or directly recorded link TF. Strict vertices inside the chassis and a triangle/box separating-axis test establish positive overlap; this is not an AABB-only inference. Positive gaps in the table are separating-axis lower bounds, not exact nearest Euclidean mesh distances.

| Offline geometry | Left-finger/chassis result | Outer-knuckle/chassis result |
|---|---:|---:|
| Planned arm + preshape q | 0.404698 mm separation | 0.771168 mm separation |
| Planned arm + failure-time q | 2.124145 mm separation | 2.509233 mm separation |
| Actual arm + original preshape q | 1.738442 mm interior witness; 38 intersecting triangles | 1.378474 mm interior witness; 58 triangles |
| Actual arm + actual q | **0.022485 mm interior witness; 4 triangles** | 0.355340 mm separation |
| Direct public link TF | **0.023113 mm interior witness; 4 triangles** | 0.355089 mm separation |

The direct-TF interior vertex is `[0.4837258995174394,-0.3899919044608628,-0.08096720999519062]` m. This independently corroborates the recorded finger/base contact pair. The counterfactuals isolate the cause: gripper closing drift improves clearance; arm/TCP tracking consumes the small planned clearance and the actual hand remains marginally intersecting. A scene API or frame-composition defect is not needed to explain this failure.

Old Moderate launch04/08 had six IK−31 failures; the old collision-disabled branch implicated forearm, wrist1, finger and knuckle versus base. Launch06 instead finds a collision-aware nominal branch on attempt1, then loses clearance in execution. This resolves neither all other arm branches nor the payload path. The postfailure valid, unexecuted IK result is additional evidence against claiming every branch impossible.

## Limits and reproduction

The first collision here means the **first recorded state-validity failure**, not independently established first physical contact time. The overlap is shallow; public TF and JointState samples are adjacent, not the exact request state. Agreement of both reconstructed poses with the native collision result supports the diagnosis but does not measure physical force or deformation. No simulator-state/GT topics, spawn target coordinates, contact-plugin output or physical-checker data enter the calculations. No production settings, metrics, or old records were changed.

A robust next planning design needs clearance consistent with actual tracking uncertainty; this report does not prescribe a collision-fitted preshape epsilon, tolerance relaxation, or unvalidated change of target height. Full-arm alternatives and attached-payload lift remain unverified.

From the AGENT repository:

```bash
source /opt/ros/noetic/setup.bash
source /media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation/install/p450-clean/setup.bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 outputs/development/ground-manipulation-batch/moderate-diagnostic/analyze_launch06.py
```

`launch06_findings.json` stores the full numbers, frames, controller samples, action endpoints, mesh witnesses and counterfactuals. The script reads the finalized bag offline and prints JSON; it creates no node or service.
