# Launch07 Easy E2E: bounded closure-geometry audit

## Conclusion

The recorded **right-inner-knuckle/perceived-target rejection is reproduced** by the accepted cuboid and measured public arm/TCP pose. It occurs in a hypothetical closure check **before any close command**, not in observed physical contact.

The independently derived contact-height correction, **raise TCP by 13.2905620873464 mm**, removes all forbidden AG95/target and AG95/chassis intersections in the bounded offline sweep, including when the recorded pose error is retained. It preserves nominal 20 mm pad overlap. It does **not** establish bilateral contact, a feasible corrected full-arm path, successful physical grasp, attachment, or lift.

## Recorded stage and geometry inputs

Launch07 `launch-07-easy-full-e2e` is an actual Ours aerial-confirmation/navigation run, not arrival-conditioned replay. It selects source585, reaches accepted Ground refinement, D_exec at 174.507 s and descend completion at 179.655 s. Close fails at 179.732 s:

`AG95 physical closure full robot state invalid; collisions=['ground/right_inner_knuckle/perceived_pick_target']`.

No grasp-close command, planning attachment or lift follows. Original run metrics/events are unchanged.

Accepted refined target map x,y,z,yaw:
`[1.911110456659964,-0.12377395118747642,0.05861850365901321,0.22062664142338037]`.

Accepted scene box in `ground/base_link`, position plus quaternion x,y,z,w:
`[0.7535450004375759,0.08016237858884923,-0.30138149634098677,0,0,-0.835604763931632,0.5493311191756404]`, dimensions `[.240,.053,.115]` m.

Preshape measured q=0.28909034878591644, conservative opening=65.607095 mm. Immediately before the failed sweep, public master q=0.29101034201867115 (179.734 s); the close-start sample is 0.2909981199688998 (179.654 s). Neither is a completed physical closure.

## Correction derived from existing pad geometry, not collision outcome

The current URDF has 55 mm outer-to-finger levers, origin angle α=44.691°, parallel pad mimic geometry, and fully open aperture 95.2 mm. For the known 53 mm span:

`w(q)=.0952+.110[cos(α+q)−cosα]`

`q_contact=acos(cosα+(.053−.0952)/.110)−α=0.45737440709566757 rad`.

The pad's downward displacement during closure is:

`Δz=.055[sin(α+q_contact)−sinα]=.0132905620873464 m`.

Therefore preserving the existing contact-overlap policy requires the configured pad edge to change from `.0156` to `.0023094379126535995` m and the grasp TCP to rise by Δz. This preserves the old calibration's approximately 40.56 µm rounding; it is not a collision-fitted clearance. Existing target height .115 m and overlap .020 m stay unchanged. For this measured target, nominal grasp map z changes from **.0805185036590132** to **.0938090657463596** m.

## Measured error and complete bounded hand sweep

Nominal old TCP base position:
`[.7535450004375761,.080162378588849,-.2794814963409868]`.

Measured failure-time TCP base position:
`[.7483448543623797,.07819298982410057,-.28351618962060243]`.
The direct public TCP TF matches this JointState/URDF FK to floating-point precision.

Relative to the nominal grasp, the measured error in target axes is **(+3.869697,-3.993176,-4.034693) mm**, with **0.011063779 rad (0.6339°)** orientation error. The target short-axis offset is therefore material.

All AG95 collision meshes (body, fingers, pads, inner/outer knuckles) are tested against the perceived oriented cuboid and rendered chassis box. Allowed contact remains exactly the four finger/finger-pad links; no knuckle exemptions are used. Collision requires a triangle-box separating-axis intersection, not overlapping AABBs.

The sweep covers actual q=0.291010342 through q_contact=0.457374407. The existing 5 mm runtime arc bound gives five q samples; a fixed independent 0.5 mm arc bound plus that runtime grid gives **41 samples**. Close-start JointState, failure JointState and direct-TF poses are each checked. These are finite checks, not continuous-motion certification.

| TCP used through closure sweep | Forbidden target intersection | AG95/chassis intersection | Pad vertical overlap at q_contact |
|---|---|---|---|
| Nominal old grasp | None | None | 33.331124 mm each |
| Measured old grasp | Right inner knuckle at 7/41 samples | None | 34.256370 / 34.218792 mm |
| Nominal + geometry-derived rise | None | None | 20.040562 mm each |
| Measured + same rise, error retained | None | None | 23.955179 / 24.416489 mm |
| Direct public TF + same rise | None | None | Same as measured |

At the old measured pose, the first sampled knuckle collision is q=0.4303964506; at q_contact there are seven intersecting triangles and a strict interior vertex `[-.0107090610,.0264036281,.0525280153]` in target coordinates, 0.096372 mm inside its nearest face. Both bracketing JointState samples reproduce the same link pair. At the corrected measured pose, every noncontact mesh has a positive target-box separating axis over the sweep; the smallest such lower bound is **8.318577 mm**.

Thus nominal old over-insertion alone is **not** proved to collide. Over-insertion combined with the actual tracking error produces the rejected geometry; the independently defined height correction removes this particular obstruction.

## Contact and capability limits

The correction retains vertical pad support but not a verified two-sided grasp at the frozen measured pose. At nominal q_contact the measured right pad enters the perceived target by about **3.977348 mm**, while the left pad remains about **3.712723 mm** away. The first sampled right-pad contact is q=0.380936864, **before** the hypothetical knuckle collision. A real close could move the object or stall at unilateral contact; neither behavior was executed or verified here.

Accordingly, the helper's nominal symmetric q_contact sweep is a conservative fixed-target geometric test, not a measured physical closure trace. Removing the forbidden obstruction does not justify treating the remaining intended-pad overlap as successful contact. Keep real grasp confirmation, measured aperture/pose checks and attachment/lift validation intact.

No new IK or full-arm trajectory was solved for the raised TCP. The tested hand geometry does not certify AUBO/chassis clearance, target versus the full arm during motion, or payload lift. No tolerance, gain, ACM, target pose, or production parameter was changed by this audit.

## Reproduction and artifacts

From AGENT:

```bash
source /opt/ros/noetic/setup.bash
source /media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation/install/p450-clean/setup.bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 outputs/development/ground-manipulation-batch/moderate-diagnostic/analyze_launch07.py
```

`launch07_findings.json` contains exact public joint/action records, target/TCP matrices, q grids, mesh triangle witnesses and corrected/unmodified cases. Inputs are accepted perception, public JointState/TF/action messages and the current robot collision model only. No simulator-state/GT topics, spawn target coordinates, physical-checker data, ROS services/nodes or simulator starts are used.
