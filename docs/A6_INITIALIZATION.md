# A6 method-independent initialization qualification

The overnight authorization permits common initialization repair before pilot,
using only sensor admission and UAV/SIM geometry. No RM4D query, A1 catalog,
candidate discovery, NBV score, Ground attempt or retrieval outcome was used.
All three original seeds and obstacle layouts remain unchanged.

## Common geometry

The fixed sensing destination is frozen: map `(-1.4, 0, 1.2), yaw=0`.
The closer/lower view addresses the earlier 4 m RGB-D range failure while
retaining ground returns from the pitched MID360. It is not a runtime GT rule.
Using the unchanged public optical transform, the entire nominal brick's
optical-depth bounds are:

| Seed | Depth interval (m) |
|---|---:|
| Easy 2026090801 | 3.3104–3.5801 |
| Moderate 2026090802 | 3.3606–3.6275 |
| Hard 2026090803 | 3.4803–3.7506 |

Depth opportunity alone does not prove FOV/occlusion/pixel-count admission.
The separate live checks require the unchanged observer to produce its stable
RGB-D handoff, followed by a frozen 5 s stable-hover MID360 capture. Raw ground
endpoints (`abs(z_map) <= .02 m`) must cover at least 100 distinct .10 m cells
(1 m²) in the common 4×4 m setup ROI centered on serialized target XY. This is
a sensor-usability diagnostic, not an A2 threshold, A2 vote or full-footprint
confirmation requirement. Policies never receive setup metadata.

## Public launch-origin collision repair

The inherited SIM UAV ground spawn `(0,0,.15), yaw=0` was not a serialized A6
seed field. Hard h1 occupies XY `[.225,.375] × [.2,.8]`, z `[0,1]`.
The actual MID360 P450 model's `rotor_2_collision` is a vertical cylinder with
center `(.1465,.147,.151)` relative to the model and radius `.128 m`.
At nominal spawn, its center-to-box XY distance is
`sqrt(.0785² + .053²) = .09471668 m`, so radial overlap is `.03328332 m`;
its z interval `[.2985,.3035]` also intersects the wall. This is a physical
collision definition, not a visual mesh or an assumed A4 prism.

Two Hard setup runs timed out at takeoff. In Hard02, before reaching the sensing
destination, the actual UAV had been displaced to approximately
`(-.5613,.3409,.0447), yaw=2.21 rad`; the actual wall remained at `(.3,.5,.5)`.
PX4 reported preflight/estimator problems. The geometry proves the setup defect;
the retained logs do not directly measure its contact impulse or prove that it
is the sole cause of every runtime symptom.

Use common launch `(-.5,0,.15), yaw=0` for **every seed and method**, leaving the
sensing destination unchanged. This gives the same rotor–h1 XY clearance
`.45292276 m`. Target, BUNKER and wall geometry is unchanged. No collision is
disabled, no simulator object is teleported during an attempt, and all bootstrap
flight costs remain measured. This is a method-independent initialization repair,
not a scene selection or policy change.

The public `air_ground_standalone.launch` already exposes all four spawn args.
A6's generated launch forwards those args through the unchanged public demo
composition. Removing exactly those four added arguments restores the original
XML tree; every original node, include, parameter and other argument is retained.
SIM and A5 source files are not edited. Invocation still uses `with_p450_env.bash`.

## Setup activation history

These are **METHOD_INDEPENDENT_SETUP**, not any of the 14 pilot methods. The
wrapper's generic `VALID_TRIAL/INVALID_TRIAL` field in these directories does
not enter pilot denominators; `kind` and `data/setup_summary.json` define their
setup outcome. Raw records are retained rather than relabeled as pilot failures.

| Directory under `outputs/a6/setup-20260908` | Outcome |
|---|---|
| easy-01 | RGB-D + ground capture PASS at old origin spawn; 847 ground cells |
| moderate-01 | Pre-task readiness timeout; no sensor task started |
| moderate-02 | Same timeout, diagnosed parent wall-clock vs simulation TF stamps |
| moderate-03 | Startup fix; RGB-D + ground capture PASS, 537 cells, old origin spawn |
| hard-01 | Takeoff timeout at origin spawn; no observation |
| hard-02 | Reproduced takeoff timeout; confirmed overlapping launch geometry |
| hard-03 | Common corrected launch: PASS, 414 cells, 5.025 s window |
| easy-02 | Common corrected launch: PASS, 851 cells, 5.094 s window |
| moderate-04 | Common corrected launch: PASS, 550 cells, 5.029 s window |

**Freeze:** after moderate-04 completed, all three original seeds had passed
with exactly the same launch and sensor poses. `configs/a6_pilot.json` was set
to `FROZEN_FOR_PILOT` immediately, before any pilot method executed. These poses
are now fixed for all 14 method slots and any proven INVALID reruns.

## Engineering correction independent of geometry

The parent setup node previously initialized before roslaunch installed
`/use_sim_time`. rospy chose its wall clock once, causing TF ages near 1.79e9 s
despite healthy live TF/action/state diagnostics. Wait for the simulation-time
parameter before node initialization; retain a wall-bounded readiness guard.
Action negotiation runs without blocking that guard on a stopped `/clock`.
No robot-efficiency metric uses wall-clock durations.
