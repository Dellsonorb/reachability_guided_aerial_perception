# Ground navigation: lost surface settings in URDF-to-SDF conversion

2026-09-09. Independent offline review of launch01 Easy navigation and
launch02 Easy navigation with corrected odometry feedback, with later
root-owned launch05/06 checks annotated below. No ROS/Gazebo processes were
started by this diagnosis. End-to-end Ground execution success is not claimed.

The smallest demonstrated defect is in collision identity during model
conversion. The rendered URDF declares zero base friction, but names its
base collision `base_link_collision` while the owning link is
`ground/base_link`. The actual installed SDFormat 9 converter interprets
that collision as a lumped collision and fails to associate the link's
Gazebo surface extension. The resulting base-box SDF has no friction
element, so the declared zeros are lost; the SDFormat schema defaults for
both ODE friction coefficients are 1.

Changing only the collision name to `ground/base_link_collision` preserves
the existing declared zero coefficients. This is a surface-setting transport
correction, not a new friction choice or altered collision envelope.

## Recorded behavior

Public `/ground/odom` pose differences and `/ground/cmd_vel` establish the
low-command dead zone. These calculations use bag timestamps, not scene GT.

| Attempt | Observed XY extent, m | Observed yaw extent, rad | Median command / reported yaw rate / pose-derived yaw rate when abs(command) is 0.01–0.09 rad/s |
|---|---|---:|---|
| launch01 | 0.5582 / 1.9331 | 2.14019 | -0.031724 / -0.031724 / 0.00000039 |
| launch02 | 0.03422 / 0.00866 | 0.037474 | -0.080000 / -0.00000007 / -0.00000010 |

Launch02 has 5,951 odometry intervals in that command band; the 95th
percentile absolute pose-derived yaw rate is 0.0000196 rad/s. Its repaired
feedback therefore reports the near-stationary physics state rather than
the velocity just assigned by the plugin. With acceleration limit 1.2 rad/s²
and controller frequency 15 Hz, the ordinary first angular dynamic-window
increment is 0.08 rad/s. The controller repeatedly searches around the
measured near-zero speed. Increasing that limit would bypass this symptom
without explaining the failed surface transport.

## Real converter evidence

The installed renderer was invoked offline and its URDF passed directly to
`sdf::readString` from the installed libsdformat9. This required no Gazebo
server. Changing variants in memory gave:

| URDF variant | Effective base-box SDF friction mu/mu2 |
|---|---|
| Original collision name, declared 0/0 | Absent / absent |
| Original collision name, declared 0.1/0.1 | Absent / absent |
| Original collision name, declared 0.8/0.8 | Absent / absent |
| Unnamed collision, original declared 0/0 | 0/0 |
| Name `ground/base_link_collision`, original declared 0/0 | 0/0 |

This excludes a special zero-value serialization issue. SDFormat's collision
extension matching uses link and lumped-collision naming relationships;
the isolated naming change restores that association. See the
[SDFormat 9 converter](https://raw.githubusercontent.com/gazebosim/sdformat/sdf9/src/parser_urdf.cc).

The effective original model has 17 joints. Fixed-joint reduction gives the
base link mass 34.321 kg and inertia Izz 3.26496 kg·m²; articulated arm links
remain separate. Its base box is unchanged at
1.026335219 × 0.782744936 × 0.395154782 m, origin
(0.018058912, 0.001357451, -0.160420741) m. The box bottom is
0.357998132 m below base_link. Wheel collisions are removed by the existing
renderer, making this box the chassis contact surface. The converter fix
does not change these definitions or other arm/gripper surface settings.

## Alternatives assessed

Automatic sleep is not supported as the immediate explanation. Gazebo 11
enables ODE automatic body disabling only when the model has no joints and
the link has no sensors; otherwise it explicitly disables that feature.
This model has 17 joints. The absence of an explicit wake call in SetVel is
real but does not establish sleeping here. The same source confirms that
velocity assignment and force application differ in their enabling behavior.
See [ODELink](https://raw.githubusercontent.com/gazebosim/gazebo/gazebo11/gazebo/physics/ode/ODELink.cc).

The plugin sets only base-link velocity, so articulated-body constraint
reconciliation was considered as a possible secondary tracking issue, not
established as the cause. Launch05 tracking below removes the immediate
justification for a broader multibody velocity reset. Also, SetLinearVel
sets the ODE body's center-of-mass velocity whereas WorldLinearVel without
an offset reports the link origin. Root's subsequent narrow frame correction
is reviewed below. No all-link velocity reset, servo, acceleration-limit
change, new friction coefficients, or collision bypass was implemented.

Base contact forces were not recorded, so the exact contributions of
contact and articulated-body dynamics to the historical slowdown cannot be
separated from these bags alone. Effective SDF is reproduced from the
installed model path; no live physics surface service was queried.

## Implemented scope and validation

At root's request, the source renderer now uses the prefixed base collision
name consistently in creation and validation. The existing renderer test's
expected name was updated. New `tests/test_ground_surface_conversion.py`
compiles a tiny parser in a temporary directory and exercises the complete
robot through real SDFormat conversion.

The new regression first failed with expected mu='0', actual=None. After
the naming change both converter tests pass. The second test verifies that
the entire converted tree is identical after normalizing only this base
collision name and intended surface element, thus covering all geometry,
inertia, joints, and other collision surfaces. Existing platform tests pass
18/18; targeted diff whitespace checks pass. No commit or installation was
performed by this agent. Root owns installation and online navigation
checks; the original two failures remain unchanged.

Recheck from the SIM checkout:

```bash
/usr/bin/python3 -m unittest discover -s tests -p 'test_ground_surface_conversion.py' -v
/usr/bin/python3 -m unittest discover -s tests -p 'test_ground_manipulator_platform.py' -v
```

## Later root-owned online checks

These results were supplied by root from the separately owned online checks;
they are not additional launches by this diagnosis. Historical attempt files
were not edited.

| Attempt | Change under test | Result and interpretation |
|---|---|---|
| launch05 Easy navigation | Installed collision-name correction; original declared friction remains 0/0 | During sim 15–20 s, commanded/reported yaw rates were 0.1530/0.1541 rad/s. Low-command tracking is restored. Navigation still reached its 120 s timeout: 2 Hz global replans reset the local planner's XY latch. Root found no target–robot contacts. |
| launch06 Easy navigation | Root changed planner_frequency to 0 | move_base returned SUCCEEDED at sim 40.725 s (approach began 12.048 s), but independently measured base-origin XY arrival error was 98.66 mm, exceeding the intended 60 mm. A navigation action status is therefore insufficient evidence of the required arrival. |

Launch05 is a useful online counterfactual to the isolated converter test:
transporting the existing zero friction restores near-matching command and
measured velocity without friction retuning, a servo, or DWA acceleration
tuning. It does not establish that every later navigation failure is solved.
The 2 Hz replan/latch issue and the origin/CoG velocity mismatch are separate
defects. Root's launch06 planner-frequency change is a navigation setting
change, distinct from this agent's renderer-only production fix.

Neither launch03 nor launch06 established lift success. Both encountered
wrist stop/settling failure; launch06 reported maximum joint speed 0.1638
rad/s at wrist3. Reaching a navigation or camera milestone must not be
reported as end-to-end retrieval success.

## Review of root's subsequent velocity-frame correction

The effective reduced base CoG is approximately
(0.024387, 0.003131, -0.102779) m relative to base_link. Root added the
conversion in `bunker_sim_runtime/src/planar_twist.hh` and its plugin caller:

```text
r_world = WorldCoGPose.position - WorldPose.position
v_CoG_world = v_base_world + omega_world cross r_world
```

This sign and frame convention are correct: the velocity of a second point
on a rigid body is the first point's velocity plus omega cross its relative
position. The caller rotates the planar base command to world coordinates,
uses world angular velocity about +Z, and obtains r in world coordinates.
The inverse relation is v_base = v_CoG - omega cross r. Thus no extra body
rotation or opposite sign belongs in the helper. It uses the existing
planar/yaw-only actuator contract; it is not a new general 6-DoF controller.

The earlier feedback-ordering change also matches physical semantics:
WorldLinearVel/WorldAngularVel are sampled before assigning this step's
command, so published twist describes the completed physics step at the
link origin rather than echoing the next requested velocity. Actual pose
and collision processing remain in the physics engine.

Root additionally implemented a fresh actual-pose arrival gate of 60 mm XY
and 0.08 rad yaw. Its online verification is pending launch07 at this report
update. The formula review establishes source-level consistency, not that
launch06's complete 98.66 mm residual is attributable only to this mismatch
or that the next run will pass. No production code was edited during this
final diagnostic/report update.

## Final online follow-up (root)

Launch07 subsequently completed the bounded check with the same Easy exact
candidate and original start. Its actual public stopped pose was
(2.469687284, −0.635087453, 2.162686313), against goal
(2.410300620, −0.633293445, 2.094395102). Errors were59.413755mm and0.068291rad,
inside the unchanged60mm/0.08rad limits. It reached fresh refinement and D_exec,
then failed the same wrist-3 lift-settling condition at0.1679rad/s. This is one
near-boundary arrival pass, not a reliability estimate or retrieval success.
The earlier pending statement describes the pre-launch source review only.
