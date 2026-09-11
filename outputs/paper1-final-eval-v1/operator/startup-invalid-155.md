# Slot 155: pre-task simulator startup failure

Original directory: `slot-155-eval-hard-001-no_occlusion/`.
The original attempt and all available raw records remain unchanged.

The frozen runner recorded `INVALID_TRIAL`, `task_started=false`, no retrieval
boolean, activation wall `1789113956.7486665`, finish wall
`1789113982.2975545`, and `RuntimeError: SIM exited before readiness`.

Read-only startup-log diagnosis found the specific failure in `attempt/runtime.log`:

- At simulation time 8.617, the Ground gripper home action aborted with
  error code -4 and `left_outer_knuckle_joint path error -0.080054`.
- At simulation time 8.620, Ground robot startup reported that its controller
  home motion failed.
- Required node `ground/spawn_ground_robot-18` exited with code 1, causing
  roslaunch to shut down the simulator before usable task execution.
- No aerial task, NBV, candidate selection or retrieval execution had started.
  The precise low-level cause of this isolated startup home deviation is not
  established; no controller, feedback, trajectory or tolerance change is made.

This is the explicit pre-usable-task simulator startup failure category in
section 6 of the frozen formal protocol, not a normal method failure being
relabelled. Its existing automatic classification is retained. Prior valid
Hard-001 failures in slots 151--154 remain valid and are not rerun.

After shutdown, no Gazebo, PX4, ROS launch or evaluation runner remained. The
same display is accessible, and tracked AGENT/SIM trees are clean. Resume only
with the one permitted same-slot/method/scene replacement in a distinct
`slot-155-eval-hard-001-no_occlusion-replacement-1/` directory, then original
slots 156--212. A failed replacement is not entitled to another replacement.

This consumes the second of at most 12 infrastructure replacement starts
(the first belongs to the USB-storage interruption at slot 131). All formal
entries and the separately documented one external bare Gazebo start count
toward 224 total starts. The original wall deadline remains
`1789495252.3554409`; no seed, method, configuration, budget, success predicate
or resource deadline is changed. First-activation sensitivity keeps this
original invalid activation unsuccessful as prespecified.
