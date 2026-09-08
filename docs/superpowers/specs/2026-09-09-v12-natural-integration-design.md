# Frozen v1.2 natural integration recovery

The user has approved exact-winner semantics and authorized autonomous minimal
runtime/platform bug repair until the existing natural A5 regression passes.
No formal slot, new pilot execution, algorithm fallback or parameter relaxation.

## Scope and diagnostic choice

Use the original natural scene and initial view (-.5,0,1.5,0), with v1.1 evidence
and exact_winner support. Preserve .10 m/.10 rad/.10 m/s settling, .5 s dwell,
existing guards, three observation windows and physical lift success criterion.
The separate frozen experimental initial view is not edited either.

First replay the live natural sequence unchanged, recording existing public
ROS TF, UAV state, control/setpoint and flight-action topics through rosbag.
No Gazebo truth is used for sensing, decisions or this diagnostic. The existing
physical checker remains evaluation-only. Preserve every activation and result.

Alternatives considered: immediately changing waits cannot establish the cause;
replacing the controller or changing capture/threshold semantics exceeds this
task. Passive raw-topic diagnosis is the smallest first step. The source shows
HOVER acknowledges command publication, while capture later freezes a measured
anchor; determine from the trace whether sequencing, stale data or platform
configuration prevents the unchanged condition from becoming true.

## Repair boundary

Only repair an observed contract violation. A root-cause-dependent patch must
first have a reproducing test, retain the predicate and initial pose, and pass
independent spec then quality review. Ordinary startup/flight-action ordering
is permitted; introducing a different control law, feasibility gate, alternate
view/ground candidate, weaker observation or success rule is not. If the latter
is required, stop with the measured evidence for research review.

Freeze A1/A2/v1.1/A3/v1.2/A4 numerical source and RM4D assets at9f3a527. In AGENT,
only ROS integration scripts/tests and new diagnostic/report outputs may change.
If SIM source needs a confirmed bug fix, use a separate SIM feature branch and
minimal platform regression, never patch frozen robot/algorithm definitions.

## Acceptance and handoff

Require real initial sensing, A1, MID360/A2, A3/A4, further observation when
selected, exact confirmed Ground handoff, BUNKER, Ground D435 refine, grasp and
independently checked lift. Keep non-winner viability diagnostic only.
After passing, run frozen-core/Hard replay/ROS/task-map regressions, document
repairs and all failed launches, commit/push the checkpoint, and retain Draft PR.
Prepare a small fresh-seed paired validation proposal without generating method
outcomes or starting any pilot/formal run. The sixteen interrupted v1.1 formal
outcomes remain unchanged and excluded from final v1.2 statistics.
