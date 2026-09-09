# Launch03: failed release attempt, not a completed unloaded hold

The original lift failure at 46.291 s remains authoritative. The closed hold
is 46.294–48.317 s. The measured-start gripper-opening attempt then fails at
49.320 s; there is no `open_hold` BEGIN/END event. The analyzer labels the
bounded 48.317–49.320 s interval `release_attempt_failed`.

The trace demonstrates a sharp wrist-rate change during opening, but does
not demonstrate target detachment or an unloaded hand.

## Native wrist behavior at 1 kHz, update end

| Interval | Native dq median, rad/s | Native dq integral, rad | Joint-angle change, rad | Joint-angle span, rad |
|---|---:|---:|---:|---:|
| Lift final 2 s | -0.168326 | -0.336966 | -0.0000391479 | 0.000308011 |
| Closed hold | -0.163121 | -0.334423 | -0.0000231420 | 0.000108838 |
| Entire failed release | -0.00191223 | -0.0257339 | +0.0000598973 | 0.000488427 |
| Release 48.317–48.420 s | -0.160591 | -0.0168401 | +0.0000210390 | 0.0000761305 |
| Transition 48.420–48.520 s | -0.100148 | -0.00883257 | +0.0000354408 | 0.000272980 |
| Later attempt 48.520–49.320 s | -0.0000410241 | -0.0000612399 | +0.00000341747 | 0.000488427 |

The descriptive subwindows are not acceptance thresholds or proven contact
state transitions. Their endpoints are included in both adjacent summaries;
do not sum their sample counts. Detailed values are in
[release-subwindows.json](release-subwindows.json).

During the final 0.8 s the native rate mean is -0.0000537224 rad/s, range
[-0.0701073,+0.0606914], with 95th-percentile absolute rate 0.0299230 rad/s.
The negative bias present throughout the closed hold is therefore largely
absent after the opening transition. The maximum wrist quaternion-vs-joint
increment discrepancy over the complete release is 3.67e-11 rad.
Before/after base setters still cause exactly zero immediate wrist q/dq
change. Neither native feedback nor the task result is replaced by these
pose-derived diagnostics.

## Target and hand movement

At release start the target model origin is at z=0.205877929 m; at failure it
is at z=0.206334482 m. Its total XYZ change is
(+0.525237,-3.270383,+0.456553) mm. It does not fall back toward its original
pre-lift height of approximately 0.056903 m. The wrist-child origin changes
only (-0.006543,-0.008537,-0.003514) mm over the same attempted release.

The target displacement expressed in wrist-child coordinates reaches only
3.31728 mm from the release-start value. Its relative coordinate changes
from (0.00231243,-0.00302059,0.21752811) m to
(-0.00097021,-0.00319388,0.21708253) m. After 48.520 s the additional relative
displacement is at most 2.09871 mm. Thus the target remains elevated near the
hand during the sampled opening attempt, with lateral movement rather than
observed separation/fall.

This supports sensitivity to the changing gripper/contact constraint state,
not an isolated full-mass-unload conclusion. The trace does not contain
reaction forces or sufficient contact geometry to quantify how much load
the hand still carries. The failed action also does not establish a fully
open gripper. Root is inspecting the controller result separately.

## Baseline for the next solver contrast

Use the closed-hold or post-command lift data as the constrained baseline:
approximately -0.16 to -0.17 rad/s native wrist rate with tiny net pose
rotation. Keep the later attempted-opening regime separate; its much smaller
rate follows a changed gripper command and is not a matched solver-only
contrast. The original native-speed failure, absence of a completed open
hold, and physical target measurements must remain separately reported.

Analysis read 28,977 selected rows with no malformed or unknown-phase rows.
Five numerical/interval-label tests pass, including a regression proving that
a failed release is not relabelled as an open hold. No online process or
production change was made by this diagnosis.
