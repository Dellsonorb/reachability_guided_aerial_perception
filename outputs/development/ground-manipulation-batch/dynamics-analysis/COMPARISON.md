# Launch01/02: native rate and pose are different diagnostic quantities

2026-09-09. Offline CSV/event analysis plus read-only extraction of recorded
arm goals and public joint states. No ROS/Gazebo starts, changed acceptance,
feedback replacement, or edited historical trials.

Use the final [launch01 report](launch-01-final/REPORT.md) and
[launch02 report](launch-02-final/REPORT.md); their JSON files contain every
phase, all six joints, missing-data counts, integrations and body diagnostics.
The earlier `launch-01`/`launch-02` folders retain preliminary measurements;
the `-final` outputs additionally include cadence and explicit closed/open
availability. Their physical measurements are unchanged.

## Main observation

The failure is not explained by a ROS-only velocity readout error or by an
unseen +/-500 Hz cancellation. At 1 kHz, native wrist rate agrees with the
parent/child angular-velocity projection, while native joint-angle increments
agree with relative-parent/child quaternion increments. Those two groups
disagree strongly with each other. A success in launch01 does not eliminate
the same discrepancy.

| Window, update-end samples | Native wrist rate range, rad/s | Native rate integral, rad | Joint-angle change, rad | Relative-quaternion change, rad | Joint-angle span, rad |
|---|---|---:|---:|---:|---:|
| Success01 final 2 s of lift | -0.198777 to -0.0596645 | -0.2638214 | +0.0001840524 | +0.0001840523 | 0.000365667 |
| Failure02 final 2 s of lift | -0.237810 to -0.137175 | -0.3365345 | -0.00001788077 | -0.00001788077 | 0.000176883 |
| Failure02 closed hold, 2.007 s | -0.243169 to -0.0642041 | -0.3187668 | -0.00003681685 | -0.00003681678 | 0.000200075 |

The native rate minus independently recomputed
`(child_angular_velocity - parent_angular_velocity) dot GlobalAxis` is at most
5.56e-17 rad/s. In failure02, the maximum per-step quaternion-minus-joint
angle increment is 3.24e-11 rad over the lift, 5.85e-12 in its final 2 s and
9.91e-12 in the closed hold. Selected samples are complete, consecutive
1 ms physics steps; no integration gap or malformed CSV row was found.

There is a small near-Nyquist modulation, but not opposite-sign cancellation.
Failure02 final-2-s update-end mean rates on even/odd iterations are
-0.161596/-0.174930 rad/s, overall mean -0.168266. All 2,001 rates are
negative. Its largest nonzero discrete FFT bin is 499.750 Hz with
abs(FFT)/N 0.00449050 rad/s. Thus lower-rate sampling can affect the observed
value, but cannot account for the 0.3365 rad integrated negative rate against
near-zero net rotation.

## Phase separation and limits of attribution

Before/after the base velocity setters, wrist position, rate and native
effort are exactly unchanged for every matched sample in both trials.
End-of-step wrist position also exactly matches next-step pre-setter position.
This excludes an immediate wrist state rewrite by these two setters as the
observed mechanism. It does not exclude coupling through the subsequent
constraint solve or other callbacks.

Root's separate upstream source inspection identified ERP/non-ERP velocity
handling in ODE QuickStep as a candidate mechanism: pose integration and
the subsequently exposed velocity can use different quantities. The measured
two-group consistency above supports investigating that distinction. This
CSV does not expose the internal ERP velocity or constraint impulses, so it
does not independently measure their decomposition or establish a solver fix.
No solver setting or native feedback is changed by this analysis.

Failure02's final-2-s wrist3 native effort median is 0.0027115, range
[-0.0104003, 0.0128276]. Other chain medians are shoulder-pan -0.0327527,
shoulder-lift 68.0539, elbow -15.2682, wrist1 -3.63084, wrist2 -0.0104648.
These are `GetForce` values, not gripping/contact reaction measurements.

## Actual commanded branch and timing

Vectors below follow the recorded trajectory order:
`[elbow, shoulder_lift, shoulder_pan, wrist1, wrist2, wrist3]`, in radians.
They are commanded final points, not IK guesses or measured poses.

| Trial | Grasp/descent final joint point | Lift final joint point |
|---|---|---|
| New01 success | [0.951306, -1.452674, 0.535250, -2.308405, -1.570809, -0.729619] | [1.127545, -1.154085, 0.535250, -2.430755, -1.570808, -0.729622] |
| New02 failure | [0.956371, -1.448804, 0.528114, -2.307210, -1.570809, -0.742391] | [1.130746, -1.150774, 0.528114, -2.430865, -1.570808, -0.742394] |
| Previous execution-batch07 failure | [1.594509, -1.247338, 0.495168, 1.271062, 1.570810, 2.437298] | [1.746532, -0.904644, 0.495168, 1.080390, 1.570809, 2.437295] |

Both new trials use the flipped wrist branch, unlike prior07. The new01/new02
branch identity therefore does not separate success from failure, although
their commanded points differ slightly and this is not an identical-trajectory
counterfactual.

New01's lift goal was stamped 40.721 s with duration 4.609743952 s; its nominal
command endpoint is 45.330743952 s and stage completion is 45.372 s. New02's
corresponding values are 40.428 s, 4.605478574 s, 45.033478574 s and a failed
stage at 47.037 s. These nominal endpoints use recorded goal stamp plus
trajectory duration, not an independently logged controller-start timestamp.
The automatically labelled tail is thus mainly ongoing motion in success01,
but begins just after nominal trajectory completion in failure02. The two
tails must not be described as identical settling intervals.

Public `/ground/joint_states` near success01 completion reported wrist rates
-0.164607 at 45.351, -0.101349 at 45.361 and -0.114492 at 45.371 s, with
stage END at 45.372. Failure02's last recorded pre-outcome sample at 47.028
is -0.189731 rad/s, followed by FAILED at 47.037. This is concrete evidence
of different sampled rates at the outcome checks, not a claim that the
acceptance criteria were changed or that pose-derived rates should replace
native feedback.

## Load and retention boundaries

Success01 records lift and retention completion; its failure-only contrast
was not triggered. During the recorded lift stage the target rises 148.894
mm, with maximum target displacement relative to wrist-child coordinates
1.4552 mm. During retention its target height change is -0.00585 mm and
relative displacement is at most 0.0887 mm. These event-bounded differences
need not equal root's separately anchored full physical-lift metric.

Failure02 physically raises the target 149.301 mm during its lift stage but
still retains its original native-speed failure. Its closed hold is exactly
47.041–49.048 s, with grasp confirmation true at both boundaries. The actual
gripper-open action fails at 49.150 s, so no open-hold event or unloaded
contrast exists. Target movement in the closed hold is diagnostic only;
unloaded behavior remains unassessed.

## Reproduction and checks

`analyze.py --trace CSV --events events.jsonl --output-dir NEW_DIRECTORY`
reads only completed event-bounded lift/retention/hold rows into NumPy and
refuses an existing output directory. It records missing/nonfinite fields as
JSON null plus counts, never as observed zeros. Standard-library plus NumPy
only; `python analyze.py --self-test` passes four numerical checks covering
rate/angle disagreement, common-frame rotation removal, missing-step exclusion
and a pure 500 Hz alternating-rate example.
