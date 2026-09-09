# Validation interpretation before any feedback adoption

This is offline semantic validation, not authorization to replace feedback or
claim that finite differences are acceptable because they yield a PASS.
Detailed simultaneous 1 ms results for all three sampling phases are in
[REPORT.md](REPORT.md) and [summary.json](summary.json).

## Genuine unloaded wrist motion, not only stationary holds

In quick-solver launch02/03, the observation motion spans 2.45851/2.45015 rad
of unwrapped wrist3 position. The subsequent pregrasp motion spans
3.20059/3.18296 rad, mostly in the opposite direction. Pose-derived absolute
speed p95 is approximately 0.361 rad/s and maxima are 0.389–0.409 rad/s.
The independent parent-relative quaternion increments reproduce this motion,
after removing parent rotation and projecting on the parent-local hinge axis.
They are not simply confirming a stationary joint.

During these pre-close intervals the target remains on its original support
at z=0.056499510 m with zero recorded height change. The open-gripper width
reported by `GROUND_REFINED` is 0.0949264 m in02 and 0.0949230 m in03. The
target does not move with the hand: the relative target/hand displacement
reaches approximately 0.83–0.87 m during the large arm movements. This is
substantial pre-grasp, open-hand movement with no carried target, rather than
the later incomplete opening attempt being relabelled unloaded.

Moving wrist rate discrepancy p95 is approximately 1e-5 rad/s during
observation and 2.3e-8 rad/s during pregrasp. Retain the larger maxima:
0.003853/0.000701 rad/s in02 and 0.004090/0.010696 rad/s in03 for observation/
pregrasp respectively. Thus agreement is strong but not pointwise exact at
every moving sample.

The largest discrepancies are localized near native joint angle zero:

- 02 at17.065–17.066 s: q -1.06238e-5 to +8.60001e-6 rad; rate discrepancy
  +0.00385338 rad/s.
- 03 at31.191–31.192 s: q +0.000359701 to -1.06917e-5 rad; discrepancy
  -0.0106962 rad/s, followed by +0.0103782 rad/s on the next step.

This is consistent with sensitivity in near-zero angle extraction or hinge
alignment, but this analysis does not establish its implementation-level
cause. The extrema are neither removed nor smoothed. Off-axis quaternion
rate residual maxima are 0.00186–0.00523 rad/s during the moving stages;
these are reported explicitly rather than hidden by the projection.

## Loaded holds and all-joint distributions

During the closed holds, grasp confirmation is true at both event boundaries
and the target remains near z=0.2058–0.2059 m. Wrist quaternion-vs-joint rate
discrepancy maxima are below 1e-8 rad/s. Actual pose-speed distributions are
not all zero:

| Joint | 02 pregrasp abs speed p95 / max | 03 pregrasp abs speed p95 / max | 02 closed hold abs speed p95 / max | 03 closed hold abs speed p95 / max |
|---|---|---|---|---|
| shoulder_pan | .00967 / .01066 | .00908 / .01052 | .000111 / .00566 | .000116 / .00560 |
| shoulder_lift | .07394 / .07862 | .07705 / .08229 | .000171 / .01008 | .000131 / .00989 |
| elbow | .05669 / .06539 | .06074 / .06617 | .000401 / .00865 | .000330 / .00847 |
| wrist1 | .33210 / .36096 | .33767 / .36071 | .00236 / .00659 | .00203 / .00560 |
| wrist2 | .35666 / .39543 | .35803 / .40313 | .00534 / .01514 | .00451 / .01508 |
| wrist3 | .36134 / .39844 | .36142 / .38801 | .02250 / .05322 | .01864 / .03416 |

All values are rad/s from actual per-step joint-position increments. Only
wrist3 has independent simultaneous parent/child quaternion validation in
the existing CSV; the other five columns are measured-position distributions,
not a claim of independent six-joint proof.

Across every selected interval and phase, there are zero timestep/iteration
gaps and zero missing/nonfinite joint/parent/child geometry pairs. All selected
phase intervals are 1 ms. The caller must still verify the real `readSim`
period before interpreting the proposed quantity as a 1 ms average.

## Existing bag poses and scope of further corroboration

Launch02's diagnostic bag contains 39,181 `/gazebo/link_states` messages and
3,919 `/ground/joint_states` messages. All six moving arm links and the
reduced base link are present. This is sufficient source material for a
coarser all-joint relative-pose check without adding CSV fields.

However, LinkStates has no header timestamp: bag receipt time is not an
independent acquisition timestamp. A future cross-topic check must report
receipt/header alignment uncertainty and must not be described as the same
simultaneous 1 ms evidence as the wrist CSV. This deliverable establishes
availability, not an unperformed six-joint quaternion comparison.

## World-solver04 is a rejected physical reference

The entire recorded04 trace was examined, including its start-up and failure.
Even though its CSV is finite and has no missing timesteps, catastrophic
outliers occur: wrist joint/quaternion rate discrepancy reaches1114.65 rad/s,
off-axis rate3066.04 rad/s, and relative target/hand displacement6.31e8 m.
That failed-LCP trace is not support for adopting the world solver or a
velocity semantic. Finite numeric output alone is not physical validation.

## Implication, not acceptance

The data supports calling unwrapped position difference divided by elapsed
time an **integrated-pose interval-average velocity**. It is independently
consistent with observed wrist motion over substantial nonzero movement and
loaded holds, within the stated measured errors. It is not identical by
definition to instantaneous native ODE velocity, nor a general solver fix.

A trial opt-in implementation still needs startup/reset/invalid-dt handling,
unchanged position/effort/control paths, raw native trace retention, controller
stability review, and fresh physical motion/grasp/lift/retention checks under
the original criteria. No threshold, controller, solver, or feedback code
was changed by this offline validation. Two new math checks and the existing
five analyzer checks pass, including moving-parent and off-axis cases.
