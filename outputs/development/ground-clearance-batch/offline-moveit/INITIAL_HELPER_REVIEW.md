# Initial pure clearance helper — read-only spec review

Reviewed the initial uncommitted `execution_clearance.py` and its five tests.
Production files were not edited. Five native-message tests pass, but they do
not compare trajectory initialization to native JTC or load the production URDF.
Additional offline checks below use the actual renderer and installed headers.

## Findings

1. **P1 — first commanded segment is not the sampled segment.** Initial helper
   lines 119–123 require a time-zero first waypoint and start sampling from that
   waypoint, disregarding the supplied start state's arm positions/velocities.
   Installed `init_joint_trajectory.h:345` discards the zero-time first point
   when `header.stamp == 0`; lines 416–460 bridge from the current **desired**
   trajectory state to the first remaining waypoint. Native reproduction with
   desired hold q=0.4, message q0=0 at t=0, q1=1 at t=1 and zero derivatives
   gives native q(0.5)=0.7; this helper gives 0.5. Native first sample is 0.4;
   helper first sample is 0. The first segment can therefore cross unchecked
   geometry. The helper also rejects ordinary positive-first-time inputs that
   JTC correctly bridges. Reproducer: `jtc_start_bridge_probe.cpp`, compiled
   against installed headers; no node or controller action runs.

2. **P2 — complete start-state validation is missing.** Initial
   `state_at_positions`, lines 98–103, zips start names and positions without
   first validating their lengths, uniqueness and finiteness. A start message
   with names `[j1, gripper]` and positions `[0.4]` is accepted and returned with
   two names but one position. Empty trajectory name lists and position lists
   likewise produce unchanged states. Runtime freshness/active-joint checks
   reduce the immediate exposure, but the pure helper's stated complete-state
   contract currently permits malformed requests and silent loss of non-arm
   state. Validate complete input state once before generating samples.

3. **P2 — unexpected over-closed start is converted to an opening sweep.**
   `closure_positions(0.6, 0.457, 0.005)` returns a descending sweep from 0.6 to
   0.522, although the physical close still commands 0.70. A measured start above
   the intended checked envelope should reject as unsupported, not generate
   the opposite motion. Ordinary preshape checks should prevent this state,
   but the closure checker should fail explicitly if it occurs after descent.

4. **Documentation mismatch — sampling is maximum 1 ms spacing, not the exact
   physics-time lattice.** A linear q=t segment of length 2.5 ms yields samples
   at 0, 0.833333, 1.666667 and 2.5 ms. A controller lattice would contain 1 and
   2 ms instead. This is a valid dense spatially sampled curve, with the stated
   continuous-proof limitation, but its documentation must not identify those
   states as exact simulator update instants.

The helper itself has no maximum sample count or wall-time budget; the reviewed
design requires that the caller enforce one and reject incomplete checks.

## Checks that passed

- Actual `render_ground_robot` output has root `ground/base_link` and the
  expected authoritative box dimensions/origin. The parser accepts it.
- Its only root-rigid collision-bearing links are base_link, aubo_mount_link
  and aubo_i5_base_link. D435 and hand links are correctly outside this set.
- The proxy dimensions add 24 mm to full box lengths, preserve its collision
  origin/orientation, and leave production physical geometry untouched.
- Applying proxy ACM, accepted target update, and contact enable/disable/enable
  updates leaves guard/target false in both symmetric entries throughout.
  Existing `manipulation_scene` merging correctly preserves this restriction.
- Cubic/quintic coefficient construction follows the installed segment's
  lowest-common derivative order and has the correct endpoint conditions for
  subsequent message segments. The existing quintic overshoot test exercises
  a real non-linear curve, but does not test native initialization.
- Deep-copying the full reference state preserves attachments and non-arm
  fields when the reference is valid. A dedicated attachment regression would
  make that critical contract explicit.

## Reviewed minimal repair contract

The root's proposed repair is appropriate: require a fresh controller desired
**hold** after the preceding arm action is terminal or its cancellation/stop
trajectory has completed, supply that held q/dq/ddq explicitly, drop zero-time
message points as native JTC does, and sample the bridge to the first positive
waypoint. Support positive-first-time trajectories. Require zero header stamp
unless delayed-start behavior is modeled explicitly.

Exact desired dq=ddq=0 at one instant does not by itself prove a hold: a moving
trajectory may contain a stationary knot. Require serial motion ownership,
previous action completion/stop and no pending scheduled arm command before
accepting fresh hold feedback. Validate held point names, sizes and finite q;
require actual desired dq/ddq exactly zero under this explicit contract.

Continue to check the measured full state separately. A measured-to-held-state
geometric interpolation is useful additional validation, but is not the
controller's command bridge, whose start is its held desired state. The command
and all original controller stopping/grasp thresholds remain unchanged.

This review records the initial version; it does not claim the later repair or
runtime integration has been verified.
