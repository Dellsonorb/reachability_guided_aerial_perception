# Repaired pure helper — spec and separate native quality review

Result: no remaining blocking finding in the reviewed **pure-helper** scope.
This does not review runtime hold acquisition, motion ownership, RPC traversal
budgets, prospective scene restoration or final action wiring.

The initial findings in `INITIAL_HELPER_REVIEW.md` are addressed:

- The explicit held-desired input implements immediate JTC initialization:
  drop zero-time first point, bridge from the held desired state, accept a
  positive first waypoint, and reject unsupported delayed header timestamps.
  Held desired q must be finite and its full dq/ddq exactly zero.
- Complete start names/positions are validated before replacement. No silent
  zip truncation or nonfinite non-arm state is accepted; empty trajectory joint
  names reject.
- A measured closure start above the checked loaded range rejects instead of
  returning an opening sweep.
- Sampling is accurately described as at most 1 ms spacing, not an exact
  simulator-time lattice or continuous clearance guarantee.

Prospective attachment now calls `_attachment_geometry_diff` directly through
its expressly hypothetical wrapper. Real `attach_target_diff` still checks
fresh-confirmation input before calling that same geometry function. No
fabricated confirmation boolean or altered object geometry was introduced.
The original four finger/pad exceptions and target/proxy exclusion are retained.
The existing native scene round-trip regression still passes after extraction.

## Separate native polynomial comparison

`spline_crosscheck.cpp` uses the **installed**
`trajectory_interface::QuinticSplineSegment<double>` and
`joint_trajectory_controller::initJointTrajectory`, rather than duplicating
the production Python coefficient formulas. The Python driver supplies
deterministic trajectories and compares the returned positions with production
`execution_clearance.controller_samples`.

81 cases cover all 27 triples of endpoint derivative availability
(position only, position/velocity, position/velocity/acceleration), each in
nominal, immediate held-with-zero-first, and held-with-positive-first modes.
Each has two arm joints and nonuniform segment durations, including 1.1 ms and
7.3 ms. Full-state joint ordering differs from trajectory ordering. Every
sample preserves the non-arm gripper value and an attached-object sentinel;
input state, trajectory and hold messages remain unchanged.

```
PASS cases=81 scalar_positions=31138
max_absolute_error_rad=3.1558089474970075e-13
```

This is a comparison to the native numerical implementation under the stated
held-desired contract, not an online confirmation that the controller is
actually holding or that physical tracking follows the nominal spline.

## Regression checks and review boundary

Eight clearance helper tests pass. All 21 existing manipulation-scene tests
also pass, including the actual rendered model/contact mesh and native C++
scene round-trip tests. The first invocation lacked the runtime package source
directory in `PYTHONPATH`, causing two import errors; rerunning with that
existing directory included resolves both. No production fix was needed for
those invocation errors.

Native oracle programs compile with installed `moveit_core` pkg-config flags
and run offline. No ROS master, controller action or Gazebo launch was used.
Production files were not edited by this reviewer.

Reviewed file hashes:

- `execution_clearance.py`:
  `daef34259c87ea8301fa93b9da0e217d7e2bc92134ee0678936fb18bb04cdafc`
- `manipulation_scene.py`:
  `5032b8609c5ffd5115485ed5a9e1a426557e77f0e5e00b353b3a258432a3bb4b`

The runtime caller must still enforce the documented conditions: previous
serial arm action/stop is complete, no pending arm command, fresh desired hold,
fresh separately checked measured full state, exact hypothetical scene cleanup
before actuation, preserved payload/non-arm state, complete sample traversal
with a finite budget, and unchanged actual collision/grasp/controller gates.
