# Public map goal tracking under dynamic localization

This is a SIM platform translation correctness fix, not an A1–A4 change or a
new controller. The existing public API accepts TF-resolvable poses; upper
tasks use fixed `map` coordinates. Native Prometheus remains flight authority.

## Measured cause

`natural-nav120-HH1UP2` stopped at the second viewpoint's A5 settling check.
A separate bounded public-flight probe recorded full `map←odom` and actual
public map pose. For the second requested map goal, the facade reported success
at 0.1494 m map error; after 15 seconds the error was 0.2366 m while speed was
0.0885 m/s. The original fixed odom target had moved about 0.159 m in map due
to localization updates. No Gazebo model state was consumed by this probe.

The current facade converts the target once at its original request timestamp,
then repeatedly issues a fixed native odom command and checks against that same
fixed target. This is inconsistent with a fixed map goal when map←odom changes.
The corrected localization itself remains necessary and unchanged.

## Minimal correction

In the existing SIM feature/fix-sim-uav-map-localization branch:

- For a map-frame FLY_TO goal, recompute its odom representation for each fresh
  odometry snapshot, using that snapshot's stamp. Use the same transformed
  target for completion, command and feedback. Preserve map XYZ/yaw exactly.
  Publish the current transformed native target even on the success iteration,
  so the backend does not retain the preceding iteration's obsolete odom goal.
- Keep existing one-shot semantics for other TF-resolvable input frames; this
  fix does not redefine moving-frame goals or add a new fixed-frame convention.
- Retain native Move/XYZ_POS commands, health checks, cancellation, caller
  timeout ownership, TAKEOFF/HOVER/LAND and existing failure handling. Do not
  publish MAVROS setpoints or add a position controller/state machine.
- Preserve the existing TAKEOFF position_tolerance at 0.15. Add a FLY_TO-only
  tolerance defaulting to that existing tolerance, and expose it through the
  existing launch include chain. The A5 verification run uses FLY_TO 0.05 m,
  tighter than its unchanged 0.10 m settling gate; no arrival, collision or A2
  threshold is relaxed. The first public probe at shared 0.05 m exhausted its
  90 s TAKEOFF guard before any FLY_TO, revealing the unintended parameter
  coupling. This separation restores original TAKEOFF behavior, not a change
  to native takeoff/controller geometry.

A5's measured hover drift and stamped per-packet coordinate alignment remain
unchanged. A4 receives the actual observation pose, not a corrected/faked pose.

## Implementation / checks

1. Add failing unit tests exercising the real facade method with changing TF,
   nonzero original request stamp, fresh snapshot stamps, and matching native
   command/completion/feedback values. Cover unchanged non-map behavior,
   cancellation, health/TF failure and launch default/override forwarding.
2. Implement the bounded translation correction and minimal launch arguments;
   run focused facade, localization and interface/launch checks and review.
3. Build only affected existing packages in the existing p450-clean profile.
   Re-run the public-flight probe with the tighter configured tolerance, then
   restart a fresh natural A5 E2E with navigation_timeout=120. Retain failures.

No frozen baseline/model, task floor, research method, benchmark, additional
dependency or safety/evidence framework changes are part of this fix.
