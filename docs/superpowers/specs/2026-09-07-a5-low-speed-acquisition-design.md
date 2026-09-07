# A5 low-speed scan-window acquisition correction

This is an autonomous integration/runtime correction, not an A2 or A4 change.
The user explicitly requested separate diagnosis of acquisition timeouts after
task-floor validation. Task-floor planning checks now pass.

## Observed cause

In fresh run `natural-task-floor-ixrpeB`, initialization succeeded but a native
HOVER moved about 0.10 m relative to the measured anchor in 3.3 s while reported
speed was about 0.06 m/s. A5 discarded 33 stamped packets and timed out. Prior
runs exposed the same fixed-anchor condition. Public HOVER acknowledges a native
odometry hold; it does not promise indefinite 0.10 m public-map position hold.

The acquisition implementation incorrectly reused the flight-arrival spatial
tolerance as a whole-window fixed-location condition. A2 is endpoint-only;
every packet already uses its own public map TF and the final merged observation
re-expresses those endpoints exactly. Low-speed drift between packets does not
invalidate their map geometry.

## Minimal selected correction

Keep requested-viewpoint arrival unchanged. At acquisition start, retain and log
the measured anchor and the existing initial settling check. During the window,
require fresh health/TF, low speed, yaw near the anchor, and current/stamped
poses within operating bounds. Compare delayed packet position to **current**
position using the existing 0.10 m tolerance, not to the original anchor.
Thus a slowly drifting, locally consistent window can complete; a delayed packet
from another position, excessive speed, yaw departure or missing TF still breaks
the window. Recovery still discards all partial endpoints and requires fresh
packets after continuous settling within the original deadline.

Rejected alternatives: repeatedly enlarge the fixed anchor radius (does not
separate arrival from acquisition semantics); shorten windows or grant per-packet
votes (does not address the nonrepetitive scan's sparse coverage and would change
observation meaning). No controller/SIM change or moving-flight mapping module.

The final stamped pose goes to A4; merged points and one whole-window A2 vote
are unchanged. This is low-speed scan acquisition with between-packet alignment,
not within-packet deskew or an assertion that a viewpoint remains mathematically
fixed throughout the dwell.

## Implementation / verification plan

Files: `scripts/a5_ros_support.py`, `scripts/run_a5_sim.py`,
`tests/test_a5_ros_support.py`; update A5 acquisition documentation.

- [ ] Add failing pure and adapter tests: slow common current/stamped drift beyond
  anchor succeeds, flight arrival still rejects it, distant delayed stamp and
  excessive speed/yaw/out-of-bounds remain rejected.
- [ ] Add an acquisition mode to the current readiness check; use the same
  speed/yaw/health checks but no original-anchor position condition. Keep packet
  current/stamped relative-position consistency and explicit bounds in capture.
- [ ] Run focused Python 3.10 and 3.8 adapter tests and independent review.
- [ ] Repeat natural A5 from fresh aerial/MID360 input. Do not reuse partial
  failed clouds, fabricate FREE evidence or modify frozen modules.
