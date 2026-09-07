# A5 Ground / RM4D integration frame calibration

User-authorized integration calibration, not a change to frozen RM4D or A1–A4.

## Source of the reference plane

SIM `air_ground_standalone.launch` passes `bunker_z` (default 0.36 m) both
to Ground spawn and to `map -> ground/odom`. The planar runtime publishes
`ground/odom -> ground/base_link` with local z=0. This defines the public
nominal base reference plane. Its exact value is read from public TF, not
fitted from LiDAR and not supplied as an empirical AGENT offset.
The runtime BUNKER collision box has center z=-0.160420741 and height
0.395154782 (bottom -0.357998132 m relative to base). Thus 0.36 m is the
existing nominal mounting/contact convention, not exact collision settlement.
SIM AUBO mounting z is 0.042+0.08=0.122 m, matching frozen RM4D.

## Boundary

Let h be the public nominal base plane height. A5 checks horizontal Ground
reference frames and equal SIM/frozen BUNKER-to-AUBO mounting transforms.
Use `T_reference_map = Trans(0,0,-h)` and its inverse. Public map XY/yaw
are preserved; Ground spawn XY/yaw are NOT subtracted from RM4D queries.

The original exact map grasp is preserved. Reuse SIM's existing query-only
1e-6 rad regularization, then transform query into the frozen reference.
Call the frozen API in its `world` convention. Convert returned global
transforms into public map, rename `T_world_*` to `T_map_*`, and retain
unchanged local transforms, validation flags, joints, margins, order and scores.
A1 gets the adapted map result; A2 remains on physical ground z=0.
Save the original baseline result and explicit frame calibration alongside
the adapted result for ordinary debugging and validation, without new systems.

## Validation and scope

Tests cover sign, inverse, nonzero spawn XY/yaw, unchanged exact grasp,
unchanged baseline result, equal AUBO-relative TCP/flange transforms,
and rejection of nonhorizontal or mismatched-mount inputs.
M-R3/M-R4 validation remains valid in its original coordinates. New SIM
validation compares representative target/candidate local geometry and calls
actual MoveIt IK and planning without execution; this is additional integration
validation, not a new baseline validation claim.

After calibration passes, resume A5 with fresh actual MID360 data. Sampling
window timeouts remain a separate runtime issue. No baseline/model changes,
ground tolerance changes, A2 z shifts, A1–A4 modifications, or new framework.
