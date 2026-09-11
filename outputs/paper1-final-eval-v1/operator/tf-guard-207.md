# Slot 207: preserve the observed TF-guard failure

Read-only diagnosis during the final scheduled collection. The original
attempt/entry, metrics and physical summary are unchanged. No extra simulator
launch, replacement, algorithm/configuration change or threshold change.

Slot 207 reached normal takeoff, initial hover, AIR_OBSERVE and AIR_HANDOFF.
Runtime perception produced a target reference (122 points, 261 mask pixels)
at simulation stamp 34.542. The adapter then emitted FAILED at simulation
35.132: `A5 Ground map TF is stale`.

The frozen `_a5_frame_calibration` checks the age of its locally received
`ground/odom -> ground/base_link` transform against `[0, 0.5]` seconds. It does
not log that local transform's stamp or computed age on rejection. Thus the
message alone cannot distinguish old local data from a negative age due to
asynchronous clock/TF receipt.

The retained diagnostics.bag shows regular ground odometry and base transforms
around the failure (roughly every 0.02 simulation seconds). For example TF
receipt/stamp pairs are (35.110, 35.109), (35.130, 35.130), (35.151, 35.149).
No process death or broken publishing contract is demonstrated in the retained
logs. These recorder observations do not establish what the adapter's separate
subscriber buffer and local clock contained at the rejection instant.

Accordingly, this is a VALID_TRIAL failure of the frozen runtime's TF acceptance
guard, with unresolved low-level cause; it is not evidence of a proven
infrastructure-invalid trial. Keep its primary binary outcome false and do not
replace it. Report it separately from physical reachability/sensing-budget
limits. Future development may add age/stamp diagnostics, but not in this cohort.
