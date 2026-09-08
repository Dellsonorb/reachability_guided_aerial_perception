# Activated natural A5 regression — FAIL (not INVALID_TRIAL)

The original natural settings and approved exact_winner/v1.1 options were used.
After adapter readiness, the passive checker connected before task activation.
PREFLIGHT, takeoff, aerial perception and RM4D initialization completed. The
original pre-window hover-stability check then failed. Both adapter and checker
exited 1. The original failure cleanup landed the UAV and the owned Gazebo/ROS
runtime was shut down normally.

No MID360 window completed; no A3/A4 decision, confirmation, Ground action,
D_exec or retrieval was reached. This genuine activated regression failure is
preserved without threshold changes, outcome-based retries or an invalid-trial
label. It is not one of the 16 interrupted v1.1 formal outcomes and is not a
final v1.2 statistical sample. v1.2 natural acceptance remains unpassed.

See [review report](../../../../docs/EXACT_POSE_SUPPORT_V12.md) for the precise
stage, observed public capture pose, diagnosis limits and original run settings.
`adapter.log` contains the observed states; `physical_summary.json` is the
existing checker's raw failure result. No trajectory or success is inferred.
