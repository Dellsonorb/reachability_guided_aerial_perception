# Authorized Paper 1 evaluation execution

Authorized by the user on 2026-09-11 Asia/Shanghai. Use AGENT `3d13a95`,
`configs/paper1_eval.json`, and `paper1-eval-finite-scan-v1` exactly as prepared.
SIM is `a0ae8e3`; original RM4D is `e9d4312` plus the existing task-domain asset.
Use the same local rendering environment as the evaluated twelve-task batch:
`P450_GAZEBO_DISPLAY=:0` and
`P450_GAZEBO_XAUTHORITY=/run/user/1000/gdm/Xauthority` (no credential contents logged).
No method/configuration changes, scene retesting, new seeds, or old 560-slot runs.

Run all 212 slots in frozen order. At most 12 documented infrastructure-invalid
replacements, one per original slot, at most 224 total starts. Valid method
failures are retained without retry. Unknown classification is not permission
for an automatic retry. Every new attempt gets its own directory.

The hard deadline is the earliest EVALUATION_ENTRY `start_wall` in this directory
plus 432000 seconds (120 elapsed hours). Derive it from persisted entries after
any session/process restart; do not reset it. The 24–36 hours is only an estimate.

Maximum new total footprint: 500 GiB, including temporary/compression data and
external SIM/PX4/cache growth above `resource-baseline.json`. Data disk must
retain at least 100 GiB free. Check time/space before each start and while running;
reserve the existing startup/task/cleanup time and room for shutdown. Use one
Gazebo task at a time. Resource exhaustion leaves the study incomplete, not a
smaller denominator. No historical or unresolved data removal.

The operator's pre-start allowance is 1800 seconds and 32 GiB of free budget
for the next task and its cleanup (existing 180-second startup, 1200-second task
guard and owned-child cleanup; image/native-data growth monitored during runs).
This is a resource reservation, not a robot threshold, trial time extension,
or additional observation budget. Stop early if the remaining reservation
cannot fit; the 120-hour/500-GiB/100-GiB-free hard bounds remain unchanged.

Use the prospective post-pair CSV gzip and RGB-D retention rules in
`docs/PAPER1_FORMAL_EXPERIMENT_PLAN.md` section 7. Keep all compact records,
nonimage diagnostics, compressed native CSV, failures and invalids. For the
predeclared successful image-retention scenes keep every method's RGB-D bag;
other successful image bags may be omitted only after physical/stage/missingness
review and representative frame export. Record any such removal explicitly.

The fixed run checkout stays at `3d13a95` throughout acquisition. Commit results
only after the last authorized launch, so the ordinary recorded runtime commit
does not change merely because a result document was committed. Historical
outputs and evaluation tags remain untouched. Do not merge main.

Slots 1–3 were launched individually with the unchanged public entry. Starting
at slot 4, `operator/serial-execution-recipe.py` records the one-time operator
loop used in this session: the same explicit public command per frozen slot,
resource checks, no automatic retry, and post-pair offline analysis/retention.
It is a run recipe, not a new robot runtime or experiment framework. Its
`progress.jsonl` includes actual starts, stage/resource samples and any stop.
Do not rerun the recipe blindly: completed directories are never overwritten;
resolve the existing active process and records first. Epoch deadline stays fixed.

Pre-run engineering cleanup: sent SIGINT to the orphaned process group 1201811
(Gazebo PID 1201814), pointing to the already completed historical pilot slot 5.
Both exited; no historical files were deleted. This was not a new formal start.
