# Pilot-2 execution notes — in progress

No formal runs have started. Pilot-2 methods: **0/14** at this preparation
checkpoint. This file is updated as activations finish; it is not a completed
experiment claim. Protocol/seeds were committed before any method outcome in
`18fb57d`, with the serialized configuration and shared v1.1 switch in `ed87bd6`.
Pilot-1 and all frozen algorithm files remain unchanged.

## Preparation

- Nominal depth setup passes the necessary gate for all three seeds:
  Easy 3.319–3.585 m, Moderate 3.303–3.565 m, Hard 3.443–3.711 m.
  This does not establish actual FOV or observed support.
- Core regression: 433 tests, 427 passed and 6 OpenCV-only skipped; system
  Python `test_a5_target_support` passes all 11 including those 6.
- Independent review of `56ece4a..ed87bd6`: no important correctness/fairness
  findings; 39 focused tests and all six method CLI parses pass.
- System volume initially had only about 120 MB free. Moved the two prior
  project-owned temporary SIM review snapshots (not Git worktrees) from `/tmp`
  to `/media/lu/P450_PAPER/AGENT/runtime-scratch/`, preserving both; this freed
  about 600 MB. Bags go directly to the experiment volume and are Git-ignored.

## Setup activation history

`outputs/a6/pilot2/setup/easy-01`: no method started. The invocation omitted
the existing wrapper's display variables. Aerial RGB-D produced no frame and
`spawn_pick_target` reported its camera timeout at sim 35.808 s. Consequently
the common setup could not find `pick_target`. This is a launch-environment
error, not scene rejection or a method failure. Keep the directory and repeat
the identical Easy seed with the correct public wrapper environment; no SIM
source/threshold/timeout change is needed.

Every subsequent invocation must pass the established rendering inputs before
the existing environment-cleaning wrapper, for example:

```bash
P450_PX4_ROOT=/media/lu/P450_PAPER/P450-PAPER/workspaces/dependencies/px4 \
P450_GAZEBO_DISPLAY=:0 \
P450_GAZEBO_XAUTHORITY=/run/user/1000/gdm/Xauthority \
/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation/scripts/with_p450_env.bash \
  /usr/bin/env ROS_MASTER_URI=http://127.0.0.1:11951 \
  GAZEBO_MASTER_URI=http://127.0.0.1:11952 \
  /usr/bin/python3 -u -B scripts/run_a6_attempt.py \
  --config configs/a6_pilot2.json --setup-scene easy \
  --output-dir outputs/a6/pilot2/setup/easy-02
```

Use the serialized slot instead of `--setup-scene` only after all three setup
checks pass. A new output directory is required for each activation.

All three live setup checks now pass, without changing any pose, seed or gate:

| Setup | Window (sim s) | Ground cells | Mean ground z (m) |
|---|---:|---:|---:|
| Easy `easy-02` | 5.030 | 856 | 0.0000385 |
| Moderate `moderate-01` | 5.088 | 559 | 0.0000261 |
| Hard `hard-01` | 5.032 | 396 | 0.0000560 |

Each obtained an accepted aerial target and available v1.1 runtime perception
reference. No setup executed RM4D or policy scoring. Common initial pose remains
map (-1.4,0,1.2), yaw=0 for all methods. Setup bags close normally and can be
read: Easy 59,662 / Moderate 57,545 / Hard 61,131 messages, including RGB/depth,
controller state and TF. No overflow/drop warning was found. Unused action goal
and result streams correctly have no messages during sensor-only setup.

The mechanism reporter's independent review found two ordinary reporting bugs:
truncated compressed NPZ aborted aggregation, and sensor-only setup records
were counted as method attempts. Both are fixed with reproducing tests: missing
NPZ stays unavailable without losing other outcomes; setup is excluded while
INVALID method activations remain. Focused rereview: 25 reporter/scoring tests
pass and no remaining actionable findings. Natural v1.1 fixture still reports
two confirmed candidates and zero alias-retained exact candidates; this is not
presented as a rescue result.

Final pre-activation regression: 451 tests, 445 passed with 6 OpenCV-only skips;
the system Python's 11 target-support tests pass, covering all 6. The diff from
`56ece4a` leaves `src/`, A5 runtime/target-support code, and Pilot-1 configuration
unchanged. Setup, reporter and code review preparation gates are satisfied.
