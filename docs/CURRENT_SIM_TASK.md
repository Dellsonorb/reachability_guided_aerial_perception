# Current sensor-driven retrieval task

Development profile: `configs/current_sim_task.json`, `sensor-retrieval-screened-handoff-v1`.
This is the current program entry, not a formal experiment or an automatic matrix.
The default target is the known red brick class/dimensions in the configured initial
camera search region; this is not arbitrary object/world search.

## Start and inspect

From the AGENT repository, a fresh natural scene and Ours:

```bash
python3 scripts/run_retrieval.py run --output-dir outputs/tasks/natural-001
```

To run a named development scene with Generic (scene geometry is SIM setup only):

```bash
python3 scripts/run_retrieval.py run \
  --scene-file configs/dev_multiscene_paired.json --scene-id paired-moderate-01 \
  --method generic --output-dir outputs/tasks/moderate-generic-001
```

`--dry-run` prints the exact command/profile without creating output or starting
robots. `--sim-root` and `--rm4d-root` point to the installed platform and frozen
baseline checkout. Defaults describe the current workstation, including the
existing temporary frozen RM4D checkout; on a different host supply a persistent
checkout of `rm4d-aubo-baseline-v1`. `P450_PX4_ROOT` can override the existing PX4
location. The existing SIM `with_p450_env.bash` prepares Noetic/MoveIt/Gazebo.
No package download or robot installation occurs in this command.

The runtime needs the installed SIM on `feature/fix-checker-observation-retention`
(includes the earlier execution/feedback fixes), the existing Python3.10 RM4D
environment, and `assets/rm4d_ground_task_v1/rmap.npy`. Scripts themselves run in
Noetic Python3.8; the numerical worker runs in the existing Python3.10 environment.
Do not substitute the positive-z literature map for the calibrated task asset.

Only one task can own local ports11951/11952. The entry rejects a busy port and an
existing output directory; it never kills another task or replaces old results.
This entry is explicitly single-host: it sets ROS_IP/ROS_HOSTNAME to127.0.0.1 and
ROS_IPV6 tooff in the setup parent and all children. This prevents the observed
old Gazebo LAN reconnect from entering a recycled TCPROS port. It is not the
network configuration for distributed/real robots. Before arming, the task also
waits for its own fresh public UAV/Ground TF, bounded by the existing30s preflight
timeout; another node's ready buffer is insufficient.
If the local Gazebo renderer needs an existing X session, set the same
`P450_GAZEBO_DISPLAY`/`P450_GAZEBO_XAUTHORITY` pair used by SIM; these are platform
environment inputs, not perception data. No GUI/GT clicking is needed.

While running or after termination:

```bash
python3 scripts/run_retrieval.py status outputs/tasks/natural-001
```

This reads recorded state, window/confirmation count, first failure and retrieval
result; it does not control robots. `IN_PROGRESS` is an unfinished recorded attempt,
not a liveness guarantee. `null` retrieval is not success or failure. The command
exit is0 only for independently checked retrieval success; other outcomes exit
nonzero. The existing runner tears down only children it launched.

## Actual behavior and common components

P450 normally takes off, flies to the common initial pose and detects the target
using aerial RGB-D. Frozen RM4D plus the task-domain asset produces exact evaluated
candidates. Real MID360 endpoints/public TF update A2 and the v1.4 operational
sidecar; exact-winner footprints form manipulation support. Generic/Ours use the
same viewpoints, visibility, flight cost, three-window cap, sensing and execution;
only the existing NBV gain weighting differs.

New explicit `--handoff-stop screened_candidate` changes the common task stop:
after each completed real observation, confirmed candidates are screened in the
existing relevance/stable order, at most four, using perceived target/full robot/
gripper/payload planning and12mm development clearance. If a preview passes, active
sensing stops. Confirmation alone cannot trigger handoff; rejected preview can
continue sensing within the remaining budget. Interface failures remain failures.
At the final stop a missing confirmed/screened candidate is not manufactured.

The shared screen is a prediction, **not D_exec**. BUNKER still navigates/stops;
D435 obtains fresh near-field geometry; actual-arrival collision-aware planning,
contact closure, physical lift and retention must succeed. No preview trajectory
is blindly replayed after movement/refinement. Twelve mm is a development allowance
based on prior geometry/execution discrepancies, not a calibrated safety guarantee.

The integrated-pose velocity mode is explicitly enabled only for this SIM backend;
native ODE feedback and physical contact/motion diagnostics remain recorded.
Ground-truth model/link diagnostics and the independent checker are measurement
only and never feed target detection, belief, ranking or execution selection.

## Outputs and compatibility

`task.json` records current config, scene setup, method and actual Git commits.
`entry.json`/`launcher.log` retain launcher exit/error even if SIM fails before an
attempt starts. Repeated execution-screen duration sums its actual intervals;
the active duration is the elapsed span including interleaved compute/hover time.
`attempt/attempt.json` holds result/failure; `attempt/data/events.jsonl` holds task
states, `metrics.json` simulation-time stages/paths, and rounds contain raw
observation/field/decisions. Independent physical checks are in
`attempt/physical_summary.json`. Bag/native diagnostics remain local under existing
ignore rules; missing path samples stay missing, not interpolated into efficiency.

Historical `run_a5_sim.py`, `run_a6_attempt.py` and configs retain legacy stopping
by default. Old paired/pilot/formal-development records are not recomputed or mixed
with the new stopping behavior. The worker's `decision.json` preserves the NBV
proposal; `A5_DECISION`/`A6_ACTIVE_STOP` events record the actual screened task stop.

## Known boundaries

Static horizontal ground, fixed-height occlusion surrogate, finite grid and
endpoint-opportunity visibility remain approximations. An ideal visible ground
cell may receive no actual LiDAR endpoint. Whole-footprint repeated support is
required; genuine unknown remains blocking for confirmation. Failing in the
three-window budget is an explained task failure, not an instruction to add votes
or automatically rerun. Arbitrary target search, calibrated real-world performance
and statistically established task-weighting advantage are not claimed.
