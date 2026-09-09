# Target-frame startup diagnosis

The failed process rejected an old local TF-buffer result immediately after MoveIt startup while the recorded TF stream continued publishing fresh transforms. The original `launch-01-natural-latest` remains a `VALID_TRIAL` with reason `target planning-frame transform is stale`; this offline investigation does not change its outcome.

## Recorded evidence

`native_tf_evidence.json` is the output of `analyze_recorded_tf.py`, which reads only `/tf` and `/tf_static` from the preserved diagnostics bag into installed native `tf2_ros.Buffer` instances. It does not initialize a ROS node, contact a master, replay ROS topics, start Gazebo, or apply a planning scene.

- Ground stopped at sim 154.669 (status log emitted at 154.671).
- MoveIt began loading the robot model at 154.678 and reported ready at 155.952.
- The demo reported the stale-transform error at 155.978 and FAILED at 155.984.
- The static `map -> ground/odom` transform was recorded at 11.414.
- The dynamic `ground/odom -> ground/base_link` stream contains 150 samples from sim 154 to 157, with maximum stamp gap 0.021 s.
- Native lookup of `ground/base_link <- map` immediately before failure gives stamp 155.961, age 0.017 s, below the unchanged 0.50 s freshness limit.
- A separate native buffer with its dynamic input frozen at initialization time retains stamp 154.661 and is 1.317 s old at failure. This demonstrates the delayed-callback mechanism; the failed process did not log its internal TF stamp, so that exact cached stamp is a model, not a measured process value.

The original ROS log is `launch-01-natural-latest/ros/14aec120-ac4d-11f1-98f0-252022cc4023/rosout.log`, particularly lines 286–287, 307 and 312–313.

## Installed implementation evidence

`/opt/ros/noetic/lib/python3/dist-packages/tf2_ros/buffer.py` calls `can_transform` before `lookup_transform_core`. With `Time(0)`, an existing stale transform satisfies availability immediately. Its nonzero timeout polls using ROS time and `rospy.sleep`; it does not enforce freshness and can hang if simulation time stops with a missing transform.

`/opt/ros/noetic/lib/python3/dist-packages/tf2_ros/transform_listener.py` inserts transforms from Python subscriber callbacks. `moveit_commander/move_group.py` directly constructs the native `MoveGroupInterface` wrapper. Disassembly of installed MoveIt 1.1.16 `_moveit_move_group_interface.so` shows the wrapper constructor at `0x64370` calls the native `MoveGroupInterface` constructor at `0x64574` without calling `PyEval_SaveThread` in that constructor. The Boost.Python value holder also directly calls this wrapper at `0x647cf`. This supports the startup callback-starvation explanation. The bag establishes fresh upstream publication independently of that explanation.

## Focused change

Only the target-frame lookup block in `_update_manipulation_target` changes. It performs nonblocking native TF lookups and yields via wall sleep until the existing freshness check passes, shutdown occurs, or the existing 0.5 s lookup budget expires on a monotonic wall clock. Each sleep is at most 0.05 s and no longer than the remaining budget. Missing, stale, and out-of-range future transforms cannot reach scene application. Accepted pose, dimensions, scene collision rules, freshness limits, and target geometry are unchanged.

The focused test uses native ROS messages, native TF buffering, native pose transformation, and the real perceived-target scene diff. External MoveIt transports and clock progression are controlled offline. The regression inserts an actual newer transform into the native buffer during the wait; it does not restamp a stale transform.

## Verification

After correcting the initial scene fixture to include its required finger links, the new test suite produced six intended failures before the production change (stale startup, missing arrival, missing timeout, still-stale updates, frozen-time stale timeout, shutdown). After the fix:

- `test_manipulation_target_tf.py`: 9 passed, no skips.
- `test_full_robot_manipulation.py`: 20 passed, no skips. Its existing target-TF method was updated to expect a nonblocking lookup and an advancing wall-clock fake for stale rejection; no other method changed.
- `test_air_ground_pick_demo.py`: 32 passed, no skips.
- `git diff --check`: passed.

Commands from the SIM worktree:

```bash
source install/p450-clean/setup.bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover -s tests -p test_manipulation_target_tf.py -v
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover -s tests -p test_full_robot_manipulation.py -v
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover -s tests -p test_air_ground_pick_demo.py -v
git diff --check
```

Diagnostic reproduction from the AGENT repository:

```bash
source /opt/ros/noetic/setup.bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 outputs/development/ground-clearance-batch/offline-tf/analyze_recorded_tf.py outputs/development/ground-clearance-batch/launch-01-natural-latest/diagnostics.bag
```

No online validation, build, or commit was performed by this task. The tests verify the focused recovery and rejection behavior; they do not establish a successful natural Ground execution.
