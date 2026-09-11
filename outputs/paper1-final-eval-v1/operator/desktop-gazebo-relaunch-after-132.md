# Desktop crash-notifier Gazebo relaunch

At 2026-09-11 13:42:33 Asia/Shanghai, an unplanned bare Gazebo process was
launched in `update-notifier-crash.service`, PID 642410 / process group 642408.
Its command referenced the old slot 003 Gazebo log path; its cwd was /home/lu.
The exact desktop/user action triggering the crash-notifier relaunch is unknown.
It was not an evaluation entry, scene setup or a rerun initiated by the operator.
No complete old task was rerun, and no new slot 003 result was created.

Slot 132's actual Gazebo PID was 640315 and its public wrapper exited normally
after physical CHECKS_PASS. The unexpected process used default Gazebo port
11345 with no ROS_MASTER_URI or GAZEBO_MASTER_URI override. Frozen formal runs
use ROS 11951 and Gazebo 11952 (run_a6_attempt.py). It was therefore a separate
server, not the source of formal sensor/control topics. The existing reducer
reported no protocol issues, clock-reset issue or missing binary result.
No evidence supports invalidating completed slots 131-replacement or 132.

The post-pair no-Gazebo guard correctly stopped before compression/deletion or
slot 133. Scoped SIGINT was sent only to the verified old process group 642408.
The cleanup did not delete user files or historical results, or change application
services or runtime settings. The original physical results stay valid and are not rerun.

Record this as one additional external bare-simulator start, separate from the
133 public formal task activations (including the single invalid original).
Conservatively count it against the same total ceiling of 224 starts. It is not
an extra scene/method observation and supplies no outcome to the primary analysis.
The operator's remaining-start check includes this +1; the frozen task manifest,
12 invalid-replacement reserve and all robot parameters remain unchanged.

Resume only the interrupted offline post-pair retention, then original slot 133.
