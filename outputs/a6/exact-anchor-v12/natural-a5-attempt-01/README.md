# Pre-activation startup failure (not a method outcome)

The first v1.2 natural-regression launch did not enter `adapter.run()` and did
not command takeoff, observe, select a Ground pose, or attempt retrieval.
`adapter.log` records the same-topic ROS publisher-registration race and fatal
`A5 status subscriber did not connect before startup` after two seconds.

The existing passive checker was stopped with SIGINT and wrote its generic
`timed out waiting for LIFT` summary. That summary is not an execution failure:
the task never activated. The owned runtime was shut down normally. This
directory is retained as a pre-activation invalid startup, not a formal sample
or a reclassification of any of the 16 retained v1.1 outcomes.

Minimal orchestration repair: the optional standalone A5 diagnostic subscriber
wait now uses the same 10 s wall-time allowance already used by A6. Tests cover
reconnection after 3 s and a bounded 10 s failure. It occurs before task timing
and robot actions; no sensor, controller, navigation, manipulation or method
parameter changed. A fresh natural-a5-attempt-02 is used for the rerun.
