# A5 status transport for a connected external checker

In `natural-map-goal-SikHiu`, publisher logs show real PREFLIGHT, ARMING,
COMMAND_CONTROL and TAKEOFF at 17:49:24.079–.091. The checker transport connected
only at .111 despite earlier ROS master registration. Latching retained only
TAKEOFF; this is a transport race, not missing execution transitions.

Before any A5 event, replace the inherited one-slot status publisher with the
same topic/type, latch enabled and a queue of 10. Add an opt-in
`--wait-for-status-subscriber` startup flag: wait at most two wall seconds for an
actual transport connection before calling `run`, or fail before any flight.
Normal operation does not require a subscriber. During E2E validation the
unchanged physical checker is the sole status subscriber at startup.

Publish each real transition once. Do not replay or synthesize events, alter the
physical checker, persist event history or introduce lifecycle infrastructure.
Unit-test opt-in/default behavior, queue capacity, ready/timeout/shutdown paths;
verify the actual fresh E2E sequence through the existing checker.
