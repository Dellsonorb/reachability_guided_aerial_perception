# Second pre-activation startup failure

No task or robot action started. The same topic-registration race persisted
through the 10 s diagnostic connection wait. This is retained separately from
attempt 01; extending the wait alone was not a sufficient fix. The checker was
stopped, and the dedicated runtime shut down normally.

Local rospy source and master.log show the inherited publisher's last handle
being released, the topic removed, and the same URI registered again while an
already subscribed checker was connecting. The adapter now acquires its
replacement publisher before releasing the inherited handle, avoiding that
registration gap. Queue size 10, latch and status payloads are unchanged for
new connections. Existing connections retain their original queue size, so the
next checker starts after the adapter's readiness log and before adapter.run().
This is startup-only orchestration, not a policy or execution-parameter change.

The raw checker `FAIL: timed out waiting for LIFT` is expected after cancellation
of an unactivated task, not a retrieval failure. Neither startup is a formal
sample or a reclassification of any preserved v1.1 trial.
