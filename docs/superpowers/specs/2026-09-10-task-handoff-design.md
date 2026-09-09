# Reusable task entry and screened Ground handoff

User authorization: autonomous development toward reusable sensor-driven aerial/ground retrieval, not preservation of historical module implementation details. Historical results remain immutable by convention, not a new framework.

## Evidence and choice

The twelve-task development batch has eight successful robot executions but one incomplete independent verification. README describes an obsolete failure state; successful configurations require multiple scattered opt-ins. Five tasks already confirmed candidates in window two, but all twelve continued through window three because uncertainty gain remained positive. Hard still lacks real repeated ground support. These are different problems.

Options considered: only document existing behavior; replace A4 with a new footprint-completion objective now; or first make the existing task executable/reusable and stop when its actual prerequisites pass, while diagnosing Hard's attainable coverage. Choose the third: it directly removes wasted sensing and a reproduced checker defect without inventing a new score from four failures.

## Design

- SIM checker retains received air/ground pose stamp/frame records for its bounded invocation; do not discard early handoff observations due to message count. Preserve all original freshness, sequence, contact, physical lift, controller and landing checks. Add a test reproducing more than 5000 callbacks. Existing failed results are not changed.
- AGENT adds a named current-development task profile/entry, reusing the current runner, public flight/MoveIt interfaces, v1.4 ground presence, exact winners, 12 mm clearance and explicit integrated SIM feedback. The entry runs one scene/method, not a matrix or automatic retry. Setup geometry belongs to SIM setup only; runtime algorithms still see sensor messages, never seed/target GT poses. Print command/status/output and preserve failures.
- Explicit new common `screened_candidate` stopping mode: after each real observation, if a confirmed exact candidate passes the existing bounded whole-manipulation preview, end active sensing and hand off. Confirmed alone is insufficient. A planning rejection before the budget limit permits the next existing observation; interface exceptions propagate. At a normal final stop, no accepted candidate remains a failure. Arrival/refinement still replans/checks actual execution. Legacy default stopping is retained for historical reproduction. Generic/Ours share the new stopping and screen; weighting/candidates/visibility/cost/budget/physical criteria remain unchanged.
- Offline Hard diagnosis asks whether any current geometric winner can reach two-vote ground support under the optimistic visibility model in the remaining one/two windows. Report idealized opportunity separately from measured support; no fabricated mapper update, no GT, no unseen-scan claim.

## Bounded validation

Maximum four online starts, including startup failures: existing Moderate01 Generic then Ours; existing Hard02 Ours then Generic. Deliberate development regressions, not fresh independent evidence. Natural current P450 sensing and autonomous selection; no archived pose replay. No replacement or additional start to force success. Preserve raw outputs and report stages and sim resources. The observed application can succeed or expose a remaining defect; do not label development complete solely from tests or a PASS.

## Boundaries and review

No RM4D baseline changes, no gate or collision relaxation, no vote fabrication, no budget increase. No formal matrix or inferential comparison. Independent code review checks common behavior, exception handling, physical criteria and task-entry version/config consistency. Review the online behavior and remaining limits before declaring the overall goal met.

## Runtime correction after start 2 (before starts 3/4)

Start 1 Moderate01 Generic physically retrieved after two windows. Start 2 Ours
failed before sensing: its own map/UAV TF subscriber could not connect, although
the setup parent's buffer had become ready. Preserve both records unchanged.

The localization TCPROS listener randomly reused port 33701. A minimal listener
on that exact port reproduced an immediate connection from the unrelated old
Gazebo process PID 1201814, via host address 100.65.37.138, sending
`ff 00 00 00 00 00 00 00 01 7f`. ROS interprets the first four bytes as header
length 255 and waits for the remainder; its serial handshake server cannot
accept waiting TF subscribers until the sender closes, about 30 seconds later.
Three such delays are recorded. Six ordinary ROS-only pub/sub nodes did not
reproduce the ten-byte input. The unrelated Gazebo was not stopped.

Configure this existing **single-host** runner's ROS_IP/ROS_HOSTNAME to
127.0.0.1 and ROS_IPV6 to off, including the setup parent. This is local transport
configuration, not a public frame change or distributed/real-robot setting.
Binding the same port through ROS's get_bind_address then accepted a local
connection and rejected the old Gazebo LAN route (connect_ex 111). It does not
promise isolation from another process intentionally using loopback.

Also require this task's own fresh public UAV/Ground TF before arming, using the
existing 30 s preflight timeout, zero-timeout lookups and wall-time yielding.
Missing/stale TF fails before takeoff; freshness and flight-health gates remain.
Tests reproduce old takeoff-before-TF behavior and cover delayed/frozen-clock
failure and stale-to-fresh recovery.

Use the remaining two starts for Moderate01 Generic then Ours at one corrected
version. This explicitly replaces the planned Hard02 online pair to validate
the demonstrated runtime defect within the unchanged four-start cap, not to
select favorable scenes. Hard analysis remains recorded-state only this stage.
Do not pool starts 1/2 with starts 3/4 as a same-version comparison.
