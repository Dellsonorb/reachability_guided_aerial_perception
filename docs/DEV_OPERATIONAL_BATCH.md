# Bounded operational-consistency development batch

This is development, not validation of a final method or a formal matrix.
Historical v1.1/v1.2/v1.3 versions and results remain untouched. Starting
method: c7824ad (v1.3 endpoint-local AMBIGUOUS disks, exact-winner anchors).

## Budget and setup fixed before online outcomes

At most **12 online launches**, including startup failures: 2 known-problem
regressions, 6 launches for three Generic/Ours pairs, 4 reserved launches.
Reserves require a documented diagnosis, fix check, or invalid platform run;
they are not retries until success. Stop after this batch, never run slot 17
of the interrupted formal study. Unused reserves need not be spent.

The serialized config is `configs/dev_operational_batch.json`. First run
known Moderate seed 405111274 and Hard-002 seed 958985919 with Ours. Then run
Easy 1026751387 (Generic/Ours), Moderate 1026806174 (Ours/Generic), and Hard
1448047368 (Generic/Ours). These three seeds are the first unique eligible
draws of `Random(2026090903).randrange(1, 2**31)`, excluding every seed in the
four historical a6 configuration files (including reserved formal scenes).
Copy the existing six scene-parameter draws and tier obstacles unchanged.
No outcome, score, candidate count, viewpoint or mechanism-based filtering.
Known regressions are explicitly reused development cases, not fresh tests.

Common initial view remains map (-1.4, 0, 1.2, 0); 3 observation windows of
5 simulation seconds. Other common runtime/method settings start unchanged.
No GT enters runtime perception, gating or NBV. Scene metadata only configures
spawn and the necessary offline RGB-D setup check. Final test data stay separate.

## Execution and diagnosis

Reuse the existing one-attempt runner, raw observation saving, simulation-time
metrics and read-only replay. No new execution framework. Check after each
known regression whether local evidence/continuous geometry are compatible
with ground confirmation and predicted observation opportunities. Distinguish
actual collision, missing real ground support, evidence/grid aliasing,
prediction limitations, and downstream execution/runtime failure.

Generic/Ours share all components except their gain weighting. Any needed
consistency repair gets a failing regression first, a clear version note and
shared implementation. Do not pool different method versions within a pair;
use a reserved same-version rerun if justified and budget permits, otherwise
report the incomplete/mixed pair. No invented observations, obstacle removal,
result-driven parameter adjustment or relaxed physical grasp/collision checks.

Report raw grid blocking → operational blocking → confirmation → D_exec →
retrieval, T/E/A evidence, exact-winner/non-winner diagnostics, same-state
Generic/Ours scores, observation windows, UAV simulation distance/time and
failure reasons. Non-winners remain diagnostic unless a justified consistency
change is explicitly documented; no silent reselection.

## Launch ledger

No online launches at config freeze. Each attempt's unique directory under
`outputs/development/operational-batch/` records slot, seed, method, startup,
outcome and actual observations. The final report lists every launch, including
invalid ones and any reserves. This ledger is a development note, not a new
artifact/provenance system.
