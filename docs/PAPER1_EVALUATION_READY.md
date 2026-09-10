# Paper 1: one-time evaluation preparation

2026-09-10. **Preparation only; 0/212 tasks started. Formal execution still
requires one explicit user resource/execution authorization.** No final scene
was flown, sensed, queried through RM4D or ranked by either method in this turn.

## Frozen scenes and order

Complete independent geometry and ordered tasks:
[configs/paper1_eval.json](../configs/paper1_eval.json).
The one-time [historical exclusion list](../configs/paper1_eval_excluded_seeds.json)
contains 145 numeric scene seeds from all existing config/output JSON, including
ignored development records and reserved old formal seeds. Only seed numbers
were extracted, not outcomes. Natural fixed geometry is not reused.

| Scope | Frozen count |
|---|---:|
| Distinct scenes | 96: 38 Easy, 28 Moderate, 30 Hard |
| Generic / Ours | 96 / 96; 192 primary tasks |
| RM4D-only / Fixed | 6 / 6; 12 descriptive context tasks |
| No-occlusion / No-cost | 4 / 4; 8 descriptive ablation tasks |
| Planned total | 212 |
| Additional infrastructure-invalid replacements | At most 12; at most one per original slot |
| Total starts | At most 224, including failed starts |

IID tier RNG 2026091021, seed RNG 2026091022 and order RNG 2026091023 are independent.
All 96 seed values are unique and disjoint from the 145 historical values. First
six scene draws and wall perturbations follow the approved protocol exactly.
There was no redrawing for scene difficulty, score, candidate count or outcome.
Within each tier the primary first-method order is balanced: 19/19, 14/14, 15/15.
Each Generic/Ours pair is consecutive; auxiliary methods follow that pair. The
first 2 generated IDs per tier determine controls/full-image retention; first 4
Hard IDs determine ablations. Twenty-eight tasks have predeclared successful
RGB-D retention, irrespective of method outcome.

The [setup-only description](../outputs/preparation/paper1-eval/setup-description.json)
checks all 96 nominal camera cuboids against the public optical transform,
installed color FOV and unchanged 4.0 m observer gate. Overall optical-depth
extrema are 3.2320–3.7699 m. All pass these nominal checks; no pair among wall,
target and padded BUNKER spawn XY rectangles overlaps. This is **not** full
robot IK/planning admission, visibility success, a candidate count or a guarantee
of live sensing. Existing nominal robot models/initial joints are unchanged;
normal live occlusion, hover, support or execution failures remain outcomes.

## Narrow integration delta, unchanged robot method

Algorithm/platform reference remains `paper1-eval-finite-scan-v1`:
AGENT reference `6c9fd2a` (same executable decision code as the tested batch),
SIM `a0ae8e3`, original RM4D `e9d4312` plus the existing task-domain asset.
The preparation branch only adds manifest/accounting tools and extends the
existing launcher/collector with:

- `status=FROZEN_FOR_EVALUATION`;
- `cohort=paper1-eval-finite-scan-v1-final`;
- inner `kind=EVALUATION_ATTEMPT` and outer `kind=EVALUATION_ENTRY`;
- recorded actual AGENT/SIM/RM4D commits and the evaluated-version reference.

The evaluated `src/`, task profile, asset, all control/planning settings and SIM
source/install remain unchanged. The new mode requires the same four flags:
integrated SIM velocity, full robot manipulation, 12 mm execution clearance and
native dynamics diagnostics. Other cohorts retain their old behavior. The old
560-slot `FROZEN_FOR_FORMAL` configuration cannot be promoted through this entry.
Ground replay, manual setup-view override and ODE contrasts are not final tasks.

The same public wrapper still performs busy-port checks, writes entry/config
records, propagates interrupts to owned-child cleanup, and returns the physical
result. Only explicit slot selection/labels differ. Shared numerical adapter
arguments and diagnostic topics are equal to the original current profile.
Static sensor-mount metadata is for **offline location accounting only**; robot
decisions continue to use public runtime TF. No final-scene GT enters them.

## Commands — no automatic matrix loop

Read-only preview of one predeclared command:

```bash
python3 scripts/paper1_eval_design.py preview --slot 1

python3 scripts/run_retrieval.py evaluation \
  --config configs/paper1_eval.json --slot 1 \
  --output-dir outputs/paper1-final-eval-v1/slot-001-eval-hard-015-generic \
  --dry-run
```

After the separate explicit authorization, remove `--dry-run` to run that one
slot. No method/scene/initial-pose override exists on this evaluation entry.
Use a new output directory for every authorized attempt; do not rerun a valid
failure. A documented invalid replacement uses the **same original slot** and
a separately named directory, never edits the original. The manifest's
`execution_authorization` text records the preparation-time permission state;
it is not a newly invented approval service or an automatic launch switch.

Status and offline analysis:

```bash
python3 scripts/run_retrieval.py status \
  outputs/paper1-final-eval-v1/slot-001-eval-hard-015-generic

PYTHONPATH=scripts OPENBLAS_NUM_THREADS=1 \
  /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python \
  scripts/analyze_paper1_eval.py --config configs/paper1_eval.json \
  --results-dir outputs/paper1-final-eval-v1 \
  --output outputs/preparation/paper1-eval/empty-cohort-analysis.json
```

The latter was exercised on the **unstarted** cohort and on synthetic/development
fixtures, not on final method data. It reports INCOMPLETE and no final inference.
The existing old formal CLI/analyzer is not used to choose the new cohort.

## Accounting and interpretation

Only the predeclared paired **physical retrieval success** supplies the primary
test. The analyzer uses aggregate IID paired McNemar and the approved two 97.5%
Clopper–Pearson bounds, not historical equal-tier weighting. Missing pairs retain
full-denominator best/worst effect bounds; incomplete or unresolved cohorts
receive no final p-value. Auxiliaries remain descriptive. Nonsignificance is not
similarity, and the ±5 pp similarity criterion is not enlarged after results.

Counts separate capture attempts/accepted windows, nonzero-offset commands and
packet-measured observation-location changes (translation > .20 m, yaw > .20 rad).
The first accepted packet transform and known sensor mount define each observed
pose; partial path sums are never full distances. Active elapsed time includes
shared preview, Ground time starts with actual navigation, and post-terminal
cleanup is excluded. Missing metrics remain missing. Joint-success efficiency
is explicitly conditional, accompanied by full-denominator success-resource
curves so quick failure is not an efficiency win.

Known physical failures stay failures. Wrong-cohort/scene/config/flag records,
duplicate valid attempts, unknown primary outcomes and unresolved entry-only
starts are reported, not silently repaired or pooled. A named invalid original
may have only one replacement and all starts consume the cap. Actual-version
differences are surfaced; selected valid tasks must share the evaluated runtime.
An already documented INVALID original retains its configuration/version defects
in `invalid_attempt_diagnostics`; those excluded defects do not invalidate a
correct permitted replacement. Unknown or mismatched entry-only starts still
consume the launch allowance and remain unresolved, never invented failures.
No statistic changes a task decision or schedules another trial.

## Offline verification performed

- 68 manifest/setup/public-entry/attempt/metrics tests and 81 analysis/collector/
  legacy-statistics/figure tests pass in the existing interpreters (149 total).
- Independent specification and code-quality reviews pass. Review corrections
  were limited to offline invalid/start accounting and missing timestamps.
- The frozen manifest and setup-only report regenerate byte-for-byte in a
  temporary directory; all 212 command previews use the four common flags.
- Public-entry `--dry-run` creates no task directory or process. The
  [empty-cohort analysis](../outputs/preparation/paper1-eval/empty-cohort-analysis.json)
  has zero activations, 212 NOT_RUN slots, 96 incomplete pairs, no primary
  inference and missing-outcome risk-difference bounds `[-1,1]`.
- The current task profile, evaluation-reference JSON, `src/` and assets have
  no changes from AGENT `6c9fd2a`. SIM is clean at `a0ae8e3`; original RM4D is
  clean at `e9d4312`; both historical evaluation tags remain unchanged.
- No online setup check, method observation, final task or historical cleanup
  was performed. The four pre-existing untracked A5 diagnostic directories
  remain untouched and are not part of this preparation commit.

The analyzer was also exercised read-only on twelve retained development tasks:
33 accepted windows, 14 measured translation changes, zero yaw-only changes and
14 nonzero-offset commands were recoverable. The failed task's missing Ground
duration remains null. These are accounting regression inputs, not final data.

## Exact resource authorization requested

The following are **proposed ceilings awaiting the user's one authorization**,
not a claim that this turn already has permission to start:

| Resource | Meaning |
|---|---|
| Expected duration | **24–36 wall hours** at development throughput; estimate, not an entitlement or stopping rule |
| Hard elapsed deadline | **120 wall hours from the first formal launch**, including startup, cleanup, diagnostics and between-task waiting; not 120 simulation hours or 120 extra hours after the estimate |
| Task count | 212 listed tasks; at most 12 clearly justified infrastructure-invalid replacements; **224 total starts**; no valid-failure retries |
| Storage | **≤500 GiB new total campaign disk footprint**, including temporary bags/CSV/compression output and diagnostics; **≥100 GiB free** on the existing data disk |
| Hardware | Existing single workstation; one Gazebo task at a time; no added compute/storage purchase |

Check remaining time/space before a task; do not start one without room for its
existing startup/task/cleanup guards. Pause before exceeding any ceiling; never
delete unresolved data, extend the deadline, substitute seeds or stop because
one method appears ahead. Missing tasks at a ceiling mean an incomplete study,
not a smaller unannounced completed sample. The 500 GiB limit is stricter than
counting only compressed retained files; ordinary post-pair lossless CSV
compression happens with no simulator running. Preserve all historical outputs
and follow the already specified prospective RGB-D retention rule only for new
resolved tasks. No old files were removed during preparation.

Only one remaining user decision is needed: authorize execution of this exact
cohort on those resource ceilings. Until then remain at **prepared, not started**.
