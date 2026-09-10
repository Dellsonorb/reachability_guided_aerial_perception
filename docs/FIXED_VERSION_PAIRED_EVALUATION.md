# Fixed-version paired development evaluation

## Question and scope

Evaluate the complete current sensor-to-physical-retrieval program across six
new scenes, without further method development. This is descriptive development
evaluation, not an independent final test or a formal statistical claim.

Baseline: AGENT `54aa4c714e068288bd8f0b6d73c9576768706866` (runtime arrival/hover
fix at `66c20aa`); SIM `a0ae8e32889e92a86f24d2794c7cb143999915fd`; original
RM4D `e9d431299053f38a4a4319aed3dfeccc261b9fac` plus the existing independent
`assets/rm4d_ground_task_v1/rmap.npy`. The pre-run commit adds only this plan,
the scene manifest, an offline setup check and a path-adapted offline summary
script. Each task records actual commits.

No new method or runtime code is planned. Use `configs/current_sim_task.json`
unchanged: finite-scan model, exact-winner anchoring, v1.4 operational gate,
three five-second observation windows, initial view `[-1.4,0,1.2,0]`, 1 m
candidate spacing within the existing extent, cost weight 0.25. Keep 12 mm
development execution margin, SIM integrated-pose velocity mode with native
diagnostics, full robot/gripper/perceived object/payload collision checks, and
the existing physical grasp, lift and retention criteria. These margins are
not a calibrated safety guarantee.

## Scenes and order fixed before any method run

`configs/fixed_version_paired.json` contains all six explicit scenes and twelve
ordered slots. Seeds are the first unused six draws from
`random.Random(2026091013).randrange(1, 2**31)`, excluding 138 seed values in
existing config JSON and recorded attempt JSON. No outcomes were consulted.
The scene generator is exactly the preceding development distribution: six
per-seed uniform draws for target XY (+/-0.15 m around [2,0]), target yaw
(+/-pi/6), initial BUNKER XY (+/-0.1 m around [3,-2.5]) and yaw (+/-pi/36
around pi); target/base heights and Easy/Moderate/Hard obstacle templates
unchanged. No scene is redrawn for scores, visibility, candidates or success.

| Slots | Scene | Seed | Method order |
|---|---|---:|---|
| 1-2 | fixed-easy-01 | 1563189545 | Generic, Ours |
| 3-4 | fixed-moderate-01 | 1857094799 | Ours, Generic |
| 5-6 | fixed-hard-01 | 204914204 | Generic, Ours |
| 7-8 | fixed-easy-02 | 1921564428 | Ours, Generic |
| 9-10 | fixed-moderate-02 | 806884938 | Generic, Ours |
| 11-12 | fixed-hard-02 | 1339075492 | Ours, Generic |

The nominal optical-depth check is only a necessary setup check. It does not
promise live FOV, unobstructed perception, sensing coverage or retrieval.
Scene metadata initializes SIM; algorithms obtain target and environment from
fresh runtime sensors. No manual candidate or historical confirmed pose input.

## Execution and fairness

Run exactly the twelve slots using the unchanged `scripts/run_retrieval.py`
entry, one fresh SIM task per slot. Generic and Ours share the complete profile,
candidate generation, finite-scan and occlusion models, cost, budget, A2,
operational gate, Ground selection and execution. Only NBV gain weighting
differs. Read-only same-state score diagnostics check this decomposition.
Non-winner support remains diagnostic, never a reselection path.

Maximum online starts: 14 including all startup failures. Two reserve starts
are exclusively for a demonstrated runtime/recording defect or a targeted
diagnosis, not a failed method retry. Retain every original attempt. If code
must change, record the defect and affected scope; do not pool different
versions or compare mixed-version scene pairs. If the two-start reserve cannot
restore a same-version pair, mark that pair incomplete rather than exceed the
cap. Normal sensing, planning, execution and grasp failures are valid outcomes
and execution continues to the next planned slot. No retry-until-success.

## Recorded outcomes and interpretation

For each task record observation windows and actual moves; raw grid blocking,
operational blocking, TARGET/ENVIRONMENT/AMBIGUOUS evidence, exact-winner and
non-winner diagnostics; confirmation; autonomous selection and fallback;
navigation arrival; D435 refine; actual-arrival collision-aware pregrasp
(`D_exec`); grasp; physical lift/hold; first terminal failure and intervention.
Keep raw validity classification and distinguish a diagnosed platform defect
in the report without silently replacing its result.

Use simulation time for active and whole-task duration. Report measured UAV
distance only when its recorder establishes coverage; otherwise keep it null
and label any observed lower bound. Compare efficiency principally for paired
tasks that both complete; a quick failure is not efficient retrieval. Report
six paired binary outcomes and discordant pairs descriptively, not statistical
superiority or a formal sample-size recommendation. Inspect Hard failures for
predicted versus actual ground support without changing votes or budgets.

## Implementation and review plan

1. Verify relevant unchanged ROS/core regressions, SIM/RM4D versions and task
   ports; run the offline depth check; independently review the manifest and
   common profile. Commit the pre-run plan before starting slot 1.
2. Execute slots in frozen order, retaining status, raw records and failures.
   Monitor tasks without intervening in robot decisions. Diagnose only clear
   software/recording defects; do not add features during this batch.
3. Reuse `outputs/development/finite-scan-analysis/summarize_runs.py` with only
   batch paths, scene list and cap adapted in `fixed-version-analysis/` (not
   the old pilot summarizer, which excludes development attempts). The public
   task entry uses local slot 1 inside every task; global slots are the manifest
   and output-directory names. Use this compact summary and scoring diagnostics to produce
   a six-pair stage/failure/efficiency report, with missing measurements intact.
   Reconcile every output directory/entry (including entry-only startup failures
   and reserve starts) with the manifest: the compact summary only includes
   finished attempt records and is not the complete start ledger by itself.
4. Review findings against raw records; rerun relevant regressions, check that
   runtime code/config and dependencies stayed fixed, commit/push results on
   `feature/eval-fixed-finite-scan`, keep PR Draft, and stop. No formal matrix.

Example (change only frozen scene, method and fresh output directory per slot):

```bash
P450_GAZEBO_DISPLAY=:0 P450_GAZEBO_XAUTHORITY=/run/user/1000/gdm/Xauthority \
python3 scripts/run_retrieval.py run \
  --scene-file configs/fixed_version_paired.json --scene-id fixed-easy-01 \
  --method generic --output-dir outputs/development/fixed-version/slot-01-easy01-generic
```
