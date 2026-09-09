# v1.2 fresh-seed paired validation — prospective freeze

2026-09-09. The approved scope is **six method slots only**, one new scene in
each existing tier, paired Generic/Ours. This implements the accepted
[proposal](V12_FRESH_SEED_VALIDATION_PROPOSAL.md), not a formal study or a restart
of the interrupted v1.1 matrix. Method baseline is integration checkpoint
`4528758`; all research source and numerical settings remain unchanged.

## Fresh seeds and complete order, before any method activation

Use `random.Random(2026090901).randrange(1, 2**31)` in Easy/Moderate/Hard order,
taking the first unique eligible draws. Exclude the union of all scene seeds
in `configs/a6_pilot.json`, `configs/a6_pilot2.json`, and `configs/a6_formal.json`
(126 reserved seeds, including the unused old formal reservations). The ten
distinct seeds in historical attempt/setup metadata are all in that union.
No draw is evaluated for candidate count, scores, viewpoint choice, mechanisms,
retrieval outcome or method advantage. None of the three first draws collides
with that union; no redraw is needed. Before writing the new configuration,
the three numbers also have no occurrence in tracked configs/docs/outputs.

| Tier | Fresh scene seed | Method order | Slots |
| --- | ---: | --- | --- |
| Easy | 1249652395 | Generic, Ours | 1, 2 |
| Moderate | 405111274 | Ours, Generic | 3, 4 |
| Hard | 1450983937 | Generic, Ours | 5, 6 |

Order is fixed by shuffling `[generic, ours]` once with
`Random(2026090902)`, then alternating that pair order by tier index. First
positions differ by one because there are three pairs. No outcome changes order.

For each scene, use `Random(scene_seed)` and exactly the existing six uniform
draws: target x=`2+U(-.15,.15)`, y=`U(-.15,.15)`, yaw=`U(-pi/6,pi/6)`;
BUNKER x=`3+U(-.1,.1)`, y=`-2.5+U(-.1,.1)`, yaw=`pi+U(-pi/36,pi/36)`.
Copy tier boxes, all z values, brick/model properties and dynamics unchanged
from Pilot-2. Serialize all three scenes and six slots to
`configs/a6_v12_validation.json`, test, commit and push before method execution.

The natural A5 regressions use the nominal unperturbed scene and no explicit
scene-generation seed; no such nominal geometry is reused here. This seed
freeze does not claim control of otherwise unseeded Gazebo/scan/planner timing.
Hard-002, all earlier pilots and development regressions remain excluded.

## Unchanged method and runtime contract

Use shared `operational_gating=v1.1` and **explicit** `support_anchor=exact_winner`
for both methods. Per-cell ties retain the first original evaluation; no
environment-aware winner selection. A2 raw state/evidence and allowance formula,
operational blocking/ground votes, A4 scoring/visibility/cost and exact Ground
selection are unchanged. Non-winners are diagnostic only, never promoted.

Common sensing pose remains `(-1.4,0,1.2,0)`, launch `(-.5,0,.15,0)`; RGB-D
depth gate4.0m, three completed5s windows,20s capture wall guard,1200s task wall
guard, original hover conditions, navigation/manipulation settings and success
criteria all remain frozen. Inherit the precise window, stop, resource and
failure rules of `A6_PILOT_PROTOCOL.md`, with only the already-approved v1.1
operational gate and v1.2 exact anchor superseding its historical gate definition.
The approved MoveIt/TF readiness repair is shared by both methods.

Both methods use the same ordered viewpoint generator, visibility/occlusion,
flight cost, budget, A2 update, v1.2 support, Ground selection and execution.
Only generic unknown versus task-weighted observation gain differs. Stop uses
each method's own gain/score, no first-confirmed early stop, and one selected
Ground candidate with no fallback. `D_exec` requires arrival, fresh D435 refine
and actual collision-aware refined pregrasp execution. Retrieval additionally
requires the existing independent physical lift/retention checker.

The existing single-attempt launcher needs only optional support-anchor CLI
forwarding; old configurations keep their old defaults. This is parameter
plumbing, not a new method definition. Use the existing `FROZEN_FOR_PILOT`
launcher category with experiment label `V12_FRESH_PAIRED_VALIDATION`; never use
the historical formal config or invoke a formal matrix.

## Setup, execution and failure handling

Run three separate method-independent setup activations before the method
slots. Use the existing aerial observer, public TF/hover and5s raw MID360 check
(at least100 distinct ground cells in the original setup ROI). No RM4D, task
scores or candidate discovery participates. A genuine setup-domain conflict
requires review, not a replacement seed or pose. Ordinary proven invalid
startup can be diagnosed and repeated with the same scene.

Then run slots1–6 serially, each in a fresh SIM and separate output directory
under `outputs/a6/v12-fresh-validation`. Inspect each completed activation
before starting the next. Valid method/execution failures remain failures.
Only independently demonstrated protocol-defined INVALID_TRIAL may be repeated,
with the original record and same slot/seed/method retained. Resource gaps make
the affected metric unavailable, not a reason to erase a known primary outcome.
No result-driven parameter, scene, selection, budget or success change.

Minimal fixes for reproduced runtime/platform bugs are allowed, with tests,
affected-run accounting and fair shared versions. Never reclassify the old
16 v1.1 formal outcomes or include them in final v1.2 statistics.

## Required saved-state diagnostics and report

Per observation window, retain existing cloud/target reference, A2 raw arrays,
operational evidence, exact assessments, A3 support and same-state dual ranking.
Report:

- `raw grid blocking → object-aware blocking → confirmation → D_exec → retrieval`.
- TARGET/ENVIRONMENT/AMBIGUOUS cell counts and vote sums, association availability
  and computed allowance; mixed classes may overlap, unknown is not free.
- Exact winner identity/pose and consistency with saved A1 relevance/yaw/A3
  support; winner-blocked/non-winner-clear IDs and ground deficits, diagnostic only.
- Same-state Generic/Ours ordered candidates, visibility/cost and both gains;
  rankings are not hypothetical executed outcomes.
- Window count, first discovery, UAV/Ground path and durations using simulation
  time, missing-sample flags, stage outcomes and original failure taxonomy.
- Paired E2E outcomes and discordant pairs, descriptive only. No significance,
  formal power/sample-size inference or success-only efficiency superiority.

Reuse existing operational/scoring/resource/report scripts and shared numerical
functions. Native bags are passive diagnostic data, not controller inputs;
Gazebo truth remains confined to scene setup and independent outcome checking.
No new benchmark, evidence/security framework or method is part of validation.

## Mandatory scientific stop and final checkpoint

If a further structural sensing/handoff deadlock or required method-semantic
change is demonstrated, stop immediately for research review. Distinguish it
from finite-budget missing ground support and genuine geometry/execution
failures. Non-winner diagnostics cannot authorize a repair by reselection.

Otherwise complete all six slots, run relevant regressions and independent
review, commit/push the results, and stop at a **formal-readiness checkpoint**.
Keep PR#6 Draft during validation and at the checkpoint pending review; no
automatic merge, new pilot extension, formal sample-size selection or formal
run is authorized by this document.
