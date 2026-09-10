# New finite-scan task observations and exact-support audit

Snapshot: 2026-09-10. Production is frozen at `6cb5f0c`. This analysis reads launch02–05 records only; it does not use simulator ground-truth geometry, run simulation, change selections, or add votes. Launch02 is **Hard01 Generic**, not the historical Hard02 scene.

## Same-state acquisition checks

The [acquisition JSON](new_task_acquisition_05.json) and [compact report](new_task_acquisition_05.md) verify ten actual observation-window presence increments and reproduce all six saved selected-view opportunity arrays exactly. Actual ground presence comes only from original retained endpoints mapped using their recorded transforms and the unchanged ground/range filters. Each comparison holds the pre-observation state, requested pose, and next real observation fixed.

Here FP means nominal positive opportunity without an actual ground hit; FN means an actual ground hit outside the nominal positive-opportunity mask. These are binary mismatch counts, not calibration of phase fractions as return probabilities. The scope is the exact, geometrically viable winner-footprint union in the observed state.

| Scene / method / observed window | Actual / union cells | Old center-FOV + raw prisms FP/FN | Finite + raw prisms FP/FN | Saved finite + operational geometry FP/FN |
|---|---:|---:|---:|---:|
| Hard01 Generic / 2 | 137/174 | 0/29 | 0/23 | 0/23 |
| Hard01 Ours / 2 | 164/166 | 0/26 | 0/19 | 0/19 |
| Hard01 Ours / 3 | 166/166 | 0/5 | 0/1 | 0/0 |
| Moderate01 Ours / 2 | 159/159 | 0/3 | 0/2 | 0/0 |
| Moderate01 Generic / 2 | 161/162 | 1/3 | 1/2 | 1/1 |
| Moderate01 Generic / 3 | 159/162 | 3/3 | 3/2 | 3/0 |

The improvement is not universal perfect prediction. Saved finite all-grid FP/FN are respectively 10/162, 7/213, 0/189, 2/142, 41/112, and 22/123. In Moderate Generic window2, all-grid FP increases from old center 38 to finite operational 41, while FN decreases from159 to112. Known-height surrogates, nominal pose, partial history, and actual finite return sampling remain limitations. These are small development trajectories, not independent formal trials or evidence of calibrated probabilities.

Hard comparison plots include the viable-footprint-union outline and exact mismatch cells:

- [Hard01 Generic, window2](new_task_acquisition_05_launch-02-hard01-generic_window2.png)
- [Hard01 Ours, window2](new_task_acquisition_05_launch-03-hard01-ours_window2.png)
- [Hard01 Ours, window3](new_task_acquisition_05_launch-03-hard01-ours_window3.png)

## Nominal versus measured pose and phase

Maximum requested-versus-recorded body-position error is 0.0886–0.1418m over the six selected-view windows. All retained rays match their inferred CSV packet phases within 3.3e-6 degrees. Hard Generic window2 and Hard Ours window2 contain51 consecutive packets; the other four selected-view windows contain52. This audit uses their actual phase count and measured packet transforms; the production forecast remains the frozen nominal51-packet phase average.

The nominal-phase and moving-phase scan-only union masks agree on all four Hard/Moderate-Ours comparisons. They differ meaningfully for Moderate Generic:

- Window2: the nominal requested pose intersects all162 union cells, but measured packet transforms omit exactly cell506, matching the sole cell without actual ground presence. It already had one real presence vote; exact sources582,542,501 therefore each remain one cell short after window2.
- Window3: measured packet transforms omit exactly542,548,701, matching the three union cells without new ground hits; all already have two votes. Actual new observations close the previously missing support cells, yielding six confirmed winners.

The moving scan-only mask has FP/FN0/0 on this union in both Moderate Generic windows. This isolates those nominal-positive misses to realized-pose/scan geometry in these records; it does not imply moving scan-only is an occlusion-aware predictor or that future motion/phase is known. No inferred ray is counted as observed ground.

## Exact winners, blockers, and non-winners

The [exact-support JSON](new_task_exact_support_04.json) and [report](new_task_exact_support_04.md) reproduce all ten saved exact-anchor and candidate-assessment snapshots. `replay_exact_support.nonwinner_diagnostics` is reused without reselection. Blocking here means observed/perceived operational geometry, not simulator truth or full manipulation feasibility. Ground support is an independent count of real observation-window presence.

| Latest decision | Exact winners | Geometry blocked | Clear but missing ground | Confirmed | Blocked-winner cells with clear non-winner | Such non-winners with sufficient ground |
|---|---:|---:|---:|---:|---:|---:|
| Hard01 Generic, round2 | 36 | 30 | 6 | 0 | 1 | 0/2 |
| Hard01 Ours, round3 | 36 | 31 | 2 | 3 | 1 | 3/3 |
| Moderate01 Ours, round2 | 35 | 30 | 3 | 2 | 1 | 0/1 |
| Moderate01 Generic, round3 | 34 | 28 | 0 | 6 | 0 | 0/0 |

Every blocked winner in these four final states intersects the existing perceived expanded target rectangle. Of them, respectively29,29,28,27 also intersect retained ambiguous-endpoint disks. None is blocked by an environment full cell or incomplete ambiguous-history fallback. A coarse ambiguous cell alone is not misreported as continuous obstruction.

Source543 in each Hard task and source539 in Moderate Ours are the diagnostic blocked-winner/clear-non-winner cases. Their displaced exact winners are blocked solely by perceived target overlap; the alternatives remain unselected and unexecuted. Hard Generic's two clear alternatives still miss95 and96 support cells. Hard Ours' three alternatives have sufficient ground, but this is not another structural deadlock: the frozen policy already has three confirmed winners and selected/executed source623. Moderate Ours' one alternative still misses15 support cells; it already has two confirmed frozen winners.

Hard Generic's six clear winners miss83,79,95,96,92,90 cells respectively and then encounter a capture-anchor flight-bounds failure. No third observation occurred, so it is excluded from acquisition FP/FN. Hard Ours' remaining clear sources584,624 each miss one cell; Moderate Ours' clear sources501,500,579 miss1,2,17. These are missing real repeated ground observations, not geometry blockers.

## Preservation and status

All original task records and prior analysis snapshots remain intact. `new_task_exact_support_01` used `event` instead of the recorded `state` key solely for the task-end display; `_02` onward corrects that display. This is not a method-result or numerical-diagnostic revision.

Launch05's three sensing decisions are complete in this snapshot, but its physical execution is still pending in these analysis files; the parent owns the later physical outcome. No claim about another candidate's executability follows from this audit. No runtime files, configuration, or Git commits were changed by this follow-up.
