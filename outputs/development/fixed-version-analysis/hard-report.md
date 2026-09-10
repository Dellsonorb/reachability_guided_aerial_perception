# Fixed-version Hard tasks: offline support diagnosis

Only completed slots05,06,11,12 at AGENT `0de63c5` are analyzed. Original observations, choices and failures are unchanged. The existing acquisition/phase and exact-support helpers reproduce **12 actual endpoint-to-presence increments, eight saved selected-view opportunity arrays, and 12 exact-anchor/assessment snapshots**. No GT geometry, new sensing, reselection, runtime edits or simulator contact is used. Four single-thread, nice10 replays took18.93s total, after the parent declared all12 tasks complete.

## Why Hard02 Generic exhausted its budget

Slot12 has35 frozen exact winners:30 are blocked by observed/perceived operational geometry, but **five remain geometrically clear**. None obtains the required two real ground-presence votes over its whole footprint after three windows. This is not an all-candidate geometry deadlock or a lost vote-update problem.

| Clear exact source | Footprint cells | Missing after window2 | Missing after window3 |
|---|---:|---:|---:|
| 625 | 107 | 85 | 20 |
| 706 | 107 | 81 | 28 |
| 705 | 108 | 81 | 29 |
| 623 | 106 | 88 | 18 |
| 663 | 106 | 90 | 24 |

The closest, source623, finishes with71 cells observed in two windows,17 in three windows, and18 in only one window. Of those18 deficient cells:

- Fourteen received ground only in the initial window; neither later window revisited them with an accepted ground endpoint.
- Cell700 received ground only in window3.
- Cells507,584,707 each received ground in only one later window, despite nominal opportunity1 in both forecasts.

The first15 cells have zero saved occluded opportunity at both selected later poses. All18 have zero environment/ambiguity/target endpoint-class votes in the cells themselves: they fail repeated ground support, not the continuous footprint blocker. Source663 additionally contains four cells never observed as ground. Across the viable union,40 cells remain deficient:29 initial-only, six third-only, one second-only, four never observed. Thirty-four of40 had zero saved occluded opportunity in both later forecasts.

The later requested poses were approximately(-3.422,-2.034,1.302)m and(-3.542,-2.075,1.172)m. The third selection is **candidate0, the zero-offset/current viewpoint**, so it is a further observation window, not a second intended spatial relocation. A flight command induced by the facade/current-pose discrepancy must not be counted as a distinct planned sensing move. Additional ground hits were collected, but whole-footprint repetition remained incomplete; `VIEW_BUDGET_REACHED` is therefore the recorded outcome under the unchanged support requirement.

## Same-state forecasts versus actual observations

FP means positive nominal phase opportunity without a real ground hit; FN means a real ground hit outside that positive mask. These are mask mismatches, not calibration of opportunity as a return probability. Each row holds the pre-observation belief, requested viewpoint and next actual observation fixed; the scope is the observed state's viable exact-winner-footprint union.

| Task / observed window | Actual / union | Old center + raw prisms FP/FN | Finite + raw prisms FP/FN | Saved finite + operational FP/FN |
|---|---:|---:|---:|---:|
| Hard01 Generic / 2 | 150/163 | 1/6 | 2/4 | 2/4 |
| Hard01 Generic / 3 | 152/163 | 2/9 | 2/6 | 2/6 |
| Hard01 Ours / 2 | 150/151 | 0/3 | 0/2 | 0/0 |
| Hard01 Ours / 3 | 150/151 | 0/3 | 0/2 | 0/0 |
| Hard02 Ours / 2 | 169/169 | 0/11 | 0/8 | 0/5 |
| Hard02 Ours / 3 | 169/169 | 0/11 | 0/5 | 0/2 |
| Hard02 Generic / 2 | 126/169 | 7/12 | 7/8 | 7/8 |
| Hard02 Generic / 3 | 134/169 | 2/15 | 2/11 | 2/11 |

In Hard02 Generic, the saved model predicts positive opportunity in125/169 union cells in each later window, while actual ground hits reach126 and134. The false-negative cells have scan-only nominal opportunity but are removed by modeled occlusion. This demonstrates conservatism relative to actual returns, not that conservative geometry explains every missing real vote. Whole-grid FP/FN remains35/97 and27/109. See [window2 plot](hard_slot-12-hard02-generic_window2.png) and [window3 plot](hard_slot-12-hard02-generic_window3.png); the black outline is the viable union.

## Recorded packet and pose effects

All inferred retained-ray directions match their CSV packet phase within3.23e-6 degrees. Other Hard later windows contain51 or52 consecutive packets. **Hard02 Generic contains50 packets in each later window**, spans5.025s and5.023s, and has one observed phase-step2 gap in each; nominal production prediction remains51 packets. No dropped packet is imputed as a vote.

For Hard02 Generic, using the actual recorded phases at the nominal pose gives scan-only coverage of all169 union cells. Holding those same phases fixed and substituting measured per-packet transforms removes all seven final-forecast nominal-positive/no-ground cells in window2 and both in window3. The maximum requested-versus-recorded position differences are0.181838m and0.113185m. This supports a realized packet/TF-geometry explanation for those nominal-positive misses, without claiming drift alone caused the task failure or assuming an unrecorded51st packet would repair it.

The moving scan-only model still predicts hits without actual ground in35 cells in window2 and33 in window3. Thus much of the support deficit remains a no-ground-return/occlusion issue, not absence of every scan-ray opportunity. This analysis cannot determine unknown physical occlusion from GT and does not model all return physics.

One moving scan-only FN in window2, cell667, illustrates the exact-plane limitation: original retained row18314 has z=-0.000236938m and legitimately passes the unchanged ±0.02m ground band. Its recorded ray intersects the ideal z=0 plane in adjacent cell666. No return or grid rule is changed to eliminate this mismatch.

Other tasks also retain limitations. Hard01 Generic window3's two nominal-positive misses disappear under measured transforms; window2 cell827 remains scan-possible without real ground. Hard02 Ours' measured transforms recover three nominal scan-only missed cells in window2 and two in window3, while known-occlusion prediction still misses actual hits. None of these diagnostics gives the online planner future phase or pose knowledge.

## Final winners and diagnostic-only alternatives

| Task | Exact winners | Geometry blocked | Clear but unsupported | Confirmed |
|---|---:|---:|---:|---:|
| Hard01 Generic | 36 | 31 | 4 | 1 |
| Hard01 Ours | 35 | 31 | 1 | 3 |
| Hard02 Ours | 35 | 30 | 0 | 5 |
| Hard02 Generic | 35 | 30 | 5 | 0 |

Every blocked winner intersects the perceived expanded target rectangle; respectively29,31,29,29 also intersect retained ambiguity disks. None is blocked by an environment full cell or ambiguity-history fallback. These are operational observed/perceived predicates, not simulator truth or a manipulation-feasibility guarantee.

The existing `nonwinner_diagnostics` finds one blocked-winner/clear-non-winner case in each task. Hard01 source621's alternative candidate000082 is still missing four ground cells for Generic and one for Ours. Hard02 source666's alternative candidate000002 is fully ground-supported for Ours but still missing21 cells for Generic. No alternative was reselected or executed. The failing task therefore has no already-supported alternative uncovered by this diagnostic; Ours already has other confirmed frozen winners.

These paired trajectories have different requested poses and naturally different sensed states. The descriptive coverage difference does not isolate a causal weighting advantage or establish general reliability. The records support retaining the failure as a real repeated-support/budget limit, not relaxing support thresholds or adding method changes to force success.

## Artifacts and preservation

Full data: [Hard01 Generic](hard-slot-05-hard01-generic.json), [Hard01 Ours](hard-slot-06-hard01-ours.json), [Hard02 Ours](hard-slot-11-hard02-ours.json), [Hard02 Generic](hard-slot-12-hard02-generic.json). They include exact footprints, missing-cell histories, forecast values, nominal/moving phase checks, and every diagnostic alternative. Reproduction helper: [hard-offline.py](hard-offline.py). The earlier [lightweight report](hard01-generic-light.md) remains intact; the full replay now supplies its deferred checks.
