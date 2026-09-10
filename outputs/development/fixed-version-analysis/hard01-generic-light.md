# Fixed-version Hard01 Generic: lightweight completed-slot diagnosis

Scope: only `outputs/development/fixed-version/slot-05-hard01-generic`, AGENT `0de63c5`. The recorded task ended successfully after three observation windows and selected exact winner source542. This is a descriptive single-task diagnosis, not a Generic/Ours comparison or a method-advantage claim.

The parent requested deferral of CPU-heavy work while the remaining paired tasks run. This check used `nice -n15`, `OPENBLAS_NUM_THREADS=1`, and `OMP_NUM_THREADS=1`; numerical work took0.285s. It did not access ROS/Gazebo, read the live Ours slot, use GT geometry, change task records, or run finite-scan/packet-phase/ranking replay. The [JSON](hard01-generic-light.json) preserves exact cells and measured-pose ranges; [the narrow script](hard01-generic-light.py) reuses existing saved-state, endpoint-binning and comparison helpers.

## Actual presence and saved opportunity

All three saved `ground_presence_votes` increments reproduce exactly from original retained endpoints mapped using their recorded transforms and the existing range/ground-band conditions. The windows contain51,52,51 packets and span5.024,5.095,5.025sim seconds. They produce486,1060,1075 real ground-hit cells over the full grid; confirmation counts are0,0,1.

Each requested NBV matches one prior saved ranking candidate. Its saved positive opportunity mask equals the saved visibility mask. Full model equality replay is deferred, not claimed by this lightweight check.

| Observed window | Exact viable union | Actual ground hits | Saved finite FP/FN | Saved unoccluded FP/FN |
|---|---:|---:|---:|---:|
| 2 | 163 | 150 | 2/4 | 13/0 |
| 3 | 163 | 152 | 2/6 | 11/0 |

The scope is the geometrically viable exact-winner-footprint union of that observed state. FP denotes positive nominal phase opportunity without a real ground hit; FN denotes a real ground hit outside that positive mask. Phase opportunities are not calibrated return probabilities or generated votes.

All four window2 FN cells740,783,825,826 and all six window3 FN cells739,740,782,783,824,826 have positive saved unoccluded opportunity. Thus the saved occlusion factor—not absence of all nominal scan opportunity—removed these actual-hit cells from the final forecast. This is evidence of nominal occlusion conservatism relative to the recorded returns; attributing it specifically to assumed height/extent versus measured-pose differences requires the deferred ray replay.

The nominal-positive cells without actual ground are788,827 in window2 and428,548 in window3. Their stored phase opportunity is1. Actual requested-versus-recorded maximum body-position differences are0.166710m and0.120720m; vertical offsets span[-0.129417,+0.002975]m and[+0.026185,+0.103478]m. These establish pose discrepancy, not yet a causal attribution for individual missed cells. Packet-phase and moving-transform ray checks are deferred.

## Final exact winners

Of36 frozen exact winners,31 are geometry-blocked, five are clear, and one is confirmed. All31 blocked winners intersect the perceived expanded target rectangle;29 additionally intersect retained ambiguity disks. No final winner is blocked by an environment full cell or ambiguity-history fallback. These are observed/perceived operational blockers, not simulator ground truth.

| Clear source | Exact footprint cells | Still missing repeated ground | Confirmed |
|---|---:|---:|---|
| 624 | 88 | 1 (cell548) | no |
| 543 | 108 | 4 | no |
| 664 | 107 | 8 | no |
| 542 | 88 | 0 | yes |
| 581 | 107 | 3 | no |

Source542 has81 cells with two real presence votes and seven with three. The other clear candidates remain unconfirmed because of missing repeated ground observations, not geometry blockers. No alternative is selected by this analysis.

The existing diagnostic-only non-winner evaluator, exact model replay, and observed-phase/moving-pose analysis are explicitly deferred until the parent confirms the batch is finished. No zero non-winner count or individual TF-causality claim is inferred from their absence here.
