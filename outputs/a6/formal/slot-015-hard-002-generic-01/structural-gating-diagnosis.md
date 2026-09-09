# Slot15: scene-local structural handoff lock

SECONDARY_DIAGNOSTIC_ONLY. Scientific-stop finding, not a new trial or outcome
reclassification. This diagnosis uses only the saved runtime-perceived target,
initial A1 catalog, observation NPZs, A2/v1.1 arrays, rankings and decisions.

In each of the three windows, all 34 representative poses are blocked:
31 intersect the continuous expanded target and all 34 also overlap at least
one accumulated AMBIGUOUS cell. None has an ENVIRONMENT-cell blocker. Of the
exact candidates, 30 intersect the expanded target, three more are blocked
only by AMBIGUOUS evidence, and source624 is exact-clear. The 321 nominally
supported cells have zero positive M_operational cells and zero U_task mass.
These are overlapping cause counts, not mutually exclusive classes.

## Decisive representative/exact discrepancy

Candidate `candidate-000000`, source624, A1 row15/col24, retains yaw -pi:

| Quantity | Exact candidate | Cell-center representative |
|---|---:|---:|
| map x (m) | 2.3700811356 | 2.3690650995 |
| map y (m) | -0.5478329364 | -0.5072827596 |
| Covered cells | 88 | 99 |
| Continuous expanded-target collision | false | true |
| ENVIRONMENT cells | 0 | 0 |
| AMBIGUOUS cells | 0 | 3 |
| TARGET cells | 0 | 2 |
| Final real ground-supported cells | 87 | 94 |

The unchanged sensor/pixel formula reproduces allowance
0.04645728611061248 m. Even with this allowance and the frozen padded base
footprint, the exact rectangle has a 19.418404 mm separating map-y gap from
the perceived target. Snapping its base to the representative moves y toward
the target by 40.550177 mm and creates 21.131773 mm map-y projection overlap;
the actual continuous SAT test confirms collision. This is not an exact
target collision mislabeled as a rescue, and not merely raw TARGET-cell alias.
Representative AMBIGUOUS cells are779,780,781; TARGET coexists in779,780.

The final missing exact ground cell is real: source624 is **not confirmed**.
However, obtaining further ground evidence cannot remove its fixed
representative collision. No hypothetical ground votes were generated here.

## Persistent ambiguous boundary evidence

The other continuously target-separated exact candidates are sources543,664
and581, each blocked by cell781. This cell also blocks source624's
representative. Its three first-window occupied returns are inside the
expanded perceived box but fail positive association: one is outside the
red mask with missing depth, one is on the eroded-mask boundary despite
agreeing depth, and one is inside the eroded mask with missing depth.
They correctly retain their original AMBIGUOUS labels; no GT identity is
inferred. Windows2 and3 add no occupied endpoints in this cell, but its
accumulated AMBIGUOUS vote remains1.

The catalog, target and initial reference are fixed; the operational replay
only accumulates occupied-class votes. Thus further allowed sensing cannot
unblock this catalog. This establishes a scene-local sensing-irreparable
v1.1 handoff lock, not proof that every scene fails or that the exact pose
would physically navigate and grasp successfully. It cannot be described
solely as a three-window coverage shortfall or proven absence of all valid
physical candidates.

## Reproduction and policy boundary

[The one-activation reconstruction](structural-gating-reconstruction.py)
reproduces 102 exact assessments,102 representative assessments,36 A3 arrays,
15 raw A2 arrays,12 operational vote arrays,three shared scoring snapshots
and all three actual Generic policy actions. All checks pass. Generic's
selected IDs are1,0,1 and the recorded third-round stop remains
`VIEW_BUDGET_REACHED`. Shared Ours rankings have `NO_PREDICTED_TASK_GAIN` in
all snapshots; this is saved-state evidence, not a newly executed Ours trial.

Full values, source624 geometry, cell781 point/pixel provenance and the exact
reproduction command are in [the JSON report](structural-gating-diagnostic.json).
[The geometry figure](structural-gating-geometry.png) shows the same saved
perceived geometry, not GT. Original attempt, metrics and outcome files
remain unchanged. No algorithm, thresholds, budget, seed or success rule was
modified, and no bag or simulator was used for this diagnostic.
