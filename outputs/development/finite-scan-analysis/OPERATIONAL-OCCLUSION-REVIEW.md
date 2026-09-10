# Independent review of operational scan occlusion

2026-09-10. Read-only review and replay of the shared prediction geometry. No
production files were edited by this review; no online run, belief update,
generated return, GT geometry, or changed historical result is involved.

## Review outcome

No remaining critical geometry issue was found after the incomplete-history
fallback correction. The cylinder implementation minimizes XY distance over the
part of each segment inside the vertical slab; an independent quadratic-root
calculation agrees on 12,000 random segments. The row-vector yaw transform for the
expanded target OBB is consistent with the existing target representation.
Thirty-seven finite-scan, operational-occlusion and integration tests pass.

One issue was reported during review and fixed by the parent: a raw occupied
count exceeding the sum of supplied class counts now preserves a full-cell
blocker. The prior check caught only zero accounted counts. The independent
partial-history reproduction now passes. This is a conservative guard; full
recorded contexts used in the replay retain their original geometry.

The model uses the existing perceived target dimensions and allowance, existing
33 mm ambiguous disks extruded to the unchanged one-metre assumed height, and
full-cell prisms for environment or incomplete/unaccounted evidence. It does not
reinterpret ambiguous endpoints as target points. Operational gating and source
votes remain separate and unchanged.

## Four recorded second-window comparisons

FP/FN below compare any nonzero predicted phase opportunity with actual ground
presence in the second-state union of viable exact footprints. A phase fraction
is not a calibrated return probability.

| Recorded run | Previous finite raw-prism FP/FN | Operational geometry FP/FN | Fraction-weighted missing actual ground |
|---|---:|---:|---:|
| 05 Hard01 Generic | 0 / 21 | 0 / 21 | 21.4000 |
| 06 Hard01 Ours | 0 / 9 | 0 / 5 | 5.6000 |
| 11 Hard02 Ours | 0 / 9 | 0 / 9 | 10.0875 |
| 12 Hard02 Generic | 0 / 10 | 0 / 10 | 11.5375 |

Hard01 Ours also improves over the full grid: 168 missed actual-ground cells
become 151, with zero false positives retained. The other three full-grid counts
are unchanged (FP/FN 13/151, 15/145, 13/136 respectively). This shows both a useful
correction and persistent model limitations.

For cell 781, Hard01 Ours prediction changes from 0 to 1 at the recorded second
viewpoint, and actual ground presence changes from one to two windows. Hard01
Generic's recorded second viewpoint still predicts zero there, and actual
presence stays one. A different hypothetical Generic viewpoint pair gains
geometric opportunity; that must not be described as a successful recorded
second-window prediction at the old viewpoint.

## Bounds and transition-aware frozen candidate audit

The audit applies the configured bounds [-4,4] x [-3,3] x [0.5,3], rejects
initial yaw-only candidates using the existing 0.05 m facade tolerance, and
also rejects yaw-only transitions between the two proposed views. The refined
200 generated views become 193 admissible candidates. Every surviving completing
pair is the same pose repeated, so its second step is a permitted rescan and
does not require a new lattice offset outside the configured move set.

| First frozen state | 65-view original lattice: jointly possible pairs | 193-view midpoint lattice: jointly possible pairs |
|---|---:|---:|
| Hard01 Generic | 1 | 1 |
| Hard01 Ours | 0 | 0 |
| Hard02 Ours | 0 | 1 |
| Hard02 Generic | 0 | 2 |

For the added Hard02 Ours repeated midpoint, 2,500/6,400 phase-start pairs meet
source 582's remaining footprint requirement, and 6,241/6,400 meet source 501's.
For Hard02 Generic's first repeated midpoint, counts are 3,456/6,400 for source
623 and 6,400/6,400 for source 541. Its other repeated midpoint gives 2,304/6,400
for source 541. The two cross-yaw pairs in the earlier unfiltered diagnostic are
excluded. Hard01 Generic's sole repeated pose has 6,400/6,400 for source 623.

These are combinatorial phase-pair counts under frozen geometry, not probabilities
assuming independent future timing. The audit neither selects a method's action
nor simulates updated beliefs. Hard01 Ours still lacks an ideal completing pair;
the shared change does not establish counterfactual success there.

## Cost and limits

Single-thread prediction takes 4.75–4.90 seconds for 65 candidates and
14.29–14.76 seconds for 193 candidates on the existing numerical environment.
The additional precise occluders are computationally material. These are offline
timings, not an assurance of stable online hover during computation.

The model still assumes nominal level hover and frozen known geometry. It omits
pose drift, packet loss, future unknown occluders and calibrated uncertainty in
real acquisition. Ground intersections remain prediction only. The unchanged
physical execution checks determine whether a later task succeeds.

Reproduction: `operational_occlusion_review.py --output <new-file-in-this-folder>`.
The exact result is `operational-occlusion-review.json`; it includes unchanged-
belief checks, geometry metadata, per-cell counts, exact pair poses, phase counts
and timings. Use the existing numerical Python environment with `PYTHONPATH`
including `src:tests:scripts`, one BLAS thread, and a writable Matplotlib cache.
