# v1.4 development: separate measured ground presence from blocking

Online Moderate launch 02 reproduced a representation contradiction, not a
missing observation: source705/source623 cell781 has ground endpoints inside
the exact footprint in all three windows (6/7/4 and 8/8/7), but ground_votes
stays zero because separated AMBIGUOUS endpoints share the coarse cell.
Their nearest distances are 67.9–94.4 mm, outside the unchanged 33 mm disks.
The original launch remains a v1.3 failure. Diagnosis uses runtime data only.

Choose a small separation of positive support evidence and negative collision
evidence, rather than candidate-specific relabeling, deletion, or a voxel
refactor. Add optional `ground_presence_votes` to the derived operational view:

    P(C) = sum_k 1[an accepted measured ground-height endpoint exists in C in k]

Ground/range/validity filters and one vote per observation/cell remain the same.
Record P BEFORE occupied-priority suppression. Preserve original ground_votes,
all occupied votes and A2 raw fields exactly. P is not FREE and never erases an
obstacle. Missing historical P is unavailable; old versions keep old semantics.

For v1.4, confirmation(q) requires P(C)>=2 for every covered cell, an unclipped
footprint, AND no operational blocker. Blocking is unchanged v1.3: continuous
expanded target, cell-conservative ENVIRONMENT, endpoint-disk AMBIGUOUS with
legacy fallback for missing sub-cell history. A true collision remains blocked
even with arbitrarily many ground returns. No ground endpoint means no P vote.

This retains the existing ground cell sampling approximation (presence in a
cell, not full sub-cell surface certification). It does not claim complete
terrain support/clearance. Source584 cell780, with no ground endpoint inside
the exact sliver, highlights that existing spatial approximation; report it,
do not pretend it was measured continuously. No radius, threshold or budget
changes. No non-winner reselection.

A3 and A5 use the same view/assess_footprint; Generic/Ours share it. A4's gain
formula and raw-A2 unknown score stay unchanged. This repair affects handoff
confirmation, not occupied association or nominal manipulation relevance.

Tests: paired mixed ground/AMB outside vs inside footprint, ENVIRONMENT and
true TARGET collision remain blocked, no-ground/one-window cannot confirm,
historical v1.3 arrays unchanged, filters/per-window counting, context IO and
CLI. Replay Moderate/Hard original observations then use reserved launch04 for
one Moderate online repair check, followed by the six scheduled paired runs
on one common v1.4 version. Up to two other reserves remain after the counted
wrapper rejection. Do not retry any genuine outcome until success.
