# A3/A5 v1.2 exact-pose support anchoring

Status: user-authorized design, minimal implementation and regression only.
No slot17, new pilot, formal restart, threshold tuning or multi-support method.
The16 v1.1 formal outcomes remain interrupted-development evidence, excluded
from any eventual v1.2 final statistics; their original files are not rewritten.

## Decision and scope

For every A1 HIGH/LOW cell C, choose the original gate-valid evaluated candidate
with maximum frozen `candidate_relevance`. At equal relevance, retain the
first candidate in the original `evaluated_candidates` sequence. This already
matches A1 and A5's existing tie convention, including saturated quality ties.
Do not sort by ID, margin above saturation, travel, residual, environment or
later outcome. Recover exactly one winner, never a less-blocked alternative.

`q*_C = first argmax_{i in C, g_i=1} v_i`, `R(C)=v(q*_C)`.
Use `Footprint(q*_C)` in both A3 projection and the corresponding A5 support
assessment. M_nominal/M_operational keep their max-relevance formulas and
`U_task=unknown_score*M_operational`. A2/v1.1 evidence and continuous target
collision rules stay unchanged. A4 formulas, viewpoint generator, visibility,
cost, stopping and tie-breaking stay unchanged; changed A3 masks can change
gains and rankings. All five environment methods share the same anchor;
RM4D-only remains on its original top-one path.

Alternatives considered: storing winners by modifying A1 artifacts is not
needed because frozen evaluated candidates are retained; environment-aware
winner replacement or multi-support changes the authorized method and is not
implemented. Deterministic reconstruction is the smallest compatible revision.

## API and compatibility

- Add a small `task_relevant_uncertainty.anchors` module with frozen
  `WinnerAnchor(source_id,candidate_id,evaluation_index,x,y,yaw,relevance)` and
  `reconstruct_winner_anchors(field,evaluated_candidates)`.
- Reconstruction must cover exactly A1 HIGH/LOW cells. Check finite pose,
  cell membership, reconstructed relevance and yaw against A1. Missing/mismatched
  input raises a clear ValueError, not a cell-center fallback. Numeric comparison
  tolerance is only for roundoff, not a physical or association allowance.
- `build_task_uncertainty(...,evaluated_candidates=None)` retains explicit
  historical cell-center behavior when omitted; supplied evaluations select
  exact-winner anchoring, even when the valid set is empty. Existing PoseSupport
  fields stay compatible; xy/yaw now describe the selected exact anchor.
- Task metadata records `anchor_semantics=exact-validated-winner-v1.2` and the
  winner IDs/evaluation indices. Legacy output fields stay unchanged when the
  option is omitted, allowing exact historical numerical replay.
- `A5Config.support_anchor` and `--support-anchor` accept `cell_center` (legacy
  default) or `exact_winner` (v1.2). v1.2 invocation also explicitly uses
  `--operational-gating v1.1`; association semantics are not renamed v1.2.
- One A5 helper builds the task for deciding and for saving/rendering, and the
  A6 worker uses that same helper. A5's existing catalog is reconstructed from
  the same winner function; confirmation still requires measured ground votes,
  unclipped footprint and an unblocked exact pose. No extra candidate fallback.

## Acceptance

1. Synthetic first-tie/saturation, field mismatch, cell membership and
   exact-vs-center geometry cases; all original target/environment/ambiguous
   gate tests remain valid. A1 and A2 input arrays must not mutate.
2. Replay Hard-002 saved states into a NEW output directory. For slot15,
   source624 remains continuously and operationally unblocked at its exact
   pose, M_operational can support sensing, but87/88 ground cells still means
   NOT confirmed. Do not generate a missing ground vote or force handoff.
3. Compare all saved raw A2 arrays and derived operational evidence against
   original data. Reproduce legacy mode before attributing any difference to
   anchoring. Report v1.1 versus v1.2 support/blocking/confirmation and A4 inputs.
4. Diagnostic only: count cells whose winner is exact-blocked but another
   original gate-valid non-winner is exact-unblocked, with IDs and causes.
   Do not replace the winner or call these non-winners physically executable.
5. Verify all five environment policies use identical anchors/A2/ground gate;
   Generic/Ours share viewpoint/visibility/cost/budget and differ only in gain
   weighting. Saved A4 arrays/ranking must match the actual exact-anchor input.
6. Run the original A5 natural Gazebo regression settings documented in
   `docs/OBJECT_AWARE_GATING_V11.md`, adding only `--support-anchor exact_winner`.
   Use a new outputs/a6/exact-anchor-v12 directory and existing passive physical
   checker. Preserve any failed regression and diagnose without parameter tuning.

After code review, tests, recorded-state replay, natural regression and
documentation, commit/push the A6 checkpoint, keep PR Draft, and STOP for review.
No final inference or new formal matrix is authorized by this revision.
