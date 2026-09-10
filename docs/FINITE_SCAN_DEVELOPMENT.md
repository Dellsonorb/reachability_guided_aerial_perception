# Finite-window acquisition development

2026-09-10. Development only; historical results are unchanged. Starting AGENT
`1b01750`, SIM `a0ae8e3`. Work remains in the user's primary AGENT checkout on
`feature/dev-finite-scan`, leaving the prior branch/PR intact.

## Question and bounded online plan

Can the shared viewpoint acquisition model better describe actual finite MID360
ground-endpoint support, and does the latest complete sensor-to-lift task work?
No guarantee of Hard completion or Ours superiority is sought.

At most **six online starts**, including infrastructure failures. Before any new
method result, choose this order:

1. Natural / Ours: current task profile and latest TF fix, before acquisition changes.
2. Existing paired-hard-01 / Generic: new shared acquisition model after offline review.
3. Existing paired-hard-01 / Ours: identical common code/config to start 2.
4. Existing paired-moderate-01 / Ours: new shared acquisition model.
5. Existing paired-moderate-01 / Generic: identical common code/config to start 4.
6. Reserve for one explicit runtime defect diagnosis/regression, or unused.

The scene definitions/seeds come unchanged from `configs/dev_multiscene_paired.json`.
These are development scenes, never an independent final test set. Old Hard02
records provide additional offline prediction diagnostics. If a pair cannot be
completed on one version, retain both results and mark the pair non-comparable;
do not spend an unbounded number of starts to recover it. Every failure stays.

## Investigation and design choices

First reproduce finite scanning coverage from known scan directions, public TF
and saved runtime points, separating planned level-hover pose error, unknown scan
phase, cell-boundary aliasing and observed foreground interception. Compare:

- current ideal center-FOV/prism visibility;
- finite-window, phase-marginalized ground-cell hit opportunity;
- a simpler empirical angular-density approximation, only if it preserves the
  measured gaps and offers a useful complexity reduction.

Prediction never enters A2 or operational evidence as a vote. Real ground support,
target/environment geometry, 12 mm development execution clearance, physical
grasp/lift, flight cost and three-window budget remain unchanged in this batch.
Generic/Ours share candidate generation, acquisition and execution; only existing
gain weighting differs. Do not add a completion objective before checking whether
the shared model/candidate set actually offers the missing support opportunities.

The detailed design and implementation plan will record the chosen representation,
assumptions and tests after this initial diagnosis, under the user's autonomous
local-design authorization. Do not start or resume formal experiments.
