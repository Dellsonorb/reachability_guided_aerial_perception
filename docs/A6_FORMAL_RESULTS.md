# A6 formal simulation — execution record

## Prospective freeze, before any formal outcome

The [formal protocol](A6_FORMAL_PROTOCOL.md) defines 120 independent scenes,
40 per existing tier, and 560 valid method slots. Exact seeds/specifications
and balanced order are in `configs/a6_formal.json`. Pilot-1/2 remain separate.
At creation of this record: **0/560 method slots; 0/120 setup scenes**. No
formal outcome has informed the design or sample size.

Independent reviews cleared the seed/order/common-settings implementation,
paired analysis and stage-scoped native D435 logging. The review corrected the
analysis's input-identity checks and first-activation sensitivity wording before
execution. No A1–A5 method/asset or SIM source change is part of this freeze.

Verification before activation:

- Core: 496 tests, no failures, 21 environment-related skips.
- System Python/Noetic with its original PYTHONPATH retained: 74 tests pass,
  including TF-bag/OpenCV, launcher and paired-analysis integration cases.
- Frozen-baseline/task-map checks with the actual RM4D_ROOT: nine tests pass,
  including original positive-workspace and separate integration-asset geometry.
- Independent logging review: 44 core tests and 27 system-Python launcher
  tests pass. Default Pilot-2 command and adapter parameters are unchanged.
- Source diff against `56ece4a` is empty for research `src`, A5 runtime/target
  support, assets and Pilot-1 configuration/protocol. SIM worktree is clean.
- A first broad Noetic test command incorrectly included a Python-3.10-only
  scene-description test and overwrote the ROS PYTHONPATH. It was a test-command
  environment error, not a robot run or source defect. The corrected interpreter
  split above passes; no frozen source was changed to accommodate it.

Original online resource metrics remain primary, causal bag reconstructions
secondary. Full-trial control/TF plus handoff-to-end native D435 images project
about 680 GB total including setup/other outputs; current capacity is about
884 GB. This is an explicitly conditional storage estimate, not a worst-case
guarantee. The unadopted BZ2 probe and every pilot original are preserved.

## Running and reporting the fixed study

Use the existing P450 environment wrapper and owned ports 11951/11952. Set
`P450_PX4_ROOT` before the wrapper; set ROS/GAZEBO master environment variables
after it because the wrapper sanitizes inherited variables. Its runtime
dependencies are the unchanged local SIM and RM4D baseline/task-domain asset.
Never run two Gazebo instances concurrently.

For each frozen scene block, first run the existing launcher with
`--config configs/a6_formal.json --setup-scene SCENE_ID --output-dir NEW_DIR`.
Inspect `data/setup_summary.json`; do not use RM4D or method results to qualify
or replace a scene. Then run each prelisted slot with the same configuration,
`--slot NUMBER --output-dir NEW_DIR`. Keep all activations and classify terminal
measurements before considering an identical-slot INVALID rerun.

Generate progress summaries with `scripts/summarize_a6_pilot.py --config
configs/a6_formal.json --results-dir outputs/a6/formal --output NEW_SUMMARY`.
Despite its retained filename, this collector selects only FORMAL_ATTEMPT
records for the formal config; it never pools pilots or setup activations.
Use the existing scoring/operational/scene-description reporters uniformly.
The secondary bag resource reporter reads `diagnostics.bag`; it still contains
the complete original control/TF topic set.

The existing core interpreter has SciPy 1.15.2 for offline analysis. After all
planned valid slots complete, run `scripts/analyze_a6_formal.py --config
configs/a6_formal.json --summary FINAL_SUMMARY --output FINAL_ANALYSIS` there.
Before full completion it reports INCOMPLETE and no inferential p-value. The
primary exact-null interpretation, approximate versus conservative intervals,
secondary Holm family and missingness rules are fixed in the protocol.

No interim efficacy analysis, sample-size revision, scene replacement or method
tuning is permitted. Append actual progress, invalid history, stage/failure
tables and final figures as execution proceeds. Scientific stop conditions
remain those explicitly specified by the user.
