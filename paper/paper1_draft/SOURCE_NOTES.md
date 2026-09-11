# Author source notes — not manuscript text

This is a concise writing aid, not a new experiment/audit framework. Paths are
relative to the AGENT repository unless otherwise stated. Source inspection
describes the fixed evaluated implementation; it does not modify or rerun it.

## Claims and definitions

| Draft claim / equation | Existing source | Interpretation limit |
|---|---|---|
| Primary 80/96 vs 56/96; 53/27/3/13; exact p; 25pp and conservative CI | `paper/paper1_evaluation/table1_overall.csv`, `table2_paired_outcomes.csv`, `table3_mcnemar.csv`, `statistics.json` | One aggregate paired test; CI is for risk difference |
| 96 scenes, 192 primary/212 total planned, actual runtime commits | `configs/paper1_eval.json`, `paper/paper1_evaluation/reproducibility.json`, `docs/PAPER1_FIXED_EVALUATION_RESULTS.md` | No old pilots or interrupted formal slots |
| 38/28/30 tiers, 35/31, 26/22, 19/3 successes | `paper/paper1_evaluation/table5_difficulty.csv` | Descriptive heterogeneity, not three tests |
| 17/1 Hard discordances, Generic-only failures, invalid sensitivity | `docs/PAPER1_FIXED_EVALUATION_RESULTS.md`, `outputs/paper1-final-eval-v1/pairs.csv`, `paper/paper1_evaluation/statistics.json` | Keep contrary cases and all denominators |
| First-terminal failures, 40/16 totals | `paper/paper1_evaluation/table4_failure_stages.csv`, `failure_stage_mapping.csv` | Display aggregation; no result reclassification |
| Progress 89/63 confirmed, 88/63 screened/navigation, 82/59 D_exec, 80/56 retrieval | `docs/PAPER1_FIXED_EVALUATION_RESULTS.md` physical-progress table, source `analysis-latest.json` | Reached/completed is not guaranteed success; different exposed denominators |
| Conditional 53 pairs; window and time intervals | `paper/paper1_evaluation/table6_conditional_efficiency.csv`, `statistics.json` | Conditional, descriptive; no time/distance claim |
| Fixed/RM4D-only/ablations | `paper/paper1_evaluation/table7_auxiliary.csv`, `docs/PAPER1_FORMAL_EXPERIMENT_PLAN.md` section 2 | Reused scheduled counterparts; small subsets |
| Hard-015 19/96 vs 0/88 support deficit, window/confirmation histories | `paper/paper1_evaluation/qualitative_examples.json`, `figure5_caption.md` | Post-hoc frozen illustration; different per-run anchors |
| Candidate gate, margin score, stable ties | `src/reachability_guided_aerial_perception/field.py`, `model.py`; `src/task_relevant_uncertainty/anchors.py` | Partial 256-candidate validation, no center displacement or smoothing |
| Endpoint evidence, N and unknown score | `src/environment_belief/core.py` | Deficit heuristic, not probability or calibrated entropy |
| Target/environment/ambiguous blocker and actual ground presence | `src/operational_gating/core.py`, `association.py`, `subcell.py` | v1.4; no clearing history or fabricated votes |
| Continuous footprint / closed grid-cell overlap / clipped semantics | `src/task_relevant_uncertainty/geometry.py`, `core.py` | Clipped poses cannot confirm; no support is not infeasible |
| Finite-window scheduled-phase opportunity | `src/reachability_guided_nbv/finite_scan.py`, `operational_occlusion.py` | Nominal pose, surrogate heights, not guaranteed returns |
| Gains, displacement/yaw cost, stable ordering | `src/reachability_guided_nbv/core.py`, `model.py` | Generic/Ours share all except gain weighting |
| Shared windows, preview, exact handoff, 12mm clearance | `configs/current_sim_task.json`, `docs/PAPER1_FORMAL_EXPERIMENT_PLAN.md`, `scripts/run_a5_sim.py` | Current executed semantics, not old A5 stopping prose |
| SIM localization/contact assumptions and retention boundary | `docs/SIM_TO_REAL_READINESS.md` with concrete SIM source references | Sensor-fed task decisions do not imply real sensor-only global localization |
| 0.10m minimum TCP/object lift and one-second wall-clock confirmation hold | SIM `scripts/check_air_ground_pick_demo.py:427`, `:368`, `:386`; `src/demos/air_ground_pick_demo/config/demo.yaml:85`, `:91`; `scripts/run_air_ground_pick_demo.py` under that demo, `_hold_grasp_confirmation`; AGENT `scripts/run_a6_attempt.py:613` checker invocation | Existing success conditions, not a new hold experiment or simulation-time efficiency measure |

All seven tables are inserted from their existing TeX exports; Figures 1–5
are inserted from existing PDFs. No original export is changed. Equations are
a notation-level description of the current code, not a new mathematical model.
Raw data are not required to compile the supplied existing figures.

## Literature checked for the initial related-work draft

Primary records below were checked on 2026-09-12. Paraphrases are limited to the
specific reported scope; no reproduced results or broader comparative claims
are inferred from them. This is a focused initial bibliography, not an exhaustive
search establishing novelty against every prior method.

- `bircher2016nbv`: [ETH publication record](https://www.research-collection.ethz.ch/entities/publication/ae2aefd8-1e86-4601-bb36-fa175be18481)
  and [authors' planner repository](https://github.com/ethz-asl/nbvplanner).
  Receding-horizon exploration by newly observable space. Not our executed comparator.
- `border2018see`: [author paper record](https://arxiv.org/abs/1802.08617).
  Density/surface-boundary NBV, authors and ICRA 2018 DOI verified.
- `gualtieri2017viewpoint`: [author preprint v3](https://arxiv.org/abs/1609.05247v3).
  Viewpoint and grasp-detection precision; bibliography explicitly uses revised preprint.
- `breyer2022target`: [author preprint](https://arxiv.org/abs/2207.10543).
  Target-occlusion-driven views, updated grasps and execution/exploration decision;
  cited as the verified preprint rather than inventing final venue/page details.
- `li2023colag`: [author preprint](https://arxiv.org/abs/2310.13324).
  UAV sensing/waypoints supporting UGV navigation; not manipulator-stance weighting.
- `lenz2020wall`: [accepted SSRR paper record](https://arxiv.org/abs/2011.01999).
  Autonomous UGV/UAV brick-handling systems; do not assert that it uses our coupling.
- `zacharias2007capability`: [DLR institutional record](https://elib.dlr.de/51226/).
  Capability/workspace representation; citation does not assert unverified algorithm details.
- `rudorfer2025rm4d`: [author project and ICRA 2025 statement](https://mrudorfer.github.io/rm4d/),
  [author preprint](https://arxiv.org/abs/2410.06968).
  Existing forward/inverse map reused here, not a contribution of this manuscript.

The bibliography is intentionally honest about preprint versions. Final
publication metadata may be refined during author/venue editing without
changing experiment claims.
