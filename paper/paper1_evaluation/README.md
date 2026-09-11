# Paper 1 evaluation package

Publication assets from the **completed 212-task / 96-pair fixed evaluation**.
No algorithm change, simulation rerun, new test scene, or real-robot execution.
Source result checkpoint: `6fec29b`. Authoritative detailed results:
[PAPER1_FIXED_EVALUATION_RESULTS.md](../../docs/PAPER1_FIXED_EVALUATION_RESULTS.md).

## Ready assets

| Asset | Files | Interpretation |
|---|---|---|
| Overall retrieval | `table1_overall.csv/.tex` | Ours80/96, Generic56/96 |
| Paired outcomes | `table2_paired_outcomes.csv/.tex` | a/b/c/d=53/27/3/13 |
| Prespecified inference | `table3_mcnemar.csv/.tex` | b=27,c=3,p=8.43033e-6; paired risk difference +25pp, conservative95% CI[8.56,39.15]pp |
| First-terminal taxonomy | `table4_failure_stages.csv/.tex` | Generic40, Ours16; eight task-stage bins |
| Per-failure mapping | `failure_stage_mapping.csv/.tex` | Every original stage/reason retained alongside display bin |
| Difficulty | `table5_difficulty.csv/.tex` | 38Easy/28Moderate/30Hard; descriptive, no extra primary tests |
| Conditional efficiency | `table6_conditional_efficiency.csv/.tex` | 53 joint-success pairs; descriptive bootstrap intervals |
| Context / ablations | `table7_auxiliary.csv/.tex` | Prescheduled subsets only; no equivalence or necessity claims |
| Exact manifest references | `scene_seeds.csv/.tex`, `task_order.csv/.tex` | All96 seeds and212 ordered tasks, not newly generated scenes |
| Machine-readable summaries | `statistics.json`, `reproducibility.json` | Includes effect interval, versions, sensitivity and caveats |
| Figure1 | `figure1_system_pipeline.{pdf,svg,png}` | Current task chain and observation loop, schematic |
| Figure2 | `figure2_method_comparison.{pdf,svg,png}` | Common components and sole NBV gain-weighting difference |
| Figure3 | `figure3_success_by_difficulty.{pdf,svg,png}` | Descriptive success counts/rates by tier |
| Figure4 | `figure4_failure_stages.{pdf,svg,png}` | Primary first-terminal failure counts |
| Figure5 | `figure5_qualitative.{pdf,svg,png}` | Recorded Hard015 sensor-derived fields, not a synthetic camera image |

[Captions](CAPTIONS.md), [long Figure5 source caption](figure5_caption.md),
[qualitative source references](qualitative_examples.json), and
[LaTeX include example](manuscript_assets.tex) are provided. Vector PDFs/SVGs
are the publication originals; PNGs are previews. Use double-column width for
the detailed pipeline/qualitative panels and adapt text size to the selected venue.
LaTeX snippets use only standard tabular/graphicx constructs. No TeX engine is
installed in this environment, so a complete venue manuscript has **not** been compiled.

## Statistical and stage semantics

The single exact McNemar test is scene-paired, not an independent proportions
test. The **confidence interval belongs to the paired retrieval risk difference**,
not the p-value: two97.5% Clopper–Pearson intervals for discordant probabilities,
combined by the predeclared Bonferroni difference. No new test/CI is selected
after looking at the results. Raw JSON retains full precision; CSV/TeX display
rounding does not alter the underlying result.

Figure4 aggregates original first-terminal failures for presentation, without
reclassifying any VALID/INVALID trial. Observation includes initial RGB-D/frame
acceptance, hover and MID360 capture. Budget exhaustion without a confirmed
exact candidate is assigned to the confirmation checkpoint; screen failure is
separate. Ground-refine includes near-camera visibility and its collision-aware
view motions. D_exec corresponds to refined pregrasp; descend/close are grasp;
lift and retention are combined. Not-reached stages do not create failures.
This explains the different display bins from the original raw `active` stage.
The full row-level mapping is exported. The current cohort has zero primary
navigation and D_exec-stage terminal failures; that is not a universal guarantee.

Only the jointly successful53 pairs support conditional observation/time
comparisons. Ours uses fewer windows descriptively; location-change and time
intervals cross zero. There is **no distance-saving claim**; incomplete paths
remain incomplete. Prespecified full-denominator success-resource curves remain
in [the original package](../../outputs/paper1-final-eval-v1/resource-curves.svg).
Do not characterize p>0.05 as equal performance, or failed quick tasks as efficient.

Figure5 is the lowest-scheduled eligible Hard discordant pair with retained
snapshots: slots1/2, seed1395226981. It was selected after outcomes, for illustration,
not to estimate a population effect. Reproduction is pinned to this pair: absent
raw files produce an explicit missing-data error, never a substitute scene under
the same caption. It uses each run's original map origin,
actual packet poses and continuous exact anchor; the two anchors are not assumed
identical. Dotted lines are observation order, **not** reconstructed flight paths.
Ground-presence votes are not occupancy-free clearance. The scene itself is not
reconstructed from Gazebo truth, and no extra observations were generated.

## Reproduce the assets, not new robot runs

On the current workstation, from the AGENT root:

```bash
bash paper/paper1_evaluation/regenerate.bash
```

This only runs the two new offline exporters. Set `PAPER1_PYTHON` to a compatible
existing numerical Python elsewhere; NumPy/SciPy/Matplotlib are required. The
observed environment versions are in `reproducibility.json`. No dependency
installation, controller startup, device access or scene activation is performed.
The output stays under this directory; original results are opened read-only.

For a compact Git-only clone lacking large sensing files, reproduce tables and
Figures1–4 separately:

```bash
PYTHONPATH=src:scripts python3 scripts/build_paper1_evaluation_package.py
```

Figure5 needs retained `observation_*.npz`, per-round fields/operational evidence
and packet transforms for the selected pair. These large raw files are **local,
not in the compact Git results**. Missing local SIM checkout/tag probes are
reported as null metadata; they do not block the statistical tables. The frozen
analysis and compact physical/attempt records allow independent primary-outcome
recalculation, but a fresh clone alone is not a full raw-sensor reproduction package.

## Exact version distinctions and current replay limits

- AGENT release tag `paper1-eval-finite-scan-v1` resolves to
  `6c9fd2a5c3d8777c4c5a33bc85ba605af53a1166`.
- Its documented executable ancestry is `1a6e85b42e9d9fd642454de499672004503c3772`.
- Actual formal collection used **`3d13a95c3aac8e530aeb9d54f4ab75cc0b37b81a`**,
  including the authorized cohort/entry/accounting preparation. Use this revision
  plus the frozen manifest when describing the executed study, not just the tag.
- SIM tag and actual runtime: **`a0ae8e32889e92a86f24d2794c7cb143999915fd`**.
- Original RM4D: **`e9d431299053f38a4a4319aed3dfeccc261b9fac`** plus this AGENT's
  separate `assets/rm4d_ground_task_v1/{rmap.npy,metadata.json}`. Original positive-z
  literature asset alone is not the executed Ground-task asset.
- Scene geometry/order/seeds: `configs/paper1_eval.json`; common profile:
  `configs/current_sim_task.json`; installed environment description:
  `configs/evaluation_version.json`. Historical preparation status strings in
  the latter are not the final completion status.

Current-host offline numerical reproduction is verified. A clean-machine full
task rerun still needs the recorded SIM installed/overlay tree, correct graphics
environment, PX4/submodules, compatible ROS/Python stacks and task asset. Some
runtime paths are workstation-specific; baseline checkout under `/tmp` must be
recreated/persisted elsewhere. Do not substitute either repository's main or
reapply the stacked PR patches on top of their cumulative tip.
Same seeds do not guarantee bitwise identical asynchronous scan/controller or
physics outcomes. No212-task rerun or clean-machine execution was performed here.

## Manuscript gaps and next writing step

The requested five numbered figures and core statistical tables are supplied.
Remaining work is manuscript integration, not missing primary experiments:

1. Write the current methods section using [METHODS_SCOPE.md](METHODS_SCOPE.md).
   **Do not copy `docs/PAPER1_SIMULATION_METHODS.md`: it is the old560-slot/v1.1
   draft** with different anchoring, scan model, selection/stopping and statistics.
2. Finish related work, contribution framing and the full paper narrative;
   fit caption length/figure typography to the selected venue and compile its template.
3. If a camera-view/contact montage or video is desired, extract it separately
   from retained real simulation frames. Figure5 is an evidence-map comparison,
   not a visual proof of contact/lift; the existing physical outcome records supply that.
4. Before external data release, decide how to distribute retained raw data and
   dependencies.129 full successful image bags were previously removed under the
   approved policy and cannot be regenerated from four representative frames.
5. State idealized simulation localization/contact evidence and transfer limits.
   The completed study supports success in its defined population, not real-world
   accuracy, arbitrary terrain, longer retention or unseen object generalization.

The [sim-to-real audit](../../docs/SIM_TO_REAL_READINESS.md) proposes a future
stationary sensing/feedback stage. No real adapter or real execution is included
in this package. Keep formal results and future development data separate.
