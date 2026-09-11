# Figure and table captions

**Figure 1 — System pipeline.** A sensor-derived grasp target is queried against
the unchanged RM4D baseline using the separate task-domain asset. Validated
manipulation support and MID360 endpoint evidence jointly define footprint-aware
task deficit. Under a shared finite observation budget the system either selects
a screened exact base candidate or requests a further viewpoint. The shared
Ground pipeline performs navigation/stopping, fresh D435 refinement, collision-aware
manipulation and physical lift/retention. Schematic only, not a reconstruction of
one trial. Public simulation localization/contact backends and independent physical
scoring are distinguished from task decisions in the readiness audit.

**Figure 2 — Generic versus manipulation-aware NBV.** Both policies evaluate the
same ordered candidate set and finite-scan observation opportunity `w(v,x)`, with
the same occlusion model, cost, updating, observation budget and execution rules.
Generic weights observation deficit globally; Ours weights it by operational
manipulation support `M_op`. Only finite validated support contributes task mass;
unassessed capability is not asserted infeasible. `alpha=1-exp(-1/tau)`. These
gains are observation-reduction surrogates, not calibrated expected entropy.
After their decisions diverge, actual later observations can differ.

**Figure 3 — Retrieval success by difficulty.** Physical retrieval outcomes on
38 Easy,28 Moderate and30 Hard paired independent scenes from the prospectively
specified mixture. Counts above bars expose the denominators. Tier results are
descriptive; the sole primary paired inference is across all96 scenes, not three
additional significance tests. The overall paired difference is +25.00pp,
conservative at-least-95% interval[+8.56,+39.15]pp, exact McNemar p=8.43e-6.

**Figure 4 — First-terminal failure distribution.** Primary Generic/Ours failures
are displayed in eight task checkpoints, totaling40 and16 respectively. Sensing,
hover/capture and early TF acceptance are observation failures; budget exhaustion
without a confirmed candidate is assigned to confirmation. No-screenable-candidate
is a separate checkpoint; descend/close and lift/retention are combined as shown.
Every original label/reason remains in `failure_stage_mapping.csv`. Downstream
not-reached stages do not add failures. Counts use all96 scenes per method and
do not compare failure rates among unequal exposed stage denominators.

**Figure 5 — Recorded Hard-scene observation difference.** Hard015 (slots1/2), the
first scheduled eligible Hard pair with Generic failure/Ours success and retained
windows, is selected post hoc for illustration. Actual packet poses, ground-presence
votes and task deficit show different closed-loop observation states. The two
continuous anchors are per-run runtime candidates, not an assumed identical pose.
Ground support is insufficient in19/96 shown Generic anchor cells, versus0/88 for
Ours; confirmed histories are0/0/0 and0/0/3. Both acquire three accepted windows;
Generic fails without confirmation, whereas Ours passes the shared execution
screen and eventually the physical retrieval checks. This is neither a camera
frame nor an additional statistical sample. See `figure5_caption.md` and
`qualitative_examples.json` for source/stamp/NaN/occupancy details.

**Tables 1–3.** Prespecified primary physical retrieval outcomes, complete paired
table, and the single two-sided exact McNemar test. The interval is for the
paired risk difference (Ours−Generic), obtained from the specified simultaneous
Clopper–Pearson bounds; there is no “confidence interval of the p-value.”

**Table 4.** Presentation aggregation of original first-terminal causes, as in
Figure4. Zero counts are retained. Status and primary outcomes are unchanged.

**Table 5.** Descriptive tier counts and differences; no post-stratification or
outcome-driven reweighting of the realized38/28/30 mixture.

**Table 6.** Paired conditional efficiency on53 joint successes, with10000
whole-scene bootstrap resamples and the predeclared seed. These are descriptive
estimates, not an unconditional cost effect. Time and relocation intervals include
zero. Missing distances are not lower-bound substitutes or evidence of savings.

**Table 7.** Prescheduled context controls and ablations reuse the original Ours
counterpart outcomes. N6/N4 subsets are descriptive, not powered equivalence or
universal component-necessity experiments.
