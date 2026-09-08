# Formal Simulation and Paired Analysis Implementation Plan

> **For agentic workers:** Use executing-plans for serial simulator ownership;
> use subagent-driven-development for the independent configuration/statistics
> tasks and requesting-code-review before the first formal activation. The user
> authorizes routine decisions and conditional formal execution, so do not pause
> for a routine plan approval. Keep the existing A6 feature branch.

**Goal:** Complete a prospectively specified, independent paired simulation
study and report its outcomes without tuning frozen research methods or scenes.

**Architecture:** Reuse the one-attempt launcher, saved-metric collector, frozen
A6 policies and v1.1 gate. Add only a serialized formal configuration, offline
paired analysis, and the minimal metadata support for more than three scenes.
Use native stage-scoped rosbag recording for bounded storage, not a new framework.

**Tech Stack:** Existing Python 3.10 core, Python 3.8/ROS Noetic, Gazebo, JSON,
NumPy/Matplotlib and the already installed SciPy for offline statistics only.
No new dependency installation, algorithm or runtime middleware.

## Task 1 — prospective specification (root)

- [x] Write `docs/A6_FORMAL_PROTOCOL.md` before any formal method outcome:
  120 independent scenes, 40 per tier, four methods per scene and two additional
  Hard ablations (560 valid slots), independent draw and balanced order; explicit
  primary null, effect/discordance planning assumptions, boundary-safe interval,
  secondary analyses, actual common platform limitations and missingness rules.
- [x] Retain the exact existing initialization, six-draw scene distribution,
  3-window/5 s acquisition, stopping, exact Ground selection, v1.1 allowance
  formula, success criterion and INVALID taxonomy by reference and direct tests.
- [x] Fix all seeds before execution. Qualify each scene with the unchanged
  method-independent setup check before its method block. No scene replacement,
  outcome-based reclassification or difficulty relabeling. Setup is not a trial.
- [x] Independently review statistical validity and practical storage before
  final freeze. Explicitly separate within-tier symmetry tested by exact
  McNemar from the weaker equal-tier-average risk-difference null.

## Task 2 — configuration and existing launcher (bounded agent)

Files: `scripts/a6_formal_design.py`, `configs/a6_formal.json`,
`tests/test_a6_formal_design.py`, `scripts/run_a6_attempt.py`,
`scripts/summarize_a6_pilot.py`, `scripts/a6_scene_description.py`.

- [x] RED: generated formal configuration must contain 120 unique non-pilot
  seeds, 560 contiguous slot IDs, four core methods per scene, Hard-only
  ablations, balanced core method positions within each tier, and identical
  shared runtime settings to Pilot-2. Use the first eligible draws from
  `random.Random(202609083)`, not an outcomes-based filter.
- [x] Implement pure `build_config(pilot1, pilot2)` using independent seed/order
  RNGs (202609083 / 202609084). Scene IDs are `easy-001`, `moderate-001`, etc.;
  `tier` retains the original tier. Preserve the original six random draws and
  exact tier boxes. Apply the generated JSON with apply_patch; no execution.
- [x] RED: launcher accepts `FROZEN_FOR_FORMAL` in addition to the unchanged
  pilot status, rejects unlisted slots, takes an explicit setup scene ID, and
  emits `FORMAL_ATTEMPT` only for formal configuration. Existing pilot records
  remain `PILOT_ATTEMPT`. Setup always remains METHOD_INDEPENDENT_SETUP.
- [x] Minimal launcher changes are metadata/status/scene-ID handling only:

```python
expected_kind = ('FORMAL_ATTEMPT' if config['status'] == 'FROZEN_FOR_FORMAL'
                 else 'PILOT_ATTEMPT')
```

  The existing collector selects this expected kind from the supplied config;
  it must not pool pilots. Scene description reads `scene.get('tier',scene['id'])`
  for descriptive target ranges while preserving scene ID in its output.
- [x] Run `PYTHONPATH=src:tests CORE_PY -m unittest test_a6_formal_design
  test_a6_attempt test_a6_pilot2 test_a6_report test_a6_scene_description -q`.
  Substitute the existing conda interpreter for CORE_PY. Existing pilot tests
  must pass unchanged. Obtain spec and quality review.

## Task 3 — offline paired statistics (root, independent review)

Files: `scripts/analyze_a6_formal.py`, `tests/test_a6_formal_analysis.py`.

- [x] RED: `exact_mcnemar(0,0)==1`, `exact_mcnemar(6,0)==0.03125`, symmetry
  under swapping methods, missing pairs never become failures, and all-zero
  discordances still have a nonzero-width risk-difference interval.
- [x] Implement primary counts from the existing collector's paired records:

```python
discordant = b + c
p = 1. if not discordant else min(1., 2.*binom.cdf(min(b,c), discordant, .5))
delta = sum((b_h-c_h)/n_h for h in tiers)/len(tiers)
```

  Use SciPy only in the offline analysis interpreter. Record the exact-null
  interpretation and prospective power assumptions in the output, not only p.
- [x] Compute conservative simultaneous Clopper–Pearson bounds for the six
  tier-specific discordance probabilities (alpha/6 for each interval):

```python
lower = sum(L_b[h]-U_c[h] for h in tiers)/3.
upper = sum(U_b[h]-L_c[h] for h in tiers)/3.
```

  Preserve missing/incomplete as unavailable; do not present final inference
  before all prespecified primary pairs complete. Add the separately labeled
  first-activation INVALID-as-failure sensitivity from the existing collector.
- [x] Also report the approximate equal-tier paired-mean interval using
  `SE^2=sum_h s_h^2/n_h / 9`, with `s_h^2=((b_h+c_h)-n_h*delta_h^2)/(n_h-1)`.
  Mark sparse directional counts and zero variance; total zero variance means
  no approximate interval, not `[0,0]`. Always retain the conservative exact
  bound. Neither interval is silently substituted for the exact primary test.
- [x] Test power planning independently of observed pilot outcomes and record
  sensitivity to discordance assumptions. Secondary endpoint/resource summaries
  have explicit available denominators and no success-conditioned main claim.
  Hard ablations are secondary; no extra w/o-task run duplicates Generic.
- [x] Run focused tests in the core interpreter, then independent statistical
  and code review. Do not change a method or stopping policy to simplify analysis.

## Task 4 — storage, freeze and serial execution (root)

- [x] Measure existing `rosbag compress --bz2 --output-dir NEW_DIR BAG` on a
  representative closed pilot bag, preserving the pilot original. Verify native
  topic counts/timestamps and readability; use this solely for storage planning.
- [x] The measured BZ2 ratio is `.7395`, insufficient for this study; native
  rewriting also collapses publisher connection metadata. Do not use that path
  for formal archives. Add opt-in `diagnostic_image_scope=ground_handoff_to_end`
  to the formal generator/config only. Existing Pilot-2 recording stays unchanged.
  In `scripts/run_a6_attempt.py`, keep all non-image topics in diagnostics.bag
  for the full task; start a second native LZ4 recorder for the two raw D435
  image topics after observing `A5_SELECTED` or `A6_RM4D_SELECTED` in the existing
  flushed events.jsonl. Never subscribe to demo status, delay robot action, or
  change the existing deadline. Both events precede return/landing and Ground
  navigation. No handoff means no D435 refinement images are needed.
- [x] RED tests in `tests/test_a6_attempt.py` verify the common non-image topic
  set, exact two image topics, no start on score/confirmation/FAILED, one start
  on either actual handoff event, unchanged timeout/exit handling and cleanup.
  Use an ordinary polling wait around the existing adapter process, not a new
  ROS protocol/service or lifecycle framework. Record image command/exit/path;
  diagnostics failures cannot erase an already measured task failure.
- [x] Use Pilot-2 event times and existing bag indices to quantify selection-
  scoped native image storage before formal launch. Keep all pilot originals
  untouched and retain the temporary compression probe for transparent review.
- [ ] Commit/push reviewed protocol, exact serialized seeds/order and tested
  orchestration before any formal activation. No merge to main.
- [ ] For each prelisted scene block run the unchanged setup check, then each
  prelisted method in its fixed order, one fresh simulator at a time. Invoke
  `scripts/run_a6_attempt.py --config configs/a6_formal.json --slot N
  --output-dir outputs/a6/formal/slot-NNN-SCENE-METHOD-01` in the existing P450
  environment on the owned ports 11951/11952. Never change order by method result.
- [ ] Inspect outcome/terminal measurements and new failures after each attempt.
  Retain every valid failure; a demonstrated INVALID gets a new directory with
  the same slot/seed/method. Do not automatically retry all non-successes.
- [ ] Run existing mechanism/scoring/scene-description and secondary bag resource
  reports uniformly. Original online resources remain primary. Inspect native bag
  scope/readability after shutdown; no archive conversion is required.
- [ ] Use small source/output checkpoints. Stop only for the user's specified
  scientific conditions; ordinary process/logging/timeout problems are diagnosed
  and repaired within scope, not a reason to ask for routine approval.

## Task 5 — final analysis and handoff (root)

- [ ] Run final paired analysis once all planned valid slots are complete.
  Report the E2E outcome table, discordant pairs, per-tier effects, confirmation
  → D_exec → retrieval, resources, all failures/INVALIDs, same-state diagnostics,
  mechanism counts and secondary Hard ablations. Report null/negative outcomes.
- [ ] Generate compact data-driven figures with Matplotlib and save JSON/CSV,
  paper-ready methods/results text, commands and acknowledged model limitations.
  No raster-generation model or scene GT is used to invent results.
- [ ] Run necessary core/Noetic regression, inspect source boundaries/worktree,
  commit/push final research checkpoint and obtain independent final review.
  Mark the persistent goal complete only when experiments and analysis are
  genuinely complete; otherwise obey the scientific stop rule with evidence.
