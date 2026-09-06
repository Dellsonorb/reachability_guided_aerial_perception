# A2 implementation plan

**Goal:** Independent map-frame endpoint evidence with `unknown_score`, preserving
the approved static horizontal-ground model and frozen A1.

**Architecture:** A standalone `environment_belief` package uses NumPy for rigid
transforms, endpoint classification and per-observation cell votes. Local Agg
figures and compact NPZ/JSON files support three synthetic offline scenes.

**Execution:** Implement inline on `feature/a2-mid360-environment-belief` using
test-driven development; user already approved implementation. No A1 edits.

## Tasks

- [x] Core: `src/environment_belief/core.py`, package exports, and
  `tests/test_environment_belief.py`. First write/run failing tests for rigid
  transforms, input filtering, per-observation vote caps, occupied priority,
  ground thresholds, empty/ambiguous observations, order independence and
  snapshot isolation. Implement only this endpoint algorithm, then run the
  focused suite.
- [x] Outputs: `src/environment_belief/outputs.py` and
  `tests/test_environment_outputs.py`. First test NPZ/JSON round trips and raw
  state/unknown-score/count PNG output, including empty grids. Implement
  ordinary serialization and backend-local plotting. Run focused tests.
- [x] Synthetic validation: `src/environment_belief/synthetic.py`,
  `scripts/run_a2_offline_validation.py`, `tests/test_environment_synthetic.py`.
  Test nearest ground/box hits and clear / overflight-obstacle /
  occlusion-multiview semantics before implementing the fixture generator.
  Produce physically occluded sensor-frame return points; no labels enter A2.
  Add a side-view plot that shows the overflight ray above the low box.
- [x] Run the three demos into `outputs/a2/`, inspect numerical results and
  PNGs, and write `docs/A2_ENVIRONMENT_BELIEF.md` covering mathematics, APIs,
  observed results, limits and the future A3 boundary. Add no fusion code.
- [x] Final verification: run all A2 and existing A1 unit tests once with
  warnings as errors, compile new modules/scripts, inspect scope and verify
  frozen A1 files unchanged. Commit A2 locally and stop; no push/PR requested.

Focused test command (replace the module suffix for each task):

```bash
PYTHONPATH=src MPLCONFIGDIR=/tmp/a2-mpl XDG_CACHE_HOME=/tmp/a2-cache \
/media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python \
  -W error -m unittest tests.test_environment_belief -v
```

Final suite: the same environment and interpreter with
`-W error -m unittest discover -s tests -v`. No real RM4D reruns.

Completed: 95 unit tests passed (72 existing A1 + 23 A2), modules/scripts
compiled, all saved intermediate/final arrays matched saved-input replay.
Review identified an internal grid-edge roundoff issue; a failing boundary
test reproduced it, and direct edge comparisons fixed it without epsilon
rounding. Review then reported no remaining findings. A1 files are unchanged.
