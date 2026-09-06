# A3 Task-Relevant Uncertainty Implementation Plan

> Execute inline with test-driven development and completion verification.
> User approved implementation; work on the requested A3 branch in the existing
> dedicated workspace. The small geometry/core/output steps are sequential.

**Goal:** A1 + footprint + A2 -> nominal/operational support and U_task.
**Architecture:** A new pure NumPy package; frozen A1/A2 public inputs only.
**Tech Stack:** Existing Python 3.10, NumPy 2, Matplotlib/Agg, unittest.

## Task 1: Geometry

Files: `src/task_relevant_uncertainty/geometry.py`,
`tests/test_task_geometry.py`.

- [x] Write tests for rectangle dimensions/rotation, overlap including contact,
  non-center overlap, yaw pi/4 corner separation, clipping and finite inputs.
- [x] Run focused test module and confirm missing implementation failure.
- [x] Implement FootprintSpec, footprint_vertices, footprint_cells. For each
  candidate cell use SAT: abs((cell_center-q) dot axis) <= rectangle projection
  radius + cell halfsize*(abs(axis.x)+abs(axis.y)) + 1e-12, on four axes.
- [x] Run tests to green, then commit geometry and tests.

## Task 2: Field core

Files: `src/task_relevant_uncertainty/core.py`, package `__init__.py`,
`tests/test_task_uncertainty.py`. Use existing A1 test fixtures to construct
real A1 fields and the A2 updater for observations; no changes to those fixtures.

- [x] Write tests for NaN/no support, blocked-only zero, unknown not blocking,
  FREE score product, overlapping max and source selection, clipped support,
  A1 state independence, no-inverse status, alignment and input immutability.
- [x] Run focused module and confirm missing implementation failure.
- [x] Implement typed output enums/records and pure builder using Task 1 SAT.
  Preserve all approved semantic qualifiers and provide covered-cell relations.
- [x] Run tests to green, then commit core and tests.

## Task 3: Offline outputs and scenes

Files: `src/task_relevant_uncertainty/outputs.py`, `synthetic.py`,
`scripts/run_a3_offline_validation.py`, `tests/test_task_outputs.py`.

- [x] First test NPZ/JSON round trip, no support/NaN rendering, no backend switch,
  exactly four scene semantics and direct CLI execution in a temporary folder.
- [x] Run tests and confirm missing implementation failure.
- [x] Implement field.npz, summary.json, supports.json, fixed-scale raw-cell
  plotting and four scenarios from the approved spec. Synthetic A1 inputs are
  visibly labeled as fixtures, not newly IK-validated observations. A2 uses
  endpoint observations; known-free scene stores N=2 and N=8 snapshots.
- [x] Run focused tests to green. Generate results into `outputs/a3/`, inspect
  main PNGs and numeric support/score outputs. Commit this batch after review.

## Task 4: Handoff

- [x] Write `docs/A3_TASK_RELEVANT_UNCERTAINTY.md` with equations, frame/footprint
  definition, array API, source relations, numeric results, pictures, limits,
  invocation and A4 input boundary (no A4 implementation).
- [x] Bounded code review for algorithm/geometry/interface correctness only.
- [x] Final test suite, compile new files, diff check and confirm no main-tracked
  A1/A2 files were modified. Commit locally, leave A3 branch, stop without push.

## Commands

Focused tests (replace module suffix):

```bash
PYTHONPATH=src MPLCONFIGDIR=/tmp/a3-mpl XDG_CACHE_HOME=/tmp/a3-cache \
/media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python -W error \
  -m unittest tests.test_task_geometry -v
```

Final suite: same interpreter/environment with `-m unittest discover -s tests -v`.
Demo: same interpreter/environment with
`scripts/run_a3_offline_validation.py --output-root outputs/a3`.
Compile: `python -m compileall -q src/task_relevant_uncertainty scripts/run_a3_offline_validation.py`.
Scope: `git diff main --diff-filter=MD --name-only` must be empty (only new A3 files).

## Completion

120 tests passed with warnings as errors (95 frozen A1/A2 + 25 A3). New modules
and CLI compiled. Four final scenes plus the N=2 intermediate exactly match
recomputation from their saved A1/A2 inputs. Bounded geometry/core and
outputs/scenario reviews reported no remaining findings. A1/A2 main-tracked
files are unchanged. A3 stays local on its feature branch; no A4 work.
