# A1 Validated Manipulation Interest Field Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a minimal ROS-independent Python prototype that converts a map-frame Brick grasp TCP and the frozen RM4D planner's evaluated candidates into a coverage-aware, joint-margin-driven two-dimensional Validated Manipulation Interest Field.

**Architecture:** A small typed core validates grasp/grid data, rasterizes all `evaluated_candidates`, and keeps unsupported cells as `UNASSESSED`. A lazy CLI adapter calls the frozen `BasePlacementAPI` once per grasp, while output and plotting modules serialize compact field artifacts and expose an RViz-ready payload without importing ROS.

**Tech Stack:** Python 3.10, NumPy, Matplotlib (headless), standard-library `dataclasses`, `enum`, `json`, `csv`, `argparse`, and `unittest`; external read-only RM4D AUBO baseline using PyBullet/SciPy.

---

## File structure

- `pyproject.toml`: package metadata and NumPy/Matplotlib dependencies.
- `.gitignore`: ignore Python caches, environments, and transient outputs.
- `src/reachability_guided_aerial_perception/model.py`: enums and validated data structures.
- `src/reachability_guided_aerial_perception/field.py`: validity gate, margin relevance, coverage, and rasterization.
- `src/reachability_guided_aerial_perception/outputs.py`: NPZ/JSON/CSV bundle and RViz payload conversion.
- `src/reachability_guided_aerial_perception/visualization.py`: headless three-panel offline plot.
- `src/reachability_guided_aerial_perception/cli.py`: one-grasp frozen-RM4D command.
- `src/reachability_guided_aerial_perception/__init__.py`: public API exports.
- `scripts/run_offline_validation.py`: three-scenario real-map runner using one long-lived RM4D API.
- `examples/scenarios/*.json`: map-frame nominal, boundary, and no-inverse inputs.
- `tests/test_model.py`: input/grid validation.
- `tests/test_field.py`: scoring, gates, coverage, yaw aggregation, and live-call seam.
- `tests/test_outputs.py`: serialization and RViz encoding.
- `tests/test_visualization.py`: headless plot smoke test.
- `README.md`: method, limitations, commands, and future MID360 fusion boundary.

### Task 1: Package and validated core types

**Files:**
- Create: `.gitignore`
- Create: `pyproject.toml`
- Create: `src/reachability_guided_aerial_perception/__init__.py`
- Create: `src/reachability_guided_aerial_perception/model.py`
- Create: `tests/test_model.py`

- [ ] **Step 1: Write failing model tests**

```python
import unittest

from reachability_guided_aerial_perception.model import GraspTCP, GridSpec


class ModelTest(unittest.TestCase):
    def test_grasp_requires_map_frame_and_unit_quaternion(self):
        grasp = GraspTCP("g", "map", (1, 2, 0.4), (0, 0, 0, 1))
        self.assertEqual("map", grasp.as_request()["frame_id"])
        with self.assertRaisesRegex(ValueError, "map"):
            GraspTCP("g", "world", (1, 2, 0.4), (0, 0, 0, 1))
        with self.assertRaisesRegex(ValueError, "unit"):
            GraspTCP("g", "map", (1, 2, 0.4), (0, 0, 0, 2))

    def test_centered_grid_uses_ros_xy_row_column_convention(self):
        grid = GridSpec.centered((1.0, 2.0), 3.0, 3.0, 0.1)
        self.assertEqual((-0.5, 0.5), (grid.origin_x, grid.origin_y))
        self.assertEqual((30, 30), grid.shape)
        self.assertEqual((15, 15), grid.cell_index(1.01, 2.01))
        self.assertIsNone(grid.cell_index(4.0, 2.0))
```

- [ ] **Step 2: Run the tests and verify RED**

Run:

```bash
PYTHONPATH=src /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python \
  -W error -m unittest tests.test_model -v
```

Expected: import failure because the package does not exist.

- [ ] **Step 3: Implement the package metadata and core types**

Implement `GraspTCP` with finite-value and unit-quaternion checks and an
`as_request()` method that emits only the four frozen fields. Implement
`GridSpec` with positive/divisible geometry, `centered()`, `shape`, and
`cell_index()` using `row=floor((y-origin_y)/resolution)` and
`column=floor((x-origin_x)/resolution)`.

Define these exact enums and immutable records in `model.py`:

```python
class FieldStatus(str, Enum):
    PARTIALLY_ASSESSED = "PARTIALLY_ASSESSED"
    NO_INVERSE_REACHABLE = "NO_INVERSE_REACHABLE"


class CellState(IntEnum):
    UNASSESSED = -1
    INFEASIBLE = 0
    LOW = 1
    HIGH = 2


@dataclass(frozen=True)
class FieldConfig:
    minimum_joint_margin_rad: float = 0.01
    joint_margin_saturation_rad: float = 0.5
    high_relevance_threshold: float = 0.8
```

Add `AssessmentCoverage`, `ManipulationInterestField`, and
`OccupancyGridPayload` records with the fields named in the design spec.

- [ ] **Step 4: Run model tests and verify GREEN**

Run the Task 1 command. Expected: 2 tests pass.

- [ ] **Step 5: Commit**

```bash
git add .gitignore pyproject.toml src tests/test_model.py
git commit -m "feat: add validated interest field data model"
```

### Task 2: Margin-only field construction and coverage

**Files:**
- Create: `src/reachability_guided_aerial_perception/field.py`
- Create: `tests/test_field.py`
- Modify: `src/reachability_guided_aerial_perception/__init__.py`

- [ ] **Step 1: Write failing scoring and gate tests**

Create candidate helpers that vary `joint_margin_rad`, residuals, and validity
flags. Assert these exact behaviors:

```python
def test_relevance_is_joint_margin_only(self):
    low_residual = candidate(margin=0.25, position_residual=0.0)
    high_residual = candidate(margin=0.25, position_residual=0.00099)
    self.assertEqual(0.5, candidate_relevance(low_residual, FieldConfig()))
    self.assertEqual(
        candidate_relevance(low_residual, FieldConfig()),
        candidate_relevance(high_residual, FieldConfig()),
    )

def test_every_validity_gate_forces_zero(self):
    for change in (
        {"rm4d_reachable": False},
        {"ik_valid": False},
        {"collision_free": False},
        {"footprint_collision": True},
        {"joint_margin_rad": 0.009},
        {"valid": False},
    ):
        self.assertEqual(0.0, candidate_relevance(candidate(**change), FieldConfig()))
```

- [ ] **Step 2: Run the focused tests and verify RED**

```bash
PYTHONPATH=src /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python \
  -W error -m unittest tests.test_field.FieldScoringTest -v
```

Expected: import failure for `candidate_relevance`.

- [ ] **Step 3: Implement the validity gate and score**

```python
def candidate_relevance(candidate, config):
    margin = candidate.get("joint_margin_rad")
    valid = (
        candidate.get("rm4d_reachable") is True
        and candidate.get("ik_valid") is True
        and candidate.get("collision_free") is True
        and candidate.get("footprint_collision") is not True
        and candidate.get("valid") is True
        and margin is not None
        and math.isfinite(float(margin))
        and float(margin) >= config.minimum_joint_margin_rad
    )
    if not valid:
        return 0.0
    return min(1.0, max(0.0, float(margin) / config.joint_margin_saturation_rad))
```

- [ ] **Step 4: Verify scoring GREEN**

Run the focused command. Expected: scoring tests pass.

- [ ] **Step 5: Write failing rasterization tests**

Test that two valid yaws in one cell produce the maximum margin, the selected
yaw and its residual diagnostics; an invalid-only cell is `INFEASIBLE`; an
empty cell remains `UNASSESSED` with `NaN` relevance. Test that coverage uses
the full summary and marks truncation when `deduplicated > evaluated`.

```python
field = build_field_from_result(grasp, result_with_three_candidates(), grid, config)
self.assertEqual(CellState.HIGH, field.cell_state[1, 1])
self.assertEqual(1.0, field.relevance[1, 1])
self.assertEqual(0.5, field.best_yaw[1, 1])
self.assertEqual(CellState.INFEASIBLE, field.cell_state[1, 2])
self.assertTrue(np.isnan(field.relevance[0, 0]))
self.assertEqual(CellState.UNASSESSED, field.cell_state[0, 0])
self.assertTrue(field.coverage.validation_truncated)
```

- [ ] **Step 6: Run rasterization tests and verify RED**

Run all of `tests.test_field`. Expected: missing `build_field_from_result`.

- [ ] **Step 7: Implement rasterization, coverage, and live API call**

Initialize relevance/best-yaw/diagnostic arrays with `NaN`, state with
`UNASSESSED`, and count arrays with zero. For each finite candidate inside the
grid, increment evaluated count, compute the margin-only score, and retain the
highest-scoring valid candidate. After aggregation, label evaluated-only cells
`INFEASIBLE` and feasible cells `LOW` or `HIGH`.

If `summary.inverse_reachable == 0`, return an all-undefined field with
`FieldStatus.NO_INVERSE_REACHABLE`. Otherwise use
`FieldStatus.PARTIALLY_ASSESSED`.

Implement the live seam exactly as:

```python
def build_field(grasp, rm4d_api, grid=None, config=FieldConfig()):
    grid = grid or GridSpec.centered(grasp.position_xyz[:2], 3.0, 3.0, 0.1)
    result = rm4d_api.plan(grasp.as_request(), top_k=1)
    return build_field_from_result(grasp, result, grid, config)
```

Validate matching `map` frame/grasp ID and required result collections.

- [ ] **Step 8: Verify field GREEN and full suite**

```bash
PYTHONPATH=src /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python \
  -W error -m unittest discover -s tests -v
```

Expected: all model and field tests pass.

- [ ] **Step 9: Commit**

```bash
git add src/reachability_guided_aerial_perception tests/test_field.py
git commit -m "feat: build coverage-aware margin interest field"
```

### Task 3: Compact outputs and RViz payload

**Files:**
- Create: `src/reachability_guided_aerial_perception/outputs.py`
- Create: `tests/test_outputs.py`
- Modify: `src/reachability_guided_aerial_perception/__init__.py`

- [ ] **Step 1: Write failing output tests**

Use a temporary directory and a synthetic field. Assert:

```python
paths = save_field_bundle(field, result["evaluated_candidates"], directory, config)
self.assertEqual({"field", "summary", "candidates"}, set(paths))
saved = np.load(paths["field"])
np.testing.assert_array_equal(field.cell_state, saved["cell_state"])
summary = json.loads(paths["summary"].read_text())
self.assertEqual("joint_margin_only", summary["relevance_semantics"])
self.assertEqual("PARTIALLY_ASSESSED", summary["status"])

payload = to_occupancy_grid_payload(field)
self.assertEqual([-1, 0, 50, 100], payload.data.tolist())
self.assertEqual("PARTIALLY_ASSESSED", payload.field_status)
```

- [ ] **Step 2: Run output tests and verify RED**

```bash
PYTHONPATH=src /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python \
  -W error -m unittest tests.test_outputs -v
```

Expected: missing output module/functions.

- [ ] **Step 3: Implement serialization and RViz conversion**

Write `field.npz` with every field array, `summary.json` with grid/config/
coverage/status, and `candidate_diagnostics.csv` with candidate ID, pose,
gates, margin, residuals, score, validity, and rejection reason. Use only
ordinary `Path.mkdir`, NumPy, JSON, and CSV operations.

For the RViz payload, flatten in C row-major order and encode:

```python
data = np.full(field.relevance.shape, -1, dtype=np.int8)
data[field.cell_state == CellState.INFEASIBLE] = 0
feasible = np.isin(field.cell_state, [CellState.LOW, CellState.HIGH])
data[feasible] = np.clip(np.rint(100.0 * field.relevance[feasible]), 1, 100)
```

- [ ] **Step 4: Run output tests and full suite**

Expected: all tests pass and JSON contains no non-standard `NaN` tokens.

- [ ] **Step 5: Commit**

```bash
git add src/reachability_guided_aerial_perception tests/test_outputs.py
git commit -m "feat: serialize fields and expose RViz payload"
```

### Task 4: Offline visualization

**Files:**
- Create: `src/reachability_guided_aerial_perception/visualization.py`
- Create: `tests/test_visualization.py`
- Modify: `src/reachability_guided_aerial_perception/__init__.py`

- [ ] **Step 1: Write failing headless plot test**

```python
with tempfile.TemporaryDirectory() as directory:
    output = Path(directory) / "field.png"
    render_field(field, grasp, candidates, output)
    self.assertTrue(output.is_file())
    self.assertGreater(output.stat().st_size, 10_000)
```

- [ ] **Step 2: Run visualization test and verify RED**

Expected: missing `render_field`.

- [ ] **Step 3: Implement the three-panel plot**

Use Matplotlib `Agg`. Panel 1 is the raw masked relevance grid with fixed
`[0,1]` scale; panel 2 is categorical `UNASSESSED/INFEASIBLE/LOW/HIGH`; panel
3 scatters all evaluated candidate XY locations, colors valid candidates by
margin relevance, marks invalid candidates with gray `x`, and marks the grasp
TCP with a star. Use `imshow(..., interpolation="none", origin="lower")` so
no smoothing is introduced. For `NO_INVERSE_REACHABLE`, show the status as an
explicit centered annotation.

- [ ] **Step 4: Run visualization test and full suite**

Expected: PNG smoke test and all previous tests pass with warnings treated as
errors.

- [ ] **Step 5: Commit**

```bash
git add src/reachability_guided_aerial_perception tests/test_visualization.py
git commit -m "feat: add unsmoothed VMIF visualization"
```

### Task 5: Frozen RM4D CLI and real offline scenarios

**Files:**
- Create: `src/reachability_guided_aerial_perception/cli.py`
- Create: `scripts/run_offline_validation.py`
- Create: `examples/scenarios/nominal.json`
- Create: `examples/scenarios/boundary.json`
- Create: `examples/scenarios/no_inverse.json`
- Create: `tests/test_cli.py`

- [ ] **Step 1: Write failing CLI parsing test**

Test `load_grasp()` with a valid map JSON, a world-frame rejection, and an
input containing `current_bunker_pose` to confirm the A1 interface rejects
fields that would bias candidate selection.

- [ ] **Step 2: Run CLI tests and verify RED**

Expected: missing CLI module.

- [ ] **Step 3: Implement CLI and three-scenario runner**

The CLI accepts explicit `--rm4d-root`, `--rm4d-config`, `--rm4d-map`,
`--grasp`, and `--output-dir`. It lazily adds the supplied root to `sys.path`,
imports `BasePlacementAPI`, creates one context-managed API, calls it once,
builds the field, writes the bundle/plot, and prints the summary.

The offline runner loads the same API once and processes the three checked-in
map-frame scenario files. It asserts:

```python
assert nominal.status is FieldStatus.PARTIALLY_ASSESSED
assert np.any(nominal.cell_state == CellState.HIGH)
assert boundary.status is FieldStatus.PARTIALLY_ASSESSED
assert boundary.coverage.evaluated_cells > 0
assert no_inverse.status is FieldStatus.NO_INVERSE_REACHABLE
assert no_inverse.coverage.evaluated_candidates == 0
```

Do not add timing loops, repetitions, benchmark tables, ROS, or Gazebo.

- [ ] **Step 4: Run CLI unit tests and full suite**

Expected: all tests pass without invoking PyBullet.

- [ ] **Step 5: Commit**

```bash
git add src/reachability_guided_aerial_perception/cli.py scripts examples tests/test_cli.py
git commit -m "feat: add frozen RM4D offline validation entrypoints"
```

### Task 6: Documentation and real-map validation artifacts

**Files:**
- Create: `README.md`
- Create: `outputs/a1/nominal/*`
- Create: `outputs/a1/boundary/*`
- Create: `outputs/a1/no_inverse/*`

- [ ] **Step 1: Write the README**

Document the margin-only equations, all gates, field/cell statuses, coverage
limitation, exact public API, the external baseline command, output files,
top-K distinction, and future footprint-aware MID360 formula. Explicitly say
that A1 is a partially assessed field over the frozen 256-candidate budget,
not a complete RM4D capability map.

- [ ] **Step 2: Run the three real frozen-RM4D scenarios**

```bash
PYTHONPATH=src:/media/lu/P450_PAPER/RM4D_AUBO \
  /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python \
  scripts/run_offline_validation.py \
  --rm4d-root /media/lu/P450_PAPER/RM4D_AUBO \
  --rm4d-config /media/lu/P450_PAPER/RM4D_AUBO/configs/mr4_offline_base_placement.json \
  --rm4d-map /media/lu/P450_PAPER/RM4D_AUBO/runs/formal-10m/data/rm4d_aubo_i5_joint_42/10000000/rmap.npy \
  --output-root outputs/a1
```

Expected: nominal and boundary are `PARTIALLY_ASSESSED`; no-inverse is
`NO_INVERSE_REACHABLE`; each scenario has NPZ, JSON, CSV, and PNG output.

- [ ] **Step 3: Inspect the three PNGs and summaries**

Confirm that every colored cell is backed by evaluated candidates, unassessed
cells remain visibly separate, low/high values follow joint margin, and the
no-inverse plot states the field-level condition. Record only observed counts
in the README; do not add a benchmark framework.

- [ ] **Step 4: Run final verification**

```bash
PYTHONPATH=src /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python \
  -W error -m unittest discover -s tests -v
git diff --check
git status --short
```

Expected: all tests pass, diff check is clean, and status lists only intended
README/output changes before the final commit.

- [ ] **Step 5: Commit**

```bash
git add README.md outputs/a1
git commit -m "docs: report A1 offline spatial validation"
```

### Task 7: Independent review and completion check

**Files:**
- Review all files changed since design commit `477f9af`.

- [ ] **Step 1: Request code review**

Ask a reviewer to compare implementation and outputs against the approved
design, with special attention to residual independence, coverage semantics,
no smoothing, `NO_INVERSE_REACHABLE`, map/grid conventions, and absence of
out-of-scope infrastructure.

- [ ] **Step 2: Address Critical or Important findings with TDD**

For each valid finding, add a failing regression test, observe RED, apply the
minimal fix, and observe GREEN. Do not expand scope for optional ideas.

- [ ] **Step 3: Re-run fresh final verification**

```bash
PYTHONPATH=src /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python \
  -W error -m unittest discover -s tests -v
PYTHONPATH=src /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python \
  -m compileall -q src scripts
git diff --check
git status --short --branch
git log --oneline --decorate -8
```

Expected: tests and compilation pass, whitespace is clean, and the feature
branch contains only the scoped A1 commits.
