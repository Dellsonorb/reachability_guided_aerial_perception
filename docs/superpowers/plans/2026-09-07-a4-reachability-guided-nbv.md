# A4 Reachability-Guided NBV Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the approved, one-step, ROS-independent predicted observation-gain prototype, without changing A1/A2/A3.

**Architecture:** A new `reachability_guided_nbv` package reads aligned frozen A3/A2 snapshots. It generates level-hover candidates, predicts ground-endpoint visibility against fixed-height occupied prisms, and ranks the same candidates with task-weighted and generic uncertainty-reduction surrogates. Synthetic fixtures and plots are separate from the ranking API.

**Tech Stack:** Existing Python 3.10, NumPy, Matplotlib, unittest; no new dependencies.

Execution is local on the user-designated `feature/a4-reachability-guided-nbv` branch. The design is already approved; finish the prototype and stop, without a new approval loop or A4 remote integration.

## Accepted geometry and scope

- Level UAV pose `(x,y,z,yaw)` in `map`; fixed current altitude. Default XY offsets `(-4,-2,0,2,4)` m and eight relative yaw samples; actual current pose included.
- Verified active `p450_runtime.launch` and `p450_mid360_tf_contract.yaml`: `T_uav_lidar = [Ry(.35), (.14714489037277257,0,.27696863564236896)]`. Mount translation includes the rotated 0.05 m ray-sensor offset. The older zero-pitch offset file is not loaded by the active launch.
- Actual tilted mount requires yaw search. Only a centered sensor with its vertical axis parallel to UAV vertical is sensing-yaw-equivalent under 360-degree horizontal FOV; collapse those headings.
- Target each non-OCCUPIED environment cell center at A2 `ground_z_m`; transform into sensor coordinates. Use A2 strict min/max range and elevation `[-7,52]` degrees. Known OCCUPIED cell rectangles extruded from ground to `ground+1.0 m` block a segment only if its **3D** segment intersects the prism. UNKNOWN does not occlude.
- `H_assumed=1.0 m` is an occlusion surrogate, not measured height. Visibility means one idealized new informative-endpoint opportunity, not a guaranteed scan return.
- With `tau=A2.config.unknown_scale`, `delta_u=-expm1(-1/tau)*unknown_score`; `G_task=-expm1(-1/tau)*sum(V*U_task)`. No-support cells remain masked; no generic fallback. FREE can still contribute.
- `C=distance_3d + rho_yaw*abs(wrapped_yaw_difference)`, default `rho_yaw=.25 m/rad`; `Score=G-lambda*C`, default `lambda=.25 surrogate-units/m`. These explicit illustrative values are not fitted. Generic uses `sum(V*delta_u)` with identical visibility/cost/candidates.
- A2/A3 are never updated during prediction. No voxel/height map, smoothing, path safety claim, actual flight, ROS, SIM connection, mission loop, benchmark or A5.

## Task 1 — Transform, candidates and visibility

Files: create `src/reachability_guided_nbv/{__init__,model,geometry}.py` and `tests/test_nbv_geometry.py`.

- [x] Write tests first for composite extrinsic, yaw-dependent FOV, centered-upright yaw collapse, strict range, UNKNOWN transparency, occupied target exclusion, through/above-prism distinction, rejected sensor origin, and invalid numeric/TF inputs. The principal geometric witness is:

```python
v = Viewpoint((0, 0, 1.5), 0)
# Target (4,0,0): elevation -4.706 deg at yaw=0, -43.248 at yaw=pi.
# A prism near x=1 lies below the ray; a prism near x=3 intersects it.
```

- [x] Run `PYTHONPATH=src /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python -m unittest tests.test_nbv_geometry -v`; expect missing A4 functionality before implementation.
- [x] Implement `Viewpoint`, `SensorModel`, `NBVConfig`, `generate_candidates`, `sensor_transform`, batched segment/AABB slab intersections, and `predict_visibility`. Use closed prisms; never carve XY rays.
- [x] Repeat the targeted command; require all geometric assertions to pass.

## Task 2 — Prediction and ranking

Files: create `src/reachability_guided_nbv/core.py` and `tests/test_nbv_core.py`.

- [x] Write tests for `rank_viewpoints(a3,a2,current,*,candidates=None,sensor=SensorModel(),config=NBVConfig())`, including:

```python
alpha = -np.expm1(-1 / a2.config.unknown_scale)
expected = alpha * np.nansum(result.visibility[0] * a3.task_relevant_uncertainty)
self.assertAlmostEqual(result.candidates[0].task_gain, expected)
```

- [x] Run `.../python -m unittest tests.test_nbv_core -v` under `PYTHONPATH=src`; observe failure before implementing.
- [x] Implement finite aligned snapshot checks, marginal gain, flight proxy, deterministic ranking, no-task-gain status and retained source/mask diagnostics. Preserve A1 UNASSESSED versus A2 UNKNOWN; do not re-evaluate A3 operational support.
- [x] Run geometry/core tests; check identity, positive FREE marginal gain, no-support masking, stale snapshot rejection, yaw-wrap cost and no input mutation.

## Task 3 — Three offline scenes, outputs and handoff

Files: create `src/reachability_guided_nbv/{synthetic,outputs}.py`, `scripts/run_a4_offline_validation.py`, `tests/test_nbv_offline.py`, `docs/A4_REACHABILITY_GUIDED_NBV.md`, and `outputs/a4/` results.

- [x] Write failing scene/output tests: task versus generic selects differently; known barrier changes Ours with unchanged A3 support; adding cost changes a distant small-gain preference to a nearby pose. Serialize enough arrays to reproduce the gains and retain A3 support/source fields.
- [x] Run `.../python -m unittest tests.test_nbv_offline -v`; observe missing scene/output functionality.
- [x] Implement deterministic fixtures using existing A1/A2/A3 public builders only in the fixture module. Produce JSON ranking, NPZ contribution/visibility/source arrays and unsmoothed PNG panels; include the through/above side view.
- [x] Run the three scenes with `PYTHONPATH=src MPLCONFIGDIR=/tmp/a4-mpl XDG_CACHE_HOME=/tmp/a4-cache .../python scripts/run_a4_offline_validation.py --output outputs/a4`; inspect PNGs and check numeric scenario assertions.
- [x] Request a bounded code review under the requesting-code-review skill; fix directly relevant correctness issues only.
- [x] Run `PYTHONPATH=src MPLCONFIGDIR=/tmp/a4-mpl XDG_CACHE_HOME=/tmp/a4-cache .../python -m unittest discover -s tests -v`, `git diff --check`, and confirm no frozen source/output changes versus `23791e1`. Document actual results and local commit; stop without SIM/A5 or push.

## Execution outcome — 2026-09-07

- Implemented only the new A4 package, its tests, runner, documentation and `outputs/a4/`.
- Final full suite: 147 tests passed (120 frozen A1/A2/A3 + 27 A4), 19.748 s. Three scene groups reran successfully; the five case rankings match the documented values.
- Default 200-candidate generation-to-ranking path passed an additional integration check. Controlled scene candidates deliberately isolate weighting, occlusion and cost; this is not a benchmark.
- Read-only independent review found no critical/important issues. Corrected the minor configurable-height plot label after a failing regression test; added exact strict range boundary checks.
- Verified unchanged frozen source and output paths against `23791e1`; all existing tracked files remain unchanged. Keep A4 on its local feature branch after the final commit; no push, ROS/SIM, A5 or extra framework.
