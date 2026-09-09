# v1.3 AMBIGUOUS sub-cell operational gating — design for research review

2026-09-09. **Design record; implementation authorized by the subsequent
autonomous-development update.** The initial design-only scope below records
the investigation stage, not a continuing per-module approval barrier.
Base: `a89c84d`, branch `feature/a6-formal-experiments`.
Development proceeds on `feature/v13-operational-geometry`; no remaining
fresh-validation or formal slot is launched. PR #6 stays Draft. The existing
three v1.2 validation outcomes and all earlier results remain unchanged.

## Conclusion and remaining scientific decision

The five Moderate winners really are separated from **all 109 retained
AMBIGUOUS occupied endpoints**, not just the endpoints in overlapping coarse
cells. Their minimum continuous distances are 71.850–118.215 mm. Thus the
observed whole-cell veto is a spatial representation issue, not evidence that
the measured endpoint lies inside those padded footprints.

Recommend retaining endpoint-local **closed uncertainty disks**, with a
legacy-conservative cell fallback when endpoint coverage or the declared
sensor profile is missing. Association labels and history never change.

The recommended review option is a **33 mm declared engineering disk**, under
the already used authoritative public-map TF assumption. It is independently
derived from the LiDAR's declared 32 mm sensor budget plus 1 mm numerical
rounding reserve (§4), not from candidate gaps or the RGB-D association budget.
It was a design proposal at the initial review; the subsequent authorized
development implementation explicitly declares this profile before execution.
It is not a frozen final-test protocol or a calibrated physical bound.

**This is not a calibrated total physical error bound.** The actual custom
SIM publisher does not apply the declared 10 mm Gaussian range noise / 2 mm
quantization, and public TF restamps separately received position and attitude
without retaining their source acquisition times. These are explicit limitations
on the claim, not a new requirement to build a full uncertainty-calibration
framework. Neither `.04395 m` nor an outcome-selected value is used.

Under this proposed conditional engineering model, all five would lose the
AMBIGUOUS alias veto, but **all five remain unconfirmed** for lack of actual
ground support. Their recorded v1.2 blocking and trial outcomes remain untouched.
The independent review accepted this explicitly conditional engineering scope;
the later user authorization permits local implementation without another
approval round. A stronger claim of bounded physical placement is not
supported by these records and is not silently imposed as an acceptance bar.

## 1. Frozen boundaries

- A2 raw states, evidence, observation counts, thresholds and updater unchanged.
- v1.1 TARGET/ENVIRONMENT/AMBIGUOUS classification unchanged. In particular,
  mask boundaries and missing registered depth stay AMBIGUOUS.
- Existing expanded perceived-target vs padded BUNKER continuous collision
  unchanged, even if no positive TARGET endpoint was recorded.
- v1.2 exact validated per-cell winner, relevance, yaw, tie-break and A5 exact
  candidate selection unchanged. Non-winners remain diagnostic only.
- Existing actual ground-vote definition unchanged, including per-window
  ENVIRONMENT/AMBIGUOUS suppression and the requirement of two supporting
  windows for every covered cell. No target/ambiguous cell becomes raw FREE.
- A3 and A5 use the same operational-footprint assessment; Generic and Ours
  receive the same rule and data. A4 formulas, visibility, costs and budgets
  unchanged. Uncertainty disks are not relevance smoothing.
- ENVIRONMENT whole-cell blocking remains in scope only as a diagnostic.

The previously quoted `.0439527451 m` is the natural v1.1 regression's
**computed** association allowance, not a literal constant in the accepted
implementation. [The frozen description](../../OBJECT_AWARE_GATING_V11.md)
defines its sensor/pixel formula. The current Moderate reference yields
`.04692094110278105 m` and its shared endpoint a pointwise
`.04670231584953422 m`. This discrepancy with shorthand `.04395` is made
explicit; neither formula nor recorded value is changed or repurposed.

## 2. What runtime actually retains

`run_a5_sim.py` reads finite XYZ from each PointCloud2 packet, looks up public
map TF at that packet's header stamp, and retains a multi-packet window.
`a5_ros_support.merge_cloud_chunks` transforms **each packet individually**
before re-expressing it in the last packet's sensor frame. This is not
within-packet deskew. The five-second window does not introduce five seconds
of uncompensated rigid motion.

Current Moderate `observation_01.npz` contains:

- 88,763 retained endpoint rows in `points_xyz`, final `T_map_sensor` and stamp;
- 52 monotone `chunk_stamps_s`, `chunk_point_counts`, `chunk_T_map_sensor`;
- the frozen initial RGB-D/CameraInfo/TF/target reference in a separate NPZ.

The original map endpoint and row-to-chunk membership are recoverable from
these real arrays. For row58439: chunk index34, retained packet row287,
stamp59.880 simulation seconds. Indices are into the saved finite-point
observation, **not an invented original firing ID**.

`build_operational_context` computes per-return labels but only saves four
aggregate vote arrays. `OperationalEvidenceView` and `operational_evidence.npz`
do **not** carry a vote→endpoint relation. Replaying the retained raw input
with the frozen association and A2 filters reproduces all four recorded
Moderate vote arrays exactly. This is genuine source replay, not a cell-center
backprojection or synthetic endpoint reconstruction.

PointCloud2 also contains plugin-generated timestamps, but the adapter keeps
only XYZ. Those timestamps are not physical firing times in this SIM path:
the rays update together, then the publisher assigns header+i×5 µs. The
saved `/tf` bag lacks source MAVROS position and IMU messages; source age cannot
be recovered from a fresh-looking TF publication stamp.

## 3. Minimal proposed rule

For exact winner q, preserve the already padded closed rectangle

\[
F_q=q_{xy}+\operatorname{Rot}(\psi_q)
([-0.52,0.52]\times[-0.39,0.39])\quad\mathrm{m}.
\]

For every usable AMBIGUOUS occupied endpoint e with available source data and
the declared SIM envelope profile, let p_e be its original map XY and use
profile, rho_e=.033 m:

\[
\mathcal E_e=p_e\oplus\overline B_2(0,\rho_e),\qquad
B_A(q)=\bigvee_e[\mathcal E_e\cap F_q\ne\varnothing].
\]

A disk permits an exact, inexpensive check. With
`a = Rot(-yaw) (p_e - q_xy)`, the Euclidean distance is

\[
d(p_e,F_q)=\sqrt{\max(|a_x|-.52,0)^2+
                       \max(|a_y|-.39,0)^2}.
\]

An endpoint blocks iff `d <= rho_e`, including tangent/corner contact with
the existing numerical contact tolerance. **Every** contributing endpoint
must be covered; a per-cell centroid, one representative return, average
position or majority vote would lose potentially intersecting evidence.
Do not increase spatial precision by implicitly decreasing history.

Let L be cells with positive historical AMBIGUOUS votes for which complete
source endpoints or the declared envelope profile cannot be supplied. Preserve

\[
B_L(q)=\bigvee_{C\in L}[C\cap F_q\ne\varnothing].
\]

The shared proposed result is

\[
B_{\rm op}(q)=B_{\rm ENV\;cell}(q)\lor B_L(q)\lor B_A(q)
                   \lor B_{\rm target\;continuous}(q).
\]

Out-of-grid/clipping and confirmation retain their existing handling. In
particular, `confirmed` still requires a non-clipped footprint, no operational
blocker and **all** actual ground-support cells. An operationally unblocked
pose is not a complete navigation-feasibility certificate.

Important details:

- Query all retained eligible AMBIGUOUS endpoints, including those whose
  original cells do **not** touch the footprint: an envelope may cross a cell
  boundary. Initially a simple linear scan is sufficient; no new index needed.
- An available endpoint whose envelope touches the footprint blocks even if
  its coarse cell did not. The revision is not required to be less conservative
  in every possible case.
- A missing historical observation, partially retained endpoints, unknown
  envelope profile or mixed complete/incomplete history cannot be interpreted as zero
  ambiguity. Preserve its cell blocker, plus known endpoint blockers.
- Do not remove history with later ground/TARGET votes. No reassociation,
  mask enlargement, evidence decay, raw state modification or counter reduction.
- This is a local occupied-evidence model, not a reconstructed complete obstacle
  volume. Real support and UNKNOWN semantics remain essential.
- The legacy fallback preserves v1.2 blocking, not a bound on arbitrary physical
  misregistration: an unknown physical envelope from a neighboring cell could
  extend farther. No whole-cell fallback certifies that stronger property.

### Alternatives considered

The disk plus explicit missing-source fallback is the smallest useful option.
An anisotropic radial/pose-swept envelope could be tighter, but its orientation
and error components need the same currently missing registration information;
it is unnecessary complexity for this revision. A zero-radius point test is
insufficient for uncertain endpoints. Keeping only the existing whole-cell
rule is the valid legacy fallback but preserves the demonstrated aliasing.

## 4. Proposed engineering envelope and limits of its physical interpretation

The minimal proposed SIM profile is

\[
\rho_{\rm SIM}=\underbrace{3(.010)+.002}_{\text{declared LiDAR budget}}
                 +\underbrace{.001}_{\text{numerical rounding reserve}}
               =.033\ \mathrm{m}.
\]

This is a **declared engineering region conditional on public-map TF**, using
the same style of nominal sensor allowance already accepted for v1.1, not
certified physical-error coverage. The first term is an independently sourced
LiDAR range budget, with its full 3D magnitude conservatively projected to an
XY disk. It contains no RGB-D pixel term, depth quantization, brick dimension
or target-association tolerance. The sensor declaration's actual application
is distinguished from its use as a design convention below.

The extra1 mm is a round-up reserve for computational representation, **not**
the existing1 mm RGB-D depth term and not a localization/latency allowance.
For accepted sensor ranges below40 m, float32 XYZ round-to-nearest contributes
at most `sqrt(3) * spacing(float32(40))/2`, about3.304 µm in Euclidean norm.
For the fixed mount's1e-6 per-component quaternion comparison tolerance, the
unit-quaternion difference norm is at most2e-6, corresponding to about4e-6 rad;
over a40.3 m lever arm this is below.162 mm. The corresponding translation
norm is below.002 mm. Rounding this sub-millimetre representation/profile
scale upward to1 mm avoids implying micrometre-level operational precision.
It does not bound dynamic UAV localization error. No component is derived
from the Moderate candidates, scores or outcomes.

The profile and value are **declared before any v1.3 development run**.
The old RGB-D association formula remains unchanged. The design does not
require new covariance, safety certification or a calibration framework to
continue under the existing public-frame modeling assumption. It also does
not pretend that conditioning on that assumption proves its physical accuracy.

For clarity, a **stronger, presently unsupported physical-placement claim**
would additionally need bounds of the following form; they are not new runtime
gates or mandatory infrastructure in this minimal proposal:

\[
\rho_e=b_{\rm local,e}+b_{\rm trans,e}
 +2\ell_e\sin(b_{\rm angle,e}/2)
 +(v_{\max,e}+\ell_e\omega_{\max,e})\Delta t_e
 +b_{\rm num,e}.
\]

Use the same pose-reference origin for translation, angle and lever arm
`ell_e`. If the pose is BUNKER-independent UAV base_link, include the LiDAR
mount in that lever arm; if it is sensor pose, use sensor-to-endpoint range.
Angle bounds are in [0,pi]. Translation/rotation terms describe spatial
registration at the correct acquisition time; the time term covers only
additional uncompensated acquisition/pose skew, without double-counting it.
`b_local` covers the sensor's endpoint-local uncertainty, not object identity
or brick dimensions. The existing robot padding is not silently reassigned to
absorb unknown sensor/TF errors.

| Source component | Verified implementation | Consequence |
| --- | --- | --- |
| Range noise and quantization | Active p450 model declares sigma .010 and resolution .002 inside plugin `<ray>`; custom ODE contact length passes directly to XYZ | These declarations do not establish actual simulated error or a total physical bound |
| Beam angles | The same CSV angles are used for ray casting and published XYZ | Sampling gaps are not angular measurement error or beam divergence |
| Numeric representation | XYZ float32; mount parameters read as floats; static mount contract uses 1e-6 comparisons | Computable representation error, not a bound on localization or dynamic TF error |
| Packet geometry | One ray update then packet publication; artificial per-point time offsets | Neither the nominal 100 ms update period nor offsets certify acquisition skew |
| Packet-to-map placement | Per-packet stamped TF is used; estimator separately receives position and attitude and restamps them | No source-age/covariance/placement bound in the saved input |
| Multi-packet motion | Each saved packet has its own map transform | Do not count whole-window motion again as error |

The matching Gazebo 11.15.1
[MultiRayShape implementation](https://raw.githubusercontent.com/gazebosim/gazebo-classic/gazebo11_11.15.1/gazebo/physics/MultiRayShape.cc)
returns minimum range plus ray length; `GetResRange` reads metadata without
rounding that return. It does not inject range noise in this custom path.
This is a source-level finding, not a change to SIM or the old allowance.

The proposed declared `3*sigma` convention is not a deterministic Gaussian
bound; it does not assert that this publisher applies that noise or that
any end-to-end confidence level is calibrated. It cannot replace registration
terms in a stronger physical claim. The camera pixel/depth budget that
produced `.04395` has a different purpose and supplies none of these missing
terms. The five measured candidate gaps are **not** permissible calibration data
for choosing a radius.

**Not numerically closed for the stronger claim:** source position/attitude age, their placement
accuracy, and unmodeled numerical/timing residuals cannot be bounded from this
recording. Observed adjacent TF deltas describe motion but do not bound hidden
age, localization bias or motion between samples. No zero values are imputed.

Two claims must not be confused at review:

1. **Recommended minimal design:** geometry conditional on existing authoritative
   public map TF and the explicitly declared33 mm engineering disk. Independent
   review accepted this scope; it is not independently bounded physical collision.
2. A stronger claim of physical-placement coverage would require the missing
   components to be addressed. That stronger claim is not made or required
   merely because public TF lacks source-age/covariance information.

If a later observed discrepancy challenges the public-frame assumption, the
smallest focused diagnostic would retain source pose/IMU acquisition stamps
alongside packet TF and examine method-independent static geometry. It is not
implemented or required by this minimal revision. Finite sample spread alone
must not be promoted to an absolute error bound. No new calibration framework
is proposed.

## 5. Minimal operational-evidence sidecar proposal

Add data only to the operational layer, not A2:

| Field | Meaning |
| --- | --- |
| observation index, observation row, cell ID | Identify a real retained source return and its existing vote group |
| endpoint map XY (retain Z for height/class diagnostics) | Endpoint obtained with the original per-packet public TF |
| chunk index/stamp and transform reference | Reuse existing packet arrays; do not duplicate a TF per point |
| fixed AMBIGUOUS class / association reference | Same frozen classifier and static target reference |
| envelope profile/status and radius/components | Declared local spatial uncertainty and conditional scope; unavailable is not zero |
| complete observation-cell groups / legacy-missing cells | Account for every positive ambiguity vote without fabricating a point |

Store **all** usable AMBIGUOUS occupied endpoints per window, not just one
per-observation vote. Duplicate endpoints may share a single vote but all can
matter geometrically. Vote totals still count windows, never point count.
Use the existing NPZ/JSON boundary; no database, artifact locks, hashes or new
framework. A3 support and A5 exact checks receive the same sidecar through the
existing `OperationalEvidenceView` / `assess_footprint` seam.

Complete replay from existing raw windows is allowed only when the actual
reference, filtering, per-cell class-vote totals and all source rows are
available. Aggregated arrays alone are not recoverable provenance. An empty
sidecar for positive votes is a legacy-missing cell, never evidence of no
blocker. Retain missing historical groups through all subsequent updates.
An absent/unknown sensor profile similarly leaves the group's cell fallback
active. Lack of calibrated dynamic TF covariance alone does not silently
disable the explicitly conditional SIM profile.

This section describes the design checkpoint. Implementation progress and
regression outcomes are recorded separately in the development plan/report.

## 6. Moderate: all five exact winners

All poses are original map-frame validated winners. Dimensions are the common
1.04×0.78 m padded rectangle. Full four-corner coordinates, all 109 AMBIGUOUS
points, each per-winner distance and original row/chunk identities are in the
[read-only diagnostic JSON](../../../outputs/diagnostics/v13-design/moderate-endpoints.json).
Each table row plus the common dimensions fully specifies its footprint.

| Winner / source | Exact XY (m) | Yaw (rad) | Legacy ambiguous cells | Nearest endpoint | Minimum distance (mm) | Real support |
| --- | --- | ---: | --- | ---: | ---: | --- |
| 000008 / 585 | (2.566980839, -.508691927) | 2.094395102 | 781 | 58439 | 118.214917 | 0/107 |
| 000009 / 705 | (2.621884650, -.303788116) | -2.617993878 | 781 | 58439 | 98.214917 | 0/105 |
| 000010 / 584 | (2.543283861, -.519742009) | 1.919862177 | 781 | 58439 | 91.850299 | 0/106 |
| 000035 / 623 | (2.395562698, -.493694783) | .872664626 | 781 | 58439 | 106.091317 | 0/107 |
| 000067 / 622 | (2.351026946, -.430091138) | .349065850 | 780,781 | 58439 | 71.850299 | 0/106 |

These minima are identical when considering only old blocking-cell endpoints
or **all 109** retained ambiguous endpoints. This explicitly checks neighboring
cells, rather than assuming the old coarse subset is enough.

For **each row**, the proposed SIM envelope is `closed_disk(endpoint XY, .033)`
under the conditional engineering profile in §4. More generally, with a common radius rho,
AMBIGUOUS would be geometrically separated iff rho is **strictly smaller**
than that row's minimum; equality blocks. For per-endpoint radii, every one
of the 109 inequalities `rho_e < d(p_e,F_q)` must hold. These numbers are
diagnostic separation limits, **not recommended radii**.

Under the proposed33 mm disk, **all five are AMBIGUOUS-unblocked in the offline
geometric counterfactual**: their entire endpoint-envelope unions are separated.
Residual minimum clearances, respectively, are85.215,65.215,58.850,73.091 and
38.850 mm. With no ENVIRONMENT or target collision blocker these are operationally
unblocked, but **all remain unconfirmed** with the exact ground deficits above.
This does not modify recorded v1.2 states or establish a new method outcome.
With missing original endpoints/profile, the conservative cell fallback would
still block each row. The other29 winners' true expanded-target collisions
remain blocked regardless.

### Shared endpoint and target association

All five share row58439 at map
**(2.153377439257136, -.01053885251230817, .11410059833230446) m**.
Cell781 spans x=[2.151847184,2.251847184],
y=[-.107934378,-.007934378]. Its footprint overlap says nothing about the
location of that particular point within the cell.

The perceived brick has center (2.051847184,-.007934378,.059894679),
yaw .270567388 rad and known size (.240,.053,.115) m. Its existing expanded
half-size is (.166920941,.073420941,.104420941) m. Row58439 in brick-local
coordinates is (.097140404,-.029646549,.054205920) m: inside the expanded box,
but its transverse coordinate is about3.147 mm outside the nominal half-width.
It must not be declared TARGET merely from geometric proximity.

At image pixel(u,v)=(319,234), raw red mask=true, eroded interior=false.
Projected depth3.663906003m vs registered3.635059357m gives residual
.028846646m, within the unchanged pointwise allowance .046702316m; the
backprojected surface is inside the expanded target. The deliberate mask
boundary exclusion alone prevents positive TARGET association. This reason
and geometry apply independently to each of the five footprint cases.

Candidate000067 also overlaps cell780, containing these seven AMBIGUOUS
endpoints; all are included, not reduced to the shared point:

| Observation row | Map XYZ (m) | Pixel | Why positive association fails |
| ---: | --- | --- | --- |
| 23405 | (2.142664220,-.011534689,.114277572) | (319,234) | Interior mask false; depth match true |
| 44759 | (2.063273409,-.022975858,.114403331) | (321,238) | Registered depth absent |
| 58429 | (2.057633984,-.024283048,.114099537) | (321,238) | Registered depth absent |
| 58434 | (2.113505640,-.017219489,.114100138) | (320,235) | Interior mask false and depth absent |
| 61695 | (2.056937235,-.038851111,.114063107) | (323,238) | Interior mask false and depth absent |
| 71548 | (2.052981247,-.031263774,.113919614) | (322,238) | Interior mask false and depth absent |
| 75353 | (2.058230550,-.016351912,.113790969) | (320,238) | Registered depth absent |

All seven lie in the expanded target geometry but remain AMBIGUOUS. Their
individual distances and association budgets are retained in JSON. The
minimum for000067 is still row58439 in781, not an omitted point in780.

## 7. ENVIRONMENT scope diagnosis

Independent replay matched all occupied-class vote arrays in seven fresh
snapshots and two already recorded natural integration finals. ENVIRONMENT
blocks **zero** exact winners in every snapshot:

| Records | ENV blocked winners | Minimum ENV endpoint–any-footprint XY distance |
| --- | --- | --- |
| Moderate/Ours, first window | 0/34 | .686949727 m |
| Easy/Generic, all three windows | 0/34 each | .918414782 m |
| Easy/Ours, all three windows | 0/34 each | .937571779–.937599883 m |
| Natural integration finals01/02 | 0/33;0/37 | No occupied ENV endpoints in ROI |

Moderate has8,127 ENV occupied endpoints in33 cells. None provides a
same-cell alias veto against these winners. No ENV structural deadlock is
demonstrated, so **do not expand v1.3 to change ENVIRONMENT gating**. This
finding concerns observed geometry, not clearance of unobserved space.

## 8. Regression plan under updated autonomous-development authority

The following are planned acceptance tests, not claimed passed outcomes in
this design record. The later user update authorizes their implementation.

1. **Synthetic continuous geometry:** interior, edge/corner tangency and
   separated disk vs rotated exact footprint; sub-cell alias separation;
   neighboring-cell envelope reaches footprint; zero/large supported disks;
   disjoint and intersecting endpoints in one vote group must preserve the
   intersecting one. Exact anchor vs cell-center regression remains unchanged.
2. **Missing/partial history:** positive AMBIGUOUS counts with no endpoints,
   omitted old window, only some point rows or unknown sensor profile retain the
   cell fallback. Known endpoints still block independently. A later complete
   observation cannot erase an earlier incomplete one.
3. **Frozen semantics:** TARGET continuous collision, environment cell veto,
   ambiguous association labels and all four operational vote arrays unchanged;
   A2 raw arrays byte-for-byte unchanged. Later true ground votes cannot remove
   an intersecting/legacy-fallback ambiguous blocker. Nonintersecting ambiguity
   cannot supply any ground vote. Required real ground support is unchanged.
4. **Recorded Moderate:** replay all109 AMBIGUOUS endpoints and all34 exact
   winners. Test conditional geometry with the independently declared profile;
   no radius tuning to force clearance. For the fixed33 mm profile, verify the
   recorded five aliases clear and29 real target collisions still
   block and five current ground deficits still prevent confirmation. Preserve
   original v1.2 trial outcome. Missing-source/profile replay must remain blocked.
5. **Other recorded regression:** Hard-002/source624 exact geometry and87/88
   support, v1.1 target/environment/ambiguous fixtures, existing natural saved
   states. Diagnostic non-winner alternatives remain unused for selection.
6. **Shared A3/A5/A4:** same exact-pose operational assessment in A3 and A5;
   unchanged A1 relevance and M_nominal; only justified operational blocking
   affects M_operational. Generic/Ours same-state scores use identical candidate
   set, visibility, cost, budget, updater and gate. No score-driven radius.
7. **Natural A5 E2E:** after offline semantic and engineering-profile checks.
   It is a regression, not a fresh validation/formal slot.
   Existing initial pose, hover thresholds, support requirement and success
   criteria remain frozen. Do not promise handoff from a geometry-only test.

An additional residual risk must be watched, not silently fixed: the frozen
ground-vote rule suppresses a cell in any window containing AMBIGUOUS evidence.
If every physically available view repeatedly produces such a return in the
same required cell, confirmation could remain structurally impossible even
after sub-cell blocking improves. One recorded window does not prove that
condition. This revision does not change ground accumulation or assert sensing
will necessarily resolve it. The later autonomous-development authorization
permits diagnosing and iterating on a demonstrated local inconsistency without
another per-module approval barrier, while preserving old results/fairness.

## 9. Design-checkpoint verification and reproduction

At the initial design checkpoint, the only added code was a **read-only
arithmetic/association reporter**, with
three focused tests (analytic rotated-rectangle distance, genuine recorded
replay and mismatched saved votes). It does not implement disk gating, choose a radius, produce new A3/A4
states or alter outcomes. Output contains all original-source rows, complete
per-winner distance vectors and an explicit unavailable **total physical**
radius status. The proposed33 mm engineering profile is specified in this
design, not imputed as calibrated total physical accuracy in the reporter.

From the repository, write a new diagnostic report outside the source attempt:

```bash
PYTHONPATH=src MPLCONFIGDIR=/tmp/a6-mpl XDG_CACHE_HOME=/tmp/a6-cache \
  /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python \
  scripts/diagnose_v13_ambiguous_geometry.py \
  --data-dir outputs/a6/v12-fresh-validation/slot-003-moderate-ours-01/data \
  --output /tmp/v13-moderate-review.json
```

The output path must be unused; no source record is overwritten.
Independent source and ENVIRONMENT reviews were completed without edits or
new simulation runs. Independent design review requested removal of an
unnecessary whole-system certification prerequisite; this draft now explicitly
uses the inherited conditional engineering contract. It also required separating
supported endpoints from missing-source fallback and stating that legacy cell
fallback does not bound arbitrary physical misregistration. Those corrections
are incorporated. Reporter input mismatches are handled in the implementation
plan. None of these review findings authorizes changing historical results.
Subsequent implementation, development replay, integration repair and natural
regression are recorded in [the development checkpoint](../../V13_OPERATIONAL_GEOMETRY_CHECKPOINT.md).

### Source pointers (SIM remains clean at5e25039)

- AGENT `scripts/run_a5_sim.py:374`, `scripts/a5_ros_support.py:108`:
  stamped capture and row-preserving chunk re-expression.
- AGENT `src/operational_gating/{association,core,io}.py`:
  unchanged classifier, per-window votes, continuous target test and saved data.
- SIM `src/p450/prometheus_gazebo/gazebo_models/uav_models/p450_D435i_mid360/p450_D435i_mid360.sdf.jinja:440`:
  active pointcloud type2 and declared sensor settings.
- SIM `src/p450/livox_laser_gazebo_plugins/src/livox_points_plugin.cpp:152,476`:
  one ray update, range-to-XYZ and synthetic point timestamps.
- SIM `src/p450/livox_laser_gazebo_plugins/src/livox_ode_multiray_shape.cpp:145`:
  ODE contact-depth range path.
- SIM `src/p450/prometheus_uav_control/src/uav_estimator.cpp:469,612,640`:
  separately received position/attitude and publication-time TF stamp.
- SIM `src/platform/sim_platform_bringup/config/p450_mid360_tf_contract.yaml`:
  nominal composed mount and numeric comparison tolerances, not localization
  accuracy bounds.
