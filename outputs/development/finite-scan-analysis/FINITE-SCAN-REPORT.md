# Finite scan opportunity: detached development diagnosis

2026-09-10. All data are the first two saved windows of slots 05, 06, 11 and 12
under `outputs/development/multiscene-paired`. No Gazebo launch, new endpoint,
belief update, vote, scene GT, selected historical pose, or original outcome is
changed by these diagnostics.

## Model and platform audit

The installed Livox plugin advances 10,000 CSV rows per callback modulo 800,000;
the installed sensor rate is 10 Hz. There are 80 packet phases per eight-second
cycle. A capture stopping at last-stamp minus first-stamp >= 5 s needs 51 nominal
packets. One old window contains a dropped packet; future loss is not modeled.

The publisher additionally rejects angular indices outside its configured bins.
Its `roundf` test accepts 799,252 rows and rejects 748, preserving original row and
packet identities. The CSV spans elevations [-7.2123, 52.164] degrees. The installed
SDF has 100 horizontal samples over [0, 6.2831852] rad and 360 vertical samples over
[-0.925024488, 0.122173046] rad. Production reads these from the supplied SDF and
checks packet size/rate rather than applying the old center-FOV envelope.

For a nominal level-hover pose, intersect emitted downward rays with the declared
ground plane. Enforce the existing strict range and half-open XY cells; test each
actual ray segment against existing known occupied prisms before binning packet
hits. For each of 80 cyclic starts, OR the next 51 packet hits. Their mean is a
fraction of scan starts with opportunity, not a calibrated return probability.
Unknown occluders, pose drift, dropout and within-packet motion are omitted.

Cached body-frame plane slopes avoid repeated trigonometry. A full cycle is
projected once per view; cyclic accumulation is over the small packet/cell array,
so the computation does not multiply 80 phases by 800,000 rays per candidate.
The original raw-cycle cache had 158,208 downward rays before publisher screening.

## Recorded comparison

All counts below use the second-state union of viable exact footprints; cells in
overlapping footprints are counted once. “FP/FN” compare a nonzero opportunity
mask with actual ground presence in that window, not guaranteed future sensing.
All four second-window masks from the old predictor reproduce exactly.

| Run / second window | Union cells | Actual ground | Old center+prism FP/FN | 51-packet actual-ray+prism FP/FN | Fraction-weighted missing actual ground |
|---|---:|---:|---:|---:|---:|
| 05 Hard01 Generic | 168 | 141 | 0 / 27 | 0 / 21 | 21.4000 |
| 06 Hard01 Ours | 174 | 142 | 18 / 10 | 0 / 9 | 9.7250 |
| 11 Hard02 Ours | 172 | 157 | 0 / 13 | 0 / 9 | 10.0875 |
| 12 Hard02 Generic | 170 | 162 | 0 / 18 | 0 / 10 | 11.5375 |

Across the full 1,600-cell grids, corresponding new FP/FN counts are 13/151,
0/168, 15/145 and 13/136. The correction is not perfect. Known one-metre prisms
remain an assumed surrogate and block many cells that have real ground returns.
The second-window sums of production phase opportunity reproduce the detached
implementation: 905.0125, 572.1875, 950.0625 and 949.375.

The original 18 Hard01 Ours missing-visible union cells all have zero nominal
ground-plane rays even over the entire raw 800,000-row cycle, including cell 419.
Thus their specific mismatch is scanned angular footprint geometry, not simply
the unlucky omission of some initial phases. The phase treatment remains useful
for other boundary cells and avoids a future scan-phase oracle.

The preserved first prototype uses 50 packets and all raw CSV directions, matching
the earlier diagnostic convention. It also re-infers and checks every packet's
retained directions, compares all eight windows, and compares nominal with actual
packet transforms. It is explicitly separate from the corrected 51-packet,
publisher-filtered production comparison.

At union scope, nominal observed-phase versus actual-transform scan-only masks
differ by 8/6/7/3 cells in first windows and 0/3/0/0 in second windows. Their
actual-transform scheduled-plane opportunity has no false negative in these
unions. This does not remove real foreground interceptions. The old recordings'
maximum base position error from requested pose is 0.102/0.136/0.111/0.128 m in
first windows and 0.094/0.077/0.076/0.074 m in second windows. These observed
ranges do not constitute a calibrated future tracking bound.

## Candidate and whole-footprint limits

The first-state 65-view, 2 m lattice has no ideal completing pair under nonzero
finite opportunities in any of the four recordings. Minimum remaining cells are
1/2/7/4. Adding common 1 m midpoints gives 200 views, with minimum remaining cells
1/2/0/0. The refined Hard02 inventories contain actual joint phase opportunities,
not merely individually positive cells:

- Hard02 Ours: one view pair (same new midpoint twice). For exact sources 582 and
  501, respectively 2,500 and 6,241 of the 6,400 enumerated phase-start pairs meet
  the remaining support requirement geometrically.
- Hard02 Generic: four view pairs. A repeated midpoint completes exact source
  541 for all 6,400 phase pairs and source 623 for 3,456. Other feasible pair
  counts are 3,840, 3,840 and 2,304 for source 541.

These are combinatorial phase-pair counts conditional on frozen geometry. The
two future phases need not be independent or arbitrarily realizable in time.
They do not predict task success or justify changed physical gates.

The best Hard01 Generic pair lacks cell 781: raw A2 OCCUPIED, one real ground
presence, target=1/ambiguous=1/environment=0, scan-only opportunity=1 but current
full-cell prism opportunity=0. The best Hard01 Ours pair lacks cells 419 and 459:
both UNKNOWN with zero initial presence and zero scan-only opportunity. Different
mechanisms persist even in the same scene family.

## Recorded subcell mismatch

Cell 781's four first-window ambiguous endpoints in Hard01 Generic occupy a
33 mm-expanded bounding box covering 39.93% of that cell. The five endpoints in
Hard01 Ours cover 39.55%; after two windows the latter cell has two real ground
presences despite remaining raw OCCUPIED. All those endpoint centers are inside
the existing expanded target OBB. Its declared top is about 0.1645 m, while NBV
uses the one-metre full-cell prism. Other viable raw-occupied Hard01 Ours cells
778/779/780 have complete ambiguous history and bounding-box fractions of
24.10/60.72/76.50% in the first window. All have environment=0.

This is evidence to review the shared prediction geometry separately. It does not
authorize inventing support, reclassifying ambiguous returns as target, or
relaxing the independent operational gate. Endpoint bounding boxes enclose the
existing disks; they are not bounds on all unseen geometry. Original sidecar XY
coordinates reproduce their actual retained observation rows exactly.

## Efficiency and artifacts

On the prescribed Python environment with one BLAS thread, the production model
takes 1.94–2.02 s for the original 65 views and 6.18–6.35 s for 200 midpoint views,
including actual-ray prism filtering. The raw scan-only prototype takes
1.17–1.39 s per 65 views. The compact 80-by-1,600 boolean packet hits occupy
128,000 bytes per prediction. These are local offline timings, not online hover
time guarantees.

- `finite_scan_probe.py` / `finite-scan-probe.json`: preserved 50-packet/raw-CSV
  comparison of all eight windows, phase reconstruction and nominal/moving rays.
- `prism_scan_probe.py` / `prism-scan-probe.json`: corrected publisher mask,
  51-packet ray-level known-prism comparison, pose drift and midpoint inventory.
- `completion_phases_probe.py` / `completion-phases-probe.json`: actual production
  model, joint-cell/joint-phase footprint arithmetic and classified residuals.
- `endpoint_extent_probe.py` / `endpoint-extents-probe.json`: exact retained-row
  verification, target geometry and ambiguous endpoint extents.

Numerical production code/tests were committed separately as `a0fbceb` and
`28bd5fc`; the latter adds explicit model metadata and corrects decimal horizon
roundoff. Twenty-one focused tests and 48 combined finite/legacy NBV tests pass.
The diagnostic scripts refuse existing output filenames and restrict results to
this directory. They never save generated endpoints or support into runtime data.
