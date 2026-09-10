# Finite scan acquisition, shared by Generic and Ours

## Decision and evidence

Use the known sensor scan program, not a learned density or a new completion
objective. The installed SIM MID360 uses 800,000 directions, sequential packets
of 10,000 at 10 Hz. Exact ground-plane slopes need only 158,208 downward rays for
the configured mount. An offline 65-view projection took about 1.2–1.4 seconds
without occlusion, so a density fit is not justified.

The old Hard01 Ours second-window prediction has 18 false-positive support-union
cells. All 18 lack even a full-cycle nominal ground-plane ray. This is mainly a
scanned angular footprint mismatch, not an unlucky initial scan phase. Retain
phase marginalization to describe the finite window without a future-phase oracle.
The existing idealized model remains selectable for historical replay.

## Mathematical/geometric definition

For nominal level-hover viewpoint v, transform the scheduled sensor directions
through the configured mounting and UAV pose. Intersect downward rays with the
declared horizontal ground z, using the actual half-open A2 XY cell convention.
Reject out-of-range/out-of-grid intersections and segments intersecting a known
occupied prism. Test the actual ray endpoint, not the cell center. UNKNOWN remains
transparent; the 1 m prism is still an assumed occlusion surrogate, not measured
height. A sensor at/below ground or within a known prism is an invalid viewpoint.

Let B[j,c] say that packet j has at least one unoccluded nominal ground intersection
in cell c. For K packets and P packet phases,

    a(v,c) = (1/P) sum_s 1{ sum_{j=0..K-1} B[(s+j) mod P,c] > 0 }
    G_task(v) = (1-exp(-1/tau)) sum_c a(v,c) U_task(c)
    G_generic(v) = (1-exp(-1/tau)) sum_c a(v,c) unknown_score(c)

The existing flight penalty remains. a is a fraction of scheduled start phases,
not a calibrated return/success probability. No independence between rays/cells
is assumed. For windows spanning a cycle, opportunity saturates rather than
creating more than one observation vote.

The actual capture ends when last packet stamp minus first packet stamp reaches
the configured window. Therefore nominal K=ceil(window_s/packet_period_s)+1=51
for 5 s at 10 Hz. The existing 50-packet diagnostic is retained, and 50/51 sensitivity
will be reported. Dropped packets, non-level hover, drift, noise and unobserved
occluders can still produce mismatch; no missing return becomes evidence.

## API and implementation boundary

- `reachability_guided_nbv/finite_scan.py`: validated scan schedule, cached
  level-hover ground slopes, per-ray prism filtering, cyclic packet opportunity.
  Sensor directions are loaded from the installed platform asset, never scene GT.
- `rank_viewpoints(..., scan=None)`: optional shared finite-scan predictor. Keep
  binary `visibility` as nonzero opportunity for compatibility and save the float
  `observation_opportunity` actually used in gains. Save unoccluded opportunity
  separately so the existing no-occlusion ablation removes only occlusion.
- `A5Config.scan_pattern_path` and `scan_window_s`: explicit optional integration
  config, default None preserves the legacy model. The current task entry enables
  it and resolves the selected SIM's installed scan file. The cloud window is the
  single duration source passed to the worker; no copied independent budget.
- Output metadata names the scan schedule, packet horizon and nominal-pose/
  unknown-phase assumptions. A2/A3/raw votes and operational gate are not changed.

Do not edit production code while the start-1 baseline task is still running.
After it completes, implement and test before the next online start. Generic/Ours
share the entire predictor and candidates. Do not merge results across versions.

## Candidate and test scope

Initially retain the common 2 m XY lattice and existing yaw/cost/flight bounds.
Offline inspect whether a method-independent 1 m midpoint refinement offers
otherwise missing footprint support. Change it only if that demonstrates an
actual candidate-coverage limitation; document the decision before starts 2/3.
Do not select poses using future returns or scene outcomes.

Unit tests cover cyclic phases, horizon saturation, row-to-packet identity,
mount/yaw transforms, half-open grid boundaries, range, no downward ray, invalid
sensor origin, prism through/overflight, endpoint-vs-center occlusion, no mutation,
and exact Generic/Ours gain/common-component equality. Replay old Hard snapshots
without changing observations and compare planned vs actual support, including
pose drift and missed packets. Then run the bounded online schedule in
`docs/FINITE_SCAN_DEVELOPMENT.md`, preserving every outcome.

Local design is decided under the user's explicit autonomous authorization;
there is no new approval or evidence framework. No formal runs.

## Evidence-driven consistency extension before the new online runs

The exact per-ray prototype revealed a separate contradiction: Hard01 Generic
source623 requires a second real ground presence in cell781. That cell already
has one real ground vote and only TARGET/AMBIGUOUS occupied evidence, with complete
endpoint history; its exact footprint is operationally viable. A whole-cell
one-meter NBV prism predicts zero opportunity forever. Do not restore this removed
cell-aliasing approximation in the acquisition layer.

For finite mode with an operational context, use the existing perceived target's
expanded oriented 3D box; retain each complete AMBIGUOUS endpoint's existing
33 mm disk as a vertical cylinder from ground to the same assumed 1 m height.
Do not relabel ambiguity as target, estimate its height from a target, or clear
any vote. ENVIRONMENT and missing/unsupported ambiguity history retain full-cell
prisms. Unknown remains transparent. Geometric ray filtering applies this union;
a raw OCCUPIED cell is not separately culled. The no-occlusion variant omits this
geometry union only; its raw ground intersections are still predictions, not votes.
Sensor-origin checks use the same union. This is an acquisition-model consistency
change, not an operational gate or confirmation change. `operational_occlusion.py`
holds this small pure geometry function; shared ranking passes it to the predictor.

The independent 1 m lattice probe found joint-phase completing opportunities in
both Hard02 recordings absent from their original lattice; it still found none
for Hard01. Enable the common 1 m XY lattice within the unchanged ±2 m extent,
flight bounds and altitude. Both methods receive the same superset; development
candidate IDs/order can differ from old records and are logged explicitly. This
is not a success claim or a change to the three-window budget. Do not introduce
sub-0.1 m pose search or a tuned completion objective.
