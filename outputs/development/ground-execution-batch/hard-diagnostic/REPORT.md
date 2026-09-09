# Historical fresh Hard: predicted views versus measured ground support

2026-09-09. Offline diagnosis of operational-batch launch10 Generic and launch12
Ours, each retaining its original three observation windows and VALID failure.
No online processes, new sensor observations, ground-truth scene geometry,
reselection, additional votes, or method/configuration changes were used.

The mechanisms differ. Generic's final gaps are predominantly measured
occlusion already represented in its visibility prediction. Ours' best two
winners miss their second supporting observation near the lower boundary of
the actual MID360 angular scan pattern. The uniform nominal elevation envelope
does not guarantee a scheduled ray into those cells. All their missing cells
receive real ground returns in window 3, too late to satisfy two distinct
supporting windows within n=3.

## Inputs and verification

[diagnostic.json](diagnostic.json) contains all per-window measurements, every
final missing cell of every viable exact winner, selected-mask identities,
foreground endpoint witnesses and packet phases.
The producing script is
[diagnose_hard_nbv_ground.py](../../../../scripts/diagnose_hard_nbv_ground.py).

All six cumulative presence arrays independently reproduce the originals
exactly from the retained endpoints, A2's strict range filter, the original
half-open XY cells, and the original ground-height interval [-0.02, 0.02] m.
All four selected visibility masks reproduce exactly with the existing A4
implementation. Every actual ground cell has a corresponding angular
opportunity in the conservative sensor-ray check below. No lost-vote or
coordinate-reexpression defect is found.

Both saved runtime logs identify the installed CSV, 800,000 scan rows,
10,000 rows/packet, downsample=1, and output type=2. Its SHA256 matches the
frozen SIM asset manifest:
`aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a`.
The existing plugin maps CSV azimuth directly and elevation to
90 degrees minus CSV zenith. The plugin source and sensor model have no
changes between the batch's SIM 5e25039 and the inspected checkout.

Each of the 308 captured packets has one uniquely identifiable 10,000-row
phase. Every one of the 529,371 retained endpoint directions matches its
inferred packet, with maximum angular residual < 0.0000033 degrees. This
allows inspection of scheduled ray directions, including directions whose
return was not retained. Phase is inferred and verified from observations;
it was not logged directly. Scheduled rays are used only to diagnose angular
opportunity, never as endpoints or support votes.

## Prediction and actual next window

Only windows 2 and 3 have a preceding saved selected-NBV prediction. Window 1
is the initial observation, not a prediction/next-measurement pair. “Actual
inside prediction” counts cells with at least one accepted ground endpoint
in that window. A cumulative presence vote is at most one per cell/window.

| Run/window | Selected view | Predicted cells | Actual inside prediction | Predicted without ground | All actual ground cells | Cumulative presence vote sum | Cells with presence >=2 |
|---|---|---:|---:|---:|---:|---:|---:|
| Generic 1 | Initial | — | — | — | 467 | 467 | 0 |
| Generic 2 | Candidate 1 | 906 | 891 | 15 | 1049 | 1516 | 236 |
| Generic 3 | Candidate 0 | 897 | 891 | 6 | 1069 | 2585 | 1062 |
| Ours 1 | Initial | — | — | — | 383 | 383 | 0 |
| Ours 2 | Candidate 26 | 615 | 582 | 33 | 754 | 1137 | 280 |
| Ours 3 | Candidate 0 | 629 | 621 | 8 | 790 | 1927 | 763 |

For the union of the five final viable footprints, Generic predicts 116/110
cells in windows 2/3 and actually observes 116/109 of those. Its union has
169 cells, with cumulative supported counts 0/16/132. Ours predicts 143/149
cells and actually observes 125/146 of those. Its union has 163 cells, with
cumulative supported counts 0/22/130. Overlapping footprints are counted
once in these union values.

Both policies request zero displacement at the second decision: candidate 0
equals the preceding `ranking.current`, with flight cost 0. The execution
events nevertheless have `A5_VIEWPOINT rescan=false` for both later calls:
the adapter performed a corrective flight against the current hover state.
Thus each policy requests one changed viewpoint and one same-pose rescan,
while the existing metric records two executed sensing flight actions. These
are different definitions, not evidence that the historical action count is
wrong. Final capture anchors differ from the requested pose by 0.03210 m
(Generic) and 0.03639 m (Ours); the windows then retain actual per-packet TF.

## Every viable exact winner

Triples are windows 1/2/3; pairs are predictions for windows 2/3. Footprints
are the exact original winners and can differ across the two runtime
perception instances. No non-winner substitution is evaluated here.

| Run/source | Covered cells | Predicted visible cells, 2/3 | Actual ground cells, 1/2/3 | Cumulative supported cells, 1/2/3 | Final missing cells: never/once |
|---|---:|---|---|---|---|
| Generic 583 | 108 | 75/70 | 26/89/88 | 0/16/88 | 9/11 |
| Generic 623 | 105 | 66/62 | 32/79/78 | 0/14/78 | 8/19 |
| Generic 541 | 103 | 77/74 | 17/87/87 | 0/7/87 | 6/10 |
| Generic 501 | 108 | 79/76 | 17/89/89 | 0/7/89 | 9/10 |
| Generic 580 | 101 | 64/60 | 15/76/77 | 0/6/76 | 15/10 |
| Ours 583 | 108 | 107/106 | 20/103/108 | 0/20/103 | 0/5 |
| Ours 543 | 107 | 104/104 | 15/102/107 | 0/15/102 | 0/5 |
| Ours 501 | 101 | 98/100 | 12/90/100 | 0/12/90 | 1/10 |
| Ours 541 | 106 | 96/100 | 12/85/101 | 0/12/85 | 5/16 |
| Ours 580 | 101 | 88/93 | 12/76/94 | 0/12/76 | 7/18 |

Generic's union has 37 final missing cells: 17 never observed and 20 observed
once. In both windows 2 and 3, 36 of these 37 have measured foreground
returns along rays whose continuation to z=0 enters the missing cell. They
are also excluded by the existing occupied-prism visibility model. These
are actual interception witnesses, not GT box intersections. For example,
window 2 row 697 in packet 0 has sensor origin
(-3.318063, -1.927256, 1.614198) m and measured endpoint
(0.425000, -0.663533, 0.423816) m. Its ray reaches the ground plane at
(1.757657, -0.213605, 0) m, inside missing cell 657.

The sole other Generic gap, cell 508, has 0/4/0 accepted ground points.
Window 3 predicts it visible, but its inferred packet schedule has no ray
intersecting that cell at z=0. The wider ground-height-band bounding check
still has possible opportunities, so this is a sampled-plane gap rather
than proof of no possible return anywhere in the accepted height band.
Generic's high global prediction hit rate therefore does not imply that
any complete viable footprint was observed twice.

## Ours: scan-pattern boundary and late support

The A4 sensor envelope allows -7 to 52 degrees at every azimuth. The complete
frozen scan pattern has an azimuth-dependent lower boundary: minimum
elevations are -5.8132 degrees over azimuth [-30,-20), -5.3357 over [-20,-10),
and -4.8642 over [-10,0). The same minima recur in the recorded return
directions across these windows. This is concrete evidence that the nominal
rectangular angular envelope includes directions not supplied by the scan
pattern. Finite packet sampling adds further gaps.

All 33 of Ours' unique final missing cells have zero scheduled rays into
their z=0 cell in window 2, and no measured foreground ray interceptions
there. Eighteen of these cells were nevertheless predicted visible. The
two best winners' exact deficits are:

| Cell | Winner(s) | Predicted visible in window 2 | Window-2 center FOV chunks /52 | Window-2 scheduled z=0 rays | Window-2 conservative height-band opportunities | Actual ground points, 1/2/3 |
|---|---|---|---:|---:|---:|---|
| 417 | 583 | Yes | 13 | 0 | 0 | 0/0/16 |
| 418 | 583 | Yes | 45 | 0 | 0 | 0/0/21 |
| 456 | 543 | No | 11 | 0 | 0 | 0/0/9 |
| 457 | 583,543 | Yes | 37 | 0 | 0 | 0/0/28 |
| 458 | 583,543 | Yes | 52 | 0 | 4 | 0/0/35 |
| 496 | 543 | Yes | 30 | 0 | 0 | 0/0/26 |
| 497 | 583,543 | Yes | 52 | 0 | 1 | 0/0/27 |

The height-band check extends each scheduled ray through z=[-0.02,0.02] m
and conservatively bounds that segment in XY. A zero count excludes an
accepted-height endpoint in that cell at the saved packet poses even under
this wider bound. Positive counts are only possible intersections and do
not imply a return. No such computed quantity is added to presence.

All seven cells have their centers inside the nominal range and FOV for
every packet of window 3. Recorded sensor height ranges fall from
1.52340–1.60754 m in window 2 to 1.45390–1.48713 m in window 3, with smaller
position/attitude changes also represented by TF. Their directions move
into sampled coverage and real returns arrive. Source 583 reaches 108/108
cells with ground in window 3, but only 103/108 with two supporting windows;
source 543 reaches 107/107 and only 102/107 respectively. The final five
missing support cells per winner are therefore consistent with measured
acquisition history, not suppressed evidence.

## What this establishes, and its limits

- Range exclusion does not explain final gaps: all final-missing cell centers
  remain within 3.1256–6.6038 m for Generic and 3.1115–3.8746 m for Ours,
  inside the unchanged 0.2–40 m interval.
- Nominal center FOV alone does not explain prediction misses: every
  predicted-but-unobserved cell has a center in FOV in at least one actual
  packet. Partial-window exposure and the scan's angular pattern matter.
- For all-grid predicted misses, scheduled z=0 ray counts are zero in
  14/15 and 5/6 cells for Generic windows 2/3; the remaining one per window
  has a retained ray whose endpoint does not provide ground in that same
  cell. Ours has zero scheduled z=0 rays in all 33/33 and 8/8 such cells;
  21/33 and 3/8 also have zero conservative height-band opportunities.
- Cell-center visibility and the fixed one-informative-endpoint gain
  surrogate are not calibrated probabilities. A3/A4 still score uncertainty
  over partial footprints, including already supported cells. A high score
  does not imply that every missing footprint cell can obtain the required
  second observation within the remaining budget.
- Map projection matches original voting exactly; per-packet reexpression
  round-trip errors are below 2.2e-14 m. These checks establish internal
  consistency, not calibrated TF or motion-distortion accuracy. Some actual
  ground endpoints and their z=0 ray projections fall in adjacent cells;
  none of those projections replaces the endpoint's original cell.
- Measured foreground witnesses establish interception of those sampled
  rays, not complete-cell occlusion or obstacle identity. No GT geometry or
  missing-point imputation was used. Within-packet acquisition times are
  unavailable, and future views or rescans remain unobserved.

No rescue or new success is claimed. This diagnosis supports future work on
the sensor opportunity model and the relation between partial information
gain and repeated full-footprint support. It does not justify changing the
three-window budget, relaxing support/collision gates, or rewriting these
historical failures.

Reproduce into a new directory (the script refuses an existing output):

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/hard-nbv-mpl \
XDG_CACHE_HOME=/tmp/hard-nbv-cache PYTHONPATH=src:scripts \
python scripts/diagnose_hard_nbv_ground.py \
  --source-dir outputs/development/operational-batch \
  --output-dir /tmp/hard-nbv-recheck-new \
  --scan-pattern /media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation/install/p450-clean/share/sim_platform_assets/models/MID360/scan_mode/mid360.csv
```

The CSV is an existing local sensor asset and is not copied into this report.
Omitting `--scan-pattern` retains the measured-endpoint, visibility, TF and
presence checks, without inferred scheduled-angle diagnostics.
