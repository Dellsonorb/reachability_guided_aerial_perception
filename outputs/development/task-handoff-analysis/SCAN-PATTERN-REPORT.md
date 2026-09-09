# First-two-window Hard scan-pattern diagnosis

2026-09-10. Offline extension of the saved-footprint opportunity report. It uses
the same four recordings, existing diagnostic helpers, and the installed
MID360 CSV accepted by the existing reader's hash check. No runtime change,
new observation, mapper update, added vote, GT geometry, or candidate
reselection. Existing reports are preserved.

The mechanisms differ. Hard01 Ours has concrete recorded sampling gaps inside
its nominally visible footprint region. For the other three runs, every
missing-ground cell in the viable-footprint union in window two has a measured
foreground interception witness. Replacing the score alone would not address
both causes.

## Reproduction and checks

[scan_pattern_diagnosis.py](scan_pattern_diagnosis.py) reuses
`scripts/diagnose_hard_nbv_ground.py`; [scan-pattern.json](scan-pattern.json)
contains window summaries, exact winner counts and individual zero-presence
cells. [scan-band-traces.json](scan-band-traces.json) resolves the seven
positive-band/no-foreground residual cells described below.

From the repository root, choose new filenames when reproducing; the script
refuses existing outputs and locations outside this analysis directory:

```sh
MPLCONFIGDIR=/tmp/matplotlib-task-handoff XDG_CACHE_HOME=/tmp/task-handoff-cache python outputs/development/task-handoff-analysis/scan_pattern_diagnosis.py --output outputs/development/task-handoff-analysis/scan-pattern-reproduced.json
MPLCONFIGDIR=/tmp/matplotlib-task-handoff XDG_CACHE_HOME=/tmp/task-handoff-cache python outputs/development/task-handoff-analysis/scan_pattern_diagnosis.py --trace-positive-band --output outputs/development/task-handoff-analysis/scan-band-traces-reproduced.json
```

All 407 packets and 698,036 retained return directions validate against their
uniquely inferred 10,000-row CSV packet, with maximum angular discrepancy
0.000003201 degrees. All eight recorded cumulative presence arrays reproduce
exactly from actual endpoints. All four selected visibility masks reproduce.
Every actual ground cell has a conservative scheduled accepted-height-band
opportunity. No packet-phase or lost-presence defect was found.

Each window lasts approximately 5.02 seconds and contains 50 or 51 unique
packet phases out of the CSV's 80. This does not evaluate unseen phases or
predict a longer window. Packet phase is inferred and verified against returns,
not logged directly.

## Missing ground across viable exact footprints

Cells in overlapping viable footprints are counted once. The accepted ground
height band is [-0.02, 0.02] m. A “band opportunity” initially means overlap of
the conservative XY bounding box of the scheduled ray's segment through this
band; it is not a return. A foreground witness is a recorded endpoint above
the ground obstacle threshold whose measured ray, extended to the declared
ground plane, enters the cell. Such a witness demonstrates an intercepted
ray, not complete-cell occlusion.

| Run | Window | Union cells | Missing actual ground | No scheduled band opportunity | Foreground witness | Positive band without foreground witness |
|---|---:|---:|---:|---:|---:|---:|
| 05 Hard01 Generic | 1 | 168 | 131 | 18 | 108 | 5 |
| 05 Hard01 Generic | 2 | 168 | 27 | 0 | 27 | 0 |
| 06 Hard01 Ours | 1 | 174 | 142 | 35 | 104 | 3 |
| 06 Hard01 Ours | 2 | 174 | 32 | 24 | 1 | 7 |
| 11 Hard02 Ours | 1 | 172 | 153 | 44 | 103 | 6 |
| 11 Hard02 Ours | 2 | 172 | 15 | 0 | 15 | 0 |
| 12 Hard02 Generic | 1 | 170 | 155 | 47 | 103 | 5 |
| 12 Hard02 Generic | 2 | 170 | 8 | 0 | 8 | 0 |

All window-two union misses in slots 05/11/12 were excluded by the saved
selected visibility mask. All predicted union cells were actually observed
as ground in those windows. These runs therefore have measured interception
evidence consistent with known occlusion, rather than a nominally visible
region whose predicted ground returns disappeared.

The unions contain 5/32/10/7 cells with zero presence after two windows for
slots 05/06/11/12. All 5/10/7 such cells in slots 05/11/12 have window-two
foreground witnesses and were excluded by the prediction. In their first
window, 3/9/6 respectively had no scheduled band opportunity. The same cell
can encounter different acquisition limits in successive views.

## Hard01 Ours: nominal visibility versus acquired cell coverage

Of 150 predicted-visible union cells in window two, 18 receive no ground
endpoint. All 18 have their centers inside nominal range/FOV in at least one
recorded packet, and none has a foreground witness in that window.

All 18 have **zero scheduled ray intersections with their cell at the declared
z=0 plane**. Eleven also have zero opportunity anywhere in the conservative
accepted-height-band bound. Seven retain 1–4 positive band opportunities.
Their centers can satisfy the nominal elevation envelope while the finite
scheduled rays still provide no acquired ground endpoint in the cell.

The seven positive-band cases were checked further using the existing closed
segment/AABB helper on the cell's accepted-height prism. All 15 intersections
remain valid under that check. Each ray matches a retained measured endpoint;
every endpoint is actual ground in an adjacent cell, rather than the deficit
cell:

| Deficit cell | Intersecting band rays | Cells containing their actual ground endpoints |
|---:|---:|---|
| 301 | 2 | 302 |
| 380 | 4 | 381, 420 |
| 419 | 1 | 420 |
| 498 | 3 | 499, 539 |
| 537 | 1 | 538 |
| 616 | 3 | 617, 657 |
| 655 | 1 | 656 |

Cell 419 is the sole zero-presence cell of the closest winner, source 623 /
`candidate-000004` (103 cells). In window one it has 18 foreground witnesses.
In window two its center is nominally in FOV for all 51 packets, at elevations
-6.9923 to -5.8024 degrees; the planned elevation was -6.2795 degrees. The sole
scheduled band-crossing ray is packet 47, CSV row 269531. Its actual ground
endpoint is `(1.984501571, -0.800803392, 0.000132972)` m, in cell 420. Thus the
weak positive band count does not represent a missed mapper vote for cell 419.
It is an actual ray that crosses the accepted-height volume but reaches the
measured ground in a neighboring cell.

The other 14 union misses excluded by prediction comprise 13 no-band cases
and one foreground case. Ground density alone would conceal these different
relationships to the prediction.

## Implication and limits

This directly strengthens the observation-model explanation for Hard01 Ours:
its chosen view belongs to the only ideal completing pair in the earlier
analysis, but nominal center visibility overstates acquired per-cell support.
The repeated-support bottleneck is supported by actual endpoint history;
neither a manufactured vote nor a score adjustment is justified by these
measurements.

For slots 05/11/12, window-two misses instead have positive scheduled rays
and measured interception witnesses, already excluded by the current mask.
Their first frozen candidate inventories also lack an ideal completing pair.
This keeps candidate coverage and whole-footprint completion relevant, without
proving that a hypothetical replanned sequence would fail or succeed.

The next engineering question is whether the shared predictor/candidate
geometry can represent usable, repeated footprint coverage using the actual
sensor schedule. A correction should apply identically to Generic and Ours.
These finite recordings do not separate permanent angular-envelope limits
from missing packet phases, and do not calibrate a probability of observing a
whole footprint. They do not justify a density threshold, longer budget, gain
tuning, or guaranteed counterfactual success.

All ray geometry uses recorded per-packet public TF, with no within-packet
deskew or calibrated pose-error bound. Scheduled intersections are conditional
geometric diagnostics, not geometric truth or inferred ground existence.
Foreground witnesses are not exhaustive occlusion classifications. No new
live sensing is required to reproduce these findings.
