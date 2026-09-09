# Saved Hard footprint opportunities and shared handoff review

2026-09-10. Development diagnosis only. Source: the four unchanged Hard runs
in `outputs/development/multiscene-paired`, slots 05, 06, 11 and 12. No online
start, Gazebo ground truth, mapper update, fabricated observation, runtime
candidate substitution, gain change, or historical outcome change was used.

The saved first-window model permits completing a footprint within two more
windows only for Hard01 Ours. Its selected view already participates in that
opportunity. After the actual second window, none of the four saved candidate
sets has an ideal one-window completion. These runs do not establish that
changing the mass objective alone would produce a successful retrieval.

## Reproduce and interpretation

Run from the repository root:

```sh
MPLCONFIGDIR=/tmp/matplotlib-task-handoff XDG_CACHE_HOME=/tmp/task-handoff-cache /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python outputs/development/task-handoff-analysis/diagnose_opportunities.py --summary
```

Omit `--summary` for source paths, exact winner IDs and poses, recorded
presence histograms, example missing cell IDs, view poses and opportunity
counts. The script writes JSON to stdout only. It reproduces all **376 saved
candidate visibility masks and 44 viable exact footprint cell sets** across
the eight snapshots, and checks their recorded ground deficit counts. All
checks passed. Saved `decision.next_viewpoint` also
matches each recorded policy choice. Input counts are the recorded
`ground_presence_votes`, not A2 observation counts or suppressed `ground_votes`.

For each current exact winner with no representative or operational blocker
and no clipped footprint, the arithmetic condition is

`g(c) + sum_j V_j(c) >= 2` for **every** covered cell `c`.

Here `g` is measured presence at the snapshot and each `V_j` is one existing
saved candidate visibility mask. First-window analysis enumerates all ordered
pairs, including repetition, from that snapshot's 65 valid views. Second-window
analysis uses the actual updated presence counts and its newly enumerated
views. Geometry and blocking are frozen inside each calculation. No ideal
increment is put into an evidence array or fed back to an algorithm.

This is an optimistic arithmetic inventory under a specified model, not a
return simulation or universal physical upper bound. Actual endpoints can
occur outside a cell-center visibility mask, and actual visible cells can fail
to receive ground endpoints. A positive result does not demonstrate acquisition,
navigation, manipulation, or retrieval. A zero result excludes completion only
within the stated snapshot, masks, winners, and window accounting.

## Opportunity counts

“Minimum missing” is the smallest remaining count of cells with fewer than two
opportunities, for any viable winner and any enumerated choice. “Selected
first” fixes the saved next view and optimizes only a second mask when two
windows remain. It does not reselect a runtime action.

| Saved run | After window | Viable winners | Views | Completing choices / examined | Minimum missing | Selected first minimum missing |
|---|---:|---:|---:|---:|---:|---:|
| 05 Hard01 Generic | 1 | 5 | 65 | 0 / 4,225 pairs | 1 | 11 |
| 05 Hard01 Generic | 2 | 5 | 25 | 0 / 25 views | 3 | 22 |
| 06 Hard01 Ours | 1 | 6 | 65 | 4 / 4,225 pairs | 0 | 0 |
| 06 Hard01 Ours | 2 | 6 | 41 | 0 / 41 views | 2 | 2 |
| 11 Hard02 Ours | 1 | 6 | 65 | 0 / 4,225 pairs | 8 | 9 |
| 11 Hard02 Ours | 2 | 6 | 25 | 0 / 25 views | 2 | 5 |
| 12 Hard02 Generic | 1 | 5 | 65 | 0 / 4,225 pairs | 7 | 9 |
| 12 Hard02 Generic | 2 | 5 | 25 | 0 / 25 views | 3 | 12 |

The four Hard01 Ours pairs are `(25,25)`, `(25,26)`, `(26,25)`, and `(26,26)`.
Each completes the same three winners: source/candidate 583/`candidate-000000`
(108 cells), 623/`candidate-000004` (103), and 542/`candidate-000009` (106).
The existing first choice is view 26. Views 25 and 26 share XYZ
`(-1.418536531, -2.094508377, 1.296205353)` m and have yaw approximately
`-0.010990577` and `0.774407586` rad, respectively.

These pair counts do not assert executable sequences: the cross pairs change
only yaw, which the existing facade-aware candidate filter excludes after an
ideal arrival at that XYZ. The two repeated-view pairs retain the positive
arithmetic result without requiring that yaw-only transition. Subsequent
runtime candidates would still use actual capture poses and updated beliefs.

For comparison, Hard01 Generic's `(25,25)` leaves cell 738 unsupported in each
of three closest footprints (sources 584, 624, 542). Every missing cell need not
be globally invisible: some cells are visible from different masks, but the
zero-presence cells need coverage in both remaining windows. Taking the union
of all candidate visibility masks would therefore overstate attainability.

## Actual second-window deficits

The minimum zero-presence count among the viable footprints after window two
is 3, 1, 2, and 0 for slots 05, 06, 11, and 12 respectively. In slots 05, 06,
and 11, **every** current viable footprint contains at least one cell never
observed as ground. Even arbitrary complete visibility in the last window
cannot give such a cell two distinct-window presence votes. This stronger
statement depends on the fixed current winners and two-vote rule, not on the
candidate lattice or visibility model.

Slot 12 has one exception to that zero-presence barrier: source 541,
`candidate-000011`, has 103 covered cells with recorded histogram
`[0, 95, 8, 0]` for zero/one/two/three votes. It needs the remaining window to
cover all 95 one-vote cells. None of its 25 saved views does so. The closest
mask is view 9 and still misses cells 497, 537, and 577. The actual next choice
is view 0, which leaves 12 cells deficient in the ideal arithmetic. The final
real footprint nevertheless has only one deficient cell, another concrete
reason not to interpret the cell-center model as a physical upper bound.

For Hard01 Ours source 623, the first saved chosen mask covers all 103 footprint
cells, but the recorded second window has actual ground in 102; cell 419 still
has zero presence. The next saved stay mask would leave cells 419 and 738
deficient under the arithmetic. In the real third window all 103 cells receive
ground, and only cell 419 remains below two votes. The initial optimistic
opportunity and subsequent missing support are consistent with imperfect
acquisition and changed visibility, without requiring a ranking defect.

The pre-existing [recorded endpoint diagnosis](../multiscene-paired-analysis/hard/diagnostic.json)
independently reproduces presence from all saved sensor windows. For the union
of viable footprints, predicted/actually observed-inside-prediction cells in
window two are 114/114, 150/132, 144/144, and 144/144 for slots 05/06/11/12.
For Hard02 Ours window three the corresponding values are 142/105. Counts of
observed cells and repeated support are different quantities. This report
does not assign an unseen scan outcome or newly diagnose a specific sensor
failure from these aggregate values.

## Mass objective versus footprint completion

The implemented A4 objective sums `V * U_task`, with per-cell operational
relevance and an observation-count uncertainty decay, then subtracts flight
cost. It contains neither a per-winner all-cells completion condition nor the
actual remaining ground-presence deficit. Overlapping footprints share cell
mass, and useful reductions can accumulate while every footprint retains a
small unsupported portion. The new handoff condition can correct continued
sensing after readiness; it cannot make this objective a completion objective.

There is a concrete progress tradeoff even within Ours: in Hard02 window two,
selected view 10 has task score 33.577995, slightly above view 9 at 33.325239.
For source 501/`candidate-000011`, view 10 leaves five deficient cells under the
ideal calculation while view 9 leaves two. Both fail the completion condition.
In Hard01 Ours, selected views 26 then 0 already have the best completion
opportunity/progress in their respective saved snapshots. In the other three
first snapshots no static pair succeeds at all.

Thus footprint completion is a distinct task objective and a meaningful
diagnostic, but these four failures provide no completing alternative that a
different score demonstrably overlooked. They do not justify tuning a gain,
claiming counterfactual success, or introducing an optimistic vote update.
Candidate coverage, real repeated acquisition and objective alignment remain
separate possible development issues.

## Common screened-candidate stop

The design's stop condition is a defensible bounded application rule: after
real confirmation, use the existing shared whole-manipulation preview and
hand off when it accepts. The preview is materially stronger than ground
support alone. Generic and Ours should invoke the same candidate ordering,
four-candidate limit, planning branches, and clearance checks. The historical
default remains necessary to reproduce prior results; new runs need their
explicit mode recorded because sensing duration and termination semantics
change.

The existing preview uses a fresh public Ground base transform, hypothetical
candidate-relative target geometry, and a temporary MoveIt scene. Its return
explicitly records `arrival_revalidation_required=true` and
`camera_observation_and_transit_guaranteed=false`. A preview pass is therefore
permission to attempt the next Ground stage, not a navigation or retrieval
certificate. Arrival sensing and actual manipulation checks must retain their
authority. A bounded preview rejection is also not exhaustive infeasibility:
the shared selector examines at most four confirmed winners, and each preview
has bounded branches. Continuing the existing next observation after a
planning rejection can reveal more confirmations, but does not guarantee that
the rejection will disappear.

Earlier screening adds planning work before the last observation and trades
further aerial coverage for earlier Ground execution. Measure its real elapsed
cost as well as saved observation windows. Interface exceptions must remain
failures, with no repeated preview or added sensing budget to reinterpret
them. A confirmed count should remain an observation milestone; preview
acceptance and later physical completion are separate recorded milestones.
All four historical Hard runs have zero confirmations in every round, so this
new stop cannot by itself change their sensor evidence or remove their failure.

## Replanning limits

Candidate IDs are local to each round. Actual round-two sets have 25/41/25/25
views instead of 65 because generation is centered on the new measured
viewpoint, with flight bounds and the facade filter. Height, XY and yaw also
change between requested views and capture anchors. Updated occupied evidence
changes the visibility masks. The first-snapshot pair calculation does not
enumerate hypothetical recentered candidate sets or simulate new beliefs.
The second-snapshot calculation starts from the actual measured state and
does not pretend it is the state that a different first choice would produce.
