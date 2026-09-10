# New task exact-support diagnostic snapshot

2026-09-10T04:27:02.808405+00:00

The frozen per-cell winners and their selections are unchanged. Blocking means the existing observed/perceived operational gate—not simulator ground truth or full executable feasibility. Real ground presence is evaluated separately; no extra votes are introduced.

| Run / round | Exact winners | Geometry blocked | Clear, ground missing | Confirmed | Blocked-winner cells with clear non-winner | Clear non-winners (ground supported) |
|---|---:|---:|---:|---:|---:|---:|
| launch-02-hard01-generic / 1 | 36 | 30 | 6 | 0 | 1 | 2 (0) |
| launch-02-hard01-generic / 2 | 36 | 30 | 6 | 0 | 1 | 2 (0) |
| launch-03-hard01-ours / 1 | 36 | 31 | 5 | 0 | 1 | 3 (0) |
| launch-03-hard01-ours / 2 | 36 | 31 | 5 | 0 | 1 | 3 (0) |
| launch-03-hard01-ours / 3 | 36 | 31 | 2 | 3 | 1 | 3 (3) |
| launch-04-moderate01-ours / 1 | 35 | 30 | 5 | 0 | 1 | 1 (0) |
| launch-04-moderate01-ours / 2 | 35 | 30 | 3 | 2 | 1 | 1 (0) |

Latest available decision details:

- launch-02-hard01-generic, round 2: overlapping blocking causes {'perceived_expanded_target_rectangle': 30, 'observed_ambiguous_endpoint_disks': 29}; confirmed sources []. Task-end recorded: False.
  Source 543: winner blocked by ['perceived_expanded_target_rectangle']; 2 exact-clear alternatives, 0 with sufficient real ground support. Alternatives remain diagnostic, unselected, and unexecuted.
- launch-03-hard01-ours, round 3: overlapping blocking causes {'observed_ambiguous_endpoint_disks': 29, 'perceived_expanded_target_rectangle': 31}; confirmed sources [623, 542, 581]. Task-end recorded: False.
  Source 543: winner blocked by ['perceived_expanded_target_rectangle']; 3 exact-clear alternatives, 3 with sufficient real ground support. Alternatives remain diagnostic, unselected, and unexecuted.
- launch-04-moderate01-ours, round 2: overlapping blocking causes {'perceived_expanded_target_rectangle': 30, 'observed_ambiguous_endpoint_disks': 28}; confirmed sources [582, 542]. Task-end recorded: False.
  Source 539: winner blocked by ['perceived_expanded_target_rectangle']; 1 exact-clear alternatives, 0 with sufficient real ground support. Alternatives remain diagnostic, unselected, and unexecuted.

All saved exact anchors and candidate assessments were reproduced. The JSON retains each unchanged winner, exact non-winner pose, blocker class, missing ground-cell ID, and real presence histogram. Counts across rounds are repeated snapshots, not independent candidates or runs.
