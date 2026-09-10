# New task acquisition snapshot

2026-09-10T04:29:11.754102+00:00

Saved phase opportunities are nominal geometric fractions, not calibrated return probabilities. Only actual recorded endpoints supply the verified presence increments. This is a read-only snapshot; unfinished tasks and unavailable windows remain pending.

| Run | Window | Confirmed candidates | Ground cells | Maximum requested-pose error (m) |
|---|---:|---:|---:|---:|
| launch-02-hard01-generic | 1 | 0 | 457 | 0.1308 |
| launch-02-hard01-generic | 2 | 0 | 1067 | 0.0988 |
| launch-03-hard01-ours | 1 | 0 | 361 | 0.1052 |
| launch-03-hard01-ours | 2 | 0 | 918 | 0.1418 |
| launch-03-hard01-ours | 3 | 3 | 869 | 0.1171 |
| launch-04-moderate01-ours | 1 | 0 | 687 | 0.1130 |
| launch-04-moderate01-ours | 2 | 2 | 1032 | 0.0886 |
| launch-05-moderate01-generic | 1 | 0 | 672 | 0.1191 |
| launch-05-moderate01-generic | 2 | 0 | 1294 | 0.1023 |
| launch-05-moderate01-generic | 3 | 6 | 1316 | 0.1009 |

| Run / observed window | Union cells | Actual ground | Old center FP/FN | Finite raw-prism FP/FN | Saved finite operational FP/FN |
|---|---:|---:|---:|---:|---:|
| launch-02-hard01-generic / 2 | 174 | 137 | 0/29 | 0/23 | 0/23 |
| launch-03-hard01-ours / 2 | 166 | 164 | 0/26 | 0/19 | 0/19 |
| launch-03-hard01-ours / 3 | 166 | 166 | 0/5 | 0/1 | 0/0 |
| launch-04-moderate01-ours / 2 | 159 | 159 | 0/3 | 0/2 | 0/0 |
| launch-05-moderate01-generic / 2 | 162 | 161 | 1/3 | 1/2 | 1/1 |
| launch-05-moderate01-generic / 3 | 162 | 159 | 3/3 | 3/2 | 3/0 |

Current status:

- launch-02-hard01-generic: last event `A6_TASK_END`; task-end record True; 1 verified prediction/observation pairs.
  First recorded failure: A5 measured capture anchor is outside configured flight bounds
  [Window 2 comparison plot](new_task_acquisition_05_launch-02-hard01-generic_window2.png)
- launch-03-hard01-ours: last event `A6_TASK_END`; task-end record True; 2 verified prediction/observation pairs.
  [Window 2 comparison plot](new_task_acquisition_05_launch-03-hard01-ours_window2.png)
  [Window 3 comparison plot](new_task_acquisition_05_launch-03-hard01-ours_window3.png)
- launch-04-moderate01-ours: last event `A6_TASK_END`; task-end record True; 1 verified prediction/observation pairs.
- launch-05-moderate01-generic: last event `A6_STAGE_START`; task-end record False; 2 verified prediction/observation pairs.

The accompanying JSON contains exact-footprint presence histograms, missing-cell IDs, recorded pose ranges, saved-model equality checks and optional packet-phase/moving-ray diagnostics. No phase opportunity is added to a vote array or treated as an observation.
