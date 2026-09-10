# New task acquisition snapshot

2026-09-10T04:12:31.571339+00:00

Saved phase opportunities are nominal geometric fractions, not calibrated return probabilities. Only actual recorded endpoints supply the verified presence increments. This is a read-only snapshot; unfinished tasks and unavailable windows remain pending.

| Run | Window | Confirmed candidates | Ground cells | Maximum requested-pose error (m) |
|---|---:|---:|---:|---:|
| launch-02-hard01-generic | 1 | 0 | 457 | 0.1308 |
| launch-02-hard01-generic | 2 | 0 | 1067 | 0.0988 |

| Run / observed window | Footprint-union cells | Predicted positive | Actual ground | Positive without ground | Ground outside prediction |
|---|---:|---:|---:|---:|---:|
| launch-02-hard01-generic / 2 | 174 | 114 | 137 | 0 | 23 |

Current status:

- launch-02-hard01-generic: last event `A6_TASK_END`; task-end record True; 1 verified prediction/observation pairs.
  First recorded failure: A5 measured capture anchor is outside configured flight bounds
- launch-03-hard01-ours: last event `A5_VIEWPOINT`; task-end record False; 0 verified prediction/observation pairs.

The accompanying JSON contains exact-footprint presence histograms, missing-cell IDs, recorded pose ranges, saved-model equality checks and optional packet-phase/moving-ray diagnostics. No phase opportunity is added to a vote array or treated as an observation.
