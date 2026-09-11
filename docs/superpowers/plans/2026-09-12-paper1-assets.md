# Paper 1 publication assets and transfer audit plan

**Goal:** Turn the completed fixed evaluation into reproducible paper tables and
figures, and document real-platform gaps without modifying a robot or a result.

**Design decision:** Reuse the frozen reducer and retained runtime snapshots.
PDF/SVG plus preview PNG figures are generated from code; schematic Figures1/2
are labeled conceptual, Figure5 is explicitly an outcome-selected illustration.
No generated fake camera images, simulator reruns or post-hoc statistical tests.
The user's existing autonomy instruction applies to ordinary presentation choices.
Work in the requested checkout on `docs/paper1-assets-sim-to-real`; keep original
large results in place and preserve the four unrelated untracked directories.

**Architecture:** One offline statistical/table/diagram exporter, one offline
qualitative exporter, and a source-backed read-only interface audit. Use the
existing NumPy/SciPy/Matplotlib environment, not new dependencies/frameworks.

## Tasks and checks

- [x] Root: tests first for paired counts/effect interval, missing outcomes, and
  first-terminal display mapping. Add `tests/test_paper1_publication.py` and
  `scripts/build_paper1_evaluation_package.py`; run the tests red then green.
- [x] Root: export CSV and LaTeX tables to `paper/paper1_evaluation/`, including
  margins, paired outcomes, exact McNemar and the prespecified paired effect CI.
  The CI is for the risk difference, not for a p-value. Stage8 display taxonomy
  distinguishes sensing failure from no confirmed candidate; record original
  stage/reason next to the presentation mapping, preserving missing/not-reached.
- [x] Root: Figures1/2 show the real system loop and the sole shared-policy gain
  difference; Figure3 descriptive tier proportions (no extra tests); Figure4
  primary first-terminal stage counts with N96 denominators.
- [x] Qualitative subtask: Figure5 from retained sensor-derived fields, exact
  winners and packet poses with deterministic illustrative case selection.
  Record sources and avoid claiming preselection or population representativeness.
- [x] Audit subtask: write `docs/SIM_TO_REAL_READINESS.md` from actual AGENT/SIM
  code and official upstream references. Distinguish interface from implemented
  hardware backend; cover PX4, BUNKER, sensors/TF/time, MoveIt and AG95.
- [x] Root: verify tag/commit/config/seed references, offline-vs-online
  reproducibility and missing raw-data limitations. Add package README/captions
  and a manuscript table include snippet, without editing old reports.
- [x] Final: run focused tests, inspect all figures, independent review; confirm
  runtime/config/result directories unchanged. Commit/push only new assets,
  analysis/tests/docs on the new branch; do not merge main or operate hardware.

## Acceptance

212 source outcomes and96 primary pairs remain unchanged. Tables reproduce
80/96 vs56/96 and53/27/3/13; the eight displayed failure counts sum to16/40.
Five numbered figures and captions exist in vector and preview formats. One
offline command regenerates publication assets; full raw-data limitations are
explicit. The transfer audit ranks concrete gaps, not readiness by assertion.

## Verification notes

- Offline regeneration produced all five figures in PDF/SVG/PNG and the tables.
- `unittest discover -s tests -p 'test_paper1*.py'`: 57 tests passed, including
  the frozen evaluation tests and 13 new publication/qualitative tests.
- Figures were visually inspected. Qualitative metadata distinguishes the
  worker's budget decision from the authoritative runtime screened-ready stop.
- Frozen AGENT runtime/config/assets/results have no diff; SIM and baseline
  checkouts are clean. No simulator or hardware was started.
- No TeX engine is installed: include snippets are supplied but no venue paper
  compilation is claimed. Retained raw data are necessary to regenerate Figure5.
- Independent review confirmed the statistics and raised two portability issues.
  Both were corrected: Figure5 is pinned to its selected pair instead of silently
  substituting another retained scene; raw-data integration tests explicitly skip
  absent local evidence in a compact clone while malformed present data still fail.
