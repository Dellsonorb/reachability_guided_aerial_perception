# Paper 1 draft writing plan

Goal: a complete research-paper first draft, not an engineering history.
Branch: `paper/paper1_draft`, based on publication checkpoint `eb5597f`.
The user has approved the topic, contribution scope and six-section structure;
ordinary writing choices proceed under the existing autonomy authorization.

Design: use a venue-neutral LaTeX article with separate prose sections and
BibTeX. Directly input the existing publication tables and PDF figures; do not
regenerate or copy their data. Contrast global and manipulation-support-weighted
observation deficit using the implemented finite-scan model. Avoid a new theory,
new experiment, first-ever claim, or claimed efficiency/real-world advantage.
An alternative engineering chronology would obscure the research question;
a venue-specific template would assume an undecided submission venue.

- [x] Inspect current methods, source formulas, fixed protocol, final tables and
  sim-to-real caveats; do not use the historical 560-slot methods draft.
- [x] Create the requested branch in the specified existing checkout, preserving
  unrelated untracked historical output directories.
- [x] Verify relevant literature using original papers/project or institutional
  sources; record citations and scope without unsupported novelty comparisons.
- [x] Write `main.tex`, section files, and `references.bib`. Include abstract,
  introduction, related work, method, experiments, results, discussion and conclusion.
- [x] Insert existing five figures and seven tables at their explanatory points.
  Link mathematical claims to source and numerical claims to existing assets in
  `SOURCE_NOTES.md`; keep this author aid outside the manuscript.
- [x] Check all input/figure paths, bibliography keys, labels/references, LaTeX
  delimiters and existing outcome values. Compile if an engine already exists;
  otherwise explicitly report uncompiled source, without installing a stack.
- [x] Review for unmeasured claims, survivor bias, result pooling, idealized
  localization/contact assumptions and simulation-only wording. Confirm that
  algorithm/configuration/results/assets have no diff from `eb5597f`.
- [x] Commit and push the writing branch without merging main. Report the entry
  point and remaining author/venue/layout work, not a submission-ready claim.

No simulator, sensor replay, hardware adapter or method execution belongs to this
writing task. Source inspection and static document verification are sufficient.

Verification: nine TeX files, eight resolved citation keys, 45 unique labels,
21 resolved reference occurrences, seven existing tables and five existing
figure PDFs. Primary counts/effect/interval and failure totals match the saved
assets. Independent read-only review found no blocking/high-priority scientific
or static LaTeX issue. At the original source-only checkpoint, no TeX executable
was installed and rendered layout was unverified.
All non-draft code/configuration/assets/results remained unchanged.
The existing 57 focused Paper 1 tests also passed without simulator execution.

Authorized compilation follow-up (2026-09-12): installed portable Tectonic
0.17.0 on the data disk and generated `main.pdf` (14 A4 pages), without editing
the manuscript's TeX sources or experimental assets. A cached-only rebuild
completed successfully. Bibliography/cross-references resolve; no missing
characters or box-overflow warnings were reported. Representative title,
equation, results-figure/table, qualitative and bibliography pages were visually
checked. Venue-specific layout remains future author work.
