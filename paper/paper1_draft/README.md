# Paper 1 bilingual manuscript draft

Title: **Manipulation-aware Active Perception for Aerial-Ground Cooperative
Mobile Manipulation**. Branch: `paper/paper1_draft`, starting from `eb5597f`.

English: [main.pdf](main.pdf), [main.tex](main.tex).
Chinese: [main_zh.pdf](zh/main_zh.pdf), [main_zh.tex](zh/main_zh.tex).

The manuscript includes a complete abstract,
Introduction, Related Work, Method, Experiments, Results, Discussion and
Conclusion. This is a research narrative, not the historical A1–A5 development
chronology. The confirmed simulation success difference is the central empirical
claim; the draft does not assert general efficiency or hardware superiority.

## Files and direct asset references

| Manuscript part | Source | Existing figure/table input |
|---|---|---|
| Abstract | `sections/abstract.tex` | Checked against primary statistics |
| Introduction | `sections/introduction.tex` | Research question and three contributions |
| Related Work | `sections/related_work.tex`, `references.bib` | Eight primary-source-checked references |
| Method | `sections/method.tex` | Revised Figures 1–2 in `figures/` |
| Experiments | `sections/experiments.tex` | Fixed scene/protocol/metric definitions |
| Results | `sections/results.tex` | Figures 3–5 and all seven existing table `.tex` files |
| Discussion / Conclusion | corresponding `sections/*.tex` | Explicit scope, simulation and transfer limits |
| Author source guide | `SOURCE_NOTES.md` | Claim-to-result/code pointers; not printed in the paper |

English tables directly input `../paper1_evaluation/`. Revised PDF figures
are included from `figures/`; their renderer reads the original CSVs and the
same frozen Hard-015 records. Original evaluation figures remain unchanged.
Chinese tables translate labels from those CSVs while preserving all numeric
strings. There is no regenerated dataset, new statistical analysis, new scene
or algorithm change. Prose values have been checked against the existing
CSV/JSON assets. Citation keys are resolved by BibTeX, not manually
numbered. Primary trials are **192**, independent scenes **96**, and total
planned tasks including descriptive controls/ablations **212**.

## Build

Compiled drafts: [English](main.pdf), 15 A4 pages; [Chinese](zh/main_zh.pdf),
14 A4 pages. Verified on 2026-09-12
with Tectonic 0.17.0, including a cached-only rebuild, resolved bibliography
and cross-references, and visual checks of equations, tables and figures.
There are no missing-character or overfull/underfull-box warnings. The benign
`inputenc` warning indicates that this UTF-8 engine does not need that package.

On this workstation, Tectonic 0.17.0 is installed as `tectonic`:

```bash
cd paper/paper1_draft
tectonic --keep-logs main.tex
cd zh
tectonic --keep-logs main_zh.tex
```

Tectonic handles LaTeX reruns and BibTeX automatically. The command wrapper is
`/home/lu/.local/bin/tectonic`; the official prebuilt engine is installed at
`/media/lu/P450_PAPER/TOOLS/tectonic-0.17.0/tectonic`, with package/font caches in
`/media/lu/P450_PAPER/TOOLS/tectonic-cache`. This avoids using the nearly full
system disk. Installation follows the [official Tectonic guidance](https://tectonic-typesetting.github.io/book/latest/installation/).

Alternatively, use a conventional LaTeX installation with standard `article`,
`fontenc`, `inputenc`, `geometry`, `amsmath`, `amssymb`, `graphicx`, `hyperref`,
`url`, and BibTeX:

```bash
cd paper/paper1_draft
pdflatex -interaction=nonstopmode -halt-on-error main.tex
bibtex main
pdflatex -interaction=nonstopmode -halt-on-error main.tex
pdflatex -interaction=nonstopmode -halt-on-error main.tex
```

Run from this directory so relative section/asset paths resolve. Copy both
`paper1_draft/` and sibling `paper1_evaluation/` when transferring the document.
No simulator, asset regeneration or sensor data replay is needed to compile.
The original source-only checkpoint did not have a TeX engine. The subsequent
authorized installation adds compilation without changing experimental assets.
Submission-template compatibility still requires choosing and testing that template.

The Chinese build additionally uses `fontspec`, `xeCJK`, `booktabs` and the
installed Noto Serif/Sans CJK SC fonts. Tectonic downloads TeX dependencies;
the CJK fonts are system dependencies. The Chinese version translates all
sections, captions and table labels, while sharing English-labelled vector
figures and the same bibliography to facilitate comparison.

## Editorial figures and skill application

Applied `nature-writing` and `nature-figure` from the user-specified
[nature-skills](https://github.com/Yuan1z0825/nature-skills) repository. See
[revision notes](REVISION_NOTES.md) for the argument, terminology, display
contracts and verification caveats. This is not a claim of Nature journal
submission compliance.

To regenerate only the manuscript figures and Chinese table labels:

```bash
python paper/paper1_draft/render_figures.py --skill-root /path/to/nature-skills
python -m unittest discover -s paper/paper1_draft -p test_presentation.py
```

Run these two commands from the repository root. The Python environment needs
NumPy and matplotlib; the skill's collision checker additionally needs PyMuPDF.
The current workstation has a dedicated presentation environment under the
project tools directory. Figure 5 requires the retained original local sensing
files identified by the existing qualitative manifest; a missing file is an
error, not permission to select another example. Normal PDF compilation uses
the checked-in vector figures and does not need the raw sensing data or skills.

Each figure has editable SVG, PDF, a 600-dpi PNG preview, an alignment result
and a rendered collision report. Schematics are labelled as conceptual, not
experimental images. Quantitative displays use the original denominators,
outcomes and prespecified inference. No AI-generated sensor imagery is used.

## Remaining author work

- Supply actual author names, affiliations, funding and any required declarations;
  none were invented. The author field is intentionally empty.
- Choose a venue and page budget. This is a readable, venue-neutral full draft;
  tables and explanatory captions may need compression or supplement placement.
- Compile and visually inspect equations, bibliography and floats in that template.
- Expand/update related work as needed for the venue. The initial eight citations
  cover NBV, task-oriented grasping, aerial-ground cooperation and reachability;
  no exhaustive review or first-ever claim is made. Some entries deliberately
  cite verified author preprints rather than unverified publication metadata.
- Decide external data/code availability wording. Existing public source pointers
  do not imply that all retained raw sensing files are in Git.

Do not substitute the historical `docs/PAPER1_SIMULATION_METHODS.md`, which
describes the old interrupted 560-slot study. Do not pool development results
with the independent final cohort. Publication assets and all runtime/configuration/
result files remain unchanged by this writing task; new display assets live
only inside the draft directory.
