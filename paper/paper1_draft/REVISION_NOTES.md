# Bilingual manuscript and figure revision

## Scope and skill use

This is an editorial revision of `a2be0f3`, not a new analysis or experiment.
The original evaluation package, runtime code, configurations and result records
remain unchanged. Existing PDFs remain recoverable in Git.

Applied `nature-writing` and `nature-figure` from
[Yuan1z0825/nature-skills](https://github.com/Yuan1z0825/nature-skills),
revision `c4e4c99fbeaf0168cda090f8739e1508017a610e`.
Writing axes: manuscript / algorithmic / existing full manuscript / English
source with a faithful Chinese companion / generic journal. Nature-leaning
argument and figure conventions are not a claim of Nature submission compliance.
Backend: the existing Python/matplotlib workflow, with editable vector exports.

## Argument and editorial plan

Under a shared finite sensing budget, manipulation-support weighting improved
physical retrieval in the evaluated static known-brick simulation population;
the matched comparison isolates weighting, not platform improvements.

Readers: robotics researchers. Preserve the requested Introduction, Related
Work, Method, Experiments, Results, Discussion and Conclusion. Prefer targeted
edits over rewriting already accurate derivations. The Introduction follows a
technical-challenge funnel; Results progresses from paired outcome to difficulty,
failure location, one recorded illustration, and efficiency boundaries.

Alternatives considered: recolouring existing figures would leave the dense
engineering presentation intact; imposing a journal-specific compressed template
would assume an unchosen venue. Instead, redraw manuscript-specific figures and
preserve the complete original evaluation assets and all seven source tables.

### Terminology

| English / symbol | Chinese | Boundary |
|---|---|---|
| Manipulation relevance, R | 操作相关性 | Joint margin after validity gates |
| Observation deficit, u | 观测缺失度 | Heuristic, not calibrated probability |
| Operational support, M_op | 操作可用支撑 | Does not certify navigation feasibility |
| Exact validated winner | 经验证的精确优胜候选 | Not a cell-centre approximation |
| Confirmed candidate | 已确认候选 | Actual ground support plus operational checks |
| D_exec | 执行阶段候选可用性验证 | Actual-arrival refined pregrasp |
| Physical retrieval success | 物理取回成功 | Grasp, lift and short retention |
| Generic / Ours | Generic / Ours（本文方法） | Same common components |

### Result allocation and figure contracts

| Display | Question / role | Data and permitted presentation |
|---|---|---|
| Fig. 1 | How does future manipulation guide sensing? | New vector schematic; no trial or sensor-image claim |
| Fig. 2 | What differs between the policies? | Spatial schematic and the unchanged gain formula; explicitly illustrative |
| Fig. 3 | Does physical retrieval improve, and where? | All 96 pairs, 53/27/3/13 outcomes and 38/28/30 tier counts; original paired CI only |
| Fig. 4 | Where do failed tasks stop? | All eight original first-terminal stage bins; zero bins retained |
| Fig. 5 | What measured evidence differs in one case? | Same frozen Hard-015 example; all three observation poses and native grids, no resampling or reselection |

Primary effect and inference remain in Results; difficulty is descriptive.
Generic-only outcomes, efficiency uncertainty and simulation limitations remain
visible. No new confidence intervals, tests, or outcome selection are introduced.
Captions explain encodings and sampling units rather than repeat whole Results
paragraphs. Tables remain direct inputs from the original package; Chinese table
labels are generated from those CSVs without changing numerical strings.

## Implementation and verification checklist

- [x] Add a manuscript-only renderer and checks for original counts, geometric
  anchoring and bilingual table values; observe the new checks fail first.
- [x] Export five PDF/SVG/PNG figures at manuscript width with consistent
  Generic/Ours encodings. Preserve raw quantitative and recorded spatial inputs.
- [x] Apply evidence-bounded English edits and supply a complete Chinese draft,
  with the same equations, references, figures, result values and limitations.
- [x] Run the skill's source, alignment, PDF text and collision checks; inspect
  each final figure and both compiled PDFs. Record caveats without claiming a
  generic automatic audit proves scientific correctness.
- [x] Confirm protected data/code have no diff and prepare a manuscript-only
  checkpoint of sources, derived presentation assets and build/review notes.

Author names, affiliations and venue remain unspecified. Human authors still
need to verify scholarly interpretation and decide any required AI-use disclosure
before submission. No fabricated authors, new citations or hardware results are
added.

## Editorial review and display QA

The English revision is targeted: Abstract and Introduction are shorter; Method
derivations and substantive Results paragraphs are preserved. Figure captions
now define panel roles, denominators and interpretation limits independently.
The conclusion corrects an inherited ambiguity: the **observed** paired difference
was 25 pp; only the analysis was prespecified. The Chinese draft makes the same
distinction. No new literature claims or references were added.

Results prose counts excluding figures/tables are unchanged across the five
subsections (201 / 92 / 220 / 170 / 174 token-like words, using the same regex on
both revisions). Overall Abstract counts decrease 212→190 and Introduction
578→531; these source-level counts include notation, not a venue word-count claim.
Increased Method/Results source length comes from self-contained captions,
not additional analyses. The primary estimate and test remain in the primary
Results section. Sensitivity and auxiliary qualifications remain visible because
this is a complete draft rather than a venue-constrained main-text/SI submission.

### Panel-level reading checks

| Figure / panels | Role and source | Statistical / geometry boundary | Visual check |
|---|---|---|---|
| 1, workflow | Measured evidence → operation-conditioned observation and physical handoff | Dashed connector is processing order; confirmation uses H_t/B_t independently of U_task | Explicit gate inputs; no label/arrow crossings |
| 2a / 2b | Same spatial cartoon, different contributing support | No experimental values or claimed winning view; no uncertainty interval applicable | Matched rectangles and typography; formula remains readable |
| 3a | All difficulty-tier proportions | 38/28/30 denominators; descriptive points without new tier CIs | Shared percentage axis; square/circle plus colour |
| 3b | All paired outcomes | 96 independent scene pairs, including 3 Generic-only cases | Counts directly labelled; original paired CI/test reported |
| 4 | All eight terminal bins | One first cause per failure, all-96 denominator; zero bins retained | Paired rows, direct counts, no fabricated error bars |
| 5a / 5b | All three recorded observation poses | Same Hard-015 selection, common map axes; lines encode order only | Native coordinates; labels avoid heading arrows and axes |
| 5c / 5d | Final measured ground-presence votes | No free-space certification; 19/96 versus 0/88 is per-run anchor evidence | Common 0–3 colour scale; exact polygons and all native cells retained |
| 5e / 5f | Saved task-deficit fields | NaN/no-support stays white; zero is not FREE; no resampling | Common 0–1 scale and map axes; separated notes/legend |

All final figures use manuscript width 162 mm (the current A4 text area), not
an assumed 183-mm journal production width. Main text is 7–10 pt in the figure
sources; the smallest rendered figure glyph is 5.6 pt. PDF and SVG are the
vector deliverables; PNG is a 600-dpi preview, not a TIFF submission file.
These deliberate choices explain the static preflight's width/PNG warnings.
Its mathtext warning is checked against actual exported PDF text sizes.
Alignment reports cover comparable panel groups; single-panel schematics and
the failure chart are not applicable. Collision reports and visual inspection
are presentation checks, not a proof of statistical or robotic correctness.

Independent read-only review checked bilingual formulas, numerical tables,
limitations and renderer data flow. It identified the workflow/gate ambiguity
and the inherited “prespecified effect” wording; both were corrected and rechecked.
Both languages have the same 45 labels and eight citation keys with no unresolved
references. Both PDF builds use Tectonic 0.17.0, including cached-only builds.
Chinese compilation depends on installed Noto CJK fonts, which the engine reports
as external system resources. The English inputenc warning is benign; neither
draft has missing-character, undefined-reference or overfull/underfull-box warnings.
