# Offline validation of integrated-pose velocity semantics

No feedback correction is implemented or approved by this analysis. The proposed quantity is a one-step average pose velocity, not necessarily the instantaneous native ODE rate.

| Run / window (update end) | Wrist unwrapped span, rad | Max / p95 angle discrepancy, rad | Max / p95 rate discrepancy, rad/s | Max off-axis rate, rad/s | Pose speed abs p95 / max, rad/s |
|---|---:|---|---|---:|---|
| launch-02-easy-generic-original / ground_refine_before_close | 2.4585059 | 3.8533825e-06 / 1.0183674e-08 | 0.0038533825 / 1.0183674e-05 | 0.0026899854 | 0.36068013 / 0.40860854 |
| launch-02-easy-generic-original / refined_pregrasp_before_close | 3.2005943 | 7.0102016e-07 / 2.2896862e-11 | 0.00070102016 / 2.2896862e-08 | 0.0035078431 | 0.36134128 / 0.39844183 |
| launch-02-easy-generic-original / lift_tail_2s_proxy | 0.00017688265 | 5.8489721e-12 / 1.9219627e-12 | 5.8489721e-09 / 1.9219627e-09 | 0.00084148964 | 0.020250731 / 0.045835852 |
| launch-02-easy-generic-original / closed_hold | 0.00020007481 | 9.9063053e-12 / 3.887161e-12 | 9.9063053e-09 / 3.887161e-09 | 0.0011608164 | 0.02249941 / 0.053216759 |
| launch-03-easy-measured-release / ground_refine_before_close | 2.4501454 | 4.0904249e-06 / 9.3812183e-09 | 0.0040904249 / 9.3812183e-06 | 0.0018590709 | 0.36069525 / 0.40895905 |
| launch-03-easy-measured-release / refined_pregrasp_before_close | 3.1829629 | 1.0696158e-05 / 2.2974825e-11 | 0.010696158 / 2.2974825e-08 | 0.0052313833 | 0.36141746 / 0.38801422 |
| launch-03-easy-measured-release / lift_tail_2s_proxy | 0.00030801064 | 6.229108e-12 / 2.6882538e-12 | 6.2291081e-09 / 2.6882538e-09 | 0.0017285235 | 0.037053764 / 0.079655088 |
| launch-03-easy-measured-release / closed_hold | 0.00010883811 | 3.3159985e-12 / 1.8828906e-12 | 3.3159985e-09 / 1.8828906e-09 | 0.0011226117 | 0.018635547 / 0.034161544 |
| launch-04-easy-world-solver / entire_trace_failed_physics | 3.1353534 | 1.1146461 / 1.673184e-15 | 1114.6461 / 1.673184e-12 | 3066.0436 | 0.0036011641 / 3128.5522 |

## Scope, missing data and physical state

- launch-02-easy-generic-original: original quick solver; pre-close motion and loaded holds; {'total_csv_rows': 154401, 'retained_rows': 111480, 'malformed_rows': 0, 'unknown_phase_rows': 0, 'columns': 87}.
  - ground_refine_before_close 11.889–24.638 s: 12749 validated pairs; 0 dt/iteration gaps; 0 missing/nonfinite geometry pairs. Target XYZ change [-1.415423334094612e-11, -1.0403361505595399e-10, 0.0] m; relative-to-wrist maximum movement 0.8635278611869366 m.
  - refined_pregrasp_before_close 24.638–34.398 s: 9760 validated pairs; 0 dt/iteration gaps; 0 missing/nonfinite geometry pairs. Target XYZ change [-1.0835776720341528e-11, -7.964295889451023e-11, 0.0] m; relative-to-wrist maximum movement 0.8250468304738902 m.
  - lift_tail_2s_proxy 45.037–47.037 s: 2000 validated pairs; 0 dt/iteration gaps; 0 missing/nonfinite geometry pairs. Target XYZ change [9.41387011714756e-05, -6.140480895283629e-06, -7.554415017158034e-05] m; relative-to-wrist maximum movement 0.00010756404095090346 m.
  - closed_hold 47.041–49.048 s: 2007 validated pairs; 0 dt/iteration gaps; 0 missing/nonfinite geometry pairs. Target XYZ change [-0.00015066901663285392, 0.0006841166524764186, 3.161234899601295e-05] m; relative-to-wrist maximum movement 0.0007194540726545275 m.
- launch-03-easy-measured-release: original quick solver; pre-close motion and loaded holds; {'total_csv_rows': 152499, 'retained_rows': 109185, 'malformed_rows': 0, 'unknown_phase_rows': 0, 'columns': 87}.
  - ground_refine_before_close 11.923–23.955 s: 12032 validated pairs; 0 dt/iteration gaps; 0 missing/nonfinite geometry pairs. Target XYZ change [-1.3358203432289883e-11, -9.818279522733064e-11, 0.0] m; relative-to-wrist maximum movement 0.8686564765354553 m.
  - refined_pregrasp_before_close 23.955–33.629 s: 9674 validated pairs; 0 dt/iteration gaps; 0 missing/nonfinite geometry pairs. Target XYZ change [-1.0740297540223764e-11, -7.894118692064467e-11, 0.0] m; relative-to-wrist maximum movement 0.8293505666214676 m.
  - lift_tail_2s_proxy 44.291–46.291 s: 2000 validated pairs; 0 dt/iteration gaps; 0 missing/nonfinite geometry pairs. Target XYZ change [0.0001631270686786035, -0.00019717940511468357, -9.214230397808221e-05] m; relative-to-wrist maximum movement 0.00026122636790163373 m.
  - closed_hold 46.294–48.317 s: 2023 validated pairs; 0 dt/iteration gaps; 0 missing/nonfinite geometry pairs. Target XYZ change [0.00015561471545511019, -0.00030552195074269595, -4.836729436036302e-05] m; relative-to-wrist maximum movement 0.00035407253225690986 m.
- launch-04-easy-world-solver: LCP-failed world solver, not adoption evidence; {'total_csv_rows': 41853, 'retained_rows': 41853, 'malformed_rows': 0, 'unknown_phase_rows': 0, 'columns': 87}.
  - entire_trace_failed_physics 0.28200000000000003–14.232 s: 13950 validated pairs; 0 dt/iteration gaps; 0 missing/nonfinite geometry pairs. Target XYZ change [-2.2799796939310113e-07, 2.7726936817629166e-06, -0.000999695838702394] m; relative-to-wrist maximum movement 630775140.4749832 m.

## Limitations

- Independent link-quaternion validation is available only for wrist3. Other five joint pose speeds are descriptive, not independently validated here.
- Unwrapped delta uses shortest angular difference each step; an actual rotation exceeding pi per step would be ambiguous.
- Quaternion comparison removes parent motion: Rrelative = Rparent inverse * Rchild, delta Rrelative projected on the parent-local joint axis. Off-axis residuals are reported, not discarded.
- A finite-difference quantity is a one-step interval-average velocity. Its agreement with actual pose motion does not establish equivalence to native instantaneous solver velocity.
- No filtering, threshold relaxation, native-rate substitution, trial relabelling, or controller change is made.
- Event-bounded pre-close motion and measured open-gripper width support an unloaded movement classification; contact reaction forces are not recorded.
- Loaded holds retain grasp confirmation and elevated target motion. Failed opening attempts are not certified unloaded states.
- The world-solver launch04 reported LCP failure. Its complete trace is a failed-physics reference only, never adoption evidence.
- A future feedback implementation still requires startup/reset/dt handling, actual readSim-period checks, control stability review and bounded physical validation with original acceptance.
