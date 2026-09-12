# Confirmation factor study

Goal: Decide whether candidate completion planning merits further development,
not make it win. Start `853602f`, branch `feature/dev-confirmation-factor-study`
in the requested existing AGENT checkout; reuse local recorded sensors/assets.
Use writing-plans, inline execution under explicit autonomous authorization,
test-driven implementation and independent requesting-code-review. No new model,
candidate set, thresholds, execution pipeline or final-test scenes.

## Design and alternatives

Selected: same-state factor decomposition plus four one-step development runs.
Only replay would leave the one-step actual behavior unknown; rerunning a full
three-policy matrix would unnecessarily expand this focused test. Keep the
previous eight runs as cross-batch development references, not new randomized
pairs or independent test data. Same shared decision/execution code and profile
are required, but new method routing has a new commit. No statistical claim.

1. Original confirmation v1 stays byte-identical: phase tier, then horizon, then
   common motion cost and stable IDs.
2. Offline windows-first: use the original core with remaining=1; if it finds no
   plan, use the original remaining=2 result. Only preference order changes.
3. New explicit `deficit` method maximizes over individual currently unblocked,
   unclipped exact winners with `max(d)<=remaining`:
   `progress(v)=max_q sum_{x in S_q,d(x)>0} w(v,x)/sum_{x in S_q} d(x)`.
   Existing flight cost, then stable view ID break ties. This is a nominal
   fractional missing-ticket surrogate, not probability of completion. Log the
   maximizing source IDs; do not pool support cells across candidates.
   Already-confirmed and zero predicted progress delegate to Ours/handoff.
   Budget exhaustion and all-current-support real vote-budget impossibility
   stop sensing. No predictive writes. The old v1 one-step diagnostic included
   budget-impossible candidates; report its differences, do not silently relabel.

## Implementation and checks

- [ ] Add `tests/test_deficit_policy.py` before source: independent import;
  `H=[1,1,1],S=[[0,1],[2]],w=[[1,0,0],[0,0,.75]]` picks view1 (max per-candidate,
  not pooled support); `H=[0],remaining=1` means real budget impossibility;
  `H=[1],w=0` means no nominal progress. Check immutable input, ties, guards and
  original Ours/confirmation routes. Run `PYTHONPATH=src:scripts python3 -m
  unittest discover -s tests -p test_deficit_policy.py`, first red, then green.
- [ ] Implement pure `plan_deficit(h,supports,w,valid,first_cost,remaining)` and
  `apply_deficit(...)` in `src/a6_pilot/deficit.py`. Add only explicit development
  method routing to policy and existing run entry; keep old core unchanged.
- [ ] Add `analysis/confirmation_factors/analyze.py`: replay all old 541 states
  plus previous eight runs' 21 states; derive phase-first/windows-first/deficit
  differences. Use existing offline transition calibration only on actually
  commanded actions; next observations are labels, never policy inputs. Record
  candidate completion false positives and needed-cell hit misses separately.
  Write derived data only under this new analysis directory.
- [ ] Review code and replay outputs, run relevant regressions, commit before
  the first online start. No code changes during the four planned tasks.

## Online budget and fixed order

Four new full tasks, all `deficit`, using unchanged geometry from
`configs/paper1_eval.json` and shared `configs/current_sim_task.json`:
Easy003, Hard023, Moderate009, Hard015, in this exact order. These are the same
four development scenes, covering both retrieval successes, confirmation failure
and post-confirmation execution failure. No seed filtering or normal-failure
rerun. At most five starts including one demonstrated infrastructure-invalid
diagnostic/replacement reserve. Wall cap two hours from first start; new storage
25 GiB and disk free >=100 GiB. Check budget before each task and leave 30 minutes
for its existing runtime guard/cleanup. No parameter tuning.

- [ ] Run each task with existing `scripts/run_retrieval.py run --scene-file
  configs/paper1_eval.json --scene-id <listed-scene> --method deficit --output-dir
  outputs/development/confirmation-factors/slot-NN-<scene>-deficit`.
- [ ] Reuse prior `summarize_online.summarize` for stages, physical outcomes,
  simulation active/Ground times and full worker/policy wall timing; separately
  extract `deficit_plan` solver timing and proposals. Show old eight references
  with their original versions, no overwritten results or pooled final statistics.
- [ ] Review actual success/failure and model calibration, run tests, commit/push
  checkpoint. Give an explicit retain/simplify/continue-lookahead recommendation.
  Stop after this batch, do not promote default Ours or start a matrix.
