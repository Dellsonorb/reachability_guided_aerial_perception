#!/usr/bin/env python3
"""Descriptive tables/plots from the offline analysis; never evaluates a new policy."""
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from offline import write_csv, read_json, read_npz


def main():
    out = Path(__file__).parent/'results'
    root = Path('outputs/paper1-final-eval-v1')
    read = lambda name: list(csv.DictReader((out/(name+'.csv')).open()))
    accounting, trajectory, predictions = read('accounting'), read('trajectories'), read('predictions')
    primary = [r for r in accounting if r['role']=='primary']
    index = {(r['scene'], r['method']): r for r in primary}
    traj = {(r['slot'], r['round']): r for r in trajectory}
    pred = {(r['slot'], r['round']): r for r in predictions}
    pairs, failures = [], []
    for scene in sorted({r['scene'] for r in primary}):
        record = dict(scene=scene)
        for method in ('generic', 'ours'):
            task = index[scene, method]
            record[method+'_slot'] = task['slot']
            record[method+'_retrieval'] = task['retrieval_success']
            for n in (1, 2, 3):
                t = traj.get((task['slot'], str(n)), {})
                for key in ('confirmed', 'minimum_missing_cells', 'fixed_missing_tickets', 'fixed_viable'):
                    record[f'{method}_round{n}_{key}'] = t.get(key)
        pairs.append(record)
        task = index[scene, 'generic']
        if task['confirmation_failure']=='True':
            a, b = pred[task['slot'], '1'], pred[task['slot'], '2']
            failures.append(dict(slot=task['slot'], scene=scene, ours_paired_retrieval=index[scene, 'ours']['retrieval_success'],
                round1_completion_tier=a['best_tier'], historical_first_tier=a['historical_first_tier'],
                strict_improvement=a['strict_improvement'], changed_first=a['changed'],
                first_pose=a['predicted_first_pose'], second_pose=a['predicted_second_pose'],
                round2_vote_budget_possible=traj[task['slot'], '2']['vote_budget_possible_candidates'],
                round2_completion_tier=b['best_tier'], final_minimum_missing_cells=traj[task['slot'], '3']['minimum_missing_cells']))
    write_csv(out/'paired_trajectories.csv', pairs)
    write_csv(out/'generic_29_cases.csv', failures)

    # Diagnostic outcome-only packet poses for all OR-level overpredictions.
    slots = {r['slot']: r for r in csv.DictReader((root/'slots.csv').open())}
    discrepancies = []
    for row in read('actual_transitions'):
        if row['role']!='primary' or row['predicted_confirmation_all_phase']!='True' or row['actual_confirmation']=='True':
            continue
        data = root/slots[row['slot']]['selected_attempt']/'data'
        rank = read_json(data/'rounds'/f"round-{int(row['from_round']):02d}"/'ranking.json')
        view = rank['candidates'][int(row['historical_view_id'])]['viewpoint']
        target = np.array(view['position_xyz'])
        obs = read_npz(data/f"observation_{int(row['to_round']):02d}.npz")
        body = obs['chunk_T_map_sensor'] @ np.linalg.inv(np.array(rank['sensor']['T_uav_lidar']))
        positions = body[:, :3, 3]
        # Tilt from the measured body Z axis, not from simulator model states.
        tilt = np.degrees(np.arccos(np.clip(body[:, 2, 2], -1, 1)))
        disagreements = dict(packet_count=len(body), nominal_command_xyz=target.tolist(),
            mean_measured_body_xyz=positions.mean(axis=0).tolist(),
            max_command_position_error_m=float(np.linalg.norm(positions-target, axis=1).max()),
            mean_height_error_m=float(positions[:, 2].mean()-target[2]),
            maximum_body_tilt_deg=float(tilt.max()),
            pose_diagnostic_semantics='observed_nominal_model_mismatch_not_causal_attribution')
        discrepancies.append(row | disagreements)
    write_csv(out/'overprediction_pose_diagnostics.csv', discrepancies)

    facts = dict(paired_scenes=len(pairs), paired_round2_available=sum(all(r[f'{m}_round2_fixed_missing_tickets'] is not None for m in ('ours','generic')) for r in pairs))
    for key in ('minimum_missing_cells', 'fixed_missing_tickets'):
        differences = [int(r[f'ours_round2_{key}'])-int(r[f'generic_round2_{key}']) for r in pairs
                       if all(r[f'{m}_round2_{key}'] is not None for m in ('ours', 'generic'))]
        facts[key+'_round2_paired'] = dict(ours_lower=sum(x<0 for x in differences), equal=sum(x==0 for x in differences),
            ours_higher=sum(x>0 for x in differences), median_ours_minus_generic=float(np.median(differences)))
    facts['generic_failures_with_paired_ours_retrieval'] = sum(r['ours_paired_retrieval']=='True' for r in failures)
    facts['historical_first_no_completion_flag'] = {}
    for method in ('generic', 'ours'):
        rows = [r for r in predictions if r['role']=='primary' and r['method']==method and r['round']=='1']
        facts['historical_first_no_completion_flag'][method] = dict(n=len(rows),
            flagged=sum(r['historical_first_tier']=='0' for r in rows),
            confirmation_failures_flagged=sum(r['historical_first_tier']=='0' and r['confirmation_failure']=='True' for r in rows),
            confirmation_failures_unflagged=sum(r['historical_first_tier']!='0' and r['confirmation_failure']=='True' for r in rows))
    facts['cumulative_confirmation'] = {}
    for method in ('generic', 'ours'):
        ids = {r['slot'] for r in primary if r['method']==method}
        facts['cumulative_confirmation'][method] = [sum(any(int(traj.get((slot,str(k)), {}).get('confirmed', 0))>0 for k in range(1,n+1)) for slot in ids) for n in (1,2,3)]
    (out/'report_facts.json').write_text(json.dumps(facts, indent=2)+'\n')

    plt.rcParams.update({'font.size':10, 'axes.spines.top':False, 'axes.spines.right':False, 'pdf.fonttype':42})
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.9), layout='constrained')
    for method, color in [('generic','#777777'), ('ours','#0072B2')]:
        axes[0].plot([1,2,3], facts['cumulative_confirmation'][method], 'o-', color=color, label=method.capitalize())
    axes[0].set(xticks=[1,2,3], ylim=(0,96), xlabel='Accepted observation window', ylabel='Scenes with a confirmed candidate', title='a  Recorded confirmation\n96 scenes per method')
    axes[0].legend(frameon=False)
    pts = np.array([[int(r['generic_round2_fixed_missing_tickets']), int(r['ours_round2_fixed_missing_tickets'])] for r in pairs
                    if r['generic_round2_fixed_missing_tickets'] is not None and r['ours_round2_fixed_missing_tickets'] is not None])
    unique, counts = np.unique(pts, axis=0, return_counts=True)
    axes[1].scatter(unique[:,0], unique[:,1], s=15+12*counts, color='#0072B2', alpha=.65)
    lim = max(5, int(pts.max()))
    axes[1].plot([0,lim],[0,lim], '--', color='.6', linewidth=1)
    axes[1].set(xlabel='Generic: missing ground-support votes', ylabel='Ours: missing ground-support votes', title=f"b  Fixed initial anchor, window 2\n{len(pts)} paired scenes")
    counts = facts['fixed_missing_tickets_round2_paired']
    axes[1].text(.04,.95,f"{counts['ours_lower']} lower / {counts['equal']} equal / {counts['ours_higher']} higher\n(each method keeps its own initial anchor)", va='top', transform=axes[1].transAxes, fontsize=9)
    counts = [sum(r['round1_completion_tier']==str(tier) for r in failures) for tier in (2,1,0)]
    axes[2].bar(['All-phase\nnominal', 'Phase-dependent\nnominal', 'No plan in\nrestricted graph'], counts, color=['#0072B2','#E69F00','#999999'])
    for x,y in enumerate(counts): axes[2].text(x,y+.4,str(y),ha='center')
    axes[2].set(ylim=(0,max(counts)+4), ylabel=f'Generic confirmation failures (n={len(failures)})', title='c  Two-step nominal alternatives\nNot measured rescues')
    fig.savefig(out/'completion_diagnostics.pdf')
    fig.savefig(out/'completion_diagnostics.png', dpi=180)
    plt.close(fig)
    print(json.dumps(facts, indent=2))


if __name__=='__main__':
    main()
