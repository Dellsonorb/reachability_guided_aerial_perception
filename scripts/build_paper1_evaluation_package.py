#!/usr/bin/env python3
"""Offline publication tables and Figures1-4; never executes a robot or changes data."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import subprocess

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
import numpy as np

from analyze_paper1_eval import primary_statistics, validate_production_manifest

ROOT = Path(__file__).resolve().parents[1]
COLORS = {'ours': '#1868ac', 'generic': '#df7b22'}
STAGES = {
    'uav_observation': 'UAV observation',
    'confirmed_candidate': 'Confirmed candidate',
    'screened_candidate': 'Screened candidate',
    'bunker_navigation': 'BUNKER navigation',
    'd435_refinement': 'D435 refinement',
    'd_exec': 'D_exec (refined pregrasp)',
    'grasp': 'Grasp (descend / close)',
    'lift_retention': 'Lift / retention',
}


def paired_statistics(pairs):
    counts = Counter()
    for pair in pairs:
        o, g = (pair[m]['retrieval_success'] for m in ('ours', 'generic'))
        if type(o) is not bool or type(g) is not bool:
            raise ValueError('Every prescribed pair needs two explicit binary outcomes')
        counts['a' if o and g else 'b' if o else 'c' if g else 'd'] += 1
    if not pairs:
        raise ValueError('No pairs')
    return primary_statistics(dict(n_scheduled=len(pairs), **{k: counts[k] for k in 'abcd'}))


def failure_display_stage(row):
    """Presentation-only aggregation, preserving first terminal stage and reason."""
    if row['retrieval_success'] is True:
        return None
    if row['retrieval_success'] is not False:
        raise ValueError('Missing primary outcome')
    stage, reason = row.get('failure_stage'), row.get('failure_reason') or ''
    if stage == 'active':
        if 'VIEW_BUDGET_REACHED' in reason and 'without a confirmed exact candidate' in reason:
            return 'confirmed_candidate'
        if any(s in reason for s in ('capture timed out', 'did not settle')):
            return 'uav_observation'
        raise ValueError('Unrecognized active-stage reason: '+reason)
    mapping = dict(aerial_observe='uav_observation', execution_screen='screened_candidate',
                   ground_navigation='bunker_navigation', ground_refine='d435_refinement',
                   refined_pregrasp='d_exec', descend='grasp', close='grasp',
                   lift='lift_retention', retention='lift_retention')
    if stage not in mapping:
        raise ValueError('Unrecognized first terminal stage: '+str(stage))
    return mapping[stage]


def failure_table(rows):
    counts = {method: Counter(failure_display_stage(r) for r in rows
                             if r['method'] == method and r['retrieval_success'] is False)
              for method in COLORS}
    return [dict(stage=key, label=label, generic=counts['generic'][key], ours=counts['ours'][key])
            for key, label in STAGES.items()]


def latex_table(headers, rows):
    def escape(v):
        return str(v).replace('_', r'\_').replace('%', r'\%').replace('&', r'\&')
    return ('\\begin{tabular}{'+'l'*len(headers)+'}\n\\hline\n'+
            ' & '.join(map(escape, headers))+r' \\'+'\n\\hline\n'+
            '\n'.join(' & '.join(map(escape, r))+r' \\' for r in rows)+
            '\n\\hline\n\\end{tabular}\n')


def write_table(directory, name, headers, rows):
    with (directory / (name+'.csv')).open('w', newline='') as stream:
        writer = csv.writer(stream, lineterminator='\n')
        writer.writerow(headers)
        writer.writerows(rows)
    (directory / (name+'.tex')).write_text(latex_table(headers, rows))


def save_figure(fig, directory, name):
    for extension in ('pdf', 'svg', 'png'):
        fig.savefig(directory / (name+'.'+extension), bbox_inches='tight', dpi=220)
    plt.close(fig)


def box(ax, x, y, w, h, text, color='#eaf2f8', size=10):
    ax.add_patch(FancyBboxPatch((x,y), w,h, boxstyle='round,pad=0.01',
                              facecolor=color, edgecolor='#476176', linewidth=1))
    ax.text(x+w/2,y+h/2,text,ha='center',va='center',fontsize=size)


def arrow(ax, start, end, text=None):
    ax.annotate('',xy=end,xytext=start,arrowprops=dict(arrowstyle='->',color='#405164',lw=1.3))
    if text:
        ax.text((start[0]+end[0])/2,(start[1]+end[1])/2+.015,text,ha='center',fontsize=8)


def pipeline_figure(out):
    fig,ax=plt.subplots(figsize=(11,6.3));ax.set(xlim=(0,1),ylim=(0,1));ax.axis('off')
    ax.text(.5,.985,'Sensor-driven aerial perception to physical Ground retrieval',ha='center',fontsize=14)
    box(ax,.02,.79,.25,.12,'P450 RGB-D\nperceived brick grasp TCP', '#e8f3eb')
    box(ax,.36,.79,.28,.12,'Frozen RM4D + task-domain asset\nA1: validated exact support',size=9)
    box(ax,.73,.79,.25,.12,'A2: MID360 endpoint belief\nactual ground / occupied votes', size=9)
    arrow(ax,(.27,.85),(.36,.85))
    box(ax,.34,.54,.32,.12,'A3: exact BUNKER footprints\n'+r'$U_{task}(x)=u(x)M_{op}(x)$',size=11)
    arrow(ax,(.50,.79),(.50,.66));arrow(ax,(.85,.79),(.66,.61))
    box(ax,.02,.31,.27,.14,'Confirmed exact candidates\nshared full-robot screen\nREADY: select + handoff',size=9)
    arrow(ax,(.34,.56),(.24,.45))
    box(ax,.37,.31,.29,.14,'A4: finite-scan NBV\nshared candidate / cost / budget\nNOT READY: informative view',size=9)
    arrow(ax,(.55,.54),(.55,.45))
    box(ax,.74,.31,.24,.14,'SIM public flight action\nfly, settle, observe', '#e8f3eb',10)
    arrow(ax,(.66,.38),(.74,.38));arrow(ax,(.86,.45),(.86,.79),'new sensor scan')
    labels=['BUNKER\nnavigate / stop','D435\nrefine','AUBO + AG95\nplan / grasp','Load lift\nand retention']
    for i,label in enumerate(labels):
        x=.02+i*.25
        box(ax,x,.075,.21,.12,label,'#e8f3eb',10)
        if i:arrow(ax,(x-.04,.135),(x,.135))
    arrow(ax,(.15,.31),(.15,.195))
    ax.text(.50,.015,'Schematic: blue = research decisions; green = sensing / shared execution. Failure may terminate any stage.',
            ha='center',fontsize=8)
    save_figure(fig,out,'figure1_system_pipeline')


def method_figure(out):
    fig,ax=plt.subplots(figsize=(10.5,5.7));ax.set(xlim=(0,1),ylim=(0,1));ax.axis('off')
    ax.text(.5,.97,'One shared system, two observation-gain weightings',ha='center',fontsize=14)
    box(ax,.12,.76,.76,.13,'Shared state, candidate viewpoints, finite-scan opportunity '+r'$w(v,x)$'+'\n'
        'Shared occlusion, flight cost, updates, three-window budget and Ground execution', '#eef1f4',10)
    box(ax,.035,.30,.43,.34,'Generic NBV\n\n'+r'$G_g(v)=\alpha\sum_x w(v,x)u(x)$'+'\n\n'
        'Global observation deficit\nwithout manipulation weighting', '#fff1e4',12)
    box(ax,.535,.30,.43,.34,'Manipulation-aware NBV (Ours)\n\n'+r'$G_o(v)=\alpha\sum_x w(v,x)u(x)M_{op}(x)$'+'\n\n'
        'Deficit within viable\nmanipulation-support footprints', '#e6f1fb',11)
    arrow(ax,(.30,.76),(.25,.64));arrow(ax,(.70,.76),(.75,.64))
    box(ax,.15,.075,.70,.13,r'$S_m(v)=G_m(v)-\lambda C(v),\quad\alpha=1-e^{-1/\tau}$'+'\n'
        'Same score-based sensing decision and screened-candidate stopping rule', '#eef1f4',11)
    arrow(ax,(.25,.30),(.35,.205));arrow(ax,(.75,.30),(.65,.205))
    ax.text(.5,.012,'Conceptual diagram, not experimental heatmaps. Opportunity and deficit are surrogates, not calibrated entropy.',ha='center',fontsize=8)
    save_figure(fig,out,'figure2_method_comparison')


def empirical_figures(out, analysis, failures):
    fig,ax=plt.subplots(figsize=(7.1,4.2));x=np.arange(3);width=.34
    for method,off in (('generic',-width/2),('ours',width/2)):
        groups=[analysis['tiers'][t] for t in ('easy','moderate','hard')]
        success=[g['a']+g['b' if method=='ours' else 'c'] for g in groups]
        totals=[g['n_scheduled'] for g in groups]
        bars=ax.bar(x+off,100*np.array(success)/totals,width,label=method.capitalize(),color=COLORS[method])
        for bar,n,d in zip(bars,success,totals):
            ax.text(bar.get_x()+bar.get_width()/2,bar.get_height()+1.8,f'{n}/{d}',ha='center',fontsize=10)
    ax.set(xticks=x,xticklabels=['Easy','Moderate','Hard'],ylim=(0,110),ylabel='Physical retrieval success (%)')
    ax.set_yticks(np.arange(0,101,20));ax.legend(frameon=False);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
    ax.set_title('Same-version paired evaluation; tier outcomes are descriptive',fontsize=11)
    save_figure(fig,out,'figure3_success_by_difficulty')
    fig,ax=plt.subplots(figsize=(8,4.7));y=np.arange(len(failures))
    for method,off in (('generic',-.18),('ours',.18)):
        vals=[r[method] for r in failures]
        ax.barh(y+off,vals,.34,label=f'{method.capitalize()} (N=96)',color=COLORS[method])
        for yy,val in zip(y+off,vals):ax.text(val+.25,yy,str(val),va='center',fontsize=9)
    ax.set(yticks=y,yticklabels=[r['label'] for r in failures],xlabel='First-terminal failures (tasks)',xlim=(0,34))
    ax.invert_yaxis();ax.legend(frameon=False,loc='lower right');ax.grid(axis='x',alpha=.2);ax.set_axisbelow(True)
    ax.set_title('Primary failures only: Generic 40, Ours 16; not-reached is not failure',fontsize=11)
    save_figure(fig,out,'figure4_failure_stages')


def git_ref(cwd, ref):
    try:
        return subprocess.check_output(['git','-C',str(cwd),'rev-parse',ref],text=True,
                                       stderr=subprocess.DEVNULL).strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def build(results, out, config_path):
    analysis=json.loads((results/'analysis-latest.json').read_text())
    config=json.loads(config_path.read_text());validate_production_manifest(config)
    if analysis['status']!='COMPLETE' or analysis['protocol_issues']:
        raise ValueError('Only the complete frozen study may be exported')
    if out.resolve()==results.resolve() or results.resolve() in out.resolve().parents:
        raise ValueError('Publication output must be outside original results')
    stats=paired_statistics(analysis['primary']['pairs'])
    if stats!=analysis['primary_inference']:
        raise ValueError('Independently reduced primary table differs from frozen result')
    out.mkdir(parents=True,exist_ok=True)
    write_table(out,'table1_overall',['Method','Retrieval success','N','Percent'],
                [[m.capitalize(),stats[m+'_successes'],stats['n'],f"{100*stats[m+'_successes']/stats['n']:.2f}"]
                 for m in ('ours','generic')])
    write_table(out,'table2_paired_outcomes',['Paired outcome','Symbol','Scenes'],
                [[label,k,stats[k]] for label,k in [('Both success','a'),('Ours only','b'),('Generic only','c'),('Both failure','d')]])
    low,high=stats['conservative_exact_rd_interval']
    write_table(out,'table3_mcnemar',['b','c','Exact p','Difference (pp)','95% CI low (pp)','CI high (pp)'],
                [[stats['b'],stats['c'],f"{stats['p_value']:.12g}",100*stats['risk_difference'],f'{100*low:.6f}',f'{100*high:.6f}']])
    failures=failure_table(analysis['slots'])
    write_table(out,'table4_failure_stages',['Stage','Generic failures','Ours failures'],
                [[r['label'],r['generic'],r['ours']] for r in failures])
    write_table(out,'failure_stage_mapping',['Slot','Scene','Method','Original stage','Original reason','Paper stage'],
                [[r['slot'],r['scene'],r['method'],r['failure_stage'],r['failure_reason'],failure_display_stage(r)]
                 for r in analysis['slots'] if r['method'] in COLORS and r['retrieval_success'] is False])
    write_table(out,'table5_difficulty',['Tier','N','Ours success','Generic success','Difference pp'],
                [[t.capitalize(),g['n_scheduled'],g['a']+g['b'],g['a']+g['c'],f"{100*g['risk_difference']:.2f}"]
                 for t in ('easy','moderate','hard') for g in [analysis['tiers'][t]]])
    write_table(out,'table6_conditional_efficiency',
                ['Metric','Joint success pairs','Mean Ours minus Generic','CI low','CI high'],
                [[k,m['n_pairs'],f"{m['mean_difference_ours_minus_generic']:.6f}",
                  *[f'{v:.6f}' for v in m['mean_difference_interval']]]
                 for k,m in analysis['conditional_efficiency']['metrics'].items()])
    write_table(out,'table7_auxiliary',['Comparator','N','Ours success','Comparator success','a','b','c','d'],
                [[m,g['n_scheduled'],g['a']+g['b'],g['a']+g['c'],*[g[k] for k in 'abcd']]
                 for m,g in analysis['auxiliary'].items()])
    write_table(out,'scene_seeds',['Scene','Tier','Seed'],[[s['id'],s['tier'],s['seed']] for s in config['scenes']])
    scenes={s['id']:s for s in config['scenes']}
    write_table(out,'task_order',['Slot','Scene','Seed','Method'],
                [[s['slot'],s['scene'],scenes[s['scene']]['seed'],s['method']] for s in config['slots']])
    pipeline_figure(out);method_figure(out);empirical_figures(out,analysis,failures)
    version=json.loads((ROOT/'configs/evaluation_version.json').read_text())
    refs=dict(release_tag='paper1-eval-finite-scan-v1',
        agent_tag_commit=git_ref(ROOT,'paper1-eval-finite-scan-v1^{}'),
        sim_tag_commit=git_ref(version['sim_root'],'paper1-eval-finite-scan-v1^{}'),
        actual_collection_versions=analysis['runtime_versions'],
        source_results_commit='6fec29b6ad54f76f5bdceb6b0fbee4e709fd374b',
        runtime_ancestry_commit=version['agent_runtime_commit'],
        config_files=['configs/paper1_eval.json','configs/current_sim_task.json','configs/evaluation_version.json',
                      'assets/rm4d_ground_task_v1/metadata.json'],
        original_rm4d_config=version['rm4d_config_relative'],
        numerical_environment=version['environment_observed_2026_09_10'],
        tag_probe_semantics='Local read-only Git probes; null means unavailable here, not a changed frozen runtime.',
        seed_count=len(config['scenes']), task_count=len(config['slots']),
        exact_replay_limitation='Asynchronous Gazebo/ROS/PX4 dynamics and scan phase are not bitwise replayable.',
        raw_data_limitation='Compact Git package cannot replay all sensor windows; retained local NPZ/bags required for qualitative/mechanism reproduction.')
    (out/'reproducibility.json').write_text(json.dumps(refs,indent=2)+'\n')
    (out/'statistics.json').write_text(json.dumps(dict(primary=stats,failures=failures,
            sensitivity=analysis['first_activation_invalid_as_failure'],
            conditional_efficiency=analysis['conditional_efficiency'],auxiliary=analysis['auxiliary']),indent=2)+'\n')
    print(json.dumps(dict(output=str(out),n=stats['n'],ours=stats['ours_successes'],generic=stats['generic_successes'],
                         failure_totals={m:sum(r[m] for r in failures) for m in COLORS})))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results',type=Path,default=ROOT/'outputs/paper1-final-eval-v1')
    parser.add_argument('--out',type=Path,default=ROOT/'paper/paper1_evaluation')
    parser.add_argument('--config',type=Path,default=ROOT/'configs/paper1_eval.json')
    args=parser.parse_args();build(args.results,args.out,args.config)


if __name__=='__main__':main()
