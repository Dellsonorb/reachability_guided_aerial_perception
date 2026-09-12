#!/usr/bin/env python3
"""Manuscript presentation only: read frozen CSVs/fields, never rerun a method.

Use the existing Python workflow. New schematics are explicitly conceptual;
empirical panels contain only saved results. No sampling, smoothing or new tests.
"""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm, ListedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch, Polygon, Rectangle

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
ASSETS = HERE.parent / 'paper1_evaluation'
OUT = HERE / 'figures'
GENERIC, OURS = '#B56B36', '#246A92'
INK, MUTED, PALE = '#25343D', '#60717A', '#F2F5F6'
ENV, TARGET, FOOTPRINT = '#805A79', '#E9C45F', '#008B8B'
WIDTH_MM = 162


def load_tables():
    names = dict(overall='table1_overall', paired='table2_paired_outcomes',
                 inference='table3_mcnemar', failures='table4_failure_stages',
                 difficulty='table5_difficulty', efficiency='table6_conditional_efficiency',
                 auxiliary='table7_auxiliary')
    result = {}
    for key, name in names.items():
        with (ASSETS / f'{name}.csv').open(newline='') as handle:
            result[key] = list(csv.DictReader(handle))
    return result


def footprint_vertices(anchor, footprint):
    corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]], dtype=float)
    corners *= [footprint['half_length_m'], footprint['half_width_m']]
    yaw = anchor['yaw']
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    return corners @ rotation.T + [anchor['x'], anchor['y']]


TRANSLATION = {
    'Method': '方法', 'Retrieval success': '取回成功数', 'N': '场景数', 'Percent': '成功率（\\%）',
    'Generic': 'Generic', 'Ours': 'Ours', 'Paired outcome': '配对结果', 'Symbol': '符号',
    'Scenes': '场景数', 'Both success': '两者均成功', 'Ours only': '仅 Ours 成功',
    'Generic only': '仅 Generic 成功', 'Both failure': '两者均失败',
    'Exact p': '精确 p 值', 'Difference (pp)': '差值（百分点）',
    '95% CI low (pp)': '95\\% CI 下限', 'CI high (pp)': 'CI 上限',
    'Stage': '首次终止阶段', 'Generic failures': 'Generic 失败数', 'Ours failures': 'Ours 失败数',
    'UAV observation': '无人机观测', 'Confirmed candidate': '候选确认',
    'Screened candidate': '候选筛选', 'BUNKER navigation': 'BUNKER 导航',
    'D435 refinement': 'D435 精定位', 'D_exec (refined pregrasp)': '$D_{\\mathrm{exec}}$（精定位后预抓取）',
    'Grasp (descend / close)': '抓取（下降／闭合）', 'Lift / retention': '提升／保持',
    'Tier': '难度', 'Ours success': 'Ours 成功数', 'Generic success': 'Generic 成功数',
    'Difference pp': '差值（百分点）', 'Easy': '简单', 'Moderate': '中等', 'Hard': '困难',
    'Metric': '指标', 'Joint success pairs': '共同成功配对数',
    'Mean Ours minus Generic': '平均差（Ours−Generic）', 'CI low': 'CI 下限', 'CI high': 'CI 上限',
    'windows': '观测窗口数', 'resolved_location_changes': '可判定位置变化次数',
    'T_active_sim': '主动感知时间（s）', 'T_ground_sim': '地面执行时间（s）',
    'T_task_sim': '全任务时间（s）', 'Comparator': '对照', 'Comparator success': '对照成功数',
    'fixed': '固定视点', 'rm4d_only': '仅 RM4D', 'no_cost': '去除飞行成本',
    'no_occlusion': '去除遮挡预测',
}


def chinese_table(rows):
    def cell(value):
        return TRANSLATION.get(value, value.replace('_', r'\_').replace('%', r'\%'))
    lines = [r'\begin{tabular}{' + 'l' * len(rows[0]) + '}', r'\toprule']
    lines.append(' & '.join(map(cell, rows[0])) + r' \\')
    lines.append(r'\midrule')
    lines.extend(' & '.join(map(cell, row)) + r' \\' for row in rows[1:])
    lines.extend([r'\bottomrule', r'\end{tabular}'])
    return '\n'.join(lines) + '\n'


def style():
    plt.rcParams.update({'font.family': 'sans-serif', 'font.sans-serif': ['DejaVu Sans'], 'font.size': 8,
                         'axes.labelsize': 8, 'axes.titlesize': 9,
                         'xtick.labelsize': 7.5, 'ytick.labelsize': 7.5,
                         'text.color': INK, 'axes.labelcolor': INK,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'axes.linewidth': .6, 'legend.frameon': False,
                         'svg.fonttype': 'none', 'pdf.fonttype': 42,
                         'savefig.facecolor': 'white', 'svg.hashsalt': 'paper1-editorial-v2'})


def save(fig, name, auditor, **alignment):
    fig.canvas.draw()
    require_matplotlib_panel_alignment = auditor
    require_matplotlib_panel_alignment(fig, json_out=OUT / f'{name}.alignment.json', tolerance_pt=1.5,
            gutter_tolerance_pt=1.5, strict=True, **alignment)
    fig.savefig(OUT / f'{name}.pdf', metadata={'CreationDate': None, 'ModDate': None})
    fig.savefig(OUT / f'{name}.svg', metadata={'Date': None})
    fig.savefig(OUT / f'{name}.png', dpi=600)
    plt.close(fig)


def panel(ax, letter, title):
    ax.annotate(letter, xy=(0, 1), xycoords='axes fraction', xytext=(0, 9),
                textcoords='offset points', fontsize=10, weight='bold', va='bottom')
    ax.annotate(title, xy=(0, 1), xycoords='axes fraction', xytext=(13, 9),
                textcoords='offset points', fontsize=8.5, va='bottom')


def box(ax, xy, size, title, detail='', color=PALE):
    x, y = xy
    w, h = size
    ax.add_patch(Rectangle((x, y), w, h, facecolor=color, edgecolor='#C5D1D6', lw=.6))
    ax.text(x+w/2, y+h*.66 if detail else y+h/2, title, ha='center', va='center',
            fontsize=8.5, weight='bold')
    if detail:
        ax.text(x+w/2, y+h*.27, detail, ha='center', va='center', fontsize=8)


def arrow(ax, start, end, color=MUTED, style='-'):
    ax.add_patch(FancyArrowPatch(start, end, arrowstyle='-|>', mutation_scale=9,
                                color=color, linewidth=.9, linestyle=style, shrinkA=1, shrinkB=1))


def pipeline(auditor):
    fig, ax = plt.subplots(figsize=(WIDTH_MM/25.4, 99/25.4))
    fig.subplots_adjust(left=.01, right=.99, top=.98, bottom=.02)
    ax.set(xlim=(0, 104), ylim=(0, 100)); ax.axis('off')
    ax.text(1, 96, 'Future manipulation determines where to observe', fontsize=10, weight='bold')
    box(ax, (1, 74), (28, 17), 'Perceive the target', 'P450 aerial RGB-D')
    box(ax, (36, 74), (28, 17), 'Validate base poses', 'RM4D + joint margin')
    box(ax, (71, 74), (28, 17), 'Project exact footprints', 'Exact support with $B_t=0$', '#E7F1F7')
    arrow(ax, (29,82), (36,82)); arrow(ax, (64,82), (71,82))
    box(ax, (1, 47), (28, 18), 'Ground evidence', 'MID360 → $u_t, H_t, B_t$')
    box(ax, (36, 47), (28, 18), 'Task-weighted deficit', '$U_{task}=u_t M_{op}$', '#E7F1F7')
    box(ax, (71, 47), (28, 18), 'Confirm + screen', r'$H_t\geq2$, $B_t=0$; full robot')
    arrow(ax, (85,74), (85,69)); arrow(ax, (85,69), (50,69)); arrow(ax, (50,69), (50,65))
    arrow(ax, (29,56), (36,56)); arrow(ax, (64,56), (71,56), style='--')
    arrow(ax, (85,47), (85,35))
    ax.text(82, 40, 'Not ready',ha='right',fontsize=7.5)
    box(ax, (71, 21), (28, 14), 'Next observation', r'$G(U_{task})-\lambda C$', '#E7F1F7')
    box(ax, (36, 21), (28, 14), 'Fly, settle, observe', 'Actual sensor update')
    arrow(ax,(71,28),(64,28)); arrow(ax,(36,28),(15,28)); arrow(ax,(15,28),(15,47))
    arrow(ax,(99,56),(102,56)); arrow(ax,(102,56),(102,16))
    ax.text(100,40,'Ready',ha='right',fontsize=7.5)
    stages = [(1,'Navigate + stop'),(26,'D435 refinement'),(51,'Grasp'),(76,'Lift + retain')]
    for x, label in stages:
        box(ax, (x, 2), (23, 12), label, color='#EDF3ED')
    for x in (24,49,74):
        arrow(ax, (x,8), (x+2,8))
    # The vertical handoff enters the common execution sequence, not an NBV branch.
    arrow(ax, (102,16), (12,16)); arrow(ax, (12,16), (12,14))
    save(fig, 'figure1_system_pipeline', auditor)


def method_comparison(auditor):
    fig, axes = plt.subplots(1, 2, figsize=(WIDTH_MM/25.4, 96/25.4))
    fig.subplots_adjust(left=.04, right=.97, top=.81, bottom=.28, wspace=.15)
    for i, ax in enumerate(axes):
        ax.set(xlim=(0,10), ylim=(0,6)); ax.axis('off')
        ax.add_patch(Rectangle((.2,.3),9.6,5.4,facecolor='#F8F9FA',edgecolor='#CDD6DA',lw=.6))
        # Geometric cartoons only: no numerical heatmap or simulated measurements.
        ax.add_patch(Rectangle((.7,1),4.1,4, facecolor='#E3E8EB', edgecolor='none'))
        ax.text(2.75,3.2,'Other\nunknown\nspace',ha='center',va='center',fontsize=8)
        for x,y,yaw in [(6.15,1.3,-.18),(7.25,1.75,.2)]:
            q=dict(x=x,y=y,yaw=yaw)
            ax.add_patch(Polygon(footprint_vertices(q,dict(half_length_m=1.0,half_width_m=.67)),
                                 facecolor='#D9E8EE',edgecolor=FOOTPRINT,lw=.85))
        ax.add_patch(Rectangle((6.15,3.2),1.2,.4,facecolor=TARGET,edgecolor=INK,lw=.6))
        ax.text(6.75,4.25,'Known target',ha='center',fontsize=8)
        if i == 0:
            ax.add_patch(Rectangle((.7,1),4.1,4,fill=False,edgecolor=GENERIC,lw=1.4))
            ax.add_patch(Rectangle((5, .5),3.6,2.5,fill=False,edgecolor=GENERIC,lw=1.4))
        else:
            ax.add_patch(Rectangle((5,.5),3.6,2.5,fill=False,edgecolor=OURS,lw=1.4))
        panel(ax, chr(97+i), ['Generic NBV', 'Manipulation-aware NBV'][i])
    fig.text(.5,.955,'Only the spatial weighting changes',ha='center',fontsize=10,weight='bold')
    fig.text(.5,.89,'Shared candidate views · finite-scan opportunity · cost · budget · execution',ha='center',fontsize=7.5)
    fig.text(.26,.20,r'$G_g(v)=\alpha\sum_x w_t(v,x)\,u_t(x)$',ha='center',fontsize=10)
    fig.text(.76,.20,r'$G_o(v)=\alpha\sum_x w_t(v,x)\,u_t(x)M_{op,t}(x)$',ha='center',fontsize=10)
    fig.text(.26,.12,'All observable deficit contributes',ha='center',fontsize=8,color=GENERIC)
    fig.text(.76,.12,'Only stance-relevant deficit contributes',ha='center',fontsize=8,color=OURS)
    fig.text(.5,.035,'Spatial schematic, not a measured map or predicted winning viewpoint.',ha='center',fontsize=7)
    save(fig,'figure2_method_comparison',auditor)


def success(data, auditor):
    fig, (ax, pairs) = plt.subplots(1,2,figsize=(WIDTH_MM/25.4, 89/25.4))
    fig.subplots_adjust(left=.125,right=.96,top=.80,bottom=.27,wspace=.50)
    panel(ax,'a','Retrieval by difficulty'); panel(pairs,'b','Paired outcomes')
    for j, row in enumerate(data['difficulty']):
        n = int(row['N']); g = int(row['Generic success']); o = int(row['Ours success'])
        xg, xo = 100*g/n,100*o/n
        ax.plot([xg,xo],[j,j],color='#B5C3CC',lw=1.5,zorder=1)
        ax.scatter([xg],[j],s=27,marker='s',color=GENERIC,zorder=3)
        ax.scatter([xo],[j],s=30,marker='o',color=OURS,zorder=3)
        ax.annotate(f'{g}/{n}',(xg,j),xytext=(-3,-14),textcoords='offset points',ha='center',fontsize=7.5,color=GENERIC)
        ax.annotate(f'{o}/{n}',(xo,j),xytext=(0,9),textcoords='offset points',ha='center',fontsize=7.5,color=OURS)
    ax.set(yticks=range(3),yticklabels=[r['Tier'] for r in data['difficulty']],
           xlim=(0,103),ylim=(2.5,-.55),xticks=[0,25,50,75,100],xlabel='Physical retrieval success (%)')
    ax.spines['left'].set_visible(False); ax.tick_params(axis='y',length=0)
    values=[int(r['Scenes']) for r in data['paired']]
    labels=['Both succeed','Ours only','Generic only','Both fail']
    colors=['#AEBCC4',OURS,GENERIC,'#DEE3E7']
    pairs.barh(range(4),values,height=.52,color=colors)
    pairs.set(yticks=range(4),yticklabels=labels,ylim=(3.5,-.5),xlim=(0,61),xlabel='Paired scenes (n = 96)',xticks=[0,20,40,60])
    pairs.spines['left'].set_visible(False); pairs.tick_params(axis='y',length=0)
    for j,n in enumerate(values): pairs.text(n+1.5,j,str(n),va='center',fontsize=8)
    handles=[Line2D([],[],marker='s',linestyle='',color=GENERIC,label='Generic'),
             Line2D([],[],marker='o',linestyle='',color=OURS,label='Ours')]
    fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.5,.995),ncol=2,fontsize=8)
    inf=data['inference'][0]
    fig.text(.5,.12,f'Paired difference +{float(inf["Difference (pp)"]):.2f} pp  |  95% CI +{float(inf["95% CI low (pp)"]):.2f} to +{float(inf["CI high (pp)"]):.2f} pp',ha='center',fontsize=8)
    mantissa,exponent=f'{float(inf["Exact p"]):.2e}'.split('e')
    power=str(int(exponent)).translate(str.maketrans('-0123456789','⁻⁰¹²³⁴⁵⁶⁷⁸⁹'))
    fig.text(.5,.045,f'Exact McNemar p = {mantissa} × 10{power}  |  Tier values are descriptive; no tier-wise tests.',ha='center',fontsize=7.5)
    save(fig,'figure3_success_by_difficulty',auditor)


def failure(data, auditor):
    fig, ax = plt.subplots(figsize=(WIDTH_MM/25.4,96/25.4))
    fig.subplots_adjust(left=.32,right=.95,top=.88,bottom=.17)
    labels=['UAV observation','Candidate confirmation','Candidate screening','BUNKER navigation',
            'D435 refinement','Refined pregrasp (D_exec)','Grasp: descent / closure','Lift / retention']
    for offset,method,color in [(-.17,'Generic',GENERIC),(.17,'Ours',OURS)]:
        counts=[int(r[f'{method} failures']) for r in data['failures']]
        ys=np.arange(8)+offset
        ax.barh(ys,counts,height=.28,color=color,label=method)
        for y,n in zip(ys,counts): ax.text(n+.45,y,str(n),va='center',fontsize=7.5,color=color)
    ax.set(yticks=range(8),yticklabels=labels,ylim=(7.6,-.6),xlim=(0,32),
           xlabel='First-terminal failures (out of 96 tasks per method)',xticks=[0,5,10,15,20,25,30])
    ax.spines['left'].set_visible(False); ax.tick_params(axis='y',length=0)
    ax.legend(loc='upper right',fontsize=8)
    fig.text(.04,.96,'Where the complete task stopped',fontsize=10,weight='bold')
    fig.text(.04,.025,'Zero bins are retained. Unreached downstream stages are not additional failures.',fontsize=7.5)
    save(fig,'figure4_failure_stages',auditor)


def qualitative(auditor):
    sys.path.insert(0,str(ROOT/'scripts'))
    from paper1_qualitative_figures import load_examples
    metadata=json.loads((ASSETS/'qualitative_examples.json').read_text())
    cases=load_examples(metadata['results_root'])
    fig, axes=plt.subplots(3,2,figsize=(WIDTH_MM/25.4,208/25.4))
    fig.subplots_adjust(left=.09,right=.88,top=.92,bottom=.19,hspace=.66,wspace=.32)
    support_cmap=ListedColormap(['#F0F1F2','#C4DED8','#73B3A5','#237765'])
    support_norm=BoundaryNorm(np.arange(-.5,4.5),4)
    bounds=(min(c['extent'][0] for c in cases),max(c['extent'][1] for c in cases),
            min(c['extent'][2] for c in cases),max(c['extent'][3] for c in cases))
    poses_all=np.asarray([p for c in cases for p in c['poses']])
    xb=(min(bounds[0],poses_all[:,0].min())-.3,bounds[1]+.1)
    yb=(min(bounds[2],poses_all[:,1].min())-.3,bounds[3]+.1)
    for col,case in enumerate(cases):
        op,fields=case['_operational'],case['_fields']; extent=case['extent']
        color=[GENERIC,OURS][col]; anchor=case['anchor']
        verts=footprint_vertices(anchor,case['footprint'])
        for row in range(3):
            ax=axes[row,col]
            if row < 2:
                im=ax.imshow(op['ground_presence_votes'],origin='lower',extent=extent,
                             interpolation='nearest',cmap=support_cmap,norm=support_norm)
                occupied=np.ma.masked_where(op['environment_occupied_votes']==0,np.ones_like(op['environment_occupied_votes']))
                ax.imshow(occupied,origin='lower',extent=extent,interpolation='nearest',cmap=ListedColormap([ENV]),vmin=0,vmax=1)
            else:
                deficit_im=ax.imshow(fields['a3_task_relevant_uncertainty'],origin='lower',extent=extent,
                                     interpolation='nearest',cmap='magma',vmin=0,vmax=1)
            ax.add_patch(Polygon(op['target_vertices_xy'],facecolor=TARGET,edgecolor=INK,lw=.65))
            ax.add_patch(Polygon(verts,fill=False,edgecolor=FOOTPRINT,lw=1.25))
            ax.plot(anchor['x'],anchor['y'],'+',color=FOOTPRINT,ms=4)
            ax.set_xlabel('map x (m)',labelpad=2); ax.set_ylabel('map y (m)',labelpad=2)
            # Equal metric scale, shared display bounds, native data untouched.
            # Pad only the viewing frame to match the measured axes aspect.
            pos=ax.get_position(original=True)
            ratio=pos.width*fig.get_figwidth()/(pos.height*fig.get_figheight())
            xx=xb if row==0 else bounds[:2]; yy=yb if row==0 else bounds[2:]
            cx,cy=np.mean(xx),np.mean(yy)
            spanx=max(xx[1]-xx[0],(yy[1]-yy[0])*ratio)
            spany=spanx/ratio
            ax.set_xlim(cx-spanx/2,cx+spanx/2); ax.set_ylim(cy-spany/2,cy+spany/2)
            ax.set_aspect('equal',adjustable='box')
            ax.tick_params(labelsize=7)
            titles=['Observation poses','Ground support','Task deficit']
            panel(ax,chr(97+row*2+col),titles[row])
        poseax=axes[0,col]; poses=np.asarray(case['poses'])
        poseax.plot(poses[:,0],poses[:,1],':',color=color,lw=.9)
        for j,(x,y,z,yaw) in enumerate(poses):
            poseax.plot(x,y,'o',color=color,ms=3)
            poseax.arrow(x,y,.36*np.cos(yaw),.36*np.sin(yaw),color=color,width=.008,head_width=.12,length_includes_head=True)
            dx=6 if x < xb[0]+.6 else -12
            offset=(18,4) if j==2 else (dx,5 if j==0 else 16)
            poseax.annotate(str(j+1),(x,y),xytext=offset,textcoords='offset points',fontsize=7.5,color=color)
        missing=anchor['operational']['ground_missing_cells']; total=len(anchor['operational']['covered_cells'])
        axes[1,col].text(.5,-.36,f'{missing}/{total} footprint cells lack ≥2 votes',transform=axes[1,col].transAxes,
                         ha='center',fontsize=7.5)
        axes[2,col].text(.5,-.36,'Confirmed: '+' → '.join(map(str,case['confirmed_counts'])),
                         transform=axes[2,col].transAxes,ha='center',fontsize=8,color=color)
    fig.text(.27,.978,'Generic · budget exhausted',ha='center',color=GENERIC,fontsize=9,weight='bold')
    fig.text(.73,.978,'Ours · retrieval success',ha='center',color=OURS,fontsize=9,weight='bold')
    r1=axes[1,1].get_position(); r2=axes[2,1].get_position()
    cb1=fig.add_axes([.915,r1.y0,.014,r1.height]); fig.colorbar(im,cax=cb1,ticks=[0,1,2,3],label='Ground-vote windows')
    cb2=fig.add_axes([.915,r2.y0,.014,r2.height]); fig.colorbar(deficit_im,cax=cb2,ticks=[0,.5,1],label='Task deficit')
    handles=[Rectangle((0,0),1,1,facecolor=ENV,label='Environment occupied'),
             Rectangle((0,0),1,1,facecolor=TARGET,label='Perceived target + allowance'),
             Line2D([],[],color=FOOTPRINT,lw=1.3,label='Exact base footprint'),
             Line2D([],[],color=MUTED,linestyle=':',marker='o',ms=3,label='Observation order, not trajectory')]
    fig.legend(handles=handles,loc='lower center',bbox_to_anchor=(.5,.04),ncol=2,fontsize=7)
    fig.text(.5,.018,'Hard-015 · Same post-hoc illustration and native evidence as the original figure',ha='center',fontsize=7)
    save(fig,'figure5_qualitative',auditor,exclude_axes=[cb1,cb2])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--skill-root',type=Path,required=True,help='nature-skills checkout (figure QA tools)')
    args=parser.parse_args()
    sys.path.insert(0,str(args.skill_root/'skills/nature-figure/scripts'))
    from audit_panel_alignment import require_matplotlib_panel_alignment
    OUT.mkdir(exist_ok=True)
    style(); data=load_tables()
    for name in ASSETS.glob('table*.csv'):
        with name.open(newline='') as handle: rows=list(csv.reader(handle))
        target=HERE/'zh'/'tables'/f'{name.stem}.tex'
        target.parent.mkdir(parents=True,exist_ok=True)
        target.write_text(chinese_table(rows))
    pipeline(require_matplotlib_panel_alignment)
    method_comparison(require_matplotlib_panel_alignment)
    success(data,require_matplotlib_panel_alignment)
    failure(data,require_matplotlib_panel_alignment)
    qualitative(require_matplotlib_panel_alignment)


if __name__=='__main__':
    main()
