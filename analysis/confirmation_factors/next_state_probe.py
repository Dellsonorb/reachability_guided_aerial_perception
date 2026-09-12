#!/usr/bin/env python3
"""Four post-hoc predictions, fixed model: old/new pose x old/new perception.

Next-state information is used ONLY to diagnose a recorded forecast failure.
No viewpoint is added to any online catalogue and no observation is generated.
"""
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from analysis.completion_aware.offline import read_json,read_npz
from environment_belief import EnvironmentGridSpec,BeliefConfig,EnvironmentBeliefGrid
from operational_gating.core import PerceivedTarget,OperationalEvidenceView
from operational_gating.subcell import AmbiguousEndpointEvidence
from reachability_guided_nbv.model import SensorModel,Viewpoint,NBVConfig
from reachability_guided_nbv.finite_scan import FiniteScan
from reachability_guided_nbv.operational_occlusion import build_operational_occlusion


def load(p):
    r=read_json(p/'ranking.json');f=read_npz(p/'fields.npz');e=read_npz(p/'operational_evidence.npz')
    grid=EnvironmentGridSpec(**r['a3_summary']['grid']);config=BeliefConfig(**r['a2_config'])
    b=EnvironmentBeliefGrid(grid,config,**{k:f['a2_'+k]for k in
        ('state','occupied_evidence','free_evidence','observation_count','unknown_score')})
    side=AmbiguousEndpointEvidence(**{k:e['ambiguous_endpoint_'+v]for k,v in
        dict(points_xy='xy',observation_indices='observation_indices',row_indices='row_indices',
             cell_ids='cell_ids',complete_vote_counts='complete_vote_counts').items()},
        profile=str(e['ambiguous_endpoint_profile']),radius_m=float(e['ambiguous_endpoint_radius_m']))
    op=OperationalEvidenceView(grid,config,PerceivedTarget(**read_json(p/'operational_summary.json')['target']),
        **{k:e[k]for k in ('environment_occupied_votes','ambiguous_occupied_votes','target_occupied_votes',
                           'ground_votes','ground_presence_votes')},ambiguous_endpoints=side)
    return r,f,e,b,build_operational_occlusion(b,op,assumed_height_m=r['config']['assumed_height_m'])


def main():
    p=ROOT/'outputs/development/confirmation-v1/slot-07-hard023-confirmation/attempt/data/rounds'
    states=[load(p/f'round-{i:02d}')for i in (1,2)]
    r=states[0][0];acq=r['acquisition'];sensor=SensorModel(**{k:r['sensor'][k]for k in
        ('T_uav_lidar','min_elevation_deg','max_elevation_deg')})
    scan=FiniteScan.from_csv(acq['scan_path'],sensor=sensor,window_s=acq['window_s'],
        packet_rows=acq['packet_rows'],packet_period_s=acq['packet_period_s'],publisher_sdf_path=acq['publisher_sdf_path'])
    ids=[81,0];poses=[Viewpoint(**s[0]['candidates'][i]['viewpoint'])for s,i in zip(states,ids)]
    d=read_json(p/'round-02/decision.json');q=next(a for a in d['assessments']if a['source_id']==500)
    h=states[1][2]['ground_presence_votes'].ravel();cells=np.array(q['operational']['covered_cells']);needed=cells[h[cells]<2]
    results=[]
    for state,s in enumerate(states):
        for pose,v in enumerate(poses):
            pred=scan.predict(s[3],v,NBVConfig(**s[0]['config']),occlusion=s[4]);w=pred.opportunity.ravel()
            if state==pose:np.testing.assert_allclose(pred.opportunity,s[1]['observation_opportunity'][ids[pose]],rtol=0,atol=1e-12)
            results.append(dict(evidence_round=state+1,pose='original_planned_second'if pose==0 else'next_reanchored_current',
                pose_xyz=list(v.position_xyz),pose_yaw=v.yaw_rad,needed_cells=len(needed),
                all_phase_hits=int((w[needed]==1).sum()),any_phase_hits=int((w[needed]>0).sum()),
                zero_opportunity_cells=needed[w[needed]==0].tolist()))
    out=Path(__file__).parent/'offline_results/next_state_probe.json'
    out.write_text(json.dumps(dict(case='previous_batch_Hard023_confirmation_source500',
        semantics='posthoc_fixed_model_factor_probe_not_policy_or_actual_return_guarantee',predictions=results),indent=2)+'\n')
    print(out.read_text())


if __name__=='__main__':main()
