#!/usr/bin/env python3
"""Joint-phase check ONLY for the nominated optimistic-only failure plans.

Reuses unchanged finite-scan projection with retained perception occluders.
No search expansion, fitted radius, phase selection, GT, or belief updates.
"""
import csv
import json
from pathlib import Path
import numpy as np
from offline import read_json, read_npz, eligible, phase_windows, completion_bounds, write_csv
from environment_belief import EnvironmentGridSpec, BeliefConfig, EnvironmentBeliefGrid
from operational_gating.core import PerceivedTarget, OperationalEvidenceView
from operational_gating.subcell import AmbiguousEndpointEvidence
from reachability_guided_nbv.model import SensorModel, Viewpoint, NBVConfig
from reachability_guided_nbv.finite_scan import FiniteScan
from reachability_guided_nbv.operational_occlusion import build_operational_occlusion


def main():
    root = Path('outputs/paper1-final-eval-v1')
    out = Path(__file__).parent/'results'
    slots = {r['slot']: r for r in csv.DictReader((root/'slots.csv').open())}
    plans = [r for r in csv.DictReader((out/'predictions.csv').open())
             if r['role']=='primary' and r['method']=='generic'
             and r['confirmation_failure']=='True' and r['round']=='1' and r['best_tier']=='1']
    scan, sensor_dict, records = None, None, []
    for plan in plans:
        p = root/slots[plan['slot']]['selected_attempt']/'data/rounds/round-01'
        ranking, decision = read_json(p/'ranking.json'), read_json(p/'decision.json')
        fields, evidence = read_npz(p/'fields.npz'), read_npz(p/'operational_evidence.npz')
        grid = EnvironmentGridSpec(**ranking['a3_summary']['grid'])
        config = BeliefConfig(**ranking['a2_config'])
        belief = EnvironmentBeliefGrid(grid, config, **{k: fields['a2_'+k] for k in
            ('state', 'occupied_evidence', 'free_evidence', 'observation_count', 'unknown_score')})
        sidecar = AmbiguousEndpointEvidence(**{k: evidence['ambiguous_endpoint_'+v] for k, v in
            dict(points_xy='xy', observation_indices='observation_indices', row_indices='row_indices',
                 cell_ids='cell_ids', complete_vote_counts='complete_vote_counts').items()},
            profile=str(evidence['ambiguous_endpoint_profile']), radius_m=float(evidence['ambiguous_endpoint_radius_m']))
        target = PerceivedTarget(**read_json(p/'operational_summary.json')['target'])
        operational = OperationalEvidenceView(grid, config, target, **{k: evidence[k] for k in
            ('environment_occupied_votes', 'ambiguous_occupied_votes', 'target_occupied_votes',
             'ground_votes', 'ground_presence_votes')}, ambiguous_endpoints=sidecar)
        occlusion = build_operational_occlusion(belief, operational, assumed_height_m=ranking['config']['assumed_height_m'])
        if scan is None:
            sensor_dict = ranking['sensor']
            sensor = SensorModel(**{k: sensor_dict[k] for k in ('T_uav_lidar', 'min_elevation_deg', 'max_elevation_deg')})
            acq = ranking['acquisition']
            scan = FiniteScan.from_csv(acq['scan_path'], sensor=sensor, window_s=acq['window_s'],
                packet_rows=acq['packet_rows'], packet_period_s=acq['packet_period_s'], publisher_sdf_path=acq['publisher_sdf_path'])
        assert ranking['sensor'] == sensor_dict
        masks, cache = [], {}
        for idx in (int(plan['first_index']), int(plan['second_index'])):
            if idx not in cache:
                view = Viewpoint(**ranking['candidates'][idx]['viewpoint'])
                pred = scan.predict(belief, view, NBVConfig(**ranking['config']), occlusion=occlusion)
                np.testing.assert_allclose(pred.opportunity, fields['observation_opportunity'][idx], rtol=0, atol=1e-12)
                mask = phase_windows(pred.packet_hits, scan.metadata['window_packets']).reshape(scan.metadata['packet_count'], -1)
                np.testing.assert_allclose(mask.mean(axis=0), pred.opportunity.ravel(), rtol=0, atol=1e-12)
                cache[idx] = mask
            masks.append(cache[idx])
        # completion_bounds handles any two sets by stacking and taking off-diagonal block.
        h = evidence['ground_presence_votes'].ravel()
        supports = [a['operational']['covered_cells'] for a in eligible(decision['assessments'])]
        phases = len(masks[0])
        joint = completion_bounds(h, supports, np.concatenate(masks).astype(float), 2)[0][:phases, phases:]
        # Unknown relative phase: report every phase offset, not a calibrated probability.
        fractions = [joint[np.arange(phases), (np.arange(phases)+offset)%phases].mean() for offset in range(phases)]
        records.append(dict(slot=int(plan['slot']), scene=plan['scene'], first_view=int(plan['first_index']),
            second_view=int(plan['second_index']), phase_pairs=joint.size, completing_phase_pairs=int(joint.sum()),
            any_joint_phase=bool(joint.any()), all_joint_phases=bool(joint.all()),
            minimum_fraction_over_relative_offsets=min(fractions), maximum_fraction_over_relative_offsets=max(fractions),
            semantics='nominal_scan_scenarios_not_return_probability'))
        print(json.dumps(records[-1]), flush=True)
    write_csv(out/'joint_phase_diagnostics.csv', records)


if __name__ == '__main__':
    main()
