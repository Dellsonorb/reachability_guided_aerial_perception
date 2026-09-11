#!/usr/bin/env python3
"""Render Figure 5 from retained runtime evidence, without rerunning perception."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


SELECTION = ('Post hoc illustrative outcome-selected example: lowest scheduled primary '
             'Hard pair with Ours retrieval success, Generic failure, and retained accepted '
             'window snapshots for both methods. Not representative, predeclared, or an '
             'additional independent test; no score-gap maximization. Reproduction is frozen '
             'to eval-hard-015, seed 1395226981, Generic slot 001 and Ours slot 002; '
             'missing local evidence does not trigger reselection.')


def select_pair(rows, tier='hard'):
    groups = {}
    for row in rows:
        if (row['tier'] == tier and row['comparison_role'] == 'primary'
                and row['status'] == 'VALID_TRIAL' and row['method'] in ('generic', 'ours')):
            groups.setdefault(row['scene'], {})[row['method']] = row
    eligible = [g for g in groups.values() if set(g) == {'generic', 'ours'}
                and g['generic']['retrieval_success'] == 'False'
                and g['ours']['retrieval_success'] == 'True']
    if not eligible:
        raise ValueError('no eligible discordant pair with retained evidence')
    first = min(eligible, key=lambda g: min(int(r['slot']) for r in g.values()))
    return [first['generic'], first['ours']]


def uav_pose(T_map_sensor, T_uav_sensor):
    matrix = np.asarray(T_map_sensor) @ np.linalg.inv(np.asarray(T_uav_sensor))
    return [*matrix[:3, 3].tolist(), float(np.arctan2(matrix[1, 0], matrix[0, 0]))]


def grid_extent(origin, resolution, shape):
    return (float(origin[0]), float(origin[0] + shape[1]*resolution),
            float(origin[1]), float(origin[1] + shape[0]*resolution))


def validate_selected_anchor(anchor, selected_event):
    if (any(anchor[k] != selected_event[k] for k in ('candidate_id', 'source_id'))
            or not np.allclose([anchor[k] for k in ('x', 'y', 'yaw')],
                               [selected_event[k] for k in ('x', 'y', 'yaw')],
                               rtol=0, atol=1e-10)):
        raise ValueError('shown exact anchor does not agree with runtime A5_SELECTED')


def load_examples(results):
    results = Path(results)
    with (results/'slots.csv').open(newline='') as handle:
        rows = list(csv.DictReader(handle))
    frozen = []
    for slot, method in [(1, 'generic'), (2, 'ours')]:
        matches = [row for row in rows if row['slot'] == str(slot)]
        if len(matches) != 1:
            raise ValueError(f'Frozen Figure 5 requires exactly one slots.csv row for slot {slot}')
        row = matches[0]
        expected = dict(scene='eval-hard-015', seed='1395226981', method=method, windows='3',
                        tier='hard', comparison_role='primary', status='VALID_TRIAL',
                        retrieval_success=str(method == 'ours'),
                        selected_attempt=f'slot-{slot:03d}-eval-hard-015-{method}/attempt')
        if any(row[key] != value for key, value in expected.items()):
            raise ValueError(f'Frozen Figure 5 metadata mismatch for slot {slot}')
        data = results/row['selected_attempt']/'data'
        required = [data/'events.jsonl']
        for n in range(1, 4):
            required.append(data/f'observation_{n:02d}.npz')
            required.extend(data/'rounds'/f'round-{n:02d}'/name for name in
                            ('decision.json', 'ranking.json', 'fields.npz',
                             'operational_evidence.npz', 'operational_summary.json'))
        missing = next((p for p in required if not p.is_file()), None)
        if missing is not None:
            raise FileNotFoundError(f'Frozen Figure 5 illustration evidence missing: {missing}; '
                                    'restore the selected raw files; no alternative pair is substituted')
        frozen.append(row)
    cases = []
    for row in frozen:
        attempt = results/row['selected_attempt']
        sources = {}

        def record(path):
            path = Path(path)
            rel = str(path.relative_to(results))
            sources[rel] = dict(path=rel)
            return path

        def read_json(path):
            return json.loads(record(path).read_text())

        def read_npz(path):
            with np.load(record(path), allow_pickle=False) as data:
                return {k: data[k].copy() for k in data.files}

        record(results/'slots.csv')
        events_path = record(attempt/'data/events.jsonl')
        events = [(i, json.loads(line)) for i, line in enumerate(events_path.read_text().splitlines(), 1)]
        accepted = {e['round']: (i, e) for i, e in events if e.get('state') == 'A5_OBSERVATION'}
        windows, decisions, poses = [], [], []
        for n in range(1, int(row['windows'])+1):
            directory = attempt/'data/rounds'/f'round-{n:02d}'
            ranking = read_json(directory/'ranking.json')
            decision = read_json(directory/'decision.json')
            packet = read_npz(attempt/'data'/f'observation_{n:02d}.npz')
            line, event = accepted[n]
            if str(packet['frame_id']) != event['sensor_frame']:
                raise ValueError('accepted packet sensor frame mismatch')
            pose = uav_pose(packet['chunk_T_map_sensor'][0], ranking['sensor']['T_uav_lidar'])
            windows.append(dict(round=n, packet_start_stamp_s=float(packet['chunk_stamps_s'][0]),
                                packet_end_stamp_s=float(packet['chunk_stamps_s'][-1]),
                                accepted_event_ros_time=event['ros_time'],
                                accepted_event_source=f'{events_path.relative_to(results)}:{line}',
                                observation_source=str((attempt/'data'/f'observation_{n:02d}.npz').relative_to(results)),
                                sensor_frame=str(packet['frame_id']),
                                packet_count=len(packet['chunk_stamps_s']),
                                first_packet_uav_map_xyz_yaw=pose))
            poses.append(pose)
            decisions.append(decision)
        operational = read_npz(directory/'operational_evidence.npz')
        fields = read_npz(directory/'fields.npz')
        summary = read_json(directory/'operational_summary.json')
        if str(operational['frame_id']) != 'map' or ranking['a3_summary']['grid']['frame_id'] != 'map':
            raise ValueError('Figure 5 requires saved map frame fields')
        anchor = decision['selected_candidate'] or decision['assessments'][0]
        runtime_line, runtime_decision = next((i, e) for i, e in reversed(events)
                                             if e.get('state') == 'A5_DECISION'
                                             and e.get('round') == n)
        selected = [(i, e) for i, e in events if e.get('state') == 'A5_SELECTED']
        selected_line, selected_event = selected[-1] if selected else (None, None)
        if row['retrieval_success'] == 'True':
            if selected_event is None:
                raise ValueError('successful illustration requires runtime A5_SELECTED')
            validate_selected_anchor(anchor, selected_event)
        extent = grid_extent(operational['origin_xy'], float(operational['resolution_m']),
                             operational['ground_presence_votes'].shape)
        cases.append(dict(slot=int(row['slot']), scene=row['scene'], method=row['method'],
                          seed=int(row['seed']), selected_attempt=row['selected_attempt'],
                          retrieval_success=row['retrieval_success'] == 'True',
                          physical_status=row['physical_status'], failure_reason=row['failure_reason'],
                          terminal_event=json.loads(row['first_terminal_event']),
                          anchor=anchor, anchor_selection=('selected continuous exact candidate' if decision['selected_candidate']
                                                         else 'first runtime assessed continuous exact candidate'),
                          anchor_semantics=decision['anchor_semantics'],
                          confirmed_counts=[d['confirmed_candidate_count'] for d in decisions],
                          task_uncertainty_mass=[d['task_uncertainty_mass'] for d in decisions],
                          worker_stop_reason=decision['stop_reason'],
                          runtime_stop_reason=runtime_decision['stop_reason'],
                          runtime_decision_source=f'{events_path.relative_to(results)}:{runtime_line}',
                          runtime_decision_ros_time=runtime_decision['ros_time'],
                          runtime_selected_event=selected_event,
                          runtime_selected_source=(f'{events_path.relative_to(results)}:{selected_line}'
                                                   if selected_event else None),
                          frame_id='map', extent=extent,
                          native_grid=ranking['a3_summary']['grid'],
                          footprint=ranking['a3_summary']['footprint'],
                          free_observations=ranking['a2_config']['free_observations'],
                          operational_semantics=summary['operational_semantics'],
                          windows=windows, poses=poses,
                          csv_poses=json.loads(row['observation_locations'])['poses_map_xyz_yaw'],
                          sources=list(sources.values()), _operational=operational, _fields=fields))
    return cases


def generate(results, outdir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap, BoundaryNorm
    from matplotlib.lines import Line2D
    from matplotlib.patches import Polygon, Rectangle, Patch

    results, outdir = Path(results).resolve(), Path(outdir).resolve()
    if outdir == results or results in outdir.parents:
        raise ValueError('output directory must be outside immutable results')
    cases = load_examples(results)
    outdir.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9, 'axes.titlesize': 10,
                         'pdf.fonttype': 42, 'ps.fonttype': 42, 'svg.hashsalt': 'paper1-figure5-v1'})
    fig, axes = plt.subplots(2, 3, figsize=(12, 8.2), gridspec_kw={'width_ratios': [1.35, 1, 1]})
    fig.subplots_adjust(left=.06, right=.96, top=.84, bottom=.2, hspace=.43, wspace=.25)
    colors = ['#e9e9e9', '#b8d7d0', '#519e93', '#17685e']
    support_cmap = ListedColormap(colors)
    support_norm = BoundaryNorm(np.arange(-.5, 4.5), 4)
    bounds = (min(c['extent'][0] for c in cases), max(c['extent'][1] for c in cases),
              min(c['extent'][2] for c in cases), max(c['extent'][3] for c in cases))
    all_poses = np.asarray([p for c in cases for p in c['poses']])
    pose_bounds = (min(bounds[0], all_poses[:, 0].min())-.45, bounds[1]+.1,
                   min(bounds[2], all_poses[:, 1].min())-.45, bounds[3]+.1)
    for row, case in enumerate(cases):
        op, fields = case['_operational'], case['_fields']
        method_color = {'generic': '#df7b22', 'ours': '#1868ac'}[case['method']]
        pose_ax, support_ax, deficit_ax = axes[row]
        extent = case['extent']
        anchor = case['anchor']
        half = case['footprint']
        corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * [half['half_length_m'], half['half_width_m']]
        yaw = anchor['yaw']
        rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        corners = corners @ rotation.T + [anchor['x'], anchor['y']]
        support_image = support_ax.imshow(op['ground_presence_votes'], origin='lower', extent=extent,
                                         interpolation='nearest', cmap=support_cmap, norm=support_norm)
        deficit_image = deficit_ax.imshow(fields['a3_task_relevant_uncertainty'], origin='lower',
                                         extent=extent, interpolation='nearest', cmap='magma', vmin=0, vmax=1)
        pose_ax.imshow(op['ground_presence_votes'], origin='lower', extent=extent,
                       interpolation='nearest', cmap=support_cmap, norm=support_norm, alpha=.7)
        for ax in (pose_ax, support_ax):
            # An occupied vote remains independent of ground-presence support.
            occupied = np.ma.masked_where(op['environment_occupied_votes'] == 0,
                                         np.ones_like(op['environment_occupied_votes']))
            ax.imshow(occupied, origin='lower', extent=extent, interpolation='nearest',
                      cmap=ListedColormap(['#70385b']), vmin=0, vmax=1)
        for ax in axes[row]:
            ax.add_patch(Polygon(op['target_vertices_xy'], facecolor='#f2cc59', edgecolor='black', linewidth=.7))
            ax.add_patch(Polygon(corners, fill=False, edgecolor='#20c6df', linewidth=1.5))
            ax.plot(anchor['x'], anchor['y'], '+', color='#20c6df', markersize=6)
            ax.set_aspect('equal')
            ax.set_xlabel('map x (m)')
            ax.set_ylabel('map y (m)')
            ax.tick_params(labelsize=8)
        for ax in (support_ax, deficit_ax):
            ax.set_xlim(bounds[:2])
            ax.set_ylim(bounds[2:])
        pose_ax.add_patch(Rectangle((extent[0], extent[2]), extent[1]-extent[0], extent[3]-extent[2],
                                   fill=False, edgecolor='#777777', linewidth=.6))
        poses = np.asarray(case['poses'])
        pose_ax.plot(poses[:, 0], poses[:, 1], ':', color=method_color, linewidth=1)
        for j, (x, y, z, yaw) in enumerate(poses):
            pose_ax.plot(x, y, 'o', color=method_color, markersize=4)
            pose_ax.arrow(x, y, .42*np.cos(yaw), .42*np.sin(yaw), color=method_color,
                          width=.015, head_width=.12, length_includes_head=True)
            dx = 8 if x < pose_bounds[0]+.8 else -18
            offset = (dx, 9 if j != 1 else 21)
            pose_ax.annotate(str(j+1), (x, y), xytext=offset, textcoords='offset points',
                             color=method_color, fontsize=9)
        pose_ax.set_xlim(pose_bounds[:2])
        pose_ax.set_ylim(pose_bounds[2:])
        status = 'failure' if row == 0 else 'retrieval success'
        pose_ax.set_title(f'({chr(97+3*row)}) {case["method"].title()} | {status}', loc='left', fontweight='bold')
        last_window = case['windows'][-1]['round']
        support_ax.set_title(f'({chr(98+3*row)}) Ground presence | window {last_window}', loc='left')
        deficit_ax.set_title(f'({chr(99+3*row)}) Task deficit | window {last_window}', loc='left')
        missing = anchor['operational']['ground_missing_cells']
        total = len(anchor['operational']['covered_cells'])
        support_ax.text(.01, -.25, f'Exact anchor: {missing}/{total} cells lack ≥2 votes',
                        transform=support_ax.transAxes, fontsize=8)
        deficit_ax.text(.01, -.25, 'Confirmed: '+ ' → '.join(map(str, case['confirmed_counts'])),
                        transform=deficit_ax.transAxes, fontsize=8)
        stamps = ' / '.join(f'{w["packet_start_stamp_s"]:.2f}' for w in case['windows'])
        pose_ax.text(0, -.25, f'Packet starts: {stamps} s (simulation clock)',
                     transform=pose_ax.transAxes, fontsize=8)
    fig.colorbar(support_image, ax=axes[:, 1], shrink=.65, pad=.025, ticks=[0, 1, 2, 3],
                 label='Ground-presence window votes')
    fig.colorbar(deficit_image, ax=axes[:, 2], shrink=.65, pad=.025,
                 label='Saved task deficit (0–1)')
    fig.suptitle('Hard015: measured observation poses and retained runtime evidence',
                 x=.06, y=.97, ha='left', fontsize=14, fontweight='bold')
    fig.text(.06, .924, 'Illustrative, post hoc outcome-selected pair · slots 001/002 · same scene seed 1395226981', fontsize=10)
    fig.text(.06, .89, 'Runtime field visualization; no camera image or reconstructed scene geometry.', fontsize=9, color='#444444')
    handles = [Patch(facecolor='#70385b', label='Environment occupied vote'),
               Patch(facecolor='#f2cc59', edgecolor='black', label='Perceived target + allowance'),
               Line2D([0], [0], color='#20c6df', label='Continuous exact base footprint'),
               Line2D([0], [0], color='#555555', marker='o', linestyle=':', label='UAV packet poses (order only)')]
    fig.legend(handles=handles, loc='lower center', bbox_to_anchor=(.51, .08), ncol=2,
               frameon=False, fontsize=8)
    fig.text(.06, .06, 'Generic: view budget reached, no confirmed exact candidate. Ours: 3 confirmed; retrieval checks pass.', fontsize=9)
    fig.text(.06, .033, 'Ground presence is not collision freedom. White deficit cells: no validated support. Native grids and continuous anchors preserved.', fontsize=8)
    for extension in ('pdf', 'svg', 'png'):
        metadata = {'CreationDate': None, 'ModDate': None} if extension == 'pdf' else {'Date': None} if extension == 'svg' else None
        fig.savefig(outdir/f'figure5_qualitative.{extension}', dpi=300, metadata=metadata)
    plt.close(fig)
    public_cases = [{k: v for k, v in c.items() if not k.startswith('_')} for c in cases]
    manifest = dict(selection_rule=SELECTION, results_root=str(results), cases=public_cases,
                    frame_semantics='T_map_uav = first chunk_T_map_sensor @ inverse(saved T_uav_lidar); all plots map frame',
                    spatial_semantics='native grid origins preserved, no resampling; exact base anchors remain continuous',
                    visualization_semantics='saved runtime fields only; no GT geometry, image synthesis, or algorithm rerun',
                    pose_line_semantics='observation order only, not the flown trajectory',
                    ground_support_semantics='ground_presence_votes >= saved free_observations; blocking remains independent',
                    anchor_rule='selected exact candidate if present, otherwise first runtime assessment; not a matched shared anchor')
    (outdir/'qualitative_examples.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
    caption = ('Figure 5. '+SELECTION+' The two runs share scene eval-hard-015 (seed 1395226981). '
               'Panels (a,d) show the first measured UAV packet pose of each accepted window, with heading arrows '
               'and dotted observation order (not a flight path). Poses are obtained from the saved sensor packet '
               'transform and inverse mount transform. Panels (b,e) display accumulated ground-presence window votes, '
               'with environment occupied votes overlaid; ground support does not imply collision freedom, and '
               'ambiguous endpoint collision gates are not depicted. Panels (c,f) show the saved task-relevant '
               'observation-deficit field on a shared 0–1 scale. White cells retain saved NaN values '
               '(no validated footprint support); zero deficit can also result from blocked support and '
               'does not mean known free space. Yellow polygons are the saved perceived target '
               'with its declared allowance, not simulator ground truth. Cyan outlines are continuous exact '
               'candidate base footprints, not representative grid centers: Generic uses the first runtime '
               'assessment; Ours uses its selected candidate. These are respective per-run anchors, not an '
               'identical shared anchor. Native map extents and 0.1 m cell size are preserved without alignment '
               'or interpolation; the small origin difference follows each run’s perceived target. '
               'The final exact anchor lacks ground support in 19/96 cells for Generic and 0/88 for Ours; '
               'confirmed-candidate histories are 0,0,0 and 0,0,3. Both runs accept three windows; '
               'Generic exhausts the view budget without a confirmed candidate, while Ours passes the '
               'shared execution screen after the third window (runtime SCREENED_CANDIDATE_READY), '
               'continues to retrieval and passes the '
               'recorded physical checks. This figure is a runtime-field visualization, not a retained camera '
               'frame or independent evidence of physical retrieval. Sources, accepted-event '
               'line numbers, distinct worker and runtime stop reasons, exact anchor assessments, '
               'the matching runtime A5_SELECTED event, terminal events and packet timestamps appear in '
               '`qualitative_examples.json`.\n')
    (outdir/'figure5_caption.md').write_text(caption)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results', type=Path, required=True)
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    manifest = generate(args.results, args.outdir)
    print(json.dumps({'scene': manifest['cases'][0]['scene'],
                      'slots': [c['slot'] for c in manifest['cases']], 'outdir': str(args.outdir)}))


if __name__ == '__main__':
    main()
