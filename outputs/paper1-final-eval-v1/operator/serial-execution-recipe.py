import datetime,json,os,signal,subprocess,time
from pathlib import Path
import shutil
from paper1_eval_design import slot_command
root=Path.cwd(); results=root/'outputs/paper1-final-eval-v1'
config=json.loads((root/'configs/paper1_eval.json').read_text())
baseline=json.loads((results/'resource-baseline.json').read_text())
previous_entries=[json.loads(p.read_text()) for p in results.glob('slot-*/entry.json')]
first=min(e['start_wall'] for e in previous_entries); deadline=first+432000
operators=results/'operator'; operators.mkdir(exist_ok=True)
environment=dict(os.environ,P450_GAZEBO_DISPLAY=':0',P450_GAZEBO_XAUTHORITY='/run/user/1000/gdm/Xauthority')
post="import json,subprocess\nfrom pathlib import Path\nimport cv2,numpy as np,rosbag,rospy\nfrom cv_bridge import CvBridge\nroot=Path('outputs/paper1-final-eval-v1')\nassert subprocess.run(['pgrep','-x','gzserver'],stdout=subprocess.DEVNULL).returncode != 0\nreport=json.loads((root/'analysis-latest.json').read_text())\nrows={r['slot']:r for r in report['slots']}\nbridge=CvBridge()\nfor directory in sorted(root.glob('slot-*/attempt')):\n    if (directory/'retention.json').exists(): continue\n    raw=json.loads((directory/'attempt.json').read_text())\n    if 'finish_wall' not in raw: continue\n    csv=directory/'ground-dynamics.csv'\n    if csv.exists(): subprocess.run(['gzip','-1','--',str(csv)],check=True)\n    row=rows[raw['slot']]\n    selected=row['selected_attempt']==str(directory.relative_to(root))\n    bag_path=directory/'diagnostics-images.bag'\n    note=dict(slot=raw['slot'],method=raw['method'],status=raw['status'],\n              retrieval_success=raw.get('retrieval_success'),\n              native_csv_storage='ground-dynamics.csv.gz (lossless gzip-1)',\n              images_original_bytes=bag_path.stat().st_size if bag_path.exists() else None,\n              images_removed=False,frames=[])\n    eligible=(selected and raw['status']=='VALID_TRIAL' and raw.get('retrieval_success') is True\n              and not raw['scene_spec']['retain_success_rgbd'])\n    if eligible and bag_path.exists():\n        physical=json.loads((directory/'physical_summary.json').read_text())\n        required={'GROUND_STOPPED','GROUND_OBSERVE','GROUND_REFINED','PREGRASP','GRASP','LIFTING','LIFT'}\n        reviewed=(physical['status']=='CHECKS_PASS' and row['D_exec'] and not row['analysis_missing']\n                  and required.issubset(physical['sequence']['states']))\n        note['physical_stage_missingness_review_passed']=bool(reviewed)\n        if reviewed:\n            events=[json.loads(line) for line in (directory/'data/events.jsonl').read_text().splitlines()]\n            targets=[('observation',next(e['ros_time'] for e in events if e['state']=='GROUND_OBSERVE')),\n                     ('accepted_refine',physical['observations']['ground']['stamp'])]\n            frame_dir=directory/'review-frames'; frame_dir.mkdir(exist_ok=True)\n            with rosbag.Bag(str(bag_path),'r') as bag:\n                for label,stamp in targets:\n                    for topic in ('/ground/d435/color/image_raw','/ground/d435/depth/image_raw'):\n                        best=None\n                        for _,message,t in bag.read_messages(topics=[topic],start_time=rospy.Time.from_sec(max(0,stamp-.5)),end_time=rospy.Time.from_sec(stamp+.5)):\n                            delta=abs(message.header.stamp.to_sec()-stamp)\n                            if best is None or delta<best[0]: best=(delta,message)\n                        assert best is not None,'No real frame near '+label\n                        delta,message=best; actual=message.header.stamp.to_sec()\n                        if '/color/' in topic:\n                            filename=label+'-color.png'; array=bridge.imgmsg_to_cv2(message,'bgr8')\n                            assert cv2.imwrite(str(frame_dir/filename),array)\n                        else:\n                            filename=label+'-depth.npz'; array=bridge.imgmsg_to_cv2(message,'passthrough')\n                            np.savez_compressed(str(frame_dir/filename),depth=array,stamp=actual,encoding=message.encoding)\n                        note['frames'].append(dict(label=label,topic=topic,requested_stamp=stamp,actual_stamp=actual,delta_s=delta,frame_id=message.header.frame_id,shape=list(array.shape),file='review-frames/'+filename))\n            note['physical_review']=dict(status=physical['status'],target_lift_m=physical['physical_target']['target_lift_m'],ground_frame=physical['observations']['ground']['frame'])\n            note['removal_reason']='Predeclared non-retention successful scene; physical/stage/missingness review and four real frame exports completed. Full RGB-D bag is not retained.'\n            bag_path.unlink()\n            note['images_removed']=True\n    if not note['images_removed']:\n        note['policy']='Full available image records retained: predefined retention scene, failure/invalid/unresolved, unavailable bag or incomplete export review.'\n    (directory/'retention.json').write_text(json.dumps(note,indent=2)+'\\n')\n    print(json.dumps(dict(slot=note['slot'],csv='lossless gzip',images_removed=note['images_removed'],images_original_bytes=note['images_original_bytes'],frames=len(note['frames']))),flush=True)\n"
def emit(kind,**fields):
    record=dict(at_wall=time.time(),kind=kind,**fields)
    print(json.dumps(record),flush=True)
    with (operators/'progress.jsonl').open('a') as stream: stream.write(json.dumps(record)+'\n')
def usage():
    used=lambda p:int(subprocess.check_output(['du','-sB1',str(p)],text=True).split()[0])
    external=sum(max(0,used(p)-n) for p,n in baseline['external_usage_bytes'].items() if Path(p).exists())
    return dict(new_gib=(used(results)+external)/2**30,data_free_gib=shutil.disk_usage(results).free/2**30,
                root_free_gib=shutil.disk_usage('/').free/2**30,remaining_h=(deadline-time.time())/3600)
def last_state(directory):
    p=directory/'attempt/data/events.jsonl'
    if not p.exists(): return None
    try:
        with p.open('rb') as stream:
            stream.seek(max(0,p.stat().st_size-32768))
            lines=stream.read().decode().splitlines()
        for line in reversed(lines):
            try:
                event=json.loads(line)
                return dict(state=event.get('state'),stage=event.get('stage'),ros_time=event.get('ros_time'))
            except ValueError: continue
    except OSError: return None
for spec in config['slots'][4-1:]:
    slot=spec['slot']
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==baseline['agent_commit']
    assert not subprocess.check_output(['git','diff','3d13a95','--','src','scripts','configs','assets'],text=True)
    entries=[json.loads(p.read_text()) for p in results.glob('slot-*/entry.json')]
    resource=usage()
    if len(entries)>=224 or resource['remaining_h']*3600<1800 or resource['new_gib']+32>=500 or resource['data_free_gib']<132:
        emit('RESOURCE_STOP_BEFORE_TASK',slot=slot,starts=len(entries),**resource)
        raise SystemExit(3)
    command=slot_command(config,root/'configs/paper1_eval.json',slot,results)
    directory=Path(command[command.index('--output-dir')+1])
    assert not directory.exists(),'Refuse to replace an existing attempt'
    assert subprocess.run(['pgrep','-x','gzserver'],stdout=subprocess.DEVNULL).returncode!=0,'Previous Gazebo still running'
    started=time.time()
    emit('START',slot=slot,scene=spec['scene'],method=spec['method'],starts_before=len(entries),**resource)
    with (operators/('slot-%03d-console.log'%slot)).open('x') as log:
        process=subprocess.Popen(command,env=environment,stdout=log,stderr=subprocess.STDOUT)
        try:
            while True:
                try:
                    code=process.wait(timeout=40)
                    break
                except subprocess.TimeoutExpired:
                    resource=usage()
                    emit('RUNNING',slot=slot,scene=spec['scene'],method=spec['method'],last=last_state(directory),**resource)
                    if resource['remaining_h']*3600<300 or resource['new_gib']>=496 or resource['data_free_gib']<=104:
                        emit('RESOURCE_INTERRUPTION_REQUIRES_REVIEW',slot=slot,**resource)
                        process.send_signal(signal.SIGINT)
                        process.wait(timeout=300)
                        raise SystemExit(3)
        except KeyboardInterrupt:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=300)
            raise
    entry=directory/'entry.json'; attempt=directory/'attempt/attempt.json'
    if not entry.exists() or not attempt.exists():
        emit('UNRESOLVED_LAUNCH_REQUIRES_REVIEW',slot=slot,started_wall=started,exit_code=code)
        raise SystemExit(2)
    raw=json.loads(attempt.read_text())
    emit('FINISHED',slot=slot,scene=spec['scene'],method=spec['method'],exit_code=code,
         status=raw.get('status'),retrieval_success=raw.get('retrieval_success'),
         reason=raw.get('classification_reason',raw.get('reason')),wall_s=time.time()-started)
    if raw.get('status')!='VALID_TRIAL' or type(raw.get('retrieval_success')) is not bool or 'finish_wall' not in raw:
        emit('CLASSIFICATION_REVIEW_REQUIRED_NO_AUTO_RETRY',slot=slot)
        raise SystemExit(2)
    expected=dict(agent=baseline['agent_commit'],sim=config['evaluation_version']['sim_commit'],rm4d=config['evaluation_version']['rm4d_baseline_commit'])
    if raw.get('runtime_versions')!=expected:
        emit('RUNTIME_VERSION_MISMATCH',slot=slot)
        raise SystemExit(2)
    primary_in_block=[s for s in config['slots'][:slot] if s['scene']==spec['scene'] and s['method'] in ('ours','generic')]
    if len(primary_in_block)==2:
        analysis=results/'analysis-latest.json'
        subprocess.run(['/media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python','scripts/analyze_paper1_eval.py','--config','configs/paper1_eval.json','--results-dir',str(results),'--output',str(analysis)],check=True,env=dict(os.environ,OPENBLAS_NUM_THREADS='1',PYTHONPATH='scripts'))
        report=json.loads(analysis.read_text())
        if report['protocol_issues']:
            emit('OFFLINE_RECORD_REVIEW_REQUIRED',slot=slot,issues=report['protocol_issues'])
            raise SystemExit(2)
        subprocess.run(['/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation/scripts/with_noetic_env.bash','/usr/bin/python3','-c',post],check=True)
        emit('POST_PAIR_COMPLETE',slot=slot,completed=report['counts']['valid_completed_slots'],**usage())
emit('ALL_PLANNED_TASKS_FINISHED',**usage())

