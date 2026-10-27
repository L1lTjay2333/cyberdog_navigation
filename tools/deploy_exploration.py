"""Install isolated, versioned robot scripts; compilation only, no motion."""
import json,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
FILES=('manual_policy.py','cloud_geometry.py','depth_geometry.py','front_cloud.py',
       'unknown_goal.py','route_preview.py','pose_history.py','depth_corridor.py','exploration_policy.py','exploration_robot_session.py')

def deploy(build_only=False):
    stamp=time.strftime('%Y%m%d_%H%M%S')
    package=ROOT/'evidence'/('exploration_package_'+stamp);package.mkdir()
    # This is a build artifact: only reviewed sensor/publisher definitions, not
    # the original manual command loop or reader thread, are included.
    source=(ROOT/'tools/manual_robot_session.py').read_text(encoding='utf-8')
    marker='\nensure_idle_camera()\n'
    if source.count(marker)!=1:raise RuntimeError('Manual bootstrap boundary changed; review before deployment')
    bootstrap=source.split(marker)[0].replace("'handoff_manual_deadman'","'handoff_exploration_executor'")
    # Exploration-only observed-ray check; leave the manual controller unchanged.
    original="""        if len(xs)<10:
            health['depth_invalid_reason']='前方检查区域有效点不足';return
"""
    replacement="""        if len(xs)<10:
            observed=observed_corridor(points,[t.translation.x,t.translation.y,t.translation.z])
            if observed is None:
                health['depth_invalid_reason']='前方测量射线覆盖不足';return
            xs.append(observed)
"""
    if bootstrap.count(original)==1:bootstrap=bootstrap.replace(original,replacement)
    elif bootstrap.count(replacement)!=1:raise RuntimeError('Depth ROI boundary changed; review build')
    bootstrap=bootstrap.replace('depth_xyz(m,depth_info,stride=8)','depth_xyz(m,depth_info,stride=16)')
    # Bind points and stamp into one immutable snapshot for the executor.
    bootstrap=bootstrap.replace("        health['depth_stamp_ns']=stamp.nanoseconds", "        health['depth_stamp_ns']=stamp.nanoseconds\n        health['exploration_depth']={'stamp':stamp.nanoseconds,'points':health['depth_points_base']}")
    (package/'exploration_bootstrap.py').write_text(bootstrap,encoding='utf-8')
    for filename in FILES:(package/filename).write_bytes((ROOT/'tools'/filename).read_bytes())
    if build_only:
        print(json.dumps({'local_package':str(package),'remote_changes':False,'motion_started':False},ensure_ascii=False))
        return
    remote='/home/mi/cyberdog_exploration_'+stamp
    ssh=['ssh','-F',str(ROOT/'config/readonly_ssh_config'),'-o','BatchMode=yes','-o','ConnectTimeout=5','cyberdog-current-nx']
    def run(args):
        p=subprocess.run(args,capture_output=True,text=True,encoding='utf-8',timeout=25)
        if p.returncode:raise RuntimeError('Deployment failed: '+(p.stderr or p.stdout)[-1200:])
        return p.stdout.strip()
    run(ssh+['mkdir -m 700 '+remote])
    run(['scp','-F',str(ROOT/'config/readonly_ssh_config'),'-o','BatchMode=yes',*[str(f) for f in package.glob('*.py')], 'cyberdog-current-nx:'+remote+'/'])
    run(ssh+['cd '+remote+' && python3 -m py_compile '+ ' '.join(FILES)+' exploration_bootstrap.py'])
    record={'remote_dir':remote,'local_package':str(package),'installed_unix':time.time(),'robot_python_compile':True,'motion_started':False}
    (ROOT/'config/exploration_deployment.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(record,ensure_ascii=False))
if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();mode=parser.add_mutually_exclusive_group(required=True);mode.add_argument('--deploy',action='store_true');mode.add_argument('--build-only',action='store_true');args=parser.parse_args()
    deploy(args.build_only)
