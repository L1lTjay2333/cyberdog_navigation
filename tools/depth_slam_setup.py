"""Reversible stock SLAM process configuration, only while algorithm manager idle."""
import json,os,signal,subprocess,time
from pathlib import Path
import yaml
from protocol.srv import AlgoTaskStatus
from lifecycle_msgs.srv import GetState

PACKAGE=Path('/home/mi/web_depth_mapping_20261011')
NS='/mi_desktop_48_b0_2d_7b_0b_4a'

def ensure_depth_slam(node,call,emit,allow_mapping=False,navigation_only=False):
    def idle(allow_failed=False):
        s=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
        allowed=(101,5) if allow_mapping else (101,)
        if allow_failed:allowed=allowed+(6,)
        if s.code!=0 or s.status not in allowed:
            raise RuntimeError('Depth input switch requires idle task101')
        return s.status
    # State 6 accepts a new localization but does not permit replacing stock
    # processes. The replacement loop below still requires idle state 101.
    idle(allow_failed=True)
    depth_running=False
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        try:args=entry.joinpath('cmdline').read_bytes().decode().strip('\0').split('\0')
        except (OSError,UnicodeError):continue
        if 'depth_scan_node.py' in args or str(PACKAGE/'depth_scan_node.py') in args:depth_running=True
    if not depth_running:
        with (PACKAGE/'depth_scan.log').open('ab') as log:
            subprocess.Popen(['python3','-u',str(PACKAGE/'depth_scan_node.py')],cwd=str(PACKAGE),
                             stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
    binaries={
        '/opt/ros2/cyberdog/lib/laser_slam/mapping':'mapping',
        '/opt/ros2/cyberdog/lib/laser_slam/localization':'localization',
        '/opt/ros2/cyberdog/lib/nav2_controller/controller_server':'controller',
        '/opt/ros2/cyberdog/lib/nav2_planner/planner_server':'planner'}
    processes={}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        try:args=entry.joinpath('cmdline').read_bytes().decode().strip('\0').split('\0')
        except (OSError,UnicodeError):continue
        if args and args[0] in binaries:
            kind=binaries[args[0]]
            if kind in processes:raise RuntimeError('Duplicate stock SLAM process; inspect before replacement')
            if '--params-file' not in args or '__ns:='+NS not in args:raise RuntimeError('Unexpected stock SLAM command')
            processes[kind]=(int(entry.name),args)
    if set(processes)!=set(binaries.values()):raise RuntimeError('Stock SLAM/navigation processes missing')
    plans=[]
    for kind,(pid,args) in processes.items():
        if navigation_only and kind in ('mapping','localization'):continue
        index=args.index('--params-file')+1;old=Path(args[index])
        document=yaml.safe_load(old.read_text())
        group=document[NS.lstrip('/')]
        if kind in ('mapping','localization'):
            name='map_builder' if kind=='mapping' else 'localization_node'
            params=group[name]['ros__parameters']
            if params.get('laser_scan_topic')=='/web_depth_scan':continue
            params['laser_scan_topic']='/web_depth_scan'
            # Camera FOV yields fewer bins than the native half-circle lidar.
            params['min_num_points']=60.0
            params['max_range']=5.5
        else:
            costmap='local_costmap' if kind=='controller' else 'global_costmap'
            params=group[costmap][costmap]['ros__parameters']['obstacle_layer']['scan']
            if params.get('topic')==NS+'/web_depth_scan':continue
            params['topic']=NS+'/web_depth_scan'
            params['obstacle_max_range']=5.5
            params['raytrace_max_range']=5.5
        backup=PACKAGE/(kind+'_original.yaml')
        if not backup.exists():backup.write_bytes(old.read_bytes())
        replacement=PACKAGE/(kind+'_depth.yaml');replacement.write_text(yaml.safe_dump(document))
        changed=list(args);changed[index]=str(replacement)
        plans.append((kind,pid,args,changed))
    for kind,pid,args,changed in plans:
        status=idle()
        if status==5:
            # Never interrupt a running map. Only unused, unconfigured Nav2
            # servers may be replaced while camera mapping continues.
            if kind not in ('controller','planner'):raise RuntimeError('Cannot replace active SLAM during mapping')
            name='controller_server_ab' if kind=='controller' else 'planner_server_ab'
            if call(name+'/get_state',GetState,GetState.Request()).current_state.id!=1:
                raise RuntimeError('Nav2 server is not unconfigured during mapping')
        os.kill(pid,signal.SIGTERM)
        until=time.monotonic()+8
        while Path('/proc/'+str(pid)).exists() and time.monotonic()<until:time.sleep(.1)
        if Path('/proc/'+str(pid)).exists():raise RuntimeError('Stock SLAM process did not stop cleanly')
        with (PACKAGE/(kind+'_depth.log')).open('ab') as log:
            replacement=subprocess.Popen(changed,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
        time.sleep(.8)
        if replacement.poll() is not None:
            with (PACKAGE/(kind+'_rollback.log')).open('ab') as log:
                subprocess.Popen(args,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
            raise RuntimeError('Depth SLAM process failed; original process restarted')
        emit(event='DEPTH_SLAM_INPUT',node=kind,pid=replacement.pid,topic=NS+'/web_depth_scan',backup=str(PACKAGE/(kind+'_original.yaml')))
    emit(event='DEPTH_SLAM_READY',topic=NS+'/web_depth_scan')
