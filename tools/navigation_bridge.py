"""Local owned navigation session. Locked until supervised hardware test."""
import json
import shlex
import subprocess
import threading
import time
from pathlib import Path

class NavigationBridge:
    def __init__(self,root):
        self.root=Path(root);self.lock=threading.Lock();self.active=False;self.cancel_event=threading.Event();self.events=[];self.last_browser_poll=time.monotonic();self.goal=None;self.session_started_unix=None;self.transport_started=None
        self.process=None
        self.finished=threading.Event();self.finished.set()
    def operator_ping(self):
        self.last_browser_poll=time.monotonic()
    def heartbeat_message(self):
        live=time.monotonic()-self.last_browser_poll<=3
        starting=self.transport_started is not None and time.monotonic()-self.transport_started<15
        return {'heartbeat':True,'operator_live':live,'cancel':self.cancel_event.is_set() or (not live and not starting)}
    def enabled(self):
        return json.loads((self.root/'config/scope.json').read_text())['motion_enabled'] is True
    def start(self,goal,expected_start=None):
        self._start(goal,False,expected_start)
    def start_localization(self):
        self._start({'action':'localize'},True)
    def _start(self,goal,localization_only,expected_start=None):
        with self.lock:
            if not localization_only and not self.enabled():raise ValueError('实机导航测试尚未开放，请先完成现场准备')
            if self.active:raise ValueError('已有导航任务，先取消或等待完成')
            self.active=True;self.events=[];self.goal=None if localization_only else dict(goal);self.session_started_unix=time.time();self.transport_started=time.monotonic();self.cancel_event.clear();self.operator_ping()
            self.finished.clear()
        request=dict(goal)
        if expected_start is not None:
            request['expected_start']={k:expected_start[k] for k in ('x','y','yaw','frame')}
        threading.Thread(target=self.run,args=(request,),daemon=True).start()
    def cancel(self,reason='用户请求取消'):
        if not self.cancel_event.is_set():
            with self.lock:self.events.append({'event':'CANCEL_REASON','message':reason,'unix':time.time()})
        self.cancel_event.set()
    def recover(self,timeout=8):
        """Cancel first; never hide an unconfirmed, submitted robot goal."""
        self.cancel('用户请求刷新重连')
        if self.active and not self.finished.wait(timeout):
            with self.lock:
                sent=any(e.get('event')=='SEND_ONCE' for e in self.events)
                terminal=any(e.get('event') in ('RESULT','STOP_TASK_TERMINAL','STOP_VERIFIED_IDLE') for e in self.events)
                process=self.process
            if sent and not terminal:
                raise ValueError('已请求取消，但导航停车尚未确认；请现场确认停车后重试，不能直接清除任务')
            if process is not None and process.poll() is None:process.terminate()
            if not self.finished.wait(3):raise ValueError('旧会话仍在退出，请稍后再次刷新重连')
        with self.lock:
            self.events=[];self.goal=None;self.session_started_unix=None;self.transport_started=None
    def run(self,goal):
        process=None
        try:
            request=dict(goal)
            scope=json.loads((self.root/'config/scope.json').read_text())
            request['scan_topic']=scope.get('navigation_scan_topic',scope.get('scan_topic','/scan'))
            request['mapping_sensor']=scope.get('mapping_sensor','lidar')
            request['navigation_sensor']=scope.get('navigation_sensor','lidar')
            if scope.get('computer_map_name'):
                request['map_name']=scope['computer_map_name']
            if 'motion_max_goal_distance_m' in scope:
                request['max_distance_m']=scope['motion_max_goal_distance_m']
            camera=(self.root/'tools/camera_handoff.py').read_text(encoding='utf-8')
            inventory=(self.root/'tools/map_inventory.py').read_text(encoding='utf-8')
            setup=(self.root/'tools/depth_slam_setup.py').read_text(encoding='utf-8') if 'depth_camera' in (request['mapping_sensor'],request['navigation_sensor']) else ''
            heartbeat=(self.root/'tools/robot_heartbeat.py').read_text(encoding='utf-8')
            heading=(self.root/'tools/navigation_heading.py').read_text(encoding='utf-8')
            code='REQUEST='+repr(request)+'\n'+camera+'\n'+inventory+'\n'+setup+'\n'+heartbeat+'\n'+heading+'\n'+(self.root/'tools/stock_navigation_session.py').read_text(encoding='utf-8')
            raw=code.encode('utf-8')
            # Read exactly the source frame without buffering later heartbeats.
            loader="""import os
header=b''
while not header.endswith(b'\\n'):
    part=os.read(0,1)
    if not part:raise RuntimeError('Source header disconnected')
    header+=part
size=int(header)
code=b''
while len(code)<size:
    part=os.read(0,size-len(code))
    if not part:raise RuntimeError('Source body disconnected')
    code+=part
exec(code)
"""
            command=['ssh','-F',str(self.root/'config/readonly_ssh_config'),'cyberdog-current-nx',
                'source /etc/mi/ros2_env.conf; python3 -u -c '+shlex.quote(loader)]
            with (self.root/'evidence/navigation_session_stderr.txt').open('w') as err:
                process=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=err)
                with self.lock:self.process=process
                process.stdin.write(str(len(raw)).encode()+b'\n'+raw);process.stdin.flush()
                def heartbeat():
                    while process.poll() is None:
                        try:
                            process.stdin.write(json.dumps(self.heartbeat_message()).encode()+b'\n');process.stdin.flush()
                        except (OSError,ValueError):break
                        time.sleep(0.5)
                threading.Thread(target=heartbeat,daemon=True).start()
                for line in process.stdout:
                    try:row=json.loads(line)
                    except ValueError:continue
                    row['local_received_unix']=time.time()
                    with self.lock:self.events.append(row);self.events=self.events[-100:]
                    with (self.root/'evidence/navigation_events.jsonl').open('a',encoding='utf-8') as log:log.write(json.dumps(row)+'\n')
                process.wait()
        except Exception as exc:
            with self.lock:self.events.append({'event':'LOCAL_ERROR','message':str(exc)})
        finally:
            try:
                if process is not None and process.stdin:process.stdin.close()
            except (OSError,ValueError):pass
            finally:
                with self.lock:self.active=False;self.process=None
                self.finished.set()
    def report(self):
        scope=json.loads((self.root/'config/scope.json').read_text())
        with self.lock:return {'enabled':scope['motion_enabled'] is True,'active':self.active,'events':list(self.events),'goal':self.goal,'session_started_unix':self.session_started_unix,'max_goal_distance_m':scope.get('motion_max_goal_distance_m'),'preparing_localization':self.active and self.goal is None,'preparing_navigation':self.active and self.goal is not None and not any(e.get('event')=='SEND_ONCE' for e in self.events)}
