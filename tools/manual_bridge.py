"""SSH manual session bridge; independent from map/navigation, fail-closed."""
import json
import shlex
import subprocess
import threading
import time
from pathlib import Path
from manual_policy import DIRECTIONS,command_velocity

class ManualBridge:
    def __init__(self, root):
        self.root = Path(root); self.lock = threading.Lock()
        self.process = None; self.started = False
        self.command = {'direction': 'stop', 'armed': False}; self.last_input = 0.
        self.state = {'connected': False, 'active': False, 'enabled': False}
        self.last_state = 0.
        self.owner = None; self.sequence = -1
        self.owner_seen = 0.
        self.auxiliary_task = None
    def enabled(self):
        return json.loads((self.root/'config/scope.json').read_text()).get('manual_motion_enabled') is True
    def connect(self, owner,auxiliary_task=None):
        if not isinstance(owner,str) or not 10<=len(owner)<=80:raise ValueError('网页会话标识无效')
        with self.lock:
            if self.started:
                if self.owner!=owner:raise ValueError('已有另一个网页持有遥控会话')
                return
            self.state={'connected':False,'active':False,'enabled':False}
            self.last_state=0.
            self.owner=owner;self.sequence=-1;self.owner_seen=time.monotonic()
            self.auxiliary_task=auxiliary_task
            self.started = True
        threading.Thread(target=self.run, daemon=True).start()
    def control(self, item):
        direction = item.get('direction')
        if direction not in DIRECTIONS: raise ValueError('未知遥控方向')
        axes=item.get('axes');command_velocity(direction,axes)
        with self.lock:
            if item.get('owner')!=self.owner or self.owner is None:raise ValueError('请先在此网页连接遥控')
            seq=item.get('sequence')
            if type(seq) is not int or seq<0:raise ValueError('指令序号无效')
            if seq<=self.sequence:return
            # One stop request always disarms. No command resumes automatically on reconnect.
            armed = item.get('armed') is True and direction != 'stop'
            if armed and not self.enabled(): raise ValueError('运动锁尚未开放，当前只能联调')
            if armed and (not self.state.get('connected') or time.monotonic()-self.last_state > 1.):
                raise ValueError('遥控连接尚未就绪或已过期')
            self.command = {'direction': direction, 'armed': armed}
            if axes is not None:self.command['axes']=axes
            self.sequence=seq
            self.last_input = self.owner_seen = time.monotonic()
    def sit(self,item,action='sit'):
        with self.lock:
            if item.get('owner')!=self.owner or self.owner is None:raise ValueError('请先连接本页遥控')
            if not self.enabled() or not self.state.get('connected') or time.monotonic()-self.last_state>1:raise ValueError('遥控未就绪')
            seq=item.get('sequence')
            if type(seq) is not int or seq<=self.sequence:return
            self.command={'direction':'stop','armed':False,'action':action,'nonce':seq}
            self.sequence=seq;self.last_input=self.owner_seen=time.monotonic()
    def posture(self,item):
        self.sit(item,action='posture')
    def stop(self):
        with self.lock:
            self.command={'direction':'stop','armed':False}
            # Administrative stop revokes the session, preventing delayed re-arming.
            self.owner=None
    def report(self,owner=None):
        with self.lock:
            if owner is not None and owner==self.owner:
                self.owner_seen=time.monotonic()
            result = dict(self.state)
            result['connected'] = bool(result.get('connected')) and time.monotonic()-self.last_state < 1.
            if not result['connected']:
                result.update(depth_valid=False,depth_min=None,front_clearance_m=None,
                    battery=None,motion_status=None,reason='遥控连接未就绪，请连接本页遥控',
                    forward_block_reason='遥控连接未就绪',
                    direction_blocks={d:'遥控连接未就绪' for d in result.get('direction_blocks',{})})
            result['enabled'] = result['connected'] and self.enabled() and result.get('enabled') is True
            result['connecting'] = self.started and not result['connected']
            result['error'] = self.state.get('error','')
            result['requested_direction'] = self.command['direction']
            if owner is not None:result['owns_session']=self.owner==owner and self.owner is not None
            return result
    def expire_owner(self,now=None):
        now=time.monotonic() if now is None else now
        with self.lock:
            expired=self.owner is not None and now-self.owner_seen>3
        if expired:self.disconnect()
        return expired
    def disconnect(self):
        self.stop()
        process=self.process
        if process is not None and process.stdin:
            try:process.stdin.close()
            except (OSError,ValueError):pass
    def run(self):
        process = None
        try:
            source = 'MOTION_ENABLED='+repr(self.enabled())+'\n'
            scope=json.loads((self.root/'config/scope.json').read_text())
            source+='SENSOR_MODE='+repr(scope.get('manual_sensor_mode','lidar'))+'\n'
            source+='AUXILIARY_TASK='+repr(self.auxiliary_task)+'\n'
            source+='MANUAL_DISTANCE_OVERRIDE='+repr(scope.get('manual_distance_override') is True)+'\n'
            source+=(self.root/'tools/camera_handoff.py').read_text(encoding='utf-8')+'\n'
            source+='import sys,types\n'
            for name in ('cloud_geometry','depth_geometry','depth_corridor','front_cloud'):
                code=(self.root/('tools/'+name+'.py')).read_text(encoding='utf-8')
                source+='m=types.ModuleType('+repr(name)+');sys.modules['+repr(name)+']=m\nexec('+repr(code)+',m.__dict__)\n'
            source += (self.root/'tools/manual_policy.py').read_text(encoding='utf-8')+'\n'
            source += (self.root/'tools/manual_robot_session.py').read_text(encoding='utf-8')
            raw = source.encode('utf-8')
            loader = "import os\nh=b''\nwhile not h.endswith(b'\\n'): h+=os.read(0,1)\nn=int(h);s=b''\nwhile len(s)<n: s+=os.read(0,n-len(s))\nexec(s)"
            cmd = ['ssh','-F',str(self.root/'config/readonly_ssh_config'),'cyberdog-current-nx',
                   'source /etc/mi/ros2_env.conf; python3 -u -c '+shlex.quote(loader)]
            with (self.root/'evidence/manual_stderr.txt').open('w',encoding='utf-8') as err:
                process = subprocess.Popen(cmd,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=err)
                self.process = process
                process.stdin.write(str(len(raw)).encode()+b'\n'+raw); process.stdin.flush()
                def pump():
                    while process.poll() is None:
                        if self.expire_owner():break
                        with self.lock:
                            item = dict(self.command)
                            if time.monotonic()-self.last_input > .3 or not self.enabled(): item['armed'] = False
                        try:
                            process.stdin.write(json.dumps(item).encode()+b'\n'); process.stdin.flush()
                        except (OSError, ValueError): break
                        time.sleep(.1)
                threading.Thread(target=pump,daemon=True).start()
                for line in process.stdout:
                    try: item = json.loads(line)
                    except ValueError: continue
                    with self.lock: self.state = item; self.last_state = time.monotonic()
                process.wait()
        except Exception as exc:
            with self.lock: self.state = {'connected':False,'active':False,'error':str(exc)}
        finally:
            if process and process.stdin: process.stdin.close()
            with self.lock:
                self.command = {'direction':'stop','armed':False}
                self.owner=None
                self.state['connected'] = False; self.started = False
