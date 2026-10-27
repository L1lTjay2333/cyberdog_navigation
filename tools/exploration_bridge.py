"""Owned standalone exploration process, with browser and robot deadman timers."""
import json,subprocess,threading,time,re

class ExplorationBridge:
    def __init__(self,root):
        self.root=root;self.lock=threading.Lock();self.active=False;self.events=[]
        self.owner=None;self.last_input=0.;self.cancel_event=threading.Event();self.process=None
        self.executing=False
    def deployment(self):
        path=self.root/'config/exploration_deployment.json'
        if not path.exists():return None
        record=json.loads(path.read_text(encoding='utf-8'))
        if not re.fullmatch(r'/home/mi/cyberdog_exploration_\d{8}_\d{6}',record.get('remote_dir','')):
            raise ValueError('探索模块部署路径无效')
        return record
    def start(self,request,owner,execute=False):
        if not isinstance(owner,str) or not 10<=len(owner)<=80:raise ValueError('探索会话标识无效')
        record=self.deployment()
        if not record or not record.get('robot_python_compile'):raise ValueError('探索模块未部署')
        with self.lock:
            if self.active:raise ValueError('已有独立探索会话')
            self.owner=owner;self.last_input=time.monotonic();self.cancel_event.clear()
            self.active=True;self.events=[];self.executing=execute
        threading.Thread(target=self.run,args=({**request,'execute':execute},record['remote_dir']),daemon=True).start()
    def heartbeat(self,owner):
        with self.lock:
            if not self.active or self.owner!=owner:raise ValueError('本页未持有探索会话')
            self.last_input=time.monotonic()
    def stop(self):
        self.cancel_event.set()
    def report(self):
        with self.lock:
            return {'active':self.active,'executing':self.executing,'events':list(self.events),
                    'deployed':bool(self.deployment()),'owner':self.owner if self.active else None,
                    'motion_verified':False}
    def run(self,request,directory):
        process=None
        try:
            cmd=['ssh','-F',str(self.root/'config/readonly_ssh_config'),'-o','BatchMode=yes','-o','ConnectTimeout=5','cyberdog-current-nx',
                 'cd '+directory+' && source /etc/mi/ros2_env.conf && python3 -u exploration_robot_session.py']
            with (self.root/'evidence/exploration_stderr.txt').open('w',encoding='utf-8') as err:
                process=subprocess.Popen(cmd,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=err,text=True,encoding='utf-8')
                self.process=process
                process.stdin.write(json.dumps(request)+'\n');process.stdin.flush()
                def pump():
                    while process.poll() is None:
                        with self.lock:live=time.monotonic()-self.last_input<2
                        cancel=self.cancel_event.is_set() or not live
                        try:
                            process.stdin.write(json.dumps({'heartbeat':True,'operator_live':live,'cancel':cancel})+'\n');process.stdin.flush()
                        except (OSError,ValueError):break
                        if cancel:break
                        time.sleep(.1)
                threading.Thread(target=pump,daemon=True).start()
                with (self.root/'evidence/exploration_events.jsonl').open('a',encoding='utf-8') as log:
                    for line in process.stdout:
                        try:event=json.loads(line)
                        except ValueError:continue
                        with self.lock:self.events.append(event);self.events=self.events[-100:]
                        log.write(json.dumps(event)+'\n');log.flush()
                code=process.wait()
                if code and not any(e.get('event')=='ERROR' for e in self.events):raise RuntimeError('探索执行器异常退出；查看evidence/exploration_stderr.txt')
        except Exception as exc:
            with self.lock:self.events.append({'event':'ERROR','message':str(exc),'unix':time.time()})
        finally:
            if process and process.stdin:
                try:process.stdin.close()
                except (OSError,ValueError):pass
            with self.lock:self.active=False;self.executing=False;self.owner=None
