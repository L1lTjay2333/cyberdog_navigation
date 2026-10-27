"""Serialized computer-owned stock mapping requests, no automatic retries."""
import json,shlex,subprocess,threading,time
class MappingBridge:
    def __init__(self,root):
        self.root=root;self.lock=threading.Lock();self.active=False;self.events=[];self.name=None
    def start(self,action):
        if action not in ('start','save'):raise ValueError('Unknown mapping operation')
        with self.lock:
            if self.active:raise ValueError('建图请求正在执行，请等待反馈')
            if action=='start' or not self.name:self.name='computer_room_'+time.strftime('%Y%m%d_%H%M%S')
            request={'action':action,'map_name':self.name,'stop_localization':action=='start'}
            request['mapping_sensor']=json.loads((self.root/'config/scope.json').read_text()).get('mapping_sensor','lidar')
            request['navigation_sensor']=json.loads((self.root/'config/scope.json').read_text()).get('navigation_sensor','lidar')
            self.active=True;self.events=[]
        threading.Thread(target=self.run,args=(request,),daemon=True).start()
    def run(self,request):
        try:
            camera=(self.root/'tools/camera_handoff.py').read_text(encoding='utf-8')
            inventory=(self.root/'tools/map_inventory.py').read_text(encoding='utf-8')
            depth=(self.root/'tools/depth_slam_setup.py').read_text(encoding='utf-8') if 'depth_camera' in (request.get('mapping_sensor'),request.get('navigation_sensor')) else ''
            code='REQUEST='+repr(request)+'\n'+camera+'\n'+inventory+'\n'+depth+'\n'+(self.root/'tools/mapping_session.py').read_text(encoding='utf-8')
            command=['ssh','-F',str(self.root/'config/readonly_ssh_config'),'cyberdog-current-nx',
                     'source /etc/mi/ros2_env.conf; python3 -u -c '+shlex.quote('import sys;exec(sys.stdin.read())')]
            result=subprocess.run(command,input=code,text=True,encoding='utf-8',capture_output=True,timeout=100)
            rows=[]
            for line in result.stdout.splitlines():
                try:rows.append(json.loads(line))
                except ValueError:pass
            if result.returncode and not any(r.get('event')=='ERROR' for r in rows):rows.append({'event':'ERROR','message':'SSH建图请求失败；勿自动重试'})
            with self.lock:self.events=rows
            if request['action']=='save' and result.returncode==0 and any(r.get('event')=='SAVED' for r in rows):
                scope_path=self.root/'config/scope.json'
                scope=json.loads(scope_path.read_text(encoding='utf-8'))
                scope['computer_map_name']=request['map_name']
                scope_path.write_text(json.dumps(scope,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
            with (self.root/'evidence/computer_mapping_events.jsonl').open('a',encoding='utf-8') as log:
                for row in rows:log.write(json.dumps(row)+'\n')
        except Exception as e:
            with self.lock:self.events=[{'event':'ERROR','message':str(e)}]
        finally:
            with self.lock:self.active=False
    def report(self):
        with self.lock:return {'active':self.active,'map_name':self.name,'events':list(self.events)}
