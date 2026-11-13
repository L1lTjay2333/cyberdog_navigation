"""Local read-only map viewer; SSH passive subscriber, no robot commands."""
import json
import subprocess
import threading
import time
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit,parse_qs
from map_coordinates import project_pose
from navigation_bridge import NavigationBridge
from localization_validity import LocalizationValidity
from motion_trail import MotionTrail
from telemetry_time import sensor_age
from sensor_consistency import sensor_consistency
from manual_bridge import ManualBridge
from mapping_bridge import MappingBridge
from navigation_decision import NavigationDecision,validate_preparation,validate_goal_distance,motion_problem,localization_problem,preview_reusable
from stock_planning_bridge import stock_preview
from route_preview import world_cell
from unknown_goal import UnknownGoal
from exploration_bridge import ExplorationBridge
from stale_task_reset import reset_stale_navigation
import math

ROOT = Path(__file__).resolve().parents[1]
from live_cloud_archive import LiveCloudArchive
cloud_archive=LiveCloudArchive(ROOT/'evidence/live_cloud_maps')
OFFLINE = False
state = {'map': None, 'connection': 'connecting'}
lock = threading.Lock()
readonly_process=None
connection_feedback='正在连接 SSH 实时订阅'
connection_check_lock=threading.Lock()
bridge=NavigationBridge(ROOT)
manual=ManualBridge(ROOT)
mapping=MappingBridge(ROOT)
selection=None
decision=NavigationDecision()
localization_validity=LocalizationValidity(json.loads((ROOT/'config/scope.json').read_text()).get('localization_rejected_map_name'))
decision_lock=threading.Lock()
planning_lock=threading.Lock()
exploration=UnknownGoal()
explorer=ExplorationBridge(ROOT)

def current_battery(manual_state,snapshot,now=None):
    """Battery feedback outlives teleoperation, not its cached session data."""
    now=time.time() if now is None else now
    candidates=[]
    if manual_state.get('connected'):
        candidates.append(('manual_bms',manual_state.get('battery')))
    if (not OFFLINE and snapshot.get('connection')=='connected' and
        0<=now-snapshot.get('local_received_unix',0)<3):
        candidates.append(('passive_bms',snapshot.get('battery')))
    valid=[(source,b) for source,b in candidates if isinstance(b,dict) and
        type(b.get('received_unix')) in (int,float) and
        sensor_age(snapshot,b['received_unix'],now)<3 and
        type(b.get('percent')) in (int,float) and math.isfinite(b['percent']) and 0<=b['percent']<=100]
    if not valid:return None,'unavailable'
    source,battery=max(valid,key=lambda pair:pair[1]['received_unix'])
    battery=dict(battery)
    # Browser timestamps must use the PC clock, never compare it to robot UTC.
    battery['received_unix']=now-sensor_age(snapshot,battery['received_unix'],now)
    return battery,source

def exploration_snapshot():
    with lock:snapshot=dict(state)
    snapshot['mapping']=mapping.report()
    return snapshot

def decision_report(snapshot,transport):
    scope=json.loads((ROOT/'config/scope.json').read_text())
    snapshot['localization_validation']=localization_validity.update(snapshot,transport,scope.get('computer_map_name'))
    with decision_lock:
        result=decision.evaluate(snapshot,transport,selection)
    if result['stop_required'] and transport['active']:bridge.cancel(result['message'])
    return result

def read_robot_once():
    try:
        from local_cloud_registration import LocalRegistration
        registration=LocalRegistration()
    except ImportError:
        registration=None
    global state,readonly_process
    command = ['ssh', '-F', str(ROOT/'config/readonly_ssh_config'), 'cyberdog-current-nx',
               "source /etc/mi/ros2_env.conf; python3 -u -c \"import sys;exec(sys.stdin.read())\""]
    with (ROOT/'evidence/map_stream_stderr.txt').open('a', encoding='utf-8') as err:
        err.write('\n--- SSH subscriber start '+str(time.time())+' ---\n');err.flush()
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                   stderr=err, text=True, encoding='utf-8')
        with lock:readonly_process=process
        received=[time.monotonic()]
        def watch_stream():
            while process.poll() is None:
                if time.monotonic()-received[0]>20:
                    with lock:state['stream_error']='实时订阅20秒未收到有效数据，正在重连'
                    process.terminate();return
                time.sleep(1)
        threading.Thread(target=watch_stream,daemon=True).start()
        # Embed the pure geometry helper; no custom files are deployed on robot.
        helper=(ROOT/'tools/cloud_geometry.py').read_text(encoding='utf-8')
        # Embed reviewed, ROS-independent teammate helpers; do not install robot files.
        scope=json.loads((ROOT/'config/scope.json').read_text())
        setup='import sys,types\nSAVED_MAP_NAME='+repr(scope.get('computer_map_name'))+'\nSCAN_TOPIC='+repr(scope.get('scan_topic','/scan'))+'\n'
        for name in ('cloud_geometry','obstacle_distance_probe','scan_distance','depth_geometry','depth_preview','saved_grid'):
            content=(ROOT/('tools/'+name+'.py')).read_text(encoding='utf-8')
            setup+='mod=types.ModuleType('+repr(name)+');sys.modules['+repr(name)+']=mod\n'
            setup+='exec('+repr(content)+',mod.__dict__)\n'
        setup+='from cloud_geometry import decode_xyz,transform_xyz\nfrom scan_distance import report_scan\n'
        process.stdin.write(setup+(ROOT/'tools/map_stream.py').read_text(encoding='utf-8'))
        process.stdin.close()
        stream_map=None;trail=MotionTrail();last_evidence=0.;map_serial=0
        for line in process.stdout:
            try:
                item = json.loads(line)
            except ValueError:
                continue
            if item.get('stream_error') and 'task' not in item:
                # A diagnostic line is not a new empty robot snapshot.
                # Keep the previous timestamps; real freshness still expires.
                with lock:state['stream_error']=item['stream_error']
                continue
            if 'map' in item:
                stream_map=item['map'];map_serial+=1
            else:item['map']=stream_map
            item['display_map_revision']=str(process.pid)+':'+str(map_serial)
            item['connection'] = 'connected'
            received[0]=time.monotonic()
            item['local_received_unix'] = time.time()
            name=mapping.report().get('map_name') if (item.get('task') or {}).get('status')==5 else scope.get('computer_map_name')
            cloud=item.get('cloud') or {}
            # Persist acquisition geometry before display-only registration.
            try:cloud_archive.add(item,name)
            except (OSError,ValueError) as exc:cloud_archive.error=str(exc)
            if registration is not None and 0<=item['emitted_unix']-cloud.get('stamp',0)<2:
                registration.apply(cloud,name)
            item.pop('archive_depth',None)
            item['cloud_archive']={'points':len(cloud_archive.voxels),'key':cloud_archive.key,'error':cloud_archive.error}
            item['mapping_sensor']=scope.get('mapping_sensor','lidar')
            item['sensor_consistency']=sensor_consistency(item,time.time())
            if item.get('map') and item.get('pose'):
                try:
                    item['marker'] = project_pose(item['map'], item['pose'])
                except ValueError as exc:
                    item['pose_error'] = str(exc)
            decision_report(item,bridge.report())
            item['trajectory']=trail.update(item,mapping.report().get('map_name') or json.loads((ROOT/'config/scope.json').read_text()).get('computer_map_name'),time.time())
            with lock:
                state = item
            if time.monotonic()-last_evidence<1:continue
            last_evidence=time.monotonic()
            temporary = ROOT/'evidence/latest_map_snapshot.tmp'
            try:
                temporary.write_text(json.dumps(item), encoding='utf-8')
                temporary.replace(ROOT/'evidence/latest_map_snapshot.json')
            except OSError:
                # Windows readers may briefly share-lock the evidence file.
                # Disk evidence failure must not terminate the live subscriber.
                pass
        code = process.wait()
        with lock:
            state['connection'] = 'disconnected'
            state['ssh_exit_code'] = code

def check_connection(reconnect=False, ip=None):
    global connection_feedback
    if not connection_check_lock.acquire(blocking=False):raise ValueError('正在检查 SSH，请稍候')
    try:
        if ip is not None:
            import ipaddress,re,urllib.request
            ip=str(ipaddress.IPv4Address(ip))
            cfg=ROOT/'config/readonly_ssh_config'
            old=cfg.read_text(encoding='utf-8')
            current=re.search(r'HostName\s+(\S+)',old).group(1)
            if ip!=current:
                model=json.load(urllib.request.urlopen('http://127.0.0.1:8766/state?points=0',timeout=3))
                remote=json.load(urllib.request.urlopen('http://127.0.0.1:8766/manual_state',timeout=3))
                if model.get('busy') or remote.get('connected') or manual.started or bridge.report()['active'] or mapping.report()['active'] or explorer.report()['active']:
                    raise ValueError('请先结束采集、遥控和任务，再切换 IP')
                new=re.sub(r'(HostName\s+)\S+',lambda m:m.group(1)+ip,old,count=1)
                temp=cfg.with_suffix('.tmp');temp.write_text(new,encoding='utf-8');temp.replace(cfg)
                with lock:
                    if readonly_process and readonly_process.poll() is None:readonly_process.terminate()
        if reconnect and (manual.started or bridge.report()['active'] or mapping.report()['active'] or explorer.report()['active']):
            raise ValueError('先结束遥控及任务切换，再重新连接数据订阅')
        from ssh_connection_check import check_ssh
        result=check_ssh(ROOT)
        connection_feedback=result['message']
        if not result['ok']:return result
        if reconnect:
            with lock:
                p=readonly_process
            if p and p.poll() is None:p.terminate()
            connection_feedback='SSH 登录成功；已请求重新建立实时数据订阅'
        return {'ok':True,'message':connection_feedback}
    except subprocess.TimeoutExpired:
        connection_feedback='SSH 连接超时：检查狗是否开机、IP 和网络'
        return {'ok':False,'message':connection_feedback}
    finally:connection_check_lock.release()

def refresh_connection():
    global selection,decision
    if not planning_lock.acquire(blocking=False):raise ValueError('任务正在切换，请稍候再刷新重连')
    try:
        if mapping.report()['active']:raise ValueError('地图正在启动或保存，请等待完成后刷新，避免丢失扫描结果')
        explorer.stop()
        manual.disconnect()
        bridge.recover()
        deadline=time.monotonic()+8
        while (manual.started or explorer.report()['active']) and time.monotonic()<deadline:time.sleep(.1)
        if manual.started or explorer.report()['active']:raise ValueError('已请求结束遥控或探索，会话尚未退出，请稍后再刷新')
        selection=None;exploration.clear()
        with decision_lock:decision=NavigationDecision()
        result=check_connection(True)
        if result['ok']:result['message']='已清理旧会话并重新连接；保留已保存地图。等待实时数据后检查定位，再重新选择目的地；不会自动行走。'
        return result
    finally:planning_lock.release()

def read_robot():
    # Reconnect only the passive subscriber. Never resend localization or motion.
    while True:
        try:
            read_robot_once()
        except (OSError,ValueError) as exc:
            with lock:
                state['connection']='disconnected'
                state['stream_error']=str(exc)
        finally:
            # An exception in local geometry/evidence processing must not leave
            # an orphan SSH subscriber behind while the next one reconnects.
            with lock:p=readonly_process
            if p and p.poll() is None:
                p.terminate()
                try:p.wait(timeout=3)
                except subprocess.TimeoutExpired:p.kill();p.wait()
            with (ROOT/'evidence/connection_events.jsonl').open('a',encoding='utf-8') as log:
                log.write(json.dumps({'unix':time.time(),'event':'SUBSCRIBER_EXIT',
                    'exit_code':p.returncode if p else None,'error':state.get('stream_error')})+'\n')
        time.sleep(3)

from module_proxy import proxy_module
from portfolio_store import portfolio_manifest, portfolio_zip, remove_work

class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        model_manual=self.path.startswith('/model/manual/')
        if model_manual:
            self.path=self.path[len('/model'):]
        if self.path == '/model/connection/check':
            self.path='/connection/check'
        elif self.path.startswith('/model/'):
            return proxy_module(self,'/model',8766)
        elif self.path.startswith('/offline/'):
            return proxy_module(self,'/offline',8768)
        global selection
        planning_acquired=False
        try:
            if OFFLINE and self.path not in {'/exploration/target','/exploration/clear','/portfolio/delete'}:
                raise ValueError('离线只读模式：不连接机器狗，不执行建图、定位或运动')
            if self.headers.get('Origin') not in {'http://127.0.0.1:8765','http://localhost:8765'}:
                raise ValueError('请求来源不受信任')
            length=int(self.headers.get('Content-Length','0'))
            if not 0<length<=4096 or self.headers.get('Content-Type')!='application/json':raise ValueError('请求格式无效')
            request=json.loads(self.rfile.read(length))
            if not isinstance(request,dict):raise ValueError('请求必须是目标对象')
            if self.path=='/portfolio/delete':
                response=remove_work(ROOT,request.get('id'))
            elif self.path=='/exploration/target':
                if explorer.report()['active']:raise ValueError('先结束自动探索会话再修改目标')
                response={'ok':True,'exploration':exploration.set_target(exploration_snapshot(),request,OFFLINE)}
            elif self.path=='/exploration/clear':
                if explorer.report()['active']:raise ValueError('先取消自动探索，再清除目标')
                exploration.clear();response={'ok':True,'message':'未知目标规划已清除；未发送运动指令'}
            elif self.path=='/exploration/heartbeat':
                explorer.heartbeat(request.get('owner'));response={'ok':True}
            elif self.path=='/exploration/stop':
                explorer.stop();response={'ok':True,'message':'已请求结束独立探索，等待STOP_SENT及现场停车确认'}
            elif self.path in {'/exploration/start','/exploration/probe','/exploration/scan'}:
                if not planning_lock.acquire(blocking=False):raise ValueError('任务正在切换')
                planning_acquired=True
                if manual.started or bridge.report()['active'] or mapping.report()['active']:raise ValueError('请先结束遥控、定位或建图切换请求')
                current=exploration_snapshot();mp=current['mapping']
                UnknownGoal.check_snapshot(current)
                if (current.get('task') or {}).get('status')!=5 or current['map']['frame']!='laser_odom':raise ValueError('先在本网页启动新场地扫描')
                if not mp.get('map_name') or not any(e.get('event')=='SCANNING' for e in mp['events']):raise ValueError('建图任务必须由本页启动')
                scan_turn=self.path.endswith('/scan')
                execute=self.path.endswith('/start') or scan_turn
                report=exploration.report(current)
                if execute and request.get('approved') is not True:raise ValueError('请先确认现场条件并允许本次自动行走')
                if execute and not scan_turn and report['status'] not in ('GOAL_ROUTE','FRONTIER_ROUTE'):raise ValueError('未知目标尚未生成可通行路线')
                explorer.start({'target':report.get('target') if execute and not scan_turn else None,'scan_turn':scan_turn,'expected_start':current['pose'],'map_name':mp['map_name']},request.get('owner'),execute)
                response={'ok':True,'message':'独立执行器已启动；等待机器人READY反馈'+('，随后低速执行' if execute else '；此会话不发布运动')}
            elif self.path=='/connection/refresh':
                response=refresh_connection()
            elif self.path=='/task/reset_stale':
                if not planning_lock.acquire(blocking=False):raise ValueError('正在处理其他任务，请稍候')
                planning_acquired=True
                with lock:snapshot=dict(state)
                response=reset_stale_navigation(ROOT,snapshot,bridge.report(),mapping.report(),
                    manual.started,explorer.report(),request.get('stopped'))
                selection=None;exploration.clear()
                localization_validity.invalidate('卡住的导航任务已手动清理；请重新定位并核对实物位置')
            elif self.path in {'/connection/check','/connection/reconnect'}:
                response=check_connection(self.path.endswith('/reconnect'),request.get('ip'))
            elif self.path in {'/manual/connect','/manual/control','/manual/stop','/manual/disconnect','/manual/sit','/manual/posture'}:
                if explorer.report()['active']:
                    if self.path=='/manual/stop':explorer.stop()
                    raise ValueError('独立探索占用控制，请先取消探索再遥控')
                if self.path=='/manual/disconnect':manual.disconnect()
                elif self.path=='/manual/stop': manual.control({**request,'direction':'stop','armed':False})
                elif self.path=='/manual/sit': manual.sit(request)
                elif self.path=='/manual/posture': manual.posture(request)
                else:
                    if not planning_lock.acquire(blocking=False):raise ValueError('正在切换任务，请稍后连接遥控')
                    planning_acquired=True
                    nav=bridge.report();mp=mapping.report()
                    if (nav['active'] and nav.get('goal') is not None) or mp['active']:raise ValueError('到点导航或建图切换请求未结束，不能同时遥控')
                    if self.path=='/manual/connect':
                        with lock:snapshot=dict(state)
                        status=(snapshot.get('task') or {}).get('status')
                        # Manual movement does not require reconstruction TF/pose readiness.
                        # Stock scanning (5) is compatible with supervised manual movement,
                        # including when camera preparation starts it after this connection.
                        auxiliary=5
                        if status==7:
                            if not nav['active'] or nav.get('goal') is not None:raise ValueError('定位任务不属于本页')
                            auxiliary=7
                        manual.connect(request.get('owner'),auxiliary)
                    else:manual.control(request)
                response={'ok':True,'message':'遥控请求已处理；以实时反馈与现场动作为准'}
            elif self.path=='/cancel':
                manual.stop();bridge.cancel();explorer.stop();response={'ok':True,'message':'已请求取消，等待执行器与现场停车确认'}
            elif self.path in {'/mapping/start','/mapping/save'}:
                if explorer.report()['active']:raise ValueError('请先取消独立探索，确认停车后再切换建图')
                if manual.started or bridge.report()['active']:raise ValueError('请先结束遥控、导航或定位会话')
                if not planning_lock.acquire(blocking=False):raise ValueError('正在处理其他任务')
                planning_acquired=True
                with lock:snapshot=dict(state)
                if snapshot.get('connection')!='connected' or time.time()-snapshot.get('local_received_unix',0)>3:raise ValueError('机器人反馈断开或过期')
                action='start' if self.path.endswith('/start') else 'save'
                expected=(101,6,8) if action=='start' else (5,)
                task=snapshot.get('task') or {}
                if task.get('status') not in expected or snapshot.get('emitted_unix',0)-task.get('received_unix',0)>2:raise ValueError('当前任务状态不允许此建图操作')
                mapping.start(action);selection=None;exploration.clear()
                response={'ok':True,'message':'已请求'+('备份旧图并扫描新场地；不会自动行走' if action=='start' else '保存建图；以原厂反馈为准')}
            elif self.path=='/relocalize':
                if explorer.report()['active']:raise ValueError('独立探索执行中，不能重新定位')
                if mapping.report()['active']:raise ValueError('建图请求未完成')
                if manual.started:raise ValueError('遥控会话已连接，请结束遥控服务再切换导航')
                if not planning_lock.acquire(blocking=False):raise ValueError('正在处理任务，请稍候')
                planning_acquired=True
                with lock:snapshot=dict(state)
                if snapshot.get('connection')!='connected' or time.time()-snapshot.get('local_received_unix',0)>3:raise ValueError('机器人数据已断开')
                task=snapshot.get('task') or {}
                if task.get('status') not in (8,101,6) or snapshot.get('emitted_unix',0)-task.get('received_unix',0)>2:raise ValueError('已有其他任务或状态过期，不能重新定位')
                if bridge.report()['active']:raise ValueError('已有定位或导航任务，请等待完成')
                snapshot['localization_validation']=localization_validity.update(snapshot,bridge.report(),json.loads((ROOT/'config/scope.json').read_text()).get('computer_map_name'))
                if request.get('force') is not True and localization_problem(snapshot) is None:
                    response={'ok':True,'reused':True,'message':'当前定位有效，已直接就绪；保留选中目标，无需重新匹配场景。位置不准时点击「强制重新定位」。'}
                else:
                    localization_validity.invalidate('正在重新定位，等待本次场景匹配成功')
                    # A localization refresh on the same saved map does not
                    # erase the operator's chosen map target. Its preview is
                    # revalidated before any subsequent navigation request.
                    bridge.start_localization();exploration.clear()
                    response={'ok':True,'reused':False,'message':'已开始重新定位，成功后刷新位置；不会行走'}
            elif self.path in {'/goal','/navigate'}:
                if explorer.report()['active']:raise ValueError('独立探索占用控制，不能同时运行原厂导航')
                if mapping.report()['active']:raise ValueError('建图请求未完成')
                if manual.started:raise ValueError('遥控会话已连接，请结束遥控服务再切换导航')
                if not planning_lock.acquire(blocking=False):raise ValueError('正在处理目标，请稍候')
                planning_acquired=True
                if bridge.report()['active']:raise ValueError('已有导航任务，先取消或等待完成')
                with lock:snapshot=dict(state)
                if snapshot.get('connection')!='connected' or time.time()-snapshot.get('local_received_unix',0)>3:raise ValueError('机器人数据已断开')
                pose=snapshot.get('pose');grid=snapshot.get('map')
                snapshot['localization_validation']=localization_validity.update(snapshot,bridge.report(),json.loads((ROOT/'config/scope.json').read_text()).get('computer_map_name'))
                problem=localization_problem(snapshot)
                if problem:raise ValueError(problem)
                if (snapshot.get('task') or {}).get('status')!=8:raise ValueError('请先完成原厂定位，再选择导航目标')
                if not pose or not grid or snapshot['emitted_unix']-pose['received_unix']>2:raise ValueError('定位数据未就绪')
                if not all(type(request.get(k)) in (int,float) for k in ('x','y')):raise ValueError('目标格式无效')
                x,y=float(request['x']),float(request['y'])
                if grid['frame']!='map' or not all(math.isfinite(v) for v in (x,y)):raise ValueError('请等待原厂map地图和定位就绪')
                ix,iy=world_cell(grid,x,y)
                if not(0<=ix<grid['width'] and 0<=iy<grid['height']) or grid['data'][iy*grid['width']+ix]!=0:raise ValueError('目标位于障碍、未知区域或地图外')
                target={'x':x,'y':y,'frame':'map'}
                validate_goal_distance(pose,target,bridge.report().get('max_goal_distance_m'))
                # The executor still replans on robot. Avoid a second SSH preview
                # for an unchanged, recently validated target on the same map.
                if self.path=='/navigate' and preview_reusable(selection,target,snapshot):
                    route=selection['preview']
                else:
                    route=stock_preview(ROOT,target)
                with lock:current=dict(state)
                current['localization_validation']=localization_validity.update(current,bridge.report(),json.loads((ROOT/'config/scope.json').read_text()).get('computer_map_name'))
                prepared=validate_preparation(current,{'x':x,'y':y,'frame':'map'},route)
                selection=prepared;exploration.clear()
                if self.path=='/navigate':
                    problem=motion_problem(current)
                    if problem:raise ValueError(problem)
                    if math.hypot(current['pose']['x']-x,current['pose']['y']-y)<=.25:raise ValueError('当前位置已在目标范围，无需行走')
                    bridge.start(selection['goal'],expected_start=current['pose'])
                response={'ok':True,**selection}
            else:raise ValueError('未知请求')
            status=200
        except (ValueError,KeyError,TypeError) as exc:
            response={'ok':False,'message':str(exc)};status=400
        finally:
            if planning_acquired:planning_lock.release()
        data=json.dumps(response,ensure_ascii=False).encode('utf-8')
        self.send_response(status);self.send_header('Content-Type','application/json; charset=utf-8');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
    def do_GET(self):
        if self.path.startswith('/model/manual_state'):
            self.path=self.path[len('/model'):]
        if self.path.startswith('/model/'):
            return proxy_module(self,'/model',8766)
        if self.path.startswith('/offline/'):
            return proxy_module(self,'/offline',8768)
        if urlsplit(self.path).path == '/manual_state':
            owner=parse_qs(urlsplit(self.path).query).get('owner',[None])[0]
            result=manual.report(owner)
            if OFFLINE:result.update(connected=False,enabled=False,active=False)
            with lock:
                snapshot=dict(state)
            result['battery'],result['battery_source']=current_battery(result,snapshot)
            distance=dict((snapshot.get('scan') or {}).get('distance') or {})
            if (snapshot.get('connection')!='connected' or
                time.time()-snapshot.get('local_received_unix',0)>1. or
                snapshot.get('emitted_unix',0)-(snapshot.get('scan') or {}).get('received_unix',0)>1.):
                distance.update(valid=False,reason='实时扫描断开或过期')
            result['distance']=distance
            data=json.dumps(result,ensure_ascii=False).encode('utf-8')
            content='application/json; charset=utf-8'
        elif self.path == '/manual':
            data=(ROOT/'manual.html').read_bytes();content='text/html; charset=utf-8'
        elif self.path == '/manual.js':
            data=(ROOT/'manual.js').read_bytes();content='text/javascript; charset=utf-8'
        elif urlsplit(self.path).path == '/state':
            bridge.operator_ping()
            with lock:
                report=dict(state)
            report['navigation']=bridge.report();report['selection']=selection
            report['mapping']=mapping.report()
            report['exploration']=exploration.report(report,OFFLINE)
            report['exploration']['executor']=explorer.report()
            report['exploration']['execution_supported']=not OFFLINE and report['exploration']['executor']['deployed']
            report['exploration']['execution_reason']='独立执行器已部署；需本次现场确认，首次实机验收仍待完成' if report['exploration']['execution_supported'] else report['exploration']['execution_reason']
            if report['exploration']['executor']['active']:
                plan_event=next((e for e in reversed(report['exploration']['executor']['events']) if e.get('event')=='PLAN'),None)
                if plan_event:report['exploration'].update(preview=plan_event.get('preview'),waypoint=plan_event.get('waypoint'))
            report['offline']=OFFLINE
            report['reset_stale_available']=not OFFLINE
            report['ssh_feedback']=connection_feedback
            if OFFLINE:
                report['navigation'].update(enabled=False,active=False)
            new_route=next((e for e in reversed(report['navigation']['events']) if e.get('event')=='REPLAN_SUCCESS'),None)
            if new_route and selection and report['navigation'].get('goal')==selection['goal']:
                points=new_route['points']
                report['selection']={**selection,'preview':{'frame':'map','points':points,'kind':'stock_nav2_preview','length_m':sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(points,points[1:]))}}
            report['decision']=decision_report(report,report['navigation'])
            known_revision=parse_qs(urlsplit(self.path).query).get('map_revision',[None])[0]
            if known_revision and known_revision==report.get('display_map_revision') and report.get('map'):
                report['map']={k:v for k,v in report['map'].items() if k!='data'}
            data = json.dumps(report).encode('utf-8')
            content = 'application/json; charset=utf-8'
        elif self.path == '/cloud/saved':
            data=json.dumps(cloud_archive.read()).encode('utf-8');content='application/json; charset=utf-8'
        elif self.path in {'/', '/index.html'}:
            data = (ROOT/'home.html').read_bytes()
            content = 'text/html; charset=utf-8'
        elif self.path.startswith('/portfolio/'):
            import mimetypes
            folder=(ROOT/'assets/portfolio').resolve()
            name=urlsplit(self.path).path[len('/portfolio/'):] or 'index.html'
            if name == 'manifest.json':
                data = json.dumps(portfolio_manifest(ROOT), ensure_ascii=False).encode('utf-8')
                content = 'application/json; charset=utf-8'
            elif name == 'portfolio.zip':
                data = portfolio_zip(ROOT)
                content = 'application/zip'
            else:
                target=(folder/name).resolve()
                if target.parent!=folder or not target.is_file() or name.endswith('.tmp') or name == 'portfolio.next.zip':
                    self.send_error(404);return
                data=target.read_bytes()
                content=mimetypes.guess_type(str(target))[0] or 'application/octet-stream'
        elif self.path == '/navigation':
            data=(ROOT/'navigation.html').read_bytes()
            content='text/html; charset=utf-8'
        elif self.path in ('/assets/home/text-ui.css','/assets/home/text-ui.js','/assets/home/navigation-ui.js','/assets/home/navigation-ui.css','/assets/home/home-ui.js','/assets/home/home-ui.css','/assets/home/dog-color.png','/assets/home/dog-dither.png'):
            data=(ROOT/self.path.lstrip('/')).read_bytes()
            content='image/png' if self.path.endswith('.png') else ('text/javascript; charset=utf-8' if self.path.endswith('.js') else 'text/css; charset=utf-8')
        elif self.path.startswith('/assets/home/dog-') and self.path.endswith('.jpg'):
            import re
            filename=self.path.rsplit('/',1)[-1]
            if not re.fullmatch(r'dog-(3[3-9]|[45][0-9]|6[0-6])\.jpg',filename):
                self.send_error(404);return
            data=(ROOT/'assets/home'/filename).read_bytes()
            content='image/jpeg'
        elif self.path == '/depth_live.js':
            data=(ROOT/'depth_live.js').read_bytes()
            content='text/javascript; charset=utf-8'
        elif self.path == '/cloud_view.js':
            data=(ROOT/'cloud_view.js').read_bytes()
            content='text/javascript; charset=utf-8'
        elif self.path == '/unknown_goal.js':
            data=(ROOT/'unknown_goal.js').read_bytes()
            content='text/javascript; charset=utf-8'
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', content)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)
    def log_message(self, *args):
        pass

if __name__ == '__main__':
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('--offline',action='store_true',help='Show recorded evidence without any robot connections')
    parser.add_argument('--resume-mapping',type=Path,help='Explicit recent no-motion restart snapshot; does not resume motion')
    args=parser.parse_args();OFFLINE=args.offline
    if args.resume_mapping and not OFFLINE:
        restored=json.loads(args.resume_mapping.read_text(encoding='utf-8'))
        if not 0<=time.time()-restored['captured_unix']<=30:raise RuntimeError('建图会话恢复凭据过期')
        mp=restored['mapping']
        if mp.get('active') or not mp.get('map_name') or not any(e.get('event')=='SCANNING' for e in mp.get('events',[])):raise RuntimeError('建图会话恢复信息无效')
        mapping.name=mp['map_name'];mapping.events=mp['events']
    if OFFLINE:
        snapshot=ROOT/'evidence/latest_map_snapshot.json'
        if snapshot.exists():
            try:state=json.loads(snapshot.read_text(encoding='utf-8'))
            except (ValueError,OSError):state={'map':None}
        state.update(connection='offline',offline=True)
    server = ThreadingHTTPServer(('127.0.0.1', 8765), Handler)
    if not OFFLINE:threading.Thread(target=read_robot, daemon=True).start()
    print(('Offline recorded map' if OFFLINE else 'Robot map viewer')+': http://127.0.0.1:8765', flush=True)
    server.serve_forever()
