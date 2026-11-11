"""Independent local static-model laboratory, port 8766; no robot controls."""
import argparse
import json
import shlex
import subprocess
import threading
import time
from datetime import datetime
from urllib.parse import urlsplit,parse_qs
from manual_bridge import ManualBridge
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from reconstruction_core import StaticModel, MovingModel, decode_depth_frame
from reconstruction_static import ConsensusStaticModel
from reconstruction_archive import FrameArchive
from reconstruction_target import TargetBox
from native_photo_capture import capture as capture_native_photo
from reconstruction_streaming import StreamingModel
from reconstruction_readiness import prepare_inputs

ROOT = Path(__file__).resolve().parents[1]
MANUAL = ManualBridge(ROOT)


def capture_failure(stderr, error):
    if 'not responding' in stderr or 'Broken pipe' in stderr or 'Connection reset' in stderr:
        return 'SSH传输中断：机器人连接超时或被断开；已保留完整收到的数据，未完成的末帧不计入，请检查网络与机器人状态'
    if 'port 22: Permission denied' in stderr:
        return 'SSH网络连接被运行环境权限限制拒绝；请在允许局域网访问的环境启动建模服务（不是深度帧错误）'
    if 'Permission denied (publickey' in stderr or 'bad permissions' in stderr:
        return 'SSH密钥认证失败；请核对本机密钥路径、文件权限和已授权公钥'
    if 'Host key verification failed' in stderr or 'REMOTE HOST IDENTIFICATION HAS CHANGED' in stderr:
        return 'SSH主机身份校验失败；保留旧记录并先核对机器人身份'
    if 'Connection timed out' in stderr or 'No route to host' in stderr or 'Connection refused' in stderr:
        return 'SSH连接不可用；请核对机器人电源、IP和同一局域网'
    if isinstance(error, BrokenPipeError):
        return 'SSH采集进程提前退出；请查看本次会话stderr确认原因，未收到可融合数据'
    return str(error)


class Laboratory:
    def __init__(self):
        self.lock = threading.Lock()
        self.model = ConsensusStaticModel()
        self.target = None
        self.native_photo = {'busy':False,'message':'停稳后可拍原生最高分辨率照片'}
        self.native_photo_last = 0.
        self.process = None
        self.active = False
        self.status = '尚未采集；只读静态实验，不控制机器狗'
        self.session = None
        self.remote = {}
        self.error = None
        self.samples = []
        self.color_pairs = 0
        self.stopped = threading.Event()
        self.started_unix = None
        self.raw_archive = None
        self.received_frames = 0
        self.unposed_archive = None
        self.fusion_paused = None
        self.preparing = False
        self.running_job = False
        self.camera = {'ready':False,'camera_ready':False,'message':'尚未检查；每次开始采集都会重新核验'}

    def take_native_photo(self, stationary):
        if stationary is not True:raise ValueError('请先停稳机器狗，再点击拍照')
        with self.lock:
            if self.native_photo['busy'] or self.preparing:
                raise ValueError('相机正在准备或拍照，请稍候')
            if time.monotonic()-self.native_photo_last<5:
                raise ValueError('原生拍照请至少间隔5秒')
            session=self.session
            self.native_photo_last=time.monotonic()
            self.native_photo={'busy':True,'message':'正在拍摄原生照片并传回电脑，请继续保持静止'}
        def job():
            try:
                report=capture_native_photo(ROOT,session)
                with self.lock:self.native_photo={'busy':False,'message':'已保存原生照片 %d×%d'%(report['width'],report['height']),'result':report}
            except Exception as error:
                with self.lock:self.native_photo={'busy':False,'message':str(error),'error':True}
        threading.Thread(target=job,daemon=True).start()

    def start(self, seconds=0, mode='moving', target_size=None):
        target = TargetBox(target_size) if target_size is not None else None
        if mode not in ('static', 'moving'):
            raise ValueError('未知采集模式')
        limit = 300 if mode=='moving' else 60
        if type(seconds) is not int or not (seconds == 0 or 1 <= seconds <= limit):
            raise ValueError('本模式采集时长须为1—%d秒'%limit)
        with self.lock:
            if self.active or self.preparing or self.running_job:
                raise ValueError('已有采集或相机准备正在进行')
            self.target = target
            self.preparing = True
            self.running_job = True
            self.stopped.clear()
            self.camera = dict(self.camera,ready=False,message='采集前正在检查深度图与同时间位姿')
            self.status = self.camera['message']
        threading.Thread(target=self.prepare_job,args=(False,seconds,mode),daemon=True).start()

    def connect_camera(self):
        with self.lock:
            if self.active or self.preparing or self.running_job:raise ValueError('请先结束当前采集或相机检查')
            self.preparing = True;self.stopped.clear()
            self.running_job = True
            self.camera = dict(self.camera,ready=False,message='正在连接/检查深度相机与扫描输入')
            self.status = self.camera['message']
        threading.Thread(target=self.prepare_job,args=(True,None,None),daemon=True).start()

    def prepare_job(self,connect,seconds,mode):
        try:
            def process_callback(process):self.process=process
            result=prepare_inputs(ROOT,connect,self.stopped,process_callback)
            with self.lock:
                self.camera=result
                self.status=result.get('message','输入检查结束')
            if self.stopped.is_set() or not result.get('ready') or seconds is None:return
            with self.lock:
                if self.stopped.is_set():return
                self.begin_capture(seconds,mode)
            self.capture(seconds)
        except Exception as exc:
            message=capture_failure(getattr(exc,'stderr',''),exc)
            with self.lock:
                self.camera=dict(self.camera,ready=False,message=message)
                self.status=message
        finally:
            with self.lock:
                self.preparing=False
                self.running_job=False
                if not self.active:self.process=None
                if self.stopped.is_set():
                    self.camera=dict(self.camera,ready=False,message='电脑准备/采集已取消；下次开始会重新检查')

    def begin_capture(self,seconds,mode):
        """Caller holds lock, after a fresh successful input check."""
        self.preparing = False
        self.model = MovingModel() if mode == 'moving' else ConsensusStaticModel()
        self.active, self.error, self.remote = True, None, {}
        self.session = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.started_unix = time.time()
        self.status = ('连接移动采集；先静止5秒，等有效帧到达后再人工低速移动' if mode == 'moving'
                       else '连接只读采集；请保持机器狗和场景静止')
        self.samples = []
        self.color_pairs = 0
        self.raw_archive = None
        self.received_frames = 0
        self.unposed_archive = None
        self.fusion_paused = None

    def stop(self):
        self.stopped.set()
        with self.lock:
            if self.preparing:self.status='已请求取消准备；等待接口返回，不会开始采集'
        process = self.process
        if process is not None and process.poll() is None:
            process.terminate()  # Stop only this SSH subscriber, never a robot task.

    def accept_record(self, record):
        """Archive validated observations even when online fusion is suspended."""
        decode_depth_frame(record)
        self.raw_archive.append(record)
        if self.raw_archive.truncated:
            raise ValueError('原始数据存储额度已满；已保存数据保留，请结束本轮并新建采集')
        self.received_frames += 1
        if record.get('color_sample') is not None:self.color_pairs += 1
        if self.fusion_paused and not isinstance(self.model, StreamingModel):
            self.status = '继续保存原始数据；在线融合已暂停：'+self.fusion_paused
            return False
        try:
            candidate = self.target.apply(record) if self.target is not None else record
            changed = self.model.add(candidate)
        except ValueError as exc:
            self.fusion_paused = str(exc)
            self.status = '继续保存原始数据；在线融合已暂停：'+self.fusion_paused
            return False
        self.fusion_paused = None
        self.status = '采集中：原始数据持续保存，模型实时融合'
        if changed:
            self.samples.append({'stamp':record['stamp'],'source_age_s':record.get('source_age_s'),
                                 'translation':record['translation'],'quaternion':record['quaternion'],**self.model.stats()})
        return changed

    def capture(self, seconds):
        output = ROOT/'evidence'/'reconstruction'/self.session
        process = None
        watchdog = None
        raw_saved_bytes = 0
        moving = isinstance(self.model, MovingModel)
        try:
            output.mkdir(parents=True, exist_ok=False)
            self.raw_archive = FrameArchive(output/'frames.jsonl',max_bytes=None)
            self.unposed_archive = FrameArchive(output/'unposed_frames.jsonl',max_bytes=None)
            if self.stopped.is_set():
                return
            if moving:
                with self.lock:
                    self.model = StreamingModel(output/'submaps')
            helper = (ROOT/'tools/reconstruction_core.py').read_text(encoding='utf-8')
            code = 'import sys,types\nm=types.ModuleType("reconstruction_core");sys.modules["reconstruction_core"]=m\nexec('+repr(helper)+',m.__dict__)\n'
            code += 'CAPTURE_SECONDS='+str(seconds)+'\n'+(ROOT/'tools/reconstruction_capture.py').read_text(encoding='utf-8')
            command = ['ssh', '-F', str(ROOT/'config/readonly_ssh_config'), 'cyberdog-current-nx',
                       'source /etc/mi/ros2_env.conf; python3 -u -c '+shlex.quote('import sys;exec(sys.stdin.read())')]
            with (output/'stderr.txt').open('w', encoding='utf-8') as err:
                process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=err,
                                           text=True, encoding='utf-8', cwd=str(ROOT))
                self.process = process
                if seconds:
                    watchdog = threading.Timer(seconds+15, self.stop)
                    watchdog.daemon = True
                    watchdog.start()
                process.stdin.write(code)
                process.stdin.close()
                for line in process.stdout:
                    if self.stopped.is_set():
                        break
                    if len(line) > 24*1024*1024:
                        raise ValueError('采集数据超过单帧上限')
                    record = json.loads(line)
                    if record.get('event') == 'unposed_depth':
                        self.unposed_archive.append(record)
                        continue
                    if record.get('event') == 'depth':
                        if abs(record.get('tf_stamp', 0)-record['stamp']) > 1e-6:
                            raise ValueError('深度帧与位姿时间不匹配，停止融合')
                        with self.lock:
                            changed = self.accept_record(record)
                        if changed and (len(self.samples) <= 2 or moving and len(self.samples)%10 == 0):
                            raw = json.dumps(record).encode('utf-8')
                            if raw_saved_bytes+len(raw) <= 8*1024*1024:
                                (output/('frame_%02d.json'%len(self.samples))).write_bytes(raw)
                                raw_saved_bytes += len(raw)
                    else:
                        with self.lock:
                            self.remote = record
                            if record.get('event') == 'waiting':self.status = '等待数据恢复：'+record.get('message','')
                            if record.get('event') == 'error':
                                self.error = record.get('message', '采集数据失效')
                                self.camera=dict(self.camera,ready=False,message=self.error)
                code = process.wait(timeout=5)
                if code and not self.stopped.is_set():
                    raise RuntimeError('相机订阅退出，请查看本次stderr；未操作机器人')
        except Exception as exc:
            try:
                details = (output/'stderr.txt').read_text(encoding='utf-8')[-4000:]
            except OSError:
                details = ''
            with self.lock:
                if not self.stopped.is_set():
                    self.error = capture_failure(details, exc)
                    self.camera=dict(self.camera,ready=False,message=self.error)
        finally:
            if watchdog:
                watchdog.cancel()
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
            with self.lock:
                self.status = self.error or (('采集结束；原始数据已保存，部分未融合：'+self.fusion_paused) if self.fusion_paused else None) or ('采集已结束；保留模型，尚未完成绝对精度验收' if self.model.accepted_frames else '未收到可融合帧；检查相机与同时间TF')
            try:
                if isinstance(self.model,StreamingModel):
                    with self.lock:self.model.finish()
                snapshot = self.report(include_points=False)
                snapshot['active'] = False
                (output/'summary.json').write_text(json.dumps(snapshot, ensure_ascii=False, indent=2), encoding='utf-8')
                (output/'samples.json').write_text(json.dumps(self.samples, indent=2), encoding='utf-8')
                with (output/'model.ply').open('wb') as model_file:
                    for chunk in self.ply_chunks():model_file.write(chunk)
            except (OSError,ValueError) as exc:
                with self.lock:
                    self.error = self.status = '模型保存失败：'+str(exc)
            finally:
                with self.lock:
                    self.active = False
                    self.process = None

    def report(self, include_points=True, since=None):
        with self.lock:
            version = f'{self.session}:{self.model.accepted_frames}'
            include_points = include_points and since != version
            streaming = isinstance(self.model,StreamingModel)
            points = self.model.preview() if streaming and include_points else [] if streaming else self.model.points()
            count = self.model.stable_count() if streaming else len(points)
            result = dict(self.model.stats(), status=self.status, active=self.active,
                          session=self.session, started_unix=self.started_unix, error=self.error,
                          remote=self.remote, stable_points=count, control_requests=0,
                          mode='moving_local_point_cloud' if isinstance(self.model,MovingModel) else 'static_local_point_cloud',
                          moving_supported=True, calibration_status='未完成真实尺寸校准')
            result['model_version'] = version
            result.update(received_frames=self.received_frames, fusion_paused=self.fusion_paused,
                          unfused_frames=max(0,self.received_frames-self.model.accepted_frames))
            result['unposed_frames'] = self.unposed_archive.frames if self.unposed_archive else 0
            result['native_photo'] = self.native_photo
            result['color_pairs'] = self.color_pairs
            result['color_geometry_verified'] = False
            result['target_box'] = {'center':self.target.center.tolist() if self.target.center is not None else None,'size':self.target.size.tolist()} if self.target else None
            result.update(preparing=self.preparing,camera=self.camera,busy=self.active or self.preparing or self.running_job)
            result['raw_archive'] = self.raw_archive.stats(self.received_frames) if self.raw_archive else None
            result['capture_reference'] = self.capture_reference()
            result['trajectory'] = getattr(self.model,'trajectory',[])
            if include_points:
                result['points'] = points[::max(1, (len(points)+29999)//30000)]
            return result

    def capture_reference(self):
        if self.model.anchor is None:
            return None
        moving = isinstance(self.model, MovingModel)
        t, q = self.model.last_pose if moving and self.model.last_pose is not None else self.model.anchor
        x, y, z, w = (float(v) for v in q)
        # Optical +Z is the camera viewing direction, not body +X or head joint yaw.
        forward = [2*(x*z+y*w), 2*(y*z-x*w), 1-2*(x*x+y*y)]
        return {'position':[float(v) for v in t], 'forward':forward,
                'frame':self.model.frame, 'session':self.session,
                'kind':'captured_camera_origin', 'live':False,
                'stamp':self.model.stamp, 'latest_fused_pose':moving,
                'label':'狗头参考（最新已融合帧）' if moving else '狗头参考（相机位置，采集时）'}

    def restore_latest(self):
        """Restore a completed local model for viewing, never start robot capture."""
        folders = sorted((ROOT/'evidence/reconstruction').glob('*'), reverse=True)
        for folder in folders:
            if not all((folder/name).is_file() for name in ('summary.json','frame_01.json','model.ply')):
                continue
            summary = json.loads((folder/'summary.json').read_text(encoding='utf-8'))
            self.received_frames = summary.get('received_frames',summary.get('accepted_frames',0))
            self.fusion_paused = summary.get('fusion_paused')
            if summary.get('stable_points',0) <= 0:
                continue  # An empty/failed capture must not hide the last usable model.
            source = json.loads((folder/'frame_01.json').read_text(encoding='utf-8'))
            if summary.get('streaming_submaps'):
                if not (folder/'submaps/manifest.json').is_file():continue
                self.model = StreamingModel.restore(folder/'submaps',summary,source)
                self.session,self.remote = folder.name,summary.get('remote',{})
                self.started_unix = summary.get('started_unix')
                self.status = '已恢复分块模型；显示为抽样预览，完整点保存在磁盘'
                return True
            from reconstruction_core import checked_pose
            import numpy as np
            anchor = checked_pose(source)
            header, content = (folder/'model.ply').read_text(encoding='ascii').split('end_header\n', 1)
            count = int(next(line.split()[2] for line in header.splitlines() if line.startswith('element vertex ')))
            if not 0 <= count <= 200000 or summary['frame'] != source['frame']:
                raise ValueError('Saved model bounds or reference frame invalid')
            points = np.asarray([[float(v) for v in line.split()] for line in content.splitlines() if line.strip()], dtype=float).reshape((-1,3))
            if len(points) != count or not np.isfinite(points).all():
                raise ValueError('Saved model data invalid')
            moving = summary.get('mode') == 'moving_local_point_cloud'
            model = MovingModel(voxel=float(summary['voxel_m'])) if moving else StaticModel(voxel=float(summary['voxel_m']))
            scaled = np.floor(points/model.voxel)
            if not np.isfinite(scaled).all() or np.any(np.abs(scaled) > np.iinfo(np.int64).max//2):
                raise ValueError('Saved model coordinates exceed bounds')
            for key, p in zip(scaled.astype(np.int64),points):
                # Rounding in ASCII export can put neighbors in the same voxel.
                cell = model.cells.setdefault(tuple(key),[np.zeros(3),0,2])
                cell[0] += p;cell[1] += 1
            model.anchor, model.frame, model.source_frame = anchor, source['frame'], source['source_frame']
            model.accepted_frames, model.stamp = int(summary['accepted_frames']), float(summary['last_stamp'])
            model.last_pose = anchor
            if moving:
                model.trajectory = summary.get('trajectory',[])[:360]
                model.travel_m = float(summary.get('travel_m',0.))
                if model.trajectory:
                    last = model.trajectory[-1]
                    model.last_pose = checked_pose(dict(source,translation=last['position'],quaternion=last['quaternion']))
            self.model, self.session, self.remote = model, folder.name, summary.get('remote',{})
            self.started_unix = summary.get('started_unix')
            self.status = '已恢复上次模型；狗头为采集帧相机参考，不是实时位置'
            return True
        return False

    def ply_chunks(self):
        with self.lock:
            frame = self.model.frame
            if isinstance(self.model,StreamingModel):
                count,batches = self.model.export_snapshot()
                note = 'submap boundary points may repeat; no global deduplication'
            else:
                points = self.model.points();count = len(points)
                batches = lambda:iter([points])
                note = 'local voxel point model; scale and drift unvalidated'
        header = 'ply\nformat ascii 1.0\ncomment frame %s\ncomment %s\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nend_header\n'%(frame,note,count)
        yield header.encode('ascii')
        for batch in batches():
            for start in range(0,len(batch),2048):
                yield ''.join('%.7f %.7f %.7f\n'%tuple(p) for p in batch[start:start+2048]).encode('ascii')

    def ply(self):
        """Small-model compatibility helper; HTTP and disk exports stream chunks."""
        return b''.join(self.ply_chunks())


LAB = Laboratory()


class Handler(BaseHTTPRequestHandler):
    def send(self, data, kind='application/json; charset=utf-8', status=200):
        self.send_response(status)
        self.send_header('Content-Type', kind)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if urlsplit(self.path).path == '/scan-manual':
            import re
            nav=(ROOT/'navigation.html').read_text(encoding='utf-8')
            panel=nav[nav.index('<div class="card" id="manual-panel">'):nav.index('</aside>',nav.index('<div class="card" id="manual-panel">'))]
            page='<html><head><meta charset="utf-8"><style>body{background:#f4f1ea;color:#494236;font:13px system-ui;margin:8px}button{padding:9px;margin:3px;border:1px solid #cdbfa5;border-radius:8px;background:#fffaf0}canvas{max-width:110px;max-height:110px}pre,details,.note,a{display:none}input{max-width:95%}h2{font-size:16px}</style></head><body>'+panel+'<script src="/manual.js"></script></body></html>'
            self.send(page.encode('utf-8'),'text/html; charset=utf-8')
        elif self.path == '/assets/home/text-ui.css':
            self.send((ROOT/'assets/home/text-ui.css').read_bytes(),'text/css; charset=utf-8')
        elif self.path == '/assets/home/text-ui.js':
            self.send((ROOT/'assets/home/text-ui.js').read_bytes(),'text/javascript; charset=utf-8')
        elif self.path == '/manual.js':
            self.send((ROOT/'manual.js').read_bytes(),'text/javascript; charset=utf-8')
        elif urlsplit(self.path).path == '/manual_state':
            owner=parse_qs(urlsplit(self.path).query).get('owner',[None])[0]
            report=MANUAL.report(owner)
            report['manual_distance_override']=json.loads((ROOT/'config/scope.json').read_text(encoding='utf-8')).get('manual_distance_override') is True
            self.send(json.dumps(report,ensure_ascii=False).encode('utf-8'))
        elif self.path == '/manual_transport.js':
            self.send((ROOT/'manual_transport.js').read_bytes(),'text/javascript; charset=utf-8')
        elif self.path in ('/', '/index.html'):
            if self.headers.get('X-Console-Proxy')!='1':
                self.send_response(302);self.send_header('Location','http://127.0.0.1:8765/model/');self.end_headers();return
            self.send((ROOT/'reconstruction.html').read_bytes(), 'text/html; charset=utf-8')
        elif urlsplit(self.path).path == '/state':
            query = parse_qs(urlsplit(self.path).query)
            result = LAB.report(include_points=query.get('points', ['1'])[0] != '0',
                                since=query.get('since', [None])[0])
            if query.get('points', ['1'])[0] == '0':
                result.pop('trajectory', None)
            self.send(json.dumps(result, ensure_ascii=False, allow_nan=False).encode('utf-8'))
        elif self.path == '/model.ply':
            self.send_response(200)
            self.send_header('Content-Type','application/octet-stream')
            self.send_header('Cache-Control','no-store')
            self.send_header('Connection','close')
            self.end_headers()
            self.close_connection = True
            try:
                for chunk in LAB.ply_chunks():self.wfile.write(chunk)
            except (BrokenPipeError,ConnectionResetError):pass
        else:
            self.send(b'not found', 'text/plain', 404)

    def do_POST(self):
        try:
            if self.headers.get('Origin') not in ('http://127.0.0.1:8766', 'http://localhost:8766'):
                raise ValueError('请求来源不受信任')
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 256 or self.headers.get('Content-Type') != 'application/json':
                raise ValueError('请求格式错误')
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict):
                raise ValueError('请求格式错误')
            if self.path in ('/manual/connect','/manual/control','/manual/stop','/manual/disconnect','/manual/posture','/manual/sit'):
                if self.path == '/manual/connect':
                    # Explicit operator takeover of scanning locomotion; verify real task afresh.
                    result=prepare_inputs(ROOT,False,threading.Event(),lambda p:None)
                    if not result.get('ready') or result.get('task')!={'status':5,'code':0}:
                        raise ValueError('扫描输入未就绪，不能接管扫描遥控')
                    MANUAL.connect(data.get('owner'),auxiliary_task=5)
                elif self.path == '/manual/posture':MANUAL.posture(data)
                elif self.path == '/manual/sit':MANUAL.sit(data)
                elif self.path == '/manual/disconnect':MANUAL.disconnect()
                elif self.path == '/manual/stop':MANUAL.control({**data,'direction':'stop','armed':False})
                else:MANUAL.control(data)
            elif self.path == '/start':
                LAB.start(data.get('seconds', 0), mode=data.get('mode','moving'), **({'target_size':data['target_size']} if 'target_size' in data else {}))
            elif self.path == '/photo/native':
                LAB.take_native_photo(data.get('stationary'))
            elif self.path == '/camera/connect':
                LAB.connect_camera()
            elif self.path == '/stop':
                LAB.stop()
            else:
                raise ValueError('未知实验操作')
            self.send(b'{"ok":true}')
        except (ValueError, TypeError, RuntimeError, OSError) as exc:
            self.send(json.dumps({'ok':False,'error':str(exc)}, ensure_ascii=False).encode('utf-8'), status=400)

    def log_message(self, *args):
        pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--capture-seconds', type=int, help='Explicitly start one bounded, read-only static capture')
    parser.add_argument('--restore-latest', action='store_true', help='View the latest saved local model without robot access')
    args = parser.parse_args()
    server = ThreadingHTTPServer(('127.0.0.1', 8766), Handler)
    if args.restore_latest:
        LAB.restore_latest()
    if args.capture_seconds is not None:
        LAB.start(args.capture_seconds)
    print('Static reconstruction laboratory: http://127.0.0.1:8766/', flush=True)
    try:
        server.serve_forever()
    finally:
        LAB.stop()
        MANUAL.disconnect()
        server.server_close()
