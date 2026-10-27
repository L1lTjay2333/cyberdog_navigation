"""Independent local static-model laboratory, port 8766; no robot controls."""
import argparse
import json
import shlex
import subprocess
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from reconstruction_core import StaticModel, MovingModel
from reconstruction_archive import FrameArchive

ROOT = Path(__file__).resolve().parents[1]


def capture_failure(stderr, error):
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
        self.model = StaticModel()
        self.process = None
        self.active = False
        self.status = '尚未采集；只读静态实验，不控制机器狗'
        self.session = None
        self.remote = {}
        self.error = None
        self.samples = []
        self.stopped = threading.Event()
        self.started_unix = None
        self.raw_archive = None

    def start(self, seconds=30, mode='static'):
        if type(seconds) is not int or not 1 <= seconds <= 60:
            raise ValueError('采集时长须为1—60秒')
        if mode not in ('static', 'moving'):
            raise ValueError('未知采集模式')
        with self.lock:
            if self.active:
                raise ValueError('已有静态采集会话')
            self.model = MovingModel() if mode == 'moving' else StaticModel()
            self.active, self.error, self.remote = True, None, {}
            self.stopped.clear()
            self.session = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            self.started_unix = time.time()
            self.status = ('连接移动采集；先静止5秒，等有效帧到达后再人工低速移动' if mode == 'moving'
                           else '连接只读采集；请保持机器狗和场景静止')
            self.samples = []
            self.raw_archive = None
        threading.Thread(target=self.capture, args=(seconds,), daemon=True).start()

    def stop(self):
        self.stopped.set()
        process = self.process
        if process is not None and process.poll() is None:
            process.terminate()  # Stop only this SSH subscriber, never a robot task.

    def capture(self, seconds):
        output = ROOT/'evidence'/'reconstruction'/self.session
        process = None
        watchdog = None
        raw_saved_bytes = 0
        moving = isinstance(self.model, MovingModel)
        try:
            output.mkdir(parents=True, exist_ok=False)
            self.raw_archive = FrameArchive(output/'frames.jsonl')
            if self.stopped.is_set():
                return
            helper = (ROOT/'tools/reconstruction_core.py').read_text(encoding='utf-8')
            code = 'import sys,types\nm=types.ModuleType("reconstruction_core");sys.modules["reconstruction_core"]=m\nexec('+repr(helper)+',m.__dict__)\n'
            code += 'CAPTURE_SECONDS='+str(seconds)+'\n'+(ROOT/'tools/reconstruction_capture.py').read_text(encoding='utf-8')
            command = ['ssh', '-F', str(ROOT/'config/readonly_ssh_config'), 'cyberdog-current-nx',
                       'source /etc/mi/ros2_env.conf; python3 -u -c '+shlex.quote('import sys;exec(sys.stdin.read())')]
            with (output/'stderr.txt').open('w', encoding='utf-8') as err:
                process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=err,
                                           text=True, encoding='utf-8', cwd=str(ROOT))
                self.process = process
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
                    if record.get('event') == 'depth':
                        if abs(record.get('tf_stamp', 0)-record['stamp']) > 1e-6:
                            raise ValueError('深度帧与位姿时间不匹配，停止融合')
                        with self.lock:
                            changed = self.model.add(record)
                            self.status = ('移动采集中：只读记录，不控制机器狗；里程计漂移尚未校正' if moving
                                           else '静态融合中；1厘米体素不代表1厘米精度')
                            if changed:
                                self.raw_archive.append(record)
                                self.samples.append({'stamp': record['stamp'], 'source_age_s': record.get('source_age_s'),
                                                     'translation':record['translation'],'quaternion':record['quaternion'], **self.model.stats()})
                        if changed and (len(self.samples) <= 2 or moving and len(self.samples)%10 == 0):
                            raw = json.dumps(record).encode('utf-8')
                            if raw_saved_bytes+len(raw) <= 8*1024*1024:
                                (output/('frame_%02d.json'%len(self.samples))).write_bytes(raw)
                                raw_saved_bytes += len(raw)
                    else:
                        with self.lock:
                            self.remote = record
                            if record.get('event') == 'error':
                                self.error = record.get('message', '采集数据失效')
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
                self.status = self.error or ('采集已结束；保留模型，尚未完成绝对精度验收' if self.model.accepted_frames else '未收到可融合帧；检查相机与同时间TF')
            try:
                snapshot = self.report(include_points=False)
                snapshot['active'] = False
                (output/'summary.json').write_text(json.dumps(snapshot, ensure_ascii=False, indent=2), encoding='utf-8')
                (output/'samples.json').write_text(json.dumps(self.samples, indent=2), encoding='utf-8')
                (output/'model.ply').write_bytes(self.ply())
            except OSError as exc:
                with self.lock:
                    self.error = self.status = '模型保存失败：'+str(exc)
            finally:
                with self.lock:
                    self.active = False
                    self.process = None

    def report(self, include_points=True):
        with self.lock:
            points = self.model.points()
            result = dict(self.model.stats(), status=self.status, active=self.active,
                          session=self.session, started_unix=self.started_unix, error=self.error,
                          remote=self.remote, stable_points=len(points), control_requests=0,
                          mode='moving_local_point_cloud' if isinstance(self.model,MovingModel) else 'static_local_point_cloud',
                          moving_supported=True, calibration_status='未完成真实尺寸校准')
            result['raw_archive'] = self.raw_archive.stats(self.model.accepted_frames) if self.raw_archive else None
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
            if summary.get('stable_points',0) <= 0:
                continue  # An empty/failed capture must not hide the last usable model.
            source = json.loads((folder/'frame_01.json').read_text(encoding='utf-8'))
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

    def ply(self):
        with self.lock:
            points, frame = self.model.points(), self.model.frame
        header = 'ply\nformat ascii 1.0\ncomment frame %s\ncomment local voxel point model; scale and drift unvalidated\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nend_header\n'%(frame, len(points))
        return (header+''.join('%.7f %.7f %.7f\n'%tuple(p) for p in points)).encode('ascii')


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
        if self.path in ('/', '/index.html'):
            self.send((ROOT/'reconstruction.html').read_bytes(), 'text/html; charset=utf-8')
        elif self.path == '/state':
            self.send(json.dumps(LAB.report(), ensure_ascii=False, allow_nan=False).encode('utf-8'))
        elif self.path == '/model.ply':
            self.send(LAB.ply(), 'application/octet-stream')
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
            if self.path == '/start':
                LAB.start(data.get('seconds', 30), mode=data.get('mode','static'))
            elif self.path == '/stop':
                LAB.stop()
            else:
                raise ValueError('未知实验操作')
            self.send(b'{"ok":true}')
        except (ValueError, TypeError) as exc:
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
        server.server_close()
