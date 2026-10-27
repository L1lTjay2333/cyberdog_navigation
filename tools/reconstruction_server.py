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
from reconstruction_core import StaticModel

ROOT = Path(__file__).resolve().parents[1]


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

    def start(self, seconds=30):
        if type(seconds) is not int or not 1 <= seconds <= 60:
            raise ValueError('采集时长须为1—60秒')
        with self.lock:
            if self.active:
                raise ValueError('已有静态采集会话')
            self.model = StaticModel()
            self.active, self.error, self.remote = True, None, {}
            self.stopped.clear()
            self.session = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            self.started_unix = time.time()
            self.status = '连接只读采集；请保持机器狗和场景静止'
            self.samples = []
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
        try:
            output.mkdir(parents=True, exist_ok=False)
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
                            self.status = '静态融合中；1厘米体素不代表1厘米精度'
                            if changed:
                                self.samples.append({'stamp': record['stamp'], 'source_age_s': record.get('source_age_s'), **self.model.stats()})
                        if changed and len(self.samples) <= 2:
                            (output/('frame_%02d.json'%len(self.samples))).write_text(json.dumps(record), encoding='utf-8')
                    else:
                        with self.lock:
                            self.remote = record
                            if record.get('event') == 'error':
                                self.error = record.get('message', '采集数据失效')
                code = process.wait(timeout=5)
                if code and not self.stopped.is_set():
                    raise RuntimeError('相机订阅退出，请查看本次stderr；未操作机器人')
        except Exception as exc:
            with self.lock:
                if not self.stopped.is_set():
                    self.error = str(exc)
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
                          mode='static_local_point_cloud', calibration_status='未完成真实尺寸校准')
            if include_points:
                result['points'] = points[::max(1, (len(points)+29999)//30000)]
            return result

    def ply(self):
        with self.lock:
            points, frame = self.model.points(), self.model.frame
        header = 'ply\nformat ascii 1.0\ncomment frame %s\ncomment static voxel point model; scale unvalidated\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nend_header\n'%(frame, len(points))
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
                LAB.start(data.get('seconds', 30))
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
    args = parser.parse_args()
    server = ThreadingHTTPServer(('127.0.0.1', 8766), Handler)
    if args.capture_seconds is not None:
        LAB.start(args.capture_seconds)
    print('Static reconstruction laboratory: http://127.0.0.1:8766/', flush=True)
    try:
        server.serve_forever()
    finally:
        LAB.stop()
        server.server_close()
