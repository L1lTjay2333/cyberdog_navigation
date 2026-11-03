"""Local offline workbench, port 8768. Has NO robot connection or control code."""
import json
import mimetypes
from datetime import datetime
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import re
import subprocess
import sys
import threading
import time
from urllib.parse import urlsplit

ROOT=Path(__file__).resolve().parents[1]
IDENTIFIER=re.compile(r'[0-9]{8}_[0-9]{6}_[0-9]{6}')
FILES={'result.json':'result.json','color_alignment.json':'color_alignment.json','reprojection.json':'reprojection.json','aligned.bin':'aligned.bin','mls.bin':'mls.bin','aligned.ply':'fusion/aligned_gray.ply','mls.ply':'mls.ply','alignment.json':'alignment.json','observed.bin':'observed.bin','supported.bin':'supported.bin','points.bin':'points.bin',
       'observed.ply':'fusion/observed_gray.ply','supported.ply':'fusion/supported_gray.ply',
       'points.ply':'processed/points_processed.ply','excluded.ply':'processed/points_excluded.ply',
       'processing.json':'processed/processing_report.json','fusion.json':'fusion/fusion_report.json'}


def read_json(path):
    if path.is_symlink() or path.stat().st_size>2*1024*1024:raise ValueError('Invalid local metadata')
    return json.loads(path.read_text(encoding='utf-8'))


class Workbench:
    def __init__(self,root):
        self.root=Path(root);self.lock=threading.Lock();self.process=None;self.log=None
        self.folder=None;self.run_id=None;self.cancelled=False;self.started=0.
        parent=self.root/'evidence/offline_runs'
        for folder in sorted(parent.glob('*'),reverse=True):
            if IDENTIFIER.fullmatch(folder.name) and self.complete(folder):
                self.folder,self.run_id=folder,folder.name;break

    def complete(self,folder):
        try:
            return (not (folder/'cancelled.json').exists() and
                    read_json(folder/'exit.json').get('code')==0 and
                    read_json(folder/'progress.json').get('phase')=='done')
        except (OSError,ValueError):return False

    def sessions(self):
        parent=(self.root/'evidence/reconstruction').resolve();rows=[]
        for folder in sorted(parent.glob('*'),reverse=True):
            try:
                if not IDENTIFIER.fullmatch(folder.name) or folder.is_symlink() or folder.resolve().parent!=parent:continue
                archive=folder/'frames.jsonl'
                if archive.is_symlink() or not 0<archive.stat().st_size<=256*1024*1024:continue
                summary=read_json(folder/'summary.json');frames=summary.get('received_frames',summary.get('accepted_frames'))
                info=summary.get('raw_archive') or {}
                if type(frames) is not int or not 2<=frames<=900 or info.get('frames')!=frames or info.get('complete') is not True:continue
                rows.append({'id':folder.name,'frames':frames,'points':summary.get('stable_points',0),
                             'capture_stopped_early':bool(summary.get('error'))})
            except (OSError,ValueError):continue
        return rows

    def start(self,session_id,mode='standard',first_frame=1):
        if mode not in ('standard','local','color') or type(first_frame) is not int or not 1<=first_frame<=891:raise ValueError('无效处理模式或起始帧')
        if not isinstance(session_id,str) or not IDENTIFIER.fullmatch(session_id):raise ValueError('请选择有效采集记录')
        with self.lock:
            if self.process is not None and self.process.poll() is None:raise ValueError('已有离线任务运行，请等待或取消')
            if session_id not in {row['id'] for row in self.sessions()}:raise ValueError('该记录没有完整可用的原始归档')
            self.run_id=datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            self.folder=self.root/'evidence/offline_runs'/self.run_id
            self.folder.mkdir(parents=True,exist_ok=False)
            self.cancelled=False;self.started=time.monotonic()
            (self.folder/'job.json').write_text(json.dumps({'session':session_id,'run':self.run_id,'mode':mode,'first_frame':first_frame}),encoding='utf-8')
            (self.folder/'progress.json').write_text(json.dumps({'phase':'queued','message':'正在启动电脑离线处理'}),encoding='utf-8')
            self.log=(self.folder/'worker.log').open('w',encoding='utf-8')
            try:
                self.process=subprocess.Popen([sys.executable,'-u',str(self.root/'tools/reconstruction_offline_worker.py'),
                    str(self.root/'evidence/reconstruction'/session_id),str(self.folder),'--mode',mode,'--first-frame',str(first_frame)],
                    stdout=self.log,stderr=subprocess.STDOUT,cwd=str(self.root))
                self.log.close();self.log=None
            except OSError:
                self.log.close();self.log=None;raise
            return {'run':self.run_id}

    def cancel(self):
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                self.cancelled=True;self.process.terminate()
                (self.folder/'cancelled.json').write_text('{"cancelled":true}',encoding='utf-8')

    def snapshot(self):
        with self.lock:
            if self.folder is None:return {'status':'idle','message':'选择电脑已保存记录，开始离线处理'}
            try:state=read_json(self.folder/'progress.json')
            except (OSError,ValueError):state={'phase':'queued','message':'正在启动电脑处理'}
            code=self.process.poll() if self.process is not None else 0
            if self.process is not None and code is None and time.monotonic()-self.started>600:
                self.process.terminate();self.cancelled=True
                (self.folder/'cancelled.json').write_text('{"cancelled":true,"timeout":true}')
            if self.cancelled:
                status='cancelled';state={'message':'离线处理已取消；原始数据保留'}
            elif code is None:status='running'
            elif self.process is not None:
                if self.log:self.log.close();self.log=None
                (self.folder/'exit.json').write_text(json.dumps({'code':int(code)}),encoding='utf-8')
                status='done' if code==0 and state.get('phase')=='done' else 'failed'
            else:status='done' if self.complete(self.folder) else 'failed'
            result={'status':status,'phase':state.get('phase'),'message':state.get('message','离线任务已结束'),'run':self.run_id}
            if status=='done':result['results']=state.get('results',{})
            elif status=='failed' and state.get('phase')!='failed':result['message']='处理进程未成功完成，请核对该任务worker.log'
            return result

    def result_file(self,run,name):
        if not IDENTIFIER.fullmatch(run) or name not in FILES:raise ValueError('Unknown result')
        folder=self.root/'evidence/offline_runs'/run
        if folder.is_symlink() or not self.complete(folder):raise ValueError('Result is incomplete or cancelled')
        path=folder/FILES[name]
        if path.is_symlink() or not path.is_file() or not path.resolve().is_relative_to(folder.resolve()):raise ValueError('Invalid result file')
        return path


BENCH=Workbench(ROOT)


class Handler(BaseHTTPRequestHandler):
    def send_json(self,value,status=200):
        self.send_bytes(json.dumps(value,ensure_ascii=False,allow_nan=False).encode('utf-8'),'application/json; charset=utf-8',status)

    def send_bytes(self,data,kind,status=200):
        self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store')
        self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)

    def send_file(self,path):
        self.send_response(200);self.send_header('Content-Type',mimetypes.guess_type(path.name)[0] or 'application/octet-stream')
        self.send_header('Content-Length',str(path.stat().st_size));self.send_header('Cache-Control','no-store');self.end_headers()
        try:
            with path.open('rb') as stream:
                while True:
                    chunk=stream.read(256*1024)
                    if not chunk:break
                    self.wfile.write(chunk)
        except (BrokenPipeError,ConnectionResetError):pass

    def do_GET(self):
        path=urlsplit(self.path).path
        try:
            if path=='/':self.send_file(ROOT/'offline_reconstruction.html')
            elif path=='/api/sessions':self.send_json({'sessions':BENCH.sessions()})
            elif path=='/api/state':self.send_json(BENCH.snapshot())
            elif path.startswith('/result/'):
                parts=path.split('/')
                if len(parts)!=4:raise ValueError('Invalid result path')
                self.send_file(BENCH.result_file(parts[2],parts[3]))
            elif path in ('/vendor/three.module.js','/vendor/three.core.js','/vendor/OrbitControls.js'):
                runtime=ROOT.parents[1]/'tools/reconstruction_runtime/web/node_modules/three'
                files={'/vendor/three.module.js':runtime/'build/three.module.js','/vendor/three.core.js':runtime/'build/three.core.js',
                       '/vendor/OrbitControls.js':runtime/'examples/jsm/controls/OrbitControls.js'}
                self.send_file(files[path])
            else:self.send_json({'error':'not found'},404)
        except (ValueError,OSError):self.send_json({'error':'文件不存在或结果尚未完成'},404)

    def do_POST(self):
        try:
            if self.headers.get('Origin') not in ('http://127.0.0.1:8768','http://localhost:8768'):raise ValueError('请求来源不受信任')
            length=int(self.headers.get('Content-Length','0'))
            if self.headers.get('Content-Type')!='application/json' or not 0<length<=256:raise ValueError('无效请求')
            data=json.loads(self.rfile.read(length))
            if not isinstance(data,dict):raise ValueError('无效请求')
            if self.path=='/api/start':result=BENCH.start(data.get('session'),data.get('mode','standard'),data.get('first_frame',1))
            elif self.path=='/api/cancel':BENCH.cancel();result={}
            else:raise ValueError('未知离线操作')
            self.send_json(dict(ok=True,**result))
        except (ValueError,TypeError,OSError) as error:self.send_json({'ok':False,'error':str(error)},400)

    def log_message(self,*args):pass


if __name__=='__main__':
    server=ThreadingHTTPServer(('127.0.0.1',8768),Handler)
    print('Offline reconstruction: http://127.0.0.1:8768/',flush=True)
    try:server.serve_forever()
    finally:BENCH.cancel();server.server_close()
