"""Read-only room reconstruction viewer. Does not connect to robot."""
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import mimetypes
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'evidence/live_room_processed_20261103'
THREE=ROOT.parents[1]/'tools/reconstruction_runtime/web/node_modules/three'
class Handler(BaseHTTPRequestHandler):
 def do_GET(self):
  routes={'/':ROOT/'live_room.html','/vendor/three.module.js':THREE/'build/three.module.js',
   '/vendor/three.core.js':THREE/'build/three.core.js','/vendor/OrbitControls.js':THREE/'examples/jsm/controls/OrbitControls.js'}
  routes.update({'/'+n:OUT/n for n in ['result.json','progress.json','raw.bin','processed.bin','gray.bin','raw.ply','processed.ply','gray.ply','gray.obj']})
  routes['/replayed.bin']=ROOT/'evidence/room_registration_replay/replayed.bin'
  routes['/depthgray.bin']=ROOT/'evidence/offline_depth_room_check_20261104/supported.bin'
  routes['/depthraw.bin']=ROOT/'evidence/offline_depth_room_check_20261104/raw.bin'
  p=routes.get(self.path.split('?')[0])
  if not p or not p.is_file():self.send_error(404);return
  data=p.read_bytes();self.send_response(200);self.send_header('Content-Type',mimetypes.guess_type(p.name)[0] or 'application/octet-stream');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)));self.end_headers()
  try:self.wfile.write(data)
  except (BrokenPipeError,ConnectionResetError):pass
 def log_message(self,*a):pass
if __name__=='__main__':
 print('Room gray model: http://127.0.0.1:8772',flush=True)
 ThreadingHTTPServer(('127.0.0.1',8772),Handler).serve_forever()
