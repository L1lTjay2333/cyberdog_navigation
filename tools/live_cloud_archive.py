"""PC-side aligned cloud archive, independent of browser lifetime."""
import json,math,time,threading,uuid
from pathlib import Path

class LiveCloudArchive:
 def __init__(self,root):
  self.root=Path(root);self.root.mkdir(parents=True,exist_ok=True)
  self.lock=threading.Lock();self.key=None;self.frame=None;self.voxels={};self.last=None;self.saved=0
  self.path=self.root/'latest.json';self.error=None
  if self.path.exists():
   try:
    d=json.loads(self.path.read_text(encoding='utf-8'));self.key=d['key'];self.frame=d['frame']
    self.voxels={tuple(math.floor(v/.05) for v in p):p for p in d['points_xyz']}
   except (ValueError,KeyError,OSError):self.error='Previous cloud archive could not be loaded'
 def add(self,s,name):
  c=s.get('cloud') or {};stamp=c.get('stamp');age=s.get('emitted_unix',0)-(stamp or 0)
  if not c.get('aligned') or c.get('frame') not in ('map','laser_odom') or not 0<=age<2:return
  if (s.get('task') or {}).get('status') not in (1,5,8):return
  if not name:return
  key=name+':'+c['frame'];signature=(key,stamp)
  with self.lock:
   if self.last==signature:return
   pts=[p for p in c.get('points_xyz',[]) if len(p)==3 and all(isinstance(v,(int,float)) and math.isfinite(v) for v in p)]
   if not pts:return
   # Write each aligned frame first, so a crash between snapshots loses no archived frame.
   session=self.root/('session_'+str(int(time.time()))+'_'+uuid.uuid4().hex[:8]) if self.key!=key else getattr(self,'session',None)
   if session is None:session=self.root/('session_'+str(int(time.time()))+'_'+uuid.uuid4().hex[:8])
   session.mkdir(exist_ok=True);self.session=session
   record={'key':key,'stamp':stamp,'frame':c['frame'],'points_xyz':pts,'registration':c.get('registration')}
   depth=s.get('archive_depth')
   if depth and depth['stamp']==stamp and depth['frame']==c['frame']:
    record['depth']=depth
   with (session/'frames.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(record)+'\n')
   if self.key!=key:self.voxels={};self.key=key;self.frame=c['frame']
   for p in pts:self.voxels[tuple(math.floor(v/.05) for v in p)]=p
   self.last=signature
   if time.monotonic()-self.saved>=2:
    data={'key':key,'frame':self.frame,'stamp':stamp,'saved_unix':time.time(),'points_xyz':list(self.voxels.values())}
    tmp=self.path.with_suffix('.tmp');tmp.write_text(json.dumps(data),encoding='utf-8');tmp.replace(self.path)
    # Each session keeps its own snapshot as well as the latest pointer.
    tmp=session/'map.tmp';tmp.write_text(json.dumps(data),encoding='utf-8');tmp.replace(session/'map.json')
    self.saved=time.monotonic();self.error=None
 def read(self):
  with self.lock:
   return {'key':self.key,'frame':self.frame,'points_xyz':list(self.voxels.values()),'error':self.error}
