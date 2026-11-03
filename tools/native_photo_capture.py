"""PC native JPEG capture. Separate from bandwidth-limited RGB-D stream."""
import base64,hashlib,json,re,struct,subprocess,zlib
from datetime import datetime
from pathlib import Path


def jpeg_size(raw):
    if not raw.startswith(b'\xff\xd8'):raise ValueError('Not JPEG')
    i=2
    while i<len(raw):
        if raw[i]!=255:raise ValueError('Invalid JPEG marker')
        while i<len(raw) and raw[i]==255:i+=1
        if i>=len(raw):break
        marker=raw[i];i+=1
        if marker in (0xd8,0xd9,0x01) or 0xd0<=marker<=0xd7:continue
        if i+2>len(raw):break
        length=struct.unpack('>H',raw[i:i+2])[0]
        if length<2 or i+length>len(raw):raise ValueError('Truncated JPEG')
        if marker in (0xc0,0xc1,0xc2):
            if length<8:raise ValueError('Invalid JPEG frame')
            h,w=struct.unpack('>HH',raw[i+3:i+7])
            if not 1<=w<=8192 or not 1<=h<=8192:raise ValueError('Image exceeds bounds')
            return w,h
        if marker==0xda:break
        i+=length
    raise ValueError('Missing JPEG dimensions')


def save(root,session,row):
    if row.get('kind')!='native_photo':raise ValueError('Invalid photo response')
    raw=base64.b64decode(row['data'],validate=True)
    if not 0<len(raw)<=20*1024*1024:raise ValueError('Photo exceeds byte bound')
    w,h=jpeg_size(raw)
    if w<1280 or h<960:raise ValueError('原厂原生照片分辨率低于建模档位')
    group=session if session and re.fullmatch(r'[0-9]{8}_[0-9]{6}_[0-9]{6}',session) else 'unassigned'
    folder=root/'evidence/native_photos'/group/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True,exist_ok=False);(folder/'photo.jpg').write_bytes(raw)
    report={k:v for k,v in row.items() if k!='data'}
    report.update(width=w,height=h,bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest(),
                  session=group,file=str((folder/'photo.jpg').resolve()),
                  stationary_confirmed_by_operator=True,pose_source=None)
    (folder/'photo.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    return report


def capture(root,session):
    code=(root/'tools/native_photo_remote.py').read_bytes()
    payload=base64.b64encode(zlib.compress(code))
    command=['ssh','-F',str(root/'config/readonly_ssh_config'),'-o','ConnectTimeout=12','cyberdog-current-nx',
        'source /etc/mi/ros2_env.conf; python3 -u -c "import sys,base64,zlib;exec(zlib.decompress(base64.b64decode(sys.stdin.buffer.read())))"']
    result=subprocess.run(command,input=payload,capture_output=True,timeout=45,cwd=str(root))
    if result.returncode:raise RuntimeError('原生拍照连接失败：'+result.stderr.decode(errors='replace')[-500:])
    rows=[json.loads(line) for line in result.stdout.splitlines() if line.startswith(b'{')]
    photos=[r for r in rows if r.get('kind')=='native_photo']
    if len(photos)!=1:raise ValueError('未收到唯一原生照片，不自动重试')
    return save(root,session,photos[0])
