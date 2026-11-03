"""PC-side sensor probe via existing pinned SSH identity. No motion commands."""
import argparse
from datetime import datetime
import json
import socket
import time
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]


def run(prepare=False):
    sys.path.insert(0,str(ROOT/'tools/_ssh_vendor'))
    import paramiko
    cfg=paramiko.SSHConfig.from_path(str(ROOT/'config/readonly_ssh_config')).lookup('cyberdog-current-nx')
    client=paramiko.SSHClient();client.set_missing_host_key_policy(paramiko.RejectPolicy())
    paths=cfg.get('userknownhostsfile',[str(Path.home()/'.ssh/known_hosts')])
    for name in ([paths] if isinstance(paths,str) else paths):
        path=Path(name).expanduser()
        if path.is_file():client.load_host_keys(str(path))
    try:
        client.connect(cfg['hostname'],username=cfg['user'],
            key_filename=[str(Path(p).expanduser()) for p in cfg['identityfile']],
            look_for_keys=False,allow_agent=False,timeout=8,banner_timeout=8,auth_timeout=12)
        _,out,_=client.exec_command('hostname',timeout=10)
        if out.read().decode().strip()!='mi-desktop':raise ValueError('Unexpected robot hostname')
        if prepare:
            helper=(ROOT/'tools/reconstruction_core.py').read_text(encoding='utf-8')
            code='import sys,types\nm=types.ModuleType("reconstruction_core");sys.modules["reconstruction_core"]=m\nexec('+repr(helper)+',m.__dict__)\nCONNECT_CAMERA=True\n'
            code+=(ROOT/'tools/reconstruction_input_probe.py').read_text(encoding='utf-8')
        else:code=(ROOT/'tools/probe_rgbd_sources.py').read_text(encoding='utf-8')
        channel=client.get_transport().open_session(timeout=10)
        channel.settimeout(50 if prepare else 25)
        # Keep outgoing writes small on the currently unreliable connection.
        # The probe stays transient; no file is installed on the robot.
        client.get_transport().sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        channel.exec_command('source /etc/mi/ros2_env.conf; python3 -u -c "import sys;exec(sys.stdin.read())"')
        payload=code.encode('utf-8')
        for start in range(0,len(payload),512):
            channel.sendall(payload[start:start+512]);time.sleep(.02)
        channel.shutdown_write()
        out=channel.makefile('rb').read().decode('utf-8')
        err=channel.makefile_stderr('rb').read().decode('utf-8')
        code=channel.recv_exit_status()
        if code!=0:raise RuntimeError('Remote probe did not complete (exit %d): %s'%(code,err[-1200:] or 'connection closed without a report; camera state is unknown'))
        report=json.loads(out.strip().splitlines()[-1])
        folder=ROOT/'evidence/rgbd_checks';folder.mkdir(parents=True,exist_ok=True)
        filename=datetime.now().strftime('%Y%m%d_%H%M%S_%f')+('_prepare' if prepare else '_inventory')+'.json'
        (folder/filename).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(report,ensure_ascii=False,indent=2))
    except Exception as error:
        folder=ROOT/'evidence/rgbd_checks';folder.mkdir(parents=True,exist_ok=True)
        report={'ready':False,'operation':'prepare' if prepare else 'inventory',
                'error':type(error).__name__+': '+str(error),'camera_state_confirmed':False,
                'motion_requests':0,'note':'A failed preparation may have changed camera state before disconnection; do not automatically retry.'}
        filename=datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_failed.json'
        (folder/filename).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        raise
    finally:client.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare-camera',action='store_true',help='Explicit stock camera prepare; no mapping or motion')
    args=parser.parse_args();run(args.prepare_camera)
