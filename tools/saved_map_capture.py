"""Save stock mapping output on the PC for reproducible depth reconstruction."""
import hashlib
import json
import re
import subprocess
from pathlib import Path

def capture_saved_map(root, name):
    if not re.fullmatch(r'computer_room_[0-9]{8}_[0-9]{6}',name):
        raise ValueError('Invalid saved map name')
    folder=Path(root)/'evidence/live_cloud_maps/saved_maps'/name
    folder.mkdir(parents=True,exist_ok=True)
    manifest={}
    for filename in ('pose.txt','graph.pbstream',name+'.pgm',name+'.yaml',name+'.json'):
        command=['ssh','-F',str(Path(root)/'config/readonly_ssh_config'),
                 '-o','BatchMode=yes','-o','ConnectTimeout=5','cyberdog-current-nx',
                 'cat /home/mi/mapping/'+filename]
        reply=subprocess.run(command,capture_output=True,timeout=30)
        if reply.returncode or not reply.stdout:
            raise RuntimeError('Map saved, but PC archive incomplete: '+filename)
        target=folder/filename
        temporary=target.with_suffix(target.suffix+'.tmp')
        temporary.write_bytes(reply.stdout)
        temporary.replace(target)
        manifest[filename]=dict(bytes=len(reply.stdout),sha256=hashlib.sha256(reply.stdout).hexdigest())
    temporary=folder/'manifest.tmp'
    temporary.write_text(json.dumps(dict(map_name=name,files=manifest),indent=2),encoding='utf8')
    temporary.replace(folder/'manifest.json')
    return str(folder)
