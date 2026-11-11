import sys,tempfile,subprocess
from pathlib import Path
from unittest.mock import patch,MagicMock
sys.path.insert(0,'tools')
from ssh_connection_check import check_ssh
with tempfile.TemporaryDirectory(dir="evidence") as t:
 root=Path(t);(root/'config').mkdir();(root/'config/readonly_ssh_config').write_text('HostName 10.239.88.115')
 with patch('ssh_connection_check.socket.create_connection',side_effect=TimeoutError),patch('ssh_connection_check.subprocess.run') as run:
  assert check_ssh(root)['stage']=='tcp';run.assert_not_called()
 for code,out,err,stage in [(0,'mi-desktop\n','','ready'),(255,'','Permission denied','authentication'),(255,'','Host key verification failed','host_key')]:
  with patch('ssh_connection_check.socket.create_connection',return_value=MagicMock()),patch('ssh_connection_check.subprocess.run',return_value=subprocess.CompletedProcess([],code,out,err)):
   assert check_ssh(root)['stage']==stage
 with patch('ssh_connection_check.socket.create_connection',return_value=MagicMock()),patch('ssh_connection_check.subprocess.run',side_effect=subprocess.TimeoutExpired('ssh',20,stderr=b'Authenticated to robot')):
  assert check_ssh(root)['stage']=='remote_command'
print('PASS network, authentication, host-key, success, command-timeout classification; no network used')

