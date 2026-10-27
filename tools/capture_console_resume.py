"""Capture current owned mapping before a deliberate idle server restart."""
import json,time
from pathlib import Path
from urllib.request import urlopen
ROOT=Path(__file__).resolve().parents[1]
with urlopen('http://127.0.0.1:8765/state',timeout=5) as response:s=json.load(response)
with urlopen('http://127.0.0.1:8765/manual_state',timeout=5) as response:m=json.load(response)
if s['navigation']['active'] or s['mapping']['active'] or s['exploration']['executor']['active'] or m.get('connected'):raise RuntimeError('已有控制或任务切换会话，不能重启')
if s['task']['status']!=5 or s['connection']!='connected' or not s['mapping']['map_name']:raise RuntimeError('没有本页运行的建图会话')
path=ROOT/'evidence/console_mapping_resume.json'
path.write_text(json.dumps({'captured_unix':time.time(),'mapping':s['mapping']},ensure_ascii=False),encoding='utf-8')
print(str(path))
