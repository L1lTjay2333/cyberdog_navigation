import json
import subprocess
import time

def stock_preview(root,goal):
    goal=dict(goal,map_name=json.loads((root/'config/scope.json').read_text(encoding='utf-8')).get('computer_map_name',''))
    code='REQUEST='+repr(goal)+'\n'+(root/'tools/stock_plan_once.py').read_text(encoding='utf-8')
    try:
        result=subprocess.run(['ssh','-F',str(root/'config/readonly_ssh_config'),'cyberdog-current-nx',
            'source /etc/mi/ros2_env.conf; python3 -u -c "import sys;exec(sys.stdin.read())"'],input=code,text=True,encoding='utf-8',capture_output=True,timeout=65)
    except subprocess.TimeoutExpired as exc:
        raise ValueError('原厂路径规划响应超时，请检查定位状态') from exc
    except OSError as exc:
        raise ValueError('路径预览连接无法建立') from exc
    rows=[]
    for line in result.stdout.splitlines():
        try:rows.append(json.loads(line))
        except ValueError:pass
    if not rows:raise ValueError('路径预览连接失败')
    row=rows[-1]
    with (root/'evidence/preview_events.jsonl').open('a',encoding='utf-8') as log:
        log.write(json.dumps({'unix':time.time(),'goal':goal,'ok':row.get('ok'),
            'message':row.get('message'),'events':[r for r in rows if r.get('event')],
            'length_m':row.get('length_m')},ensure_ascii=False)+'\n')
    if not row.get('ok'):raise ValueError(row.get('message','路径预览失败'))
    return row
