"""Read or apply the reviewed idle-controller profile. Never sends a motion goal."""
import argparse
import ast
import json
from pathlib import Path
import subprocess


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--apply-reviewed-tuning',action='store_true')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    source=(root/'tools/stock_navigation_session.py').read_text(encoding='utf-8')
    tree=ast.parse(source)
    names={'call','apply_navigation_efficiency_profile'}
    snippets=[ast.get_source_segment(source,n) for n in tree.body if
              (isinstance(n,ast.FunctionDef) and n.name in names) or
              (isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='NAVIGATION_EFFICIENCY_PARAMETERS' for t in n.targets))]
    prelude='''import json
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from lifecycle_msgs.srv import GetState, ChangeState
from protocol.srv import AlgoTaskStatus
NS='/mi_desktop_48_b0_2d_7b_0b_4a'
rclpy.init(args=[])
node=rclpy.create_node('web_navigation_profile_probe',enable_rosout=False,start_parameter_services=False)
def operator_ready():pass
def emit(**row):print(json.dumps(row),flush=True)
'''
    operation="apply_navigation_efficiency_profile()" if args.apply_reviewed_tuning else '''req=GetParameters.Request();req.names=list(NAVIGATION_EFFICIENCY_PARAMETERS)
values=call('controller_server_ab/get_parameters',GetParameters,req).values
emit(event='NAVIGATION_PROFILE_READ',parameters={name:{'type':v.type,'value':v.double_value if v.type==3 else v.integer_value if v.type==2 else v.bool_value} for name,v in zip(req.names,values)})'''
    code=prelude+'\n'.join(snippets)+'\ntry:\n'+''.join('    '+line+'\n' for line in operation.splitlines())+'finally:\n    node.destroy_node();rclpy.shutdown()\n'
    result=subprocess.run(['ssh','-F',str(root/'config/readonly_ssh_config'),'-o','BatchMode=yes','cyberdog-current-nx',
        'source /etc/mi/ros2_env.conf; python3 -u -c "import sys;exec(sys.stdin.read())"'],input=code,
        text=True,encoding='utf-8',capture_output=True,timeout=60)
    rows=[]
    for line in result.stdout.splitlines():
        try:rows.append(json.loads(line))
        except ValueError:continue
    filename='navigation_profile_applied_20261010.json' if args.apply_reviewed_tuning else 'navigation_profile_read_20261010.json'
    (root/'evidence'/filename).write_text(json.dumps({'returncode':result.returncode,'events':rows},ensure_ascii=False,indent=2),encoding='utf-8')
    for row in rows:print(json.dumps(row,ensure_ascii=False))
    if result.returncode:raise SystemExit('Profile probe failed; no motion sent. '+result.stderr[-1500:])


if __name__=='__main__':main()
