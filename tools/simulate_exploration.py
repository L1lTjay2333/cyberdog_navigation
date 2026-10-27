"""Idealized planar simulation ONLY: no ROS, SSH, sockets or physical evidence.

Sensor reveals a circular area without occlusion and always has valid depth.
This checks planning/tracking integration, not real sensor or motor behavior.
"""
import json,math
from pathlib import Path
from unknown_goal import plan
from exploration_policy import track,safe_point

def simulate(detour=False,closed=False):
    w,h,r=60,50,.1;truth=[0]*(w*h)
    if detour or closed:
        for y in (range(h) if closed else range(20,30)):truth[y*w+25]=100
    grid={'width':w,'height':h,'resolution':r,'frame':'laser_odom','origin':{'x':0.,'y':0.,'qz':0.,'qw':1.},'data':[-1]*(w*h)}
    if closed:grid['data']=list(truth) # Known fully closed wall: no viable frontier.
    pose={'x':1.05,'y':2.55,'yaw':0.,'frame':'laser_odom'}
    goal={'x':4.05,'y':2.55,'frame':'laser_odom'}
    route=None;status=None;trace=[];visited=[];waypoint=None;length=0.;phases=[]
    for step in range(1800):
        t=step*.1
        if t>=120 or length>5:return {'result':'STOPPED_BUDGET','path_length_m':length,'phases':phases,'trace':trace}
        for y in range(h):
            for x in range(w):
                if math.hypot((x+.5)*r-pose['x'],(y+.5)*r-pose['y'])<=1.8:
                    grid['data'][y*w+x]=truth[y*w+x]
        if math.hypot(goal['x']-pose['x'],goal['y']-pose['y'])<=.15:
            return {'result':'ARRIVED','elapsed_sim_s':t,'straight_distance_m':3.,'path_length_m':length,'phases':phases,'trace':trace}
        if step%5==0:
            if status=='FRONTIER_ROUTE' and waypoint and math.hypot(waypoint['x']-pose['x'],waypoint['y']-pose['y'])<.15:visited.append(waypoint)
            report=plan(grid,pose,goal,visited=visited);route=report.get('preview');waypoint=report.get('waypoint');status=report['status']
            if status not in phases:phases.append(status)
            if not route:return {'result':status,'elapsed_sim_s':t,'straight_distance_m':3.,'path_length_m':length,'phases':phases,'trace':trace}
        health={'sensor_mode':'supervised_four_way_trial','auxiliary_task':5,'task':5,'task_time':t,'status_time':t,
            'status':{'motion_id':112,'switch_status':0,'ori_error':0,'footpos_error':0,'motor_error':[0]*12},
            'battery_time':t,'battery':{'percent':80},'scan_time':t,'depth_time':t,'depth_valid':True,'depth_min':5.}
        velocity,reason=track(grid,pose,route,health,t)
        vx,_,yaw=velocity;theta=pose['yaw'];dt=.1
        if abs(yaw)<1e-9:dx,dy=vx*dt*math.cos(theta),vx*dt*math.sin(theta)
        else:dx,dy=vx/yaw*(math.sin(theta+yaw*dt)-math.sin(theta)),-vx/yaw*(math.cos(theta+yaw*dt)-math.cos(theta))
        pose['x']+=dx;pose['y']+=dy;pose['yaw']=math.atan2(math.sin(theta+yaw*dt),math.cos(theta+yaw*dt));length+=math.hypot(dx,dy)
        if not safe_point({**grid,'data':truth},pose['x'],pose['y']):raise AssertionError('Simulated footprint collision')
        if step%10==0:trace.append({'t':t,'x':pose['x'],'y':pose['y'],'yaw':pose['yaw'],'status':status,'reason':reason})
    return {'result':'SIM_TIMEOUT','path_length_m':length,'phases':phases,'trace':trace}

if __name__=='__main__':
    results={'kind':'IDEALIZED_SIMULATION_NOT_ROBOT_TEST','sensor_model':'Circular reveal; no occlusion; always valid depth; no motor dynamics',
             'straight':simulate(),'wall_detour':simulate(detour=True),'closed':simulate(closed=True)}
    dest=Path(__file__).resolve().parents[1]/'evidence/exploration_idealized_simulation.json'
    dest.write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({name:{k:v for k,v in value.items() if k!='trace'} for name,value in results.items() if isinstance(value,dict)},ensure_ascii=False))
