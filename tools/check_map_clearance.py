"""Read-only current-map footprint diagnostic; no robot commands."""
import json,math,time
from urllib.request import urlopen
from route_preview import world_cell
with urlopen('http://127.0.0.1:8765/state',timeout=5) as response:state=json.load(response)
grid,pose=state['map'],state['pose'];cx,cy=world_cell(grid,pose['x'],pose['y']);r=grid['resolution'];origin=grid['origin']
angle=2*math.atan2(origin['qz'],origin['qw']);c,s=math.cos(angle),math.sin(angle)
rows=[];counts={};w,h=grid['width'],grid['height']
for y in range(max(0,cy-20),min(h,cy+21)):
    for x in range(max(0,cx-20),min(w,cx+21)):
        value=grid['data'][y*w+x]
        if value==0:continue
        wx,wy=origin['x']+c*(x+.5)*r-s*(y+.5)*r,origin['y']+s*(x+.5)*r+c*(y+.5)*r
        dx,dy=wx-pose['x'],wy-pose['y'];yaw=pose['yaw']
        bx,by=math.cos(yaw)*dx+math.sin(yaw)*dy,-math.sin(yaw)*dx+math.cos(yaw)*dy
        if math.hypot(bx,by)<=.6:
            rows.append({'type':'unknown' if value<0 else 'occupied','x_body':bx,'y_body':by,'distance':math.hypot(bx,by),'inside_body':-.23<=bx<=.25 and abs(by)<=.13})
            counts[value]=counts.get(value,0)+1
report=state.get('exploration') or {};executor=report.get('executor') or {}
print(json.dumps({'unix':time.time(),'pose':pose,'map_age':time.time()-grid['received_unix'],'near_counts':counts,'nearest':sorted(rows,key=lambda p:p['distance'])[:25],'planner_status':report.get('status'),'executor_active':executor.get('active'),'terminal_events':executor.get('events',[])[-2:]},ensure_ascii=False))
