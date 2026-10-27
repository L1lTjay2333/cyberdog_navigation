"""Conservative static-map preview. Stock Nav2 owns real planning and avoidance."""
import heapq
import math

def world_cell(grid,x,y):
    if not all(math.isfinite(v) for v in (x,y)):
        raise ValueError('目标坐标无效')
    o=grid['origin']; a=2*math.atan2(o['qz'],o['qw']); dx=x-o['x'];dy=y-o['y']
    return math.floor((math.cos(a)*dx+math.sin(a)*dy)/grid['resolution']), math.floor((-math.sin(a)*dx+math.cos(a)*dy)/grid['resolution'])

def preview(grid,pose,x,y,radius=0.34):
    if pose['frame']!=grid['frame']:raise ValueError('位置与地图坐标系不一致')
    w,h=grid['width'],grid['height'];r=grid['resolution'];data=grid['data']
    if min(w,h,r)<=0 or len(data)!=w*h:raise ValueError('地图无效')
    start=world_cell(grid,pose['x'],pose['y']);goal=world_cell(grid,x,y)
    n=math.ceil(radius/r); offsets=[(dx,dy) for dx in range(-n,n+1) for dy in range(-n,n+1) if dx*dx+dy*dy <= (radius/r)**2]
    def free(cell):
        cx,cy=cell
        return all(0<=cx+dx<w and 0<=cy+dy<h and data[(cy+dy)*w+cx+dx]==0 for dx,dy in offsets)
    if not free(goal):raise ValueError('目标位于障碍、未知区域或距障碍过近，请选宽敞的白色区域')
    if not free(start):raise ValueError('当前位置周围空间不足，暂不生成路线')
    queue=[(0,start)];cost={start:0};parent={};seen=set()
    while queue:
        _,c=heapq.heappop(queue)
        if c in seen:continue
        seen.add(c)
        if c==goal:break
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            p=(c[0]+dx,c[1]+dy)
            if p not in seen and free(p) and cost[c]+1<cost.get(p,float('inf')):
                cost[p]=cost[c]+1;parent[p]=c
                heapq.heappush(queue,(cost[p]+abs(p[0]-goal[0])+abs(p[1]-goal[1]),p))
    if goal not in seen:raise ValueError('当前地图没有足够宽的可通行路线')
    cells=[goal]
    while cells[-1]!=start:cells.append(parent[cells[-1]])
    o=grid['origin'];a=2*math.atan2(o['qz'],o['qw'])
    points=[]
    for cx,cy in reversed(cells):
        gx=(cx+0.5)*r;gy=(cy+0.5)*r
        points.append({'x':o['x']+math.cos(a)*gx-math.sin(a)*gy,'y':o['y']+math.sin(a)*gx+math.cos(a)*gy})
    return {'frame':grid['frame'],'points':points,'length_m':(len(points)-1)*r,'clearance_m':radius,'kind':'static_preview'}
