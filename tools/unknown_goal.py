"""Incremental unknown-goal planning, without ROS or motion commands.

Unknown cells are never traversable. Plan to a reachable observation frontier,
then replan when an updated map reveals the next section. This module does not
claim the stock, mutually exclusive mapping/navigation tasks can execute it.
"""
from collections import deque
import math
import threading
import time
from route_preview import world_cell


def number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('目标坐标必须为有限数字')
    return float(value)


def validate_grid(grid):
    w, h = grid['width'], grid['height']
    r = number(grid['resolution'])
    if type(w) is not int or type(h) is not int or min(w, h, r) <= 0:
        raise ValueError('地图尺寸无效')
    if w*h > 500000 or len(grid['data']) != w*h:
        raise ValueError('地图数据无效或超过规划容量')
    origin = grid['origin']
    for key in ('x', 'y', 'qz', 'qw'):
        number(origin[key])
    if origin['qz']**2+origin['qw']**2 < 1e-12:
        raise ValueError('地图原点姿态无效')
    if not grid.get('frame'):
        raise ValueError('地图坐标系未就绪')
    return w, h, r


def cell_point(grid, cell):
    a = 2*math.atan2(grid['origin']['qz'], grid['origin']['qw'])
    x, y = ((v+.5)*grid['resolution'] for v in cell)
    return {'x': grid['origin']['x']+math.cos(a)*x-math.sin(a)*y,
            'y': grid['origin']['y']+math.sin(a)*x+math.cos(a)*y}


def integral_mask(grid, predicate):
    w, h = grid['width'], grid['height']
    sums = [0]*((w+1)*(h+1))
    for y in range(h):
        row = 0
        for x in range(w):
            row += bool(predicate(grid['data'][y*w+x]))
            sums[(y+1)*(w+1)+x+1] = sums[y*(w+1)+x+1]+row
    def count(cx, cy, n):
        x0, y0 = max(0, cx-n), max(0, cy-n)
        x1, y1 = min(w, cx+n+1), min(h, cy+n+1)
        return (sums[y1*(w+1)+x1]-sums[y0*(w+1)+x1]
                -sums[y1*(w+1)+x0]+sums[y0*(w+1)+x0])
    return count


def plan(grid, pose, target, radius=.34, visited=()):
    """Reachable known-free BFS, conservatively inflated by a square footprint.

    Candidates are at the unknown boundary outside the footprint. Frontier
    score favors proximity to the final goal but permits retreat/detours.
    No direct-line substitution and no guaranteed reachability of unknown goals.
    """
    w, h, r = validate_grid(grid)
    if pose.get('frame') != grid['frame'] or target.get('frame') != grid['frame']:
        raise ValueError('目标、定位与地图坐标系不一致，请重新设置目标')
    for p in (pose, target):
        number(p['x']); number(p['y'])
    n = math.ceil(radius/r)
    blocked = integral_mask(grid, lambda v: v != 0)
    unknown = integral_mask(grid, lambda v: v < 0)
    free = [False]*(w*h)
    for y in range(n, h-n):
        for x in range(n, w-n):
            free[y*w+x] = blocked(x, y, n) == 0
    start, goal = world_cell(grid, pose['x'], pose['y']), world_cell(grid, target['x'], target['y'])
    def inside(c): return 0 <= c[0] < w and 0 <= c[1] < h
    if not inside(start) or not free[start[1]*w+start[0]]:
        return {'status': 'START_BLOCKED', 'message': '当前位置周围未确认足够通行空间', 'preview': None}
    if inside(goal) and grid['data'][goal[1]*w+goal[0]] > 0:
        return {'status': 'GOAL_BLOCKED', 'message': '新扫描发现目标位于障碍物中，请修改目标', 'preview': None}
    parent = {start: None}; distance = {start: 0}; queue = deque([start])
    best = None; best_score = float('inf'); complete = False
    while queue:
        cell = queue.popleft()
        if cell == goal and free[cell[1]*w+cell[0]]:
            best = cell; complete = True; break
        # Stop slightly before unknown terrain; reserve the full footprint.
        if distance[cell]*r >= .20 and unknown(cell[0], cell[1], n+1):
            p = cell_point(grid, cell)
            if not any(math.hypot(p['x']-v['x'], p['y']-v['y']) < .30 for v in visited):
                score = math.hypot(p['x']-target['x'], p['y']-target['y'])+.15*distance[cell]*r
                if score < best_score:
                    best, best_score = cell, score
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = (cell[0]+dx, cell[1]+dy)
            if inside(nxt) and nxt not in parent and free[nxt[1]*w+nxt[0]]:
                parent[nxt] = cell; distance[nxt] = distance[cell]+1; queue.append(nxt)
    if best is None:
        return {'status': 'NO_ROUTE', 'message': '没有可通行路线或可到达的观察边界；不生成运动指令', 'preview': None}
    cells = []; cell = best
    while cell is not None:
        cells.append(cell); cell = parent[cell]
    points = [cell_point(grid, c) for c in reversed(cells)]
    return {'status': 'GOAL_ROUTE' if complete else 'FRONTIER_ROUTE',
            'message': '目标已扫描可达，已生成完整绕障路线' if complete else '目标尚未知，先规划至已知区域的观察边界',
            'waypoint': {**points[-1], 'frame': grid['frame']},
            'preview': {'frame': grid['frame'], 'points': points,
                        'length_m': distance[best]*r, 'clearance_m': radius,
                        'kind': 'unknown_goal_preview'}, 'reachable_cells': len(parent)}


class UnknownGoal:
    """Read-only preview session. Execution is intentionally unsupported.

    Static recorded inputs are accepted only in explicitly offline mode.
    Anchor start and goal in one mapping frame/epoch, never silently rebase.
    """
    def __init__(self):
        self.lock = threading.Lock()
        self.target = None
        self.start = None
        self.epoch = None
        self.visited = []
        self.last_key = None
        self.last_report = None
        self.last_planned = 0

    def clear(self):
        with self.lock:
            self.target = None; self.last_key = None; self.last_report = None

    @staticmethod
    def check_snapshot(snapshot, offline=False):
        grid, pose = snapshot.get('map'), snapshot.get('pose')
        if not grid or not pose:
            raise ValueError('请先启动扫描，等待地图和同坐标系位置')
        validate_grid(grid)
        if pose.get('frame') != grid['frame']:
            raise ValueError('定位与扫描地图坐标系不一致')
        number(pose['x']); number(pose['y'])
        if not offline:
            now = time.time()
            if snapshot.get('connection') != 'connected':
                raise ValueError('实时连接中断，规划已暂停')
            for item, key, limit in ((snapshot, 'local_received_unix', 3),
                                     (grid, 'received_unix', 3), (pose, 'received_unix', 2),
                                     (snapshot.get('scan') or {}, 'received_unix', 2)):
                age = now-number(item.get(key, 0))
                if not -.5 <= age <= limit:
                    raise ValueError('地图、定位或扫描数据过期，规划已暂停')

    @staticmethod
    def map_epoch(snapshot):
        return snapshot['map']['frame'], (snapshot.get('mapping') or {}).get('map_name')

    def set_target(self, snapshot, request, offline=False):
        self.check_snapshot(snapshot, offline)
        x, y = number(request.get('x')), number(request.get('y'))
        pose = snapshot['pose']; grid = snapshot['map']
        if request.get('relative', False):
            # Offsets are in fixed map axes, not changing body/joystick axes.
            x += pose['x']; y += pose['y']
        if math.hypot(x-pose['x'], y-pose['y']) > 100:
            raise ValueError('本模式目标距离上限为 100 米')
        with self.lock:
            self.target = {'x': x, 'y': y, 'frame': grid['frame']}
            self.start = {'x': pose['x'], 'y': pose['y'], 'frame': grid['frame']}
            self.epoch = self.map_epoch(snapshot)
            self.visited = []; self.last_key = None; self.last_report = None; self.last_planned = 0
        return self.report(snapshot, offline)

    def report(self, snapshot, offline=False):
        with self.lock:
            base = {'execution_supported': False, 'active': False,
                    'execution_reason': '现有原厂建图与到点导航互斥；本模式仅规划，不自动行走'}
            if not self.target:
                return {**base, 'status': 'IDLE', 'message': '可在未知区域选点，或输入地图坐标/相对偏移'}
            base.update(target=dict(self.target), start=dict(self.start),
                        start_goal_distance_m=math.hypot(self.target['x']-self.start['x'], self.target['y']-self.start['y']),
                        offline=offline)
            try:
                self.check_snapshot(snapshot, offline)
                if self.map_epoch(snapshot) != self.epoch:
                    raise ValueError('扫描场景或坐标系已切换，请重新设置未知目标')
                grid, pose = snapshot['map'], snapshot['pose']
                key = (grid.get('received_unix'), pose.get('received_unix'), pose['x'], pose['y'])
                # Cache exact snapshots; cap fresh-map computations at 2 Hz.
                if self.last_report and (key == self.last_key or time.monotonic()-self.last_planned < .5):
                    return {**base, **self.last_report}
                if math.hypot(self.target['x']-pose['x'], self.target['y']-pose['y']) <= .15:
                    result = {'status': 'AT_TARGET', 'message': '当前位置已在目标范围内（位置反馈，不代表自动导航验收）', 'preview': None}
                else:
                    old = (self.last_report or {}).get('waypoint')
                    if old and self.last_report['status'] == 'FRONTIER_ROUTE' and math.hypot(old['x']-pose['x'], old['y']-pose['y']) < .15:
                        if len(self.visited) >= 200:
                            raise ValueError('观察边界尝试次数已达上限，请重新评估目标')
                        self.visited.append(old)
                    result = plan(grid, pose, self.target, visited=self.visited)
                self.last_key = key; self.last_report = result; self.last_planned = time.monotonic()
                return {**base, **result}
            except (ValueError, KeyError, TypeError) as exc:
                return {**base, 'status': 'PAUSED', 'message': str(exc), 'preview': None}
