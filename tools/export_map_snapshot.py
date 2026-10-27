"""Save an observed occupancy grid locally without stopping stock mapping."""
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def export_snapshot(snapshot, destination):
    grid = snapshot['map']
    w,h = grid['width'],grid['height']
    if not w or not h or len(grid['data']) != w*h or grid['resolution'] <= 0:
        raise ValueError('Invalid occupancy grid')
    destination.mkdir(parents=True, exist_ok=False)
    (destination/'snapshot.json').write_text(json.dumps(snapshot,indent=2),encoding='utf-8')
    pixels = bytearray()
    for y in range(h-1,-1,-1):
        for value in grid['data'][y*w:(y+1)*w]:
            pixels.append(205 if value < 0 else 0 if value >= 65 else 254 if value <= 25 else 205)
    (destination/'map.pgm').write_bytes(('P5\n{} {}\n255\n'.format(w,h)).encode('ascii')+pixels)
    origin = grid['origin']
    yaw = 2*math.atan2(origin['qz'],origin['qw'])
    yaml = ('image: map.pgm\nresolution: {resolution}\norigin: [{x}, {y}, {yaw}]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n').format(
                resolution=grid['resolution'],x=origin['x'],y=origin['y'],yaw=yaw)
    (destination/'map.yaml').write_text(yaml,encoding='utf-8')
    (destination/'README.md').write_text(
        '# 房间地图采集快照\n\n2026-10-07：操作者确认已绕房间一圈。'
        '这是电脑保存的二维地图及原厂位置快照；原厂建图任务仍运行，以继续显示实时位置。'
        '未调用停止/保存服务，因此不是原厂完整SLAM图数据的持久化完成证明。'
        'snapshot.json保留地图原坐标系与完整栅格；map.yaml/map.pgm可供后续地图工具读取。'
        '尚未验证地图加载、重定位或自主导航。\n',encoding='utf-8')

if __name__ == '__main__':
    snapshot = json.loads((ROOT/'evidence/latest_map_snapshot.json').read_text(encoding='utf-8'))
    destination = ROOT/'maps/room_20261007_first_capture'
    export_snapshot(snapshot,destination)
    print(str(destination))
