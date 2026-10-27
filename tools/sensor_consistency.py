"""Display/diagnostic cross-check; never substitutes one sensor's distance."""
from telemetry_time import sensor_age

def sensor_consistency(snapshot,now):
    laser=snapshot.get('laser_overlay') or {}
    if snapshot.get('mapping_sensor')=='depth_camera':
        fresh=sensor_age(snapshot,laser.get('received_unix'),now)<=1
        return {'status':'available' if fresh and laser.get('points') else 'unknown',
                'message':'深度相机建图输入实时更新；橙色点为相机投影扫描' if fresh and laser.get('points') else '等待深度相机投影扫描，尚不能确认建图输入'}
    depth=(snapshot.get('cloud') or {}).get('front_check') or {}
    a=laser.get('central_range_m');b=depth.get('distance_m')
    if sensor_age(snapshot,laser.get('received_unix'),now)>1 or sensor_age(snapshot,depth.get('stamp'),now)>1:
        return {'status':'unknown','message':'等待同一时段雷达和深度相机测距'}
    if a is None or b is None:return {'status':'unknown','message':'前向有效测距不足'}
    conflict=a<.6 and b>1.5 and b-a>1.
    return {'status':'conflict' if conflict else 'available','laser_m':a,'depth_m':b,
            'message':('前向雷达近回波与深度相机不一致，地图定位精度未确认' if conflict else '实时测距可用；仍需核对场地对应关系')}
