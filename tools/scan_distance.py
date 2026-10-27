"""Teammate ROI diagnostics, not an autonomous motion permission."""
import math
from obstacle_distance_probe import scan_points, summarize
from cloud_geometry import transform_xyz

def report_scan(msg, transform, now_ns):
    stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
    age=(now_ns-stamp)*1e-9
    report={'valid':False,'source_frame':msg.header.frame_id,'target_frame':'base_link',
            'source_age_s':age,'motion_compensated':False,'control_requests':0}
    if not msg.header.frame_id or stamp==0 or not math.isfinite(age) or age<-.05 or age>1.:
        report['reason']='Missing frame, zero stamp, stale data or clock mismatch'
        return report
    try:
        source=scan_points(msg)
        p,q=transform.translation,transform.rotation
        points=transform_xyz(source,[p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
        report.update(summarize(points,[.1,5.,-.3,.3,-.1,1.],3))
        report['raw_range_min_m']=min(math.hypot(p[0],p[1]) for p in source) if source else None
        report['interpretation']='ROI统计，不保证障碍物身份或测距精度；不作为运动许可'
    except (ValueError,TypeError,AttributeError) as exc:
        report.update(valid=False,reason=str(exc))
    return report
