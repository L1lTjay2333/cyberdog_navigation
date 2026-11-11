"""Replay recorded depth with stock saved global poses and measured fixed camera TF."""
import argparse,json
from pathlib import Path
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation,Slerp
from reconstruction_core import decode_depth_frame


def correction_trajectory(depths, poses, extrinsic_rotation, extrinsic_translation):
    times = np.array([d['stamp'] for d in depths])
    local_rotation = Rotation.from_quat([d['quaternion'] for d in depths]) * extrinsic_rotation.inv()
    camera_position = np.array([d['translation'] for d in depths])
    local_position = camera_position - local_rotation.apply(
        np.tile(extrinsic_translation, (len(depths), 1)))
    dense = {}
    for depth in depths:
        for sample in depth.get('local_pose_history',[]):
            if times[0] <= sample['stamp'] <= times[-1]:
                dense[sample['stamp']] = sample
    # Exact camera acquisition poses remain authoritative at frame times.
    for t,position,quaternion in zip(times,local_position,local_rotation.as_quat()):
        dense[t] = dict(stamp=float(t),translation=position,quaternion=quaternion)
    dense_times = np.array(sorted(dense))
    dense_positions = np.array([dense[t]['translation'] for t in dense_times])
    dense_rotations = Rotation.from_quat([dense[t]['quaternion'] for t in dense_times])
    interpolate_local_rotation = Slerp(dense_times, dense_rotations)
    inside = poses[(poses[:,0] >= times[0]) & (poses[:,0] <= times[-1])]
    overlap_start=max(times[0],poses[0,0])
    overlap_end=min(times[-1],poses[-1,0])
    if overlap_end <= overlap_start:
        raise ValueError('Saved and captured trajectories do not overlap')
    if max(overlap_start-times[0],times[-1]-overlap_end)>2:
        raise ValueError('More than two seconds outside saved optimized trajectory')
    endpoints = np.array([overlap_start,overlap_end])
    endpoint_rows = np.column_stack([
        endpoints,
        np.column_stack([np.interp(endpoints,poses[:,0],poses[:,j]) for j in range(1,4)]),
        Slerp(poses[:,0],Rotation.from_quat(poses[:,4:8]))(endpoints).as_quat()])
    inside = np.concatenate([inside,endpoint_rows])
    inside = inside[np.argsort(inside[:,0])]
    _, unique = np.unique(inside[:,0],return_index=True)
    inside = inside[unique]
    # Retain observed camera motion between optimized trajectory nodes.
    anchor_times = inside[:,0]
    rotation_at_nodes = interpolate_local_rotation(anchor_times)
    position_at_nodes = np.column_stack([
        np.interp(anchor_times, dense_times, dense_positions[:,j]) for j in range(3)])
    delta_rotation = Rotation.from_quat(inside[:,4:8]) * rotation_at_nodes.inv()
    delta_position = inside[:,1:4] - delta_rotation.apply(position_at_nodes)
    interpolate_delta = Slerp(anchor_times, delta_rotation)
    clipped = np.clip(times, anchor_times[0], anchor_times[-1])
    corrected_rotation = interpolate_delta(clipped) * local_rotation
    corrected_position = interpolate_delta(clipped).apply(local_position) + np.column_stack([
        np.interp(clipped, anchor_times, delta_position[:,j]) for j in range(3)])
    # Withhold every fourth correction anchor to measure interpolation uncertainty.
    train = np.arange(len(anchor_times)) % 4 != 1
    train[0] = train[-1] = True
    check = ~train
    withheld_rotation = Slerp(anchor_times[train], delta_rotation[train])(anchor_times[check])
    withheld_position = withheld_rotation.apply(position_at_nodes[check]) + np.column_stack([
        np.interp(anchor_times[check],anchor_times[train],delta_position[train,j]) for j in range(3)])
    error = np.linalg.norm(withheld_position - inside[check,1:4],axis=1)
    report = dict(correction_anchors=len(anchor_times), local_pose_samples=len(dense_times),
                  max_correction_anchor_gap_s=float(np.diff(anchor_times).max()),
                  endpoint_correction_hold_s=float(np.max(abs(times-clipped))),
                  frames_with_endpoint_correction_hold=int(np.sum(times!=clipped)),
                  withheld_position_error_p50_p95_m=np.percentile(error,[50,95]).tolist(),
                  note='Interpolation consistency against saved poses, not absolute mapping accuracy.')
    return corrected_position, corrected_rotation, report

def run(source,folder):
    poses=np.loadtxt(folder/'stock_pose.txt')
    if np.any(np.diff(poses[:,0])<=0):raise ValueError('Non-monotonic saved poses')
    rotations=Slerp(poses[:,0],Rotation.from_quat(poses[:,4:8]))
    extrinsic=json.loads((folder/'camera_extrinsic.json').read_text(encoding='utf-8-sig'))
    er=Rotation.from_quat(extrinsic['quaternion'])
    et=np.array(extrinsic['translation'])
    with source.open() as stream:
        records = [json.loads(line) for line in stream]
    depths = [r['depth'] for r in records if r.get('depth')]
    if not depths:
        raise ValueError('No depth records')
    positions, corrected_rotations, interpolation_report = correction_trajectory(depths,poses,er,et)
    cells={};original_cells={};frames=0;skipped=0
    with (folder/'replayed_camera_poses.jsonl').open('w') as pose_log:
        for depth, position, base in zip(depths,positions,corrected_rotations):
            replay=dict(depth,translation=(position+base.apply(et)).tolist(),
                        quaternion=(base*er).as_quat().tolist())
            pose_log.write(json.dumps(dict(stamp=depth['stamp'],translation=replay['translation'],
                                            quaternion=replay['quaternion']))+'\n')
            raw_points=decode_depth_frame(depth,stride=8)
            _,raw_idx=np.unique(np.floor(raw_points/.05),axis=0,return_index=True)
            for point in raw_points[raw_idx]:
                key=tuple(np.floor(point/.05).astype(int))
                if key not in original_cells:original_cells[key]=[point.copy(),1]
                else:original_cells[key][0]+=point;original_cells[key][1]+=1
            points=decode_depth_frame(replay,stride=8)
            _,idx=np.unique(np.floor(points/.05),axis=0,return_index=True)
            for p in points[idx]:
                key=tuple(np.floor(p/.05).astype(int))
                if key not in cells:cells[key]=[p.copy(),1]
                else:cells[key][0]+=p;cells[key][1]+=1
            frames+=1
    points=np.array([v/n for v,n in cells.values() if n>=3])
    cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    cloud,_=cloud.remove_statistical_outlier(20,2)
    o3d.io.write_point_cloud(str(folder/'global_pose_replay.ply'),cloud)
    np.asarray(cloud.points).astype('<f4').tofile(folder/'global_pose_replay.bin')
    original=np.array([v/n for v,n in original_cells.values() if n>=3])
    original_cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(original))
    original_cloud,_=original_cloud.remove_statistical_outlier(20,2)
    original=np.asarray(original_cloud.points)
    o3d.io.write_point_cloud(str(folder/'local_pose_replay.ply'),original_cloud)
    corrected=np.asarray(cloud.points)
    lo=np.minimum(original[:,:2].min(0),corrected[:,:2].min(0))-.1
    hi=np.maximum(original[:,:2].max(0),corrected[:,:2].max(0))+.1
    w,h=np.ceil((hi-lo)/.025).astype(int)+1
    images=[]
    for p in (original,corrected):
        p=p[(p[:,2]>.15)&(p[:,2]<1.4)]
        xy=np.floor((p[:,:2]-lo)/.025).astype(int)
        image=np.full((h,w,3),245,np.uint8)
        image[h-1-xy[:,1],xy[:,0]]=[30,70,110]
        images.append(image)
    o3d.io.write_image(str(folder/'local_global_comparison.png'),
                      o3d.geometry.Image(np.ascontiguousarray(np.concatenate(images,axis=1))))
    report=dict(frames=frames,skipped=skipped,points=len(cloud.points),
                input='raw recorded depth',pose_source='saved stock global_pose from pose.txt',
                interpolation=interpolation_report,
                limitations=['Fixed TF read after acquisition; assumes unchanged camera mounting.',
                             'Source matches local official reference; binary parity not proven.',
                             'No wall parallelism constraint imposed; result requires visual validation.'])
    (folder/'global_pose_replay.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report))
if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('source',type=Path);parser.add_argument('folder',type=Path)
    args=parser.parse_args();run(args.source,args.folder)
