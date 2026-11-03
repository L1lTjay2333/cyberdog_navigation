"""Offline diagnostic projections. Never certifies or changes calibration.

Uses actual depth CameraInfo and cam3 from an archived stock calibration.
Source-layout, mathematical inverse and direct matrix are shown separately;
alternatives diagnose convention mistakes and are never automatically selected.
"""
import argparse
import ast
import json
from pathlib import Path
import re
import numpy as np
from capture_rgbd_samples import decode_image, save_rgb_png


def read_cam3(path):
    text = path.read_text(encoding='utf-8')
    block = text.split('\ncam3:\n', 1)[1].split('\ncam4:', 1)[0]
    def vector(name):
        return ast.literal_eval(re.search(r'\b' + name + r':\s*(\[[^\]]*\])', block).group(1))
    matrix = [ast.literal_eval(s) for s in re.findall(r'^  - (\[[^\]]*\])', block, re.M)]
    t = np.asarray(matrix, dtype=float)
    if t.shape != (4, 4) or not np.isfinite(t).all() or not np.allclose(t[3], [0, 0, 0, 1]):
        raise ValueError('Invalid calibration transform')
    return t, vector('intrinsics'), vector('distortion_coeffs'), vector('resolution')


def project(xyz, rotation, translation, intrinsics, distortion, return_depth=False):
    q = xyz @ rotation.T + translation
    front = np.isfinite(q).all(axis=1) & (q[:, 2] > .05)
    q = q[front]
    x, y = q[:, 0] / q[:, 2], q[:, 1] / q[:, 2]
    k1, k2, p1, p2, k3 = distortion
    r2 = x*x+y*y
    radial = 1+k1*r2+k2*r2*r2+k3*r2*r2*r2
    xd = x*radial + 2*p1*x*y+p2*(r2+2*x*x)
    yd = y*radial + 2*p2*x*y+p1*(r2+2*y*y)
    fx, fy, cx, cy = intrinsics
    uv = np.stack((fx*xd+cx, fy*yd+cy), axis=1)
    return (uv, q[:,2]) if return_depth else uv


def visible_pixels(edge_uv, edge_z, all_uv, all_z, width, height, tolerance=.02):
    """Nearest-depth visibility at rounded target pixels; no gap filling/splatting.

    This is a diagnostic rasterization gate, not a calibration acceptance test.
    """
    def raster(uv,z):
        ok=np.isfinite(uv).all(axis=1)&np.isfinite(z)&(z>0)
        ok &= (uv[:,0]>=0)&(uv[:,0]<width-.5)&(uv[:,1]>=0)&(uv[:,1]<height-.5)
        return np.rint(uv[ok]).astype(int),z[ok]
    pixels,z=raster(all_uv,all_z)
    buffer=np.full(width*height,np.inf)
    np.minimum.at(buffer,pixels[:,1]*width+pixels[:,0],z)
    pixels,z=raster(edge_uv,edge_z)
    visible=z<=buffer[pixels[:,1]*width+pixels[:,0]]+tolerance
    return pixels[visible],int((~visible).sum())


def foreground_edges(z, valid, threshold=.06):
    """Select the nearer endpoint of valid depth jumps, independently of direction.

    Invalid-neighbor boundaries are returned separately: a hole is not evidence
    of a physical object contour. This does not solve cross-camera occlusion.
    """
    edges = np.zeros(z.shape, dtype=bool)
    holes = np.zeros(z.shape, dtype=bool)
    for a, b in ((np.s_[:, :-1], np.s_[:, 1:]), (np.s_[:-1, :], np.s_[1:, :])):
        za, zb = z[a], z[b]
        va, vb = valid[a], valid[b]
        jump = va & vb & (np.abs(za-zb) > threshold)
        edges[a] |= jump & (za < zb)
        edges[b] |= jump & (zb < za)
        holes[a] |= va & ~vb
        holes[b] |= vb & ~va
    return edges, holes


def run(folder, calibration):
    t, intrinsics, distortion, resolution = read_cam3(calibration)
    inverse = np.linalg.inv(t)
    variants = {'source_array_column_major': (inverse[:3, :3].T, inverse[:3, 3]),
                'mathematical_inverse': (inverse[:3, :3], inverse[:3, 3]),
                'direct_matrix': (t[:3, :3], t[:3, 3]),
                # PTX uses raw_u16 * scale, adds translation unchanged and skips
                # target model 5. Public caller passes scale=1 and model=5.
                # Equivalent center projection for metric XYZ: t / 1000.
                # Runtime caller payload and corner rasterization are unverified.
                'native_ptx_public_call_assumption': (inverse[:3, :3].T, inverse[:3, 3]/1000.)}
    # Source fills rs2_extrinsics.rotation row by row; rs2 consumes column-major.
    # The separately displayed alternatives are diagnostic, never corrections.
    pairs = {}
    for line in (folder/'raw_samples.jsonl').read_bytes().splitlines():
        row = json.loads(line)
        if row.get('kind') == 'image' and 'pair_id' in row:
            pairs.setdefault(row['pair_id'], []).append(row)
    output = folder/'projection_foreground_check'; output.mkdir(exist_ok=True)
    report = {'spatial_registration_verified': False, 'automatic_selection': False,
              'calibration': str(calibration.resolve()), 'pairs': [],
              'edge_method':'near_endpoint_valid_depth_jump',
              'depth_jump_threshold_m':.06,
              'cross_camera_occlusion_verified':False,
              'visibility_gate':'nearest_depth_at_same_rounded_pixel_only',
              'visibility_tolerance_m':.02,
              'native_kernel_parity_verified':False}
    for pair_id, rows in pairs.items():
        color_rows = [r for r in rows if r['encoding'] in ('rgb8','bgr8','rgba8','bgra8')]
        depth_rows = [r for r in rows if r['encoding'] in ('16UC1','32FC1')]
        if len(color_rows) != 1 or len(depth_rows) != 1:
            raise ValueError('Each pair must have one color and one depth')
        color, depth = color_rows[0], depth_rows[0]
        dt = abs(color['stamp']-depth['stamp'])
        if dt > .05: raise ValueError('Pair exceeds timestamp gate')
        rgb = decode_image(color)
        if [rgb.shape[1], rgb.shape[0]] != resolution:
            raise ValueError('Color dimensions differ from calibration')
        info = [c for c in depth['calibration'] if c['frame_id'] == depth['frame_id']
                and c['width'] == depth['width'] and c['height'] == depth['height']
                and abs(c['stamp']-depth['stamp']) <= .2]
        if len(info) != 1: raise ValueError('Missing unique depth calibration')
        z = decode_image(depth).astype(float)*(.001 if depth['encoding']=='16UC1' else 1)
        valid = np.isfinite(z)&(z>=.15)&(z<=4)
        edge, holes = foreground_edges(z, valid)
        v, u = np.nonzero(edge & valid)
        p = info[0]['p']
        if p[0]<=0 or p[5]<=0 or p[3]!=0 or p[7]!=0: raise ValueError('Unsupported depth projection')
        xyz = np.stack(((u-p[2])*z[v,u]/p[0], (v-p[6])*z[v,u]/p[5], z[v,u]),axis=1)
        av,au=np.nonzero(valid)
        all_xyz=np.stack(((au-p[2])*z[av,au]/p[0],(av-p[6])*z[av,au]/p[5],z[av,au]),axis=1)
        result = {'pair_id':pair_id, 'time_difference_s':dt, 'depth_edge_points':len(xyz),
                  'invalid_neighbor_boundary_points':int(holes.sum()),'variants':{}}
        diagnostic = np.zeros((*z.shape,3),dtype=np.uint8)
        diagnostic[valid] = [40,40,40]
        diagnostic[holes] = [255,140,0]
        diagnostic[edge] = [0,255,80]
        save_rgb_png(output/('pair_%s_depth_boundary_types.png'%pair_id), diagnostic)
        save_rgb_png(output/('pair_%s_original.png'%pair_id), rgb)
        for name, (rotation, translation) in variants.items():
            coefficients=[0]*5 if name=='native_ptx_public_call_assumption' else distortion
            uv,ez = project(xyz, rotation, translation, intrinsics, coefficients,return_depth=True)
            all_uv,az = project(all_xyz,rotation,translation,intrinsics,coefficients,return_depth=True)
            pixels,rejected=visible_pixels(uv,ez,all_uv,az,rgb.shape[1],rgb.shape[0])
            overlay = rgb.copy()
            overlay[pixels[:,1],pixels[:,0]] = [0,255,80]
            filename = 'pair_%s_%s.png'%(pair_id,name)
            save_rgb_png(output/filename, overlay)
            result['variants'][name]={'preview':filename,'projected_edges_in_image':len(pixels),
                'occluded_edge_points_rejected':rejected}
        base,_=project(xyz,inverse[:3,:3].T,inverse[:3,3],intrinsics,distortion,return_depth=True)
        alternative,_=project(xyz,inverse[:3,:3].T,inverse[:3,3]/1000.,intrinsics,[0]*5,return_depth=True)
        # Compare identical source points only when both transforms keep all
        # candidates in front; do not silently compare different filtered rows.
        if len(base)==len(alternative)==len(xyz) and len(xyz):
            def in_image(a):
                return np.isfinite(a).all(axis=1)&(a[:,0]>=0)&(a[:,0]<resolution[0])&(a[:,1]>=0)&(a[:,1]<resolution[1])
            common=in_image(base)&in_image(alternative)
            difference=np.linalg.norm(base[common]-alternative[common],axis=1)
            if len(difference):
                result['call_assumption_sensitivity_px']={'median':float(np.median(difference)),
                    'p95':float(np.percentile(difference,95)),
                    'common_in_image_points':int(common.sum()),
                    'note':'difference between hypotheses in common image bounds, not image alignment error'}
        report['pairs'].append(result)
    if not report['pairs']:raise ValueError('No paired images')
    (output/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({'output':str(output),'pairs':len(report['pairs']),'spatial_registration_verified':False}))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path)
    parser.add_argument('--calibration',type=Path,default=Path('evidence/rgbd_calibration_20261030/params_extrinsic.yaml'))
    args=parser.parse_args();run(args.folder,args.calibration)
