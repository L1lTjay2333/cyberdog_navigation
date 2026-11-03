"""Small fixed-budget RGB Gaussian baseline. No depth precision claims or robot IO."""
import argparse,json,math
from pathlib import Path
import numpy as np
from PIL import Image
import pycolmap
from scipy.spatial import cKDTree
import torch
from gsplat import rasterization


def train(dataset,output,steps=2000):
    root=Path(dataset);out=Path(output)
    if not 1<=steps<=10000:raise ValueError('Steps must be 1..10000')
    if not torch.cuda.is_available():raise ValueError('CUDA GPU unavailable')
    gate=json.loads((root/'sfm_report.json').read_text())
    if not gate.get('reconstruction_screen_pass') or not gate.get('undistorted_dataset'):
        raise ValueError('Need successful COLMAP and undistorted images')
    data=Path(gate['undistorted_dataset']);recon=pycolmap.Reconstruction(str(data/'sparse'))
    views=[]
    for iid in sorted(recon.reg_image_ids()):
        image=recon.images[iid];camera=recon.cameras[image.camera_id]
        if camera.model.name not in ('PINHOLE','SIMPLE_PINHOLE'):raise ValueError('Expected undistorted pinhole camera')
        rgb=np.asarray(Image.open(data/'images'/image.name).convert('RGB'))
        # Initial 640-level stream already small; retain image sampling geometry.
        matrix=np.eye(4);matrix[:3]=image.cam_from_world().matrix()
        views.append((image.name,torch.tensor(matrix,dtype=torch.float32,device='cuda'),
            torch.tensor(camera.calibration_matrix(),dtype=torch.float32,device='cuda'),
            torch.tensor(rgb.copy(),dtype=torch.float32,device='cuda')/255.))
    if len(views)<24:raise ValueError('Need at least 24 registered views')
    held=views[::8];fit=[v for i,v in enumerate(views) if i%8!=0]
    entries=list(recon.points3D.values())
    entries=[p for p in entries if p.track.length()>=3 and p.error<2.]
    if len(entries)<100:raise ValueError('Not enough reliable initialization points')
    if len(entries)>30000:entries=[entries[i] for i in np.linspace(0,len(entries)-1,30000).astype(int)]
    xyz=np.array([p.xyz for p in entries]);colors=np.array([p.color for p in entries])/255.
    distances=cKDTree(xyz).query(xyz,k=4)[0][:,1:].mean(axis=1)
    scale=np.maximum(distances,1e-5)
    params=torch.nn.ParameterDict({
        'means':torch.nn.Parameter(torch.tensor(xyz,dtype=torch.float32,device='cuda')),
        'log_scales':torch.nn.Parameter(torch.tensor(np.repeat(np.log(scale)[:,None],3,axis=1),dtype=torch.float32,device='cuda')),
        'quats':torch.nn.Parameter(torch.tensor(np.tile([1.,0,0,0],(len(xyz),1)),dtype=torch.float32,device='cuda')),
        'opacity_logits':torch.nn.Parameter(torch.full((len(xyz),),-1.,device='cuda')),
        'color_logits':torch.nn.Parameter(torch.tensor(np.log(np.clip(colors,.001,.999)/(1-np.clip(colors,.001,.999))),dtype=torch.float32,device='cuda'))})
    extent=float(np.linalg.norm(np.ptp(xyz,axis=0)))
    rates={'means':max(extent,1e-3)*1e-4,'log_scales':.003,'quats':.001,'opacity_logits':.02,'color_logits':.01}
    optimizer=torch.optim.Adam([{'params':[p],'lr':rates[k]} for k,p in params.items()])
    def render(view):
        _,matrix,k,rgb=view;h,w=rgb.shape[:2]
        images,_,_=rasterization(means=params['means'],quats=params['quats'],
            scales=params['log_scales'].exp(),opacities=params['opacity_logits'].sigmoid(),
            colors=params['color_logits'].sigmoid(),viewmats=matrix[None],Ks=k[None],width=w,height=h,
            packed=False,backgrounds=torch.zeros((1,3),device='cuda'))
        return images[0]
    out.mkdir(parents=True,exist_ok=False);torch.manual_seed(1)
    for step in range(steps):
        view=fit[step%len(fit)];optimizer.zero_grad();prediction=render(view)
        loss=(prediction-view[3]).abs().mean()+1e-4*params['log_scales'].exp().mean()
        if not torch.isfinite(loss):raise ValueError('Non-finite training loss')
        loss.backward();optimizer.step()
        if step%100==0:
            print(json.dumps({'step':step,'loss':float(loss.detach())}),flush=True)
    with torch.no_grad():
        scores=[]
        for i,v in enumerate(held):
            rendered=render(v);mse=float(((rendered-v[3])**2).mean());scores.append(-10*math.log10(max(mse,1e-12)))
            Image.fromarray((rendered.clamp(0,1).cpu().numpy()*255).astype('uint8')).save(out/('heldout_%02d.png'%i))
        arrays={k:p.detach().cpu().numpy() for k,p in params.items()}
    np.savez(out/'gaussians.npz',**arrays)
    # Standard degree-0 3DGS PLY fields for compatible external viewers.
    n=len(xyz);sh=(torch.sigmoid(params['color_logits']).detach().cpu().numpy()-.5)/.28209479177387814
    values=np.column_stack((arrays['means'],np.zeros((n,3)),sh,arrays['opacity_logits'],arrays['log_scales'],arrays['quats']))
    names=['x','y','z','nx','ny','nz','f_dc_0','f_dc_1','f_dc_2','opacity','scale_0','scale_1','scale_2','rot_0','rot_1','rot_2','rot_3']
    header='ply\nformat ascii 1.0\nelement vertex %d\n'%n+''.join('property float '+name+'\n' for name in names)+'end_header'
    np.savetxt(out/'gaussians.ply',values,fmt='%.7g',header=header,comments='')
    result=dict(gaussians=n,steps=steps,training_views=len(fit),withheld_views=len(held),
        withheld_psnr_db=scores,depth_supervision=False,metric_scale_verified=False,
        geometry_accuracy_verified=False,method='experimental fixed-count RGB gsplat baseline',
        limitations=['No densification or pose refinement.','Full scene baseline; target masks are not yet integrated.',
                     'Held-out image quality does not certify shape or dimensions.'])
    (out/'report.json').write_text(json.dumps(result,indent=2));return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('dataset');p.add_argument('output');p.add_argument('--steps',type=int,default=2000)
    a=p.parse_args();print(json.dumps(train(a.dataset,a.output,a.steps)))
