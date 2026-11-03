"""Exercise actual CUDA rasterization AND gradients; import success alone is insufficient."""
import json
import torch
from gsplat import rasterization


def check():
    if not torch.cuda.is_available():raise RuntimeError('CUDA unavailable')
    means=torch.tensor([[0.,0.,2.],[.1,0.,2.]],device='cuda',requires_grad=True)
    colors=torch.tensor([[1.,0.,0.],[0.,1.,0.]],device='cuda',requires_grad=True)
    image,alpha,_=rasterization(means=means,
        quats=torch.tensor([[1.,0,0,0]]*2,device='cuda'),
        scales=torch.full((2,3),.08,device='cuda'),opacities=torch.full((2,),.8,device='cuda'),
        colors=colors,viewmats=torch.eye(4,device='cuda')[None],
        Ks=torch.tensor([[[40.,0,16],[0,40,16],[0,0,1]]],device='cuda'),width=32,height=32,packed=False)
    loss=image.square().mean();loss.backward();torch.cuda.synchronize()
    if not torch.isfinite(image).all() or image.sum()<=0 or means.grad is None or not torch.isfinite(means.grad).all():
        raise RuntimeError('Invalid rendered image or gradients')
    return dict(torch=torch.__version__,cuda=torch.version.cuda,gpu=torch.cuda.get_device_name(),
                rendered_shape=list(image.shape),backward_pass=True,peak_memory_bytes=torch.cuda.max_memory_allocated())


if __name__=='__main__':print(json.dumps(check()))
