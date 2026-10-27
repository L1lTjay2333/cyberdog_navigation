/* Real XYZ only: raw frames shown individually; only timestamped map TF fuses. */
(() => {
const canvas=document.getElementById('cloud'),ctx=canvas.getContext('2d');
const label=document.getElementById('cloudstatus'),model=new Map();
let points=[],frame='map',lastStamp=null,lastStart=null,lastTask=null,yaw=.5,pitch=.55,zoom=1,center=[0,0,0],radius=3,drag=null,viewFitted=false,robot=null;
function reset(){model.clear();points=[];lastStamp=null;viewFitted=false;draw();}
function bounds(){if(!points.length)return;const lo=[Infinity,Infinity,Infinity],hi=[-Infinity,-Infinity,-Infinity];const fitPoints=robot?[...points,[robot.x,robot.y,0]]:points;for(const p of fitPoints)for(let a=0;a<3;a++){lo[a]=Math.min(lo[a],p[a]);hi[a]=Math.max(hi[a],p[a]);}center=[(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,0];radius=Math.max(.5,(hi[0]-lo[0])/2,(hi[1]-lo[1])/2,Math.abs(lo[2]),Math.abs(hi[2]));viewFitted=true;}
function draw(){ctx.fillStyle='#101c29';ctx.fillRect(0,0,canvas.width,canvas.height);if(!points.length){ctx.fillStyle='#bdd0e0';ctx.font='18px system-ui';ctx.fillText('等待真实深度点云',30,50);return;}
 const cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch),scale=230*zoom/radius,step=Math.max(1,Math.ceil(points.length/20000)),out=[];
 const project=p=>{const x=p[0]-center[0],y=p[1]-center[1],z=p[2],u=cy*x-sy*y,v=sy*x+cy*y;return [450+u*scale,300-(cp*z-sp*v)*scale];};
 function referenceLine(a,b,color,width=1){a=project(a);b=project(b);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();}
 const spacing=radius>4?1:.5,reach=Math.ceil(radius/spacing)*spacing;
 for(let d=-reach;d<=reach;d+=spacing){referenceLine([center[0]+d,center[1]-reach,0],[center[0]+d,center[1]+reach,0],'#263b50');referenceLine([center[0]-reach,center[1]+d,0],[center[0]+reach,center[1]+d,0],'#263b50');}
 const origin=[center[0],center[1],0],axis=Math.min(1,radius);
 for(const [end,color,text] of [[[center[0]+axis,center[1],0],'#f17676','X'],[[center[0],center[1]+axis,0],'#6cdd91','Y'],[[center[0],center[1],axis],'#72b8ff','Z↑']]){referenceLine(origin,end,color,2);const p=project(end);ctx.fillStyle=color;ctx.font='15px system-ui';ctx.fillText(text,p[0]+5,p[1]-5);}
 for(let i=0;i<points.length;i+=step){const p=points[i],x=p[0]-center[0],y=p[1]-center[1],z=p[2]-center[2],u=cy*x-sy*y,v=sy*x+cy*y,depth=cp*v+sp*z,h=cp*z-sp*v;out.push([450+u*scale,300-h*scale,depth,p[2]]);}
 out.sort((a,b)=>a[2]-b[2]);for(const p of out){if(p[0]<0||p[0]>900||p[1]<0||p[1]>600)continue;const hue=200-120*Math.max(0,Math.min(1,(p[3]-center[2]+radius)/(2*radius)));ctx.fillStyle=`hsl(${hue},85%,65%)`;ctx.fillRect(p[0],p[1],2,2);}
 if(robot){
  // Position and heading are a map XY reference, not measured head-joint pose.
  const base=project([robot.x,robot.y,0]),tip=project([robot.x+.4*Math.cos(robot.yaw),robot.y+.4*Math.sin(robot.yaw),0]);
  const angle=Math.atan2(tip[1]-base[1],tip[0]-base[0]),color=robot.fresh?'#38baff':'#8b99a6';
  referenceLine([robot.x,robot.y,0],[robot.x+.4*Math.cos(robot.yaw),robot.y+.4*Math.sin(robot.yaw),0],color,4);
  ctx.fillStyle=color;ctx.strokeStyle='#ffffff';ctx.lineWidth=2;ctx.beginPath();ctx.arc(base[0],base[1],7,0,Math.PI*2);ctx.fill();ctx.stroke();
  ctx.beginPath();ctx.moveTo(tip[0],tip[1]);ctx.lineTo(tip[0]-18*Math.cos(angle)+8*Math.sin(angle),tip[1]-18*Math.sin(angle)-8*Math.cos(angle));ctx.lineTo(tip[0]-18*Math.cos(angle)-8*Math.sin(angle),tip[1]-18*Math.sin(angle)+8*Math.cos(angle));ctx.closePath();ctx.fill();ctx.stroke();
  ctx.font='16px system-ui';ctx.fillStyle=color;ctx.fillText(robot.fresh?'狗头朝向':'狗头朝向（定位过期）',tip[0]+12,tip[1]-12);
 }
 ctx.fillStyle='#bdd0e0';ctx.font='14px system-ui';ctx.fillText('拖动自由旋转 · 滚轮缩放 · 水平视角一键恢复 · 网格为z=0坐标参考',18,580);
}
canvas.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);});
canvas.addEventListener('pointermove',e=>{if(!drag)return;yaw+=(e.clientX-drag[0])*.008;pitch=Math.max(-Math.PI/2,Math.min(Math.PI/2,pitch+(e.clientY-drag[1])*.008));drag=[e.clientX,e.clientY];draw();});
canvas.addEventListener('pointerup',()=>drag=null);canvas.addEventListener('pointercancel',()=>drag=null);
canvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(.2,Math.min(8,zoom*Math.exp(-e.deltaY*.001)));draw();},{passive:false});
document.getElementById('clearcloud').onclick=()=>{reset();label.textContent='已清空电脑点云，等待下一帧。';};
document.getElementById('fitcloud').onclick=()=>{bounds();zoom=1;draw();};
document.getElementById('levelcloud').onclick=()=>{yaw=.5;pitch=.55;draw();};
document.getElementById('topcloud').onclick=()=>{yaw=0;pitch=Math.PI/2;draw();};
document.getElementById('exportcloud').onclick=()=>{if(!points.length){label.textContent='还没有点云可导出。';return;}const text=`ply\nformat ascii 1.0\ncomment frame ${frame}\ncomment measured ROS XYZ scale requires field validation\nelement vertex ${points.length}\nproperty float x\nproperty float y\nproperty float z\nend_header\n`+points.map(p=>p.join(' ')).join('\n');const url=URL.createObjectURL(new Blob([text],{type:'text/plain'}));const a=document.createElement('a');a.href=url;a.download='cyberdog_'+(frame==='map'?'map_cloud':'depth_frame')+'.ply';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
window.updateCloud=s=>{const cloud=s.cloud,status=s.task?.status;if(lastStart!==s.started_unix||(status===7&&lastTask!==7)){reset();lastStart=s.started_unix;}lastTask=status;
 const pose=s.pose;robot=pose?.frame==='map'&&[pose.x,pose.y,pose.yaw].every(Number.isFinite)?{x:pose.x,y:pose.y,yaw:pose.yaw,fresh:s.connection==='connected'&&Date.now()/1000-s.local_received_unix<3&&s.emitted_unix-pose.received_unix<3&&(status===8||status===1)}:null;
 if(!cloud?.points_xyz?.length){label.textContent=cloud?.error?'点云格式未通过检查：'+cloud.error:'尚未收到真实深度点云。开机定位后验证数据；当前不生成虚构三维地图。';return;}
 const fresh=s.connection==='connected'&&Date.now()/1000-s.local_received_unix<3&&s.emitted_unix-cloud.received_unix<3;
 if(!fresh){label.textContent='点云数据已过期，保留已采集画面，停止融合。';draw();return;}
 const signature=cloud.received_unix+':'+cloud.frame+':'+cloud.aligned;if(signature===lastStamp){draw();return;}lastStamp=signature;
 const valid=cloud.points_xyz.filter(p=>p.length===3&&p.every(Number.isFinite));
 if(cloud.aligned&&(cloud.frame==='map'||status===5&&cloud.frame===s.map?.frame)&&[5,8,1].includes(status)){
  if(model.size&&frame!==cloud.frame)model.clear();
  for(const p of valid){const key=p.map(v=>Math.floor(v/.05)).join(',');if(!model.has(key)){model.set(key,p);if(model.size>60000)model.delete(model.keys().next().value);}}
  points=Array.from(model.values());frame=cloud.frame;label.textContent=`三维测量点云 · ${points.length} 个体素 · ${frame}对齐 · 5厘米融合；尺度与场景覆盖待现场验证。`;
 }else if(model.size){points=Array.from(model.values());frame='map';label.textContent=`三维测量点云 · ${points.length} 个体素 · 当前帧等待坐标对齐，保留已融合结果。`;}
 else{label.textContent='等待当前深度帧对齐map坐标；保持当前视角，不切换到相机坐标。';return;}
 if(!viewFitted)bounds();draw();
};draw();
})();
