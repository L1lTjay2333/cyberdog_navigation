/* Real XYZ only: raw frames shown individually; only timestamped map TF fuses. */
(() => {
const canvas=document.getElementById('cloud'),ctx=canvas.getContext('2d');
const label=document.getElementById('cloudstatus'),model=new Map();
const hits=new Map();let modelFrame=null,fog=true;
let panX=0,panY=0,moveMode='pan',following=false;
let restoreRequested=false;
const pointers=new Map();
let points=[],frame='map',lastStamp=null,lastStart=null,lastTask=null,yaw=.5,pitch=.55,zoom=1,center=[0,0,0],radius=3,drag=null,viewFitted=false,robot=null;
function reset(){model.clear();hits.clear();modelFrame=null;points=[];lastStamp=null;viewFitted=false;panX=panY=0;robot=null;draw();}
function bounds(){if(!points.length)return;const lo=[Infinity,Infinity,Infinity],hi=[-Infinity,-Infinity,-Infinity];const fitPoints=robot?[...points,[robot.x,robot.y,0]]:points;for(const p of fitPoints)for(let a=0;a<3;a++){lo[a]=Math.min(lo[a],p[a]);hi[a]=Math.max(hi[a],p[a]);}center=[(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,0];radius=Math.max(.5,(hi[0]-lo[0])/2,(hi[1]-lo[1])/2,Math.abs(lo[2]),Math.abs(hi[2]));viewFitted=true;}
let drawPending=false;
function draw(){if(drawPending)return;drawPending=true;requestAnimationFrame(()=>{drawPending=false;if(!document.hidden)paintCloud();});}
function paintCloud(){ctx.fillStyle='#f0ebe1';ctx.fillRect(0,0,canvas.width,canvas.height);if(!points.length){ctx.fillStyle='#756a59';ctx.font='18px system-ui';ctx.fillText('等待真实深度点云',30,50);return;}
 const cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch),scale=230*zoom/radius,step=Math.max(1,Math.ceil(points.length/(pointers.size?8000:20000))),out=[];
 const project=p=>{const x=p[0]-center[0],y=p[1]-center[1],z=p[2],u=cy*x-sy*y,v=sy*x+cy*y;return [450+panX+u*scale,300+panY-(cp*z-sp*v)*scale];};
 function referenceLine(a,b,color,width=1){a=project(a);b=project(b);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();}
 // Ground projection of depth hits only: never mark rays/occluded space as free.
 if(fog&&model.size){
  const covered=new Set(points.map(p=>Math.floor(p[0]/.25)+','+Math.floor(p[1]/.25)));
  const reach=Math.min(12,Math.max(2,radius)),x0=Math.floor((center[0]-reach)/.25),x1=Math.ceil((center[0]+reach)/.25),y0=Math.floor((center[1]-reach)/.25),y1=Math.ceil((center[1]+reach)/.25);
  ctx.fillStyle='rgba(155,165,180,.18)';
  for(let x=x0;x<=x1;x++)for(let y=y0;y<=y1;y++){
   if(covered.has(x+','+y))continue;
   const corners=[[x*.25,y*.25,0],[(x+1)*.25,y*.25,0],[(x+1)*.25,(y+1)*.25,0],[x*.25,(y+1)*.25,0]].map(project);
   ctx.beginPath();ctx.moveTo(...corners[0]);for(const p of corners.slice(1))ctx.lineTo(...p);ctx.closePath();ctx.fill();
  }
 }
 const spacing=radius>4?1:.5,reach=Math.ceil(radius/spacing)*spacing;
 for(let d=-reach;d<=reach;d+=spacing){referenceLine([center[0]+d,center[1]-reach,0],[center[0]+d,center[1]+reach,0],'#263b50');referenceLine([center[0]-reach,center[1]+d,0],[center[0]+reach,center[1]+d,0],'#263b50');}
 const origin=[center[0],center[1],0],axis=Math.min(1,radius);
 for(const [end,color,text] of [[[center[0]+axis,center[1],0],'#f17676','X'],[[center[0],center[1]+axis,0],'#6cdd91','Y'],[[center[0],center[1],axis],'#72b8ff','Z↑']]){referenceLine(origin,end,color,2);const p=project(end);ctx.fillStyle=color;ctx.font='15px system-ui';ctx.fillText(text,p[0]+5,p[1]-5);}
 for(let i=0;i<points.length;i+=step){const p=points[i],x=p[0]-center[0],y=p[1]-center[1],z=p[2]-center[2],u=cy*x-sy*y,v=sy*x+cy*y,depth=cp*v+sp*z,h=cp*z-sp*v;out.push([450+panX+u*scale,300+panY-h*scale,depth,p[2]]);}
 out.sort((a,b)=>a[2]-b[2]);for(const p of out){if(p[0]<0||p[0]>900||p[1]<0||p[1]>600)continue;const hue=200-120*Math.max(0,Math.min(1,(p[3]-center[2]+radius)/(2*radius)));ctx.fillStyle=`hsl(${hue},85%,65%)`;ctx.fillRect(p[0],p[1],2,2);}
 if(robot){
  // Position and heading are a map XY reference, not measured head-joint pose.
  const base=project([robot.x,robot.y,0]),tip=project([robot.x+.4*Math.cos(robot.yaw),robot.y+.4*Math.sin(robot.yaw),0]);
  const angle=Math.atan2(tip[1]-base[1],tip[0]-base[0]),color=robot.fresh?'#38baff':'#8b99a6';
  referenceLine([robot.x,robot.y,0],[robot.x+.4*Math.cos(robot.yaw),robot.y+.4*Math.sin(robot.yaw),0],color,4);
  ctx.fillStyle=color;ctx.strokeStyle='#ffffff';ctx.lineWidth=2;ctx.beginPath();ctx.arc(base[0],base[1],7,0,Math.PI*2);ctx.fill();ctx.stroke();
  ctx.beginPath();ctx.moveTo(tip[0],tip[1]);ctx.lineTo(tip[0]-18*Math.cos(angle)+8*Math.sin(angle),tip[1]-18*Math.sin(angle)-8*Math.cos(angle));ctx.lineTo(tip[0]-18*Math.cos(angle)-8*Math.sin(angle),tip[1]-18*Math.sin(angle)+8*Math.cos(angle));ctx.closePath();ctx.fill();ctx.stroke();
  ctx.font='16px system-ui';ctx.fillStyle=color;ctx.fillText(robot.fresh?'机器狗 · 前向':'上次位置（定位过期）',tip[0]+12,tip[1]-12);
 }
 ctx.fillStyle='#756a59';ctx.font='14px system-ui';ctx.fillText('拖动：'+(moveMode==='pan'?'平移':'旋转')+' · 右键/Shift拖动平移 · 滚轮缩放 · 双指平移缩放',18,580);
}
canvas.style.touchAction='none';canvas.style.cursor='grab';
canvas.addEventListener('contextmenu',e=>e.preventDefault());
canvas.addEventListener('pointerdown',e=>{e.preventDefault();pointers.set(e.pointerId,[e.clientX,e.clientY]);canvas.setPointerCapture(e.pointerId);canvas.style.cursor='grabbing';following=false;followButton.textContent='跟随机器狗：关';});
canvas.addEventListener('pointermove',e=>{
 if(!pointers.has(e.pointerId))return;
 const old=[...pointers.values()],prev=pointers.get(e.pointerId);pointers.set(e.pointerId,[e.clientX,e.clientY]);
 const now=[...pointers.values()],rect=canvas.getBoundingClientRect();
 if(now.length>=2){
  const mid=a=>[(a[0][0]+a[1][0])/2,(a[0][1]+a[1][1])/2],a=mid(old),b=mid(now);
  panX+=(b[0]-a[0])*900/rect.width;panY+=(b[1]-a[1])*600/rect.height;
  const dist=a=>Math.hypot(a[0][0]-a[1][0],a[0][1]-a[1][1]),d=dist(old);if(d>0)zoom=Math.max(.05,Math.min(40,zoom*dist(now)/d));
 }else if(moveMode==='pan'||e.shiftKey||(e.buttons&6)){
  panX+=(e.clientX-prev[0])*900/rect.width;panY+=(e.clientY-prev[1])*600/rect.height;
 }else{yaw+=(e.clientX-prev[0])*.008;pitch=Math.max(-Math.PI/2,Math.min(Math.PI/2,pitch+(e.clientY-prev[1])*.008));}
 draw();
});
for(const event of ['pointerup','pointercancel','lostpointercapture'])canvas.addEventListener(event,e=>{pointers.delete(e.pointerId);canvas.style.cursor=pointers.size?'grabbing':'grab';});
canvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(.2,Math.min(8,zoom*Math.exp(-e.deltaY*.001)));draw();},{passive:false});
document.getElementById('clearcloud').onclick=()=>{reset();label.textContent='已清空电脑点云，等待下一帧。';};
document.getElementById('fitcloud').onclick=()=>{bounds();zoom=1;panX=panY=0;draw();};
document.getElementById('levelcloud').onclick=()=>{yaw=.5;pitch=.55;draw();};
document.getElementById('topcloud').onclick=()=>{yaw=0;pitch=Math.PI/2;draw();};
const fogButton=document.createElement('button');fogButton.textContent='迷雾：开';fogButton.onclick=()=>{fog=!fog;fogButton.textContent='迷雾：'+(fog?'开':'关');draw();};document.querySelector('#scene .cloudbuttons').append(fogButton);
const modeButton=document.createElement('button');modeButton.textContent='拖动：平移';modeButton.onclick=()=>{moveMode=moveMode==='pan'?'rotate':'pan';modeButton.textContent='拖动：'+(moveMode==='pan'?'平移':'旋转');};
const followButton=document.createElement('button');followButton.textContent='跟随机器狗：关';followButton.onclick=()=>{following=!following;followButton.textContent='跟随机器狗：'+(following?'开':'关');if(following&&robot?.fresh){center[0]=robot.x;center[1]=robot.y;panX=panY=0;draw();}};
const robotStatus=document.createElement('p');robotStatus.className='note';robotStatus.id='cloud-robot-status';robotStatus.textContent='等待与点云同坐标系的机器人定位';
const saveStatus=document.createElement('p');saveStatus.className='note';saveStatus.id='cloud-save-status';document.getElementById('scene').append(saveStatus);
const grayLink=document.createElement('a');grayLink.href='http://127.0.0.1:8772/';grayLink.target='_blank';grayLink.rel='noopener';grayLink.textContent='查看本轮离线灰模 / 点云对照';grayLink.style.display='inline-block';grayLink.style.padding='9px';document.getElementById('scene').append(grayLink);
document.querySelector('#scene .cloudbuttons').prepend(modeButton,followButton);document.getElementById('scene').append(robotStatus);
const fogNote=document.createElement('p');fogNote.className='note';fogNote.textContent='灰雾为深度表面观测的地面投影提示；清除迷雾不代表可通行。定位有效时边走边累计，未定位时仅显示当前相机帧。';document.getElementById('scene').append(fogNote);
document.getElementById('exportcloud').onclick=()=>{if(!points.length){label.textContent='还没有点云可导出。';return;}const text=`ply\nformat ascii 1.0\ncomment frame ${frame}\ncomment measured ROS XYZ scale requires field validation\nelement vertex ${points.length}\nproperty float x\nproperty float y\nproperty float z\nend_header\n`+points.map(p=>p.join(' ')).join('\n');const url=URL.createObjectURL(new Blob([text],{type:'text/plain'}));const a=document.createElement('a');a.href=url;a.download='cyberdog_'+(frame==='map'?'map_cloud':'depth_frame')+'.ply';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
window.updateCloud=s=>{const cloud=s.cloud,status=s.task?.status;if(lastStart!==s.started_unix||([5,7].includes(status)&&lastTask!==status)){reset();lastStart=s.started_unix;}lastTask=status;
 saveStatus.textContent=s.cloud_archive?.error?'点云保存异常：'+s.cloud_archive.error:s.cloud_archive?.points?`电脑自动保存：${s.cloud_archive.points} 个体素 · 刷新后自动恢复本轮地图`:'等待有效坐标对齐后自动保存到电脑';
 if(!restoreRequested&&s.cloud_archive?.key){
  restoreRequested=true;const key=s.cloud_archive.key,started=lastStart;
  fetch('/cloud/saved').then(r=>r.json()).then(d=>{
   const name=s.mapping?.map_name;
   if(lastStart!==started||d.key!==key||!name||key!==name+':'+d.frame||!Array.isArray(d.points_xyz)||modelFrame&&modelFrame!==d.frame)return;
   for(const p of d.points_xyz)if(p.length===3&&p.every(Number.isFinite)){const k=p.map(v=>Math.floor(v/.05)).join(',');if(!model.has(k)){model.set(k,p);hits.set(k,1);}}
   points=Array.from(model.values());modelFrame=frame=d.frame;lastStamp=null;bounds();draw();
  }).catch(()=>{saveStatus.textContent='读取已保存点云失败，请检查电脑服务';});
 }
 const pose=s.pose,target=(cloud?.aligned?cloud.frame:null)||modelFrame,poseAge=s.emitted_unix-pose?.received_unix;
 robot=pose?.frame===target&&[pose.x,pose.y,pose.yaw].every(Number.isFinite)?{x:pose.x,y:pose.y,yaw:pose.yaw,fresh:s.connection==='connected'&&Date.now()/1000-s.local_received_unix<3&&poseAge>=-.2&&poseAge<2&&[5,8,1].includes(status),expires:s.local_received_unix+Math.max(0,2-poseAge)}:null;
 robotStatus.textContent=robot?(robot.fresh?'机器狗实时位置':'上次有效位置（已过期）')+` · X ${robot.x.toFixed(2)} m · Y ${robot.y.toFixed(2)} m · 朝向 ${(robot.yaw*180/Math.PI).toFixed(0)}° · ${target}`:'尚无与点云匹配的定位；相机局部视图不标注全局狗位置';
 if(following&&robot?.fresh){center[0]=robot.x;center[1]=robot.y;panX=panY=0;}
 if(!cloud?.points_xyz?.length){label.textContent=cloud?.error?'点云格式未通过检查：'+cloud.error:'尚未收到真实深度点云。开机定位后验证数据；当前不生成虚构三维地图。';return;}
 const age=s.emitted_unix-cloud.stamp;
 const fresh=s.connection==='connected'&&Date.now()/1000-s.local_received_unix<3&&Number.isFinite(age)&&age>=-.2&&age<2;
 if(!fresh){label.textContent='点云数据已过期，保留已采集画面，停止融合。';draw();return;}
 const signature=cloud.stamp+':'+cloud.frame+':'+cloud.aligned;if(signature===lastStamp){draw();return;}lastStamp=signature;
 const valid=cloud.points_xyz.filter(p=>p.length===3&&p.every(Number.isFinite));
 if(cloud.aligned&&(cloud.frame==='map'||status===5&&cloud.frame===s.map?.frame)&&[5,8,1].includes(status)){
  if(modelFrame!==cloud.frame){model.clear();hits.clear();viewFitted=false;modelFrame=cloud.frame;}
  const batch=new Map();
  for(const p of valid){const key=p.map(v=>Math.floor(v/.05)).join(',');if(!batch.has(key))batch.set(key,p);}
  for(const [key,p] of batch){
   const n=Math.min(hits.get(key)||0,19),old=model.get(key);
   model.set(key,old?p.map((v,i)=>(old[i]*n+v)/(n+1)):p);hits.set(key,n+1);
   if(model.size>60000){const first=model.keys().next().value;model.delete(first);hits.delete(first);}
  }
  points=Array.from(model.values());frame=cloud.frame;label.textContent=`边走边更新 · ${points.length} 个体素 · ${frame} · 5厘米融合 · 灰雾表示未获深度表面观测`;
 }else if(model.size){points=Array.from(model.values());frame=modelFrame;label.textContent=`已保留 ${points.length} 个体素 · 当前帧等待坐标对齐，暂停累计。`;}
 else{
  // Optical axes (right, down, forward) -> local display (forward, left, up).
  // This is a single camera frame, never world-map fusion or robot pose.
  const optical=(cloud.source_frame||'').includes('optical');
  const displayFrame=optical?'local_forward_left_up_from_'+cloud.source_frame:cloud.source_frame;
  if(frame!==displayFrame)viewFitted=false;
  frame=displayFrame;robot=null;
  points=optical?valid.filter(p=>p[2]>=.15&&p[2]<=4).map(p=>[p[2],-p[0],-p[1]]):valid;
  label.textContent=`实时相机点云 · ${points.length} 点 · 未定位，显示相机局部坐标，不累计为地图`;
 }
 if(!viewFitted)bounds();draw();
};
setInterval(()=>{if(robot?.fresh&&Date.now()/1000>robot.expires){robot.fresh=false;robotStatus.textContent='机器人定位已过期；灰色标记为上次位置';draw();}},500);
draw();
})();
