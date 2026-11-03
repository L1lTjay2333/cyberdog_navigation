'use strict';
(()=>{
 const canvas=document.getElementById('depth-live'),label=document.getElementById('depth-live-status'),context=canvas.getContext('2d');
 let lastState=null,signature=null,previous=null,previousFrame=null;
 function unavailable(message){label.textContent=message;signature=null;previous=null;context.fillStyle='#404954';context.fillRect(0,0,canvas.width,canvas.height);}
 function render(s){
  const d=s?.depth_preview;if(!d||d.error){unavailable(d?.error?'深度输入异常：'+d.error:'尚未收到深度图；等待原厂相机输入');return;}
  // Compare sensor and emission times on the robot, then add PC transport age.
  const age=Number.isFinite(s.emitted_unix)&&Number.isFinite(s.local_received_unix)?
   Math.max(0,s.emitted_unix-d.received_unix)+Math.max(0,Date.now()/1000-s.local_received_unix):Date.now()/1000-d.received_unix;
  if(!s.offline&&(s.connection!=='connected'||!Number.isFinite(age)||age<-.2||age>2)){unavailable('深度数据已过期 / 连接中断，当前环境未知');return;}
  const {width:w,height:h}=d;
  if(!Number.isInteger(w)||!Number.isInteger(h)||w<1||h<1||w>160||h>120||d.encoding!=='uint16_le_mm'){unavailable('深度预览格式异常');return;}
  label.textContent=(s.offline?'离线回放 · ':'实时深度 · ')+w+'×'+h+' · 有效像素 '+(100*d.valid_fraction).toFixed(1)+'% · '+(s.offline?'原始时间 '+d.stamp:Math.max(0,age).toFixed(1)+'秒前')+' · 光轴深度，非机身净距';
  if(signature===d.stamp)return;
  try{
   const raw=atob(d.data_mm_b64);if(raw.length!==w*h*2)throw Error('length');
   const input=new Uint16Array(w*h),filtered=new Float32Array(w*h);
   for(let i=0;i<input.length;i++)input[i]=raw.charCodeAt(i*2)|(raw.charCodeAt(i*2+1)<<8);
   if(previousFrame!==d.source_frame||previous?.length!==input.length||d.stamp-signature>.6||d.stamp<=signature)previous=null;
   // Display only: never fill missing returns or average across depth edges.
   for(let y=0;y<h;y++)for(let x=0;x<w;x++){
    const i=y*w+x,z=input[i];if(z<150||z>4000)continue;
    const near=[],gate=Math.max(20,z*.015);
    for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){
     const xx=x+dx,yy=y+dy;if(xx<0||xx>=w||yy<0||yy>=h)continue;
     const v=input[yy*w+xx];if(v>=150&&v<=4000&&Math.abs(v-z)<=gate)near.push(v);
    }
    near.sort((a,b)=>a-b);const median=near[Math.floor(near.length/2)];
    filtered[i]=previous?.[i]>0&&Math.abs(previous[i]-median)<=gate?previous[i]*.55+median*.45:median;
   }
   previous=filtered;previousFrame=d.source_frame;
   if(canvas.width!==w||canvas.height!==h){canvas.width=w;canvas.height=h;}const image=context.createImageData(w,h);
   for(let i=0;i<w*h;i++){
    const mm=filtered[i],j=i*4;
    if(mm<150||mm>4000){image.data[j]=64;image.data[j+1]=73;image.data[j+2]=84;}
    else {const t=(mm-150)/3850;image.data[j]=255*(1-t);image.data[j+1]=210*(1-Math.abs(2*t-1));image.data[j+2]=255*t;}
    image.data[j+3]=255;
   }
   context.putImageData(image,0,0);signature=d.stamp;
  }catch(e){unavailable('深度预览数据损坏，当前环境未知');}
 }
 window.updateDepthLive=s=>{lastState=s;render(s);};
 setInterval(()=>{if(lastState)render(lastState);},500);
 unavailable('等待深度图；无返回区域显示灰色');
})();
