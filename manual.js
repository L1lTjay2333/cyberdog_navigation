'use strict';
const manualPanel=document.getElementById('manual-panel');
const manualElement=id=>document.getElementById((manualPanel?'control-':'')+id);
const manualButtons=()=> (manualPanel||document).querySelectorAll('[data-direction]');
const msg=manualElement('message'),feedback=manualElement('feedback');
let held='stop',live=false,enabled=false,pending=false,token=0;
let heldKey=null;
const pressedKeys=new Set();let heldAxes=null;
let keyboardFeedback='尚未收到方向键';
let localControlError='';
let directionBlocks={};
let anotherOwner=false;
function freshestBattery(...samples){
 const now=Date.now()/1000;
 return samples.filter(b=>b&&Number.isFinite(b.received_unix)&&Number.isFinite(b.percent)&&
  b.percent>=0&&b.percent<=100&&now-b.received_unix>=-.5&&now-b.received_unix<3)
  .sort((a,b)=>b.received_unix-a.received_unix)[0]||null;
}
const keyDirections={KeyW:'forward',KeyS:'backward',KeyA:'left',KeyD:'right',KeyJ:'shift_left',KeyK:'shift_right'};
const owner=crypto.randomUUID();let sequence=0;
async function manualPost(path,body){const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({...body,owner,sequence:++sequence})});const j=await r.json();if(!j.ok)throw Error(j.message);return j;}
function release(){pressedKeys.clear();held='stop';heldKey=null;heldAxes=null;token++;if(!live)return;manualPost('/manual/stop',{}).catch(e=>{localControlError=e.message;msg.textContent='停车请求未确认：'+e.message+'；必要时使用物理急停';});}
function setAxes(x,y){
 const norm=Math.max(1,Math.hypot(x,y));x/=norm;y/=norm;
 if(Math.abs(x)<.01)x=0;if(Math.abs(y)<.01)y=0;
 const direction=!x?(y>0?'left':y<0?'right':'stop'):(x>0?'forward':'backward')+(y>0?'_left':y<0?'_right':'');
 if(direction==='stop'){held='stop';heldAxes=null;token++;if(live)manualPost('/manual/stop',{}).catch(()=>{});return;}
 if(!live||!enabled||directionBlocks[direction]){localControlError=direction+' 未发送：'+(directionBlocks[direction]||'遥控未就绪');held='stop';heldAxes=null;return;}
 held=direction;heldAxes=[x,y];token++;
}
function updateKeys(){
 const x=Number(pressedKeys.has('KeyW'))-Number(pressedKeys.has('KeyS'));
 const yaw=Number(pressedKeys.has('KeyA'))-Number(pressedKeys.has('KeyD'));
 const side=Number(pressedKeys.has('KeyJ'))-Number(pressedKeys.has('KeyK'));
 if(!side){setAxes(x,yaw);return;}
 if(x||yaw){setAxes(0,0);localControlError='J/K 横移请单独使用；W/S+A/D 用于弧线行走';return;}
 const direction=side>0?'shift_left':'shift_right';
 if(!live||!enabled||directionBlocks[direction]){setAxes(0,0);localControlError=directionBlocks[direction]||'遥控未就绪';return;}
 held=direction;heldAxes=null;token++;
}
function focusControls(){
 const panel=manualElement('keyboard')||manualPanel||document.body;
 panel.tabIndex=0;panel.focus({preventScroll:true});
}
// A real focusable input lets the browser receive keys rather than relying on
// the embedding application's page-level keyboard shortcut forwarding.
const keyboardCapture=document.createElement('input');
keyboardCapture.id=(manualPanel?'control-':'')+'keyboard';
keyboardCapture.type='text';keyboardCapture.readOnly=true;
keyboardCapture.placeholder='点击启用键盘：WASD 转向行走 · J/K 横移';
keyboardCapture.setAttribute('aria-label','机器狗键盘控制区域');
keyboardCapture.style.cssText='box-sizing:border-box;width:100%;padding:14px;margin:8px 0;background:#142733;color:#eef5ff;border:2px solid #55d7c6;border-radius:8px;caret-color:transparent';
manualElement('connect').insertAdjacentElement('afterend',keyboardCapture);
const batteryLabel=document.createElement('p');batteryLabel.textContent='电量：等待实时反馈';
batteryLabel.id=(manualPanel?'control-':'')+'battery';
keyboardCapture.insertAdjacentElement('afterend',batteryLabel);
const postureButton=document.createElement('button');postureButton.textContent='空格：起立 / 趴下';
postureButton.className='wide';batteryLabel.insertAdjacentElement('afterend',postureButton);
postureButton.onclick=()=>{release();if(live)manualPost('/manual/posture',{}).catch(e=>{localControlError=e.message;});else localControlError='请先连接本页遥控';};
const controlNote=(manualPanel||document).querySelector?.('p.note');
if(controlNote)controlNote.textContent='W/S 前后；A/D 原地左旋/右旋；J/K 单独左移/右移。W+A/W+D 向左前/右前行走，S+A/S+D 向左后/右后倒车；摇杆横推旋转、斜推弧线行走。松开停车；空格起立/趴下，Esc 紧急停车。';
const joystick=document.createElement('canvas');joystick.width=180;joystick.height=180;
joystick.style.cssText='display:block;width:180px;height:180px;touch-action:none;border-radius:50%;margin:12px auto;background:#142733;border:1px solid #55d7c6';
joystick.setAttribute('aria-label','全方向遥控摇杆');batteryLabel.insertAdjacentElement('afterend',joystick);
const jc=joystick.getContext('2d');
function drawStick(x=0,y=0){jc.clearRect(0,0,180,180);jc.strokeStyle='#55d7c6';jc.beginPath();jc.arc(90,90,70,0,Math.PI*2);jc.stroke();jc.fillStyle='#55d7c6';jc.beginPath();jc.arc(90+x*65,90+y*65,20,0,Math.PI*2);jc.fill();}
let stickDown=false;
function moveStick(e){const r=joystick.getBoundingClientRect();let x=(e.clientX-r.left)/r.width*180-90,y=(e.clientY-r.top)/r.height*180-90;const n=Math.max(65,Math.hypot(x,y));x/=n;y/=n;drawStick(x,y);setAxes(-y,-x);}
joystick.onpointerdown=e=>{if(!live)return;e.preventDefault();release();stickDown=true;joystick.setPointerCapture(e.pointerId);moveStick(e);};
joystick.onpointermove=e=>{if(stickDown)moveStick(e);};
joystick.onpointerup=joystick.onpointercancel=joystick.onlostpointercapture=()=>{stickDown=false;drawStick();release();};drawStick();
keyboardCapture.addEventListener('focus',()=>{keyboardCapture.placeholder='键盘焦点已就绪：按住移动，松开停车';});
keyboardCapture.addEventListener('blur',()=>{keyboardCapture.placeholder='点击启用键盘：WASD 转向行走 · J/K 横移';release();});
// Mapping can request a connection, but never steal another page or send motion.
window.connectMappingControl=async()=>{
 if(anotherOwner)throw Error('其他网页持有遥控，请点击本页接管按钮');
 await manualPost('/manual/connect',{});localControlError='';
 const until=Date.now()+15000;
 while(Date.now()<until){
  await manualUpdate();
  if(live){focusControls();return;}
  if(anotherOwner)throw Error('其他网页持有遥控，请点击本页接管按钮');
  await new Promise(resolve=>setTimeout(resolve,200));
 }
 throw Error('建图已开启，遥控尚未就绪；请点击「连接建图遥控」重试连接');
};
manualElement('connect').onclick=async()=>{
 try{
  if(anotherOwner){
   // Explicit button click revokes the old page first, never copies its command.
   await manualPost('/manual/disconnect',{});
   msg.textContent='正在停止旧网页遥控，等待释放后连接本页…';
  }
  for(let i=0;i<30;i++){
   try{await manualPost('/manual/connect',{});localControlError='';focusControls();return;}
   catch(e){
    if(!anotherOwner||!e.message.includes('已有另一个网页'))throw e;
    await new Promise(resolve=>setTimeout(resolve,200));
   }
  }
  throw Error('旧会话尚未退出，请确认现场停车后重试连接');
 }catch(e){localControlError=e.message;msg.textContent=e.message;}
};
manualElement('disconnect').onclick=()=>{release();manualPost('/manual/disconnect',{}).catch(e=>msg.textContent=e.message);};
window.endManualControl=async()=>{
 held='stop';heldKey=null;heldAxes=null;pressedKeys.clear();token++;live=false;
 await manualPost('/manual/disconnect',{});
 for(let i=0;i<40;i++){
  const s=await(await fetch('/manual_state')).json();
  if(!s.connected&&!s.active)return;
  await new Promise(resolve=>setTimeout(resolve,100));
 }
 throw Error('遥控尚未完全停止，暂不切换任务');
};
manualElement('stop').onclick=release;
manualElement('stop').textContent='紧急停车（Esc）';
for(const p of (manualPanel||document).querySelectorAll('p')){
 if(p!==controlNote&&p.textContent.includes('空格')&&p.textContent.includes('停车'))p.textContent=controlNote?controlNote.textContent:'W/S 前后，A/D 原地旋转，J/K 横移；组合键及斜推摇杆弧线转向；空格起立/趴下，松开停车，Esc 紧急停车。';
}
for(const b of manualButtons()){
 b.onpointerdown=e=>{if(!live)return;e.preventDefault();if(heldKey){release();return;}b.setPointerCapture(e.pointerId);heldAxes=null;held=b.dataset.direction;token++;};
 b.onpointerup=release;b.onpointercancel=release;b.onlostpointercapture=()=>{if(held!=='stop')release();};
}
function directionCode(e){return keyDirections[e.code]?e.code:({w:'KeyW',a:'KeyA',s:'KeyS',d:'KeyD',j:'KeyJ',k:'KeyK'}[String(e.key||'').toLowerCase()]);}
window.addEventListener('keydown',e=>{
 const target=e.target;
 if(target!==keyboardCapture&&(target?.isContentEditable||['TEXTAREA','SELECT'].includes(target?.tagName)||(target?.tagName==='INPUT'&&!['checkbox','radio','button'].includes(target.type))))return;
 if(e.code==='Escape'){e.preventDefault();release();return;}
 if(e.code==='Space'||e.key===' '){
  e.preventDefault();if(e.repeat)return;release();
  keyboardFeedback='已收到空格：起立 / 趴下';
  localControlError='';
  if(live)manualPost('/manual/posture',{}).catch(e=>{localControlError=e.message;});
  else localControlError='空格未发送：请先连接本页遥控';
  return;
 }
 const code=directionCode(e);
 const direction=keyDirections[code];if(!direction)return;
 keyboardFeedback='已收到 '+code.replace('Key','')+(live?'':'；遥控连接未就绪');
 e.preventDefault();if(e.repeat||!live)return;
 if(!enabled||directionBlocks[direction]){
  localControlError=code.replace('Key','')+' 未发送：'+(directionBlocks[direction]||'运动权限未开放');
  msg.textContent=localControlError;return;
 }
 localControlError='';
 heldKey=code;pressedKeys.add(code);updateKeys();
},true);
window.addEventListener('keyup',e=>{const code=directionCode(e);if(keyDirections[code]){e.preventDefault();keyboardFeedback='已松开 '+code.replace('Key','');pressedKeys.delete(code);heldKey=pressedKeys.size?Array.from(pressedKeys)[0]:null;updateKeys();}},true);
window.addEventListener('blur',release);document.addEventListener('visibilitychange',()=>{if(document.hidden)release();});
window.addEventListener('pagehide',()=>{held='stop';fetch('/manual/stop',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({owner,sequence:++sequence}),keepalive:true}).catch(()=>{});});
setInterval(async()=>{if(held==='stop'||pending)return;pending=true;const seq=token;try{await manualPost('/manual/control',{direction:held,axes:heldAxes,armed:enabled});}catch(e){localControlError=e.message;msg.textContent=e.message;if(seq===token)release();}finally{pending=false;}},100);
let manualPolling=false;
async function manualUpdate(){if(manualPolling)return;manualPolling=true;try{
 const s=await(await fetch('/manual_state?owner='+encodeURIComponent(owner))).json();live=s.connected===true&&s.owns_session!==false;enabled=s.enabled===true;
 anotherOwner=s.connected===true&&s.owns_session===false;
 directionBlocks=s.direction_blocks||{};
 const battery=freshestBattery(s.battery,typeof latest!=='undefined'?latest?.battery:null),batteryFresh=!!battery;
 const charging=batteryFresh&&(battery.wired_charging||battery.wireless_charging);
 batteryLabel.textContent=batteryFresh?'电量：'+battery.percent+'%'+(charging?' · 充电中':' · 未充电')+' · 更新 '+Math.max(0,Date.now()/1000-battery.received_unix).toFixed(1)+' 秒前':'电量 / 充电状态：未知，等待新反馈';
 // Only clear an obsolete charging error, never motor/posture/stop failures.
 if(batteryFresh&&!charging&&s.motion_status?.switch_status!==14&&/充电/.test(localControlError))localControlError='';
 if(s.reason?.startsWith('保护停车')&&held!=='stop'){localControlError=s.reason;release();}
 const connect=manualElement('connect');connect.textContent=live?'本页已连接 · 点击聚焦键盘':anotherOwner?'停止其他页并接管本页':(manualPanel?'连接本页遥控':'连接遥控');
 const d=s.distance||{},metres=v=>Number.isFinite(v)?v.toFixed(3)+' 米':'无';
 manualElement('distance').textContent=d.valid?
 'TF/数据有效：是（不是测距标定通过）\n原始最近斜距：'+metres(d.raw_range_min_m)+
 '\n机身前向最小值：'+metres(d.forward_x_min_m)+'\n机身前向P10：'+metres(d.forward_x_p10_m)+
 '\n机身前向中位数：'+metres(d.forward_x_median_m)+'\nROI点数：'+d.roi_points+
 '\n坐标系：'+d.source_frame+' → '+d.target_frame:
 '当前统计无效：'+(d.reason||'等待实时扫描与TF');
 const depthMode=['depth_forward_trial','supervised_four_way_trial'].includes(s.sensor_mode);
 msg.textContent=localControlError||(!live?(s.connected?'其他网页正在遥控，请在原网页断开后连接本页':'点击“连接遥控”以查看实时状态'):enabled?'W/S前后 · A/D原地旋转 · J/K横移，松开停车':'已连接，等待开放本次实机运动权限');
 msg.textContent+='；键盘：'+keyboardFeedback;
 for(const b of manualButtons()){
  const block=s.direction_blocks?.[b.dataset.direction]||(s.sensor_mode==='depth_forward_trial'&&b.dataset.direction!=='forward'?'当前仅开放前进':'');
  b.disabled=!live||!enabled||!!block;b.title=block;
 }
 feedback.textContent='请求方向：'+(s.requested_direction||'stop')+'\n实际发送运动指令：'+(s.active?'是':'否')+'\n当前保护传感器：'+(depthMode?'深度相机':'雷达')+'\n前方距离：'+(depthMode&&!s.depth_valid?'当前无有效距离':metres(depthMode?s.depth_min:s.front_min))+'\n保护原因：'+(s.reason||s.forward_block_reason||'无')+'\n深度检查：'+(s.depth_valid?'通过':s.depth_invalid_reason||'等待数据')+'\n运动状态：'+JSON.stringify(s.motion_status||{})+'\n算法任务：'+s.task;
 if(!live&&held!=='stop')release();
 feedback.textContent+='\n键盘：'+keyboardFeedback+'\n四向条件：'+JSON.stringify(s.direction_blocks||{});
 feedback.textContent+='\n请求速度 [前后, 横移, 转向]：'+JSON.stringify(s.requested_velocity||[0,0,0])+'\n深度来源：'+(s.depth_source||'等待数据')+'\n最近深度帧：'+(Number.isFinite(s.depth_frame_age_s)?s.depth_frame_age_s.toFixed(2)+' 秒前':'尚未收到')+'\n最近有效测距：'+(Number.isFinite(s.depth_age_s)?s.depth_age_s.toFixed(2)+' 秒前':'尚未通过');
 feedback.textContent+='\n操控方式：W/S前后，A/D原地旋转，组合弧线；J/K单独横移。'+(s.camera_start_message?'\n相机启动：'+s.camera_start_message:'');
 feedback.textContent+='\n电池反馈来源：'+(s.battery_source||'实时BMS');
 if(batteryFresh&&!charging&&s.motion_status?.switch_status===14){
  const waiting='电池已反馈未充电，但原厂运动状态仍为14；等待运动状态自动恢复，不是网页电量缓存。';
  msg.textContent=waiting;feedback.textContent+='\n'+waiting;
 }
 if(Number.isFinite(s.front_stop_clearance_m))feedback.textContent+='\n前沿估算净距：'+(Number.isFinite(s.front_clearance_m)?(s.front_clearance_m*100).toFixed(1)+' 厘米':'无有效距离')+'；锁定阈值：'+(s.front_stop_clearance_m*100).toFixed(0)+' 厘米（基于配置轮廓）\n接近障碍限速后指令：'+JSON.stringify(s.limited_velocity||[0,0,0]);
 if(s.sit_message)feedback.textContent+='\n姿态切换：'+s.sit_message;
 }catch(e){live=false;enabled=false;for(const b of manualButtons())b.disabled=true;if(held!=='stop')release();msg.textContent='网页服务连接中断';}finally{manualPolling=false;}}
manualUpdate();setInterval(manualUpdate,250);
