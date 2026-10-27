/* Owned exploration preview/execution and explicitly approved in-place scan. */
(()=>{
 'use strict';
 const get=id=>document.getElementById(id);let state=null,pending=false,owned=false;
 const owner=crypto.randomUUID();
 async function post(path,body){
  const response=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  const result=await response.json();if(!response.ok||!result.ok)throw Error(result.message||'规划请求失败');return result;
 }
 const number=id=>{const value=get(id).value.trim();if(!value||!Number.isFinite(Number(value)))throw Error('请填写有效的 X、Y 数值');return Number(value);};
 async function select(target){
  if(pending)return;pending=true;get('unknown-status').textContent='根据当前扫描计算观察边界与绕障路线…';
  try{const result=await post('/exploration/target',target);render(result.exploration);}
  catch(error){get('unknown-status').textContent=error.message;}
  finally{pending=false;}
 }
 function render(report){
  if(!report)return;
  const distance=report.start_goal_distance_m;
  get('unknown-distance').textContent=Number.isFinite(distance)?`起点 → 终点直线距离：${distance.toFixed(2)} 米`:'起点 → 终点直线距离：未设置';
  const executor=report.executor||{},event=executor.events?.at(-1),error=executor.events?.findLast(e=>e.event==='ERROR');
  get('unknown-status').textContent=executor.active?`独立执行器：${event?.event||'连接中'} · ${event?.message||report.message}`:`${report.message}。${report.execution_reason}${error?`；执行失败：${error.message}`:event?`；最近结果：${event.event} ${event.message||''}`:''}`;
  const goal=report.target,next=report.waypoint;
  get('unknown-detail').textContent=goal?`目标 (${goal.x.toFixed(2)}, ${goal.y.toFixed(2)}) · ${goal.frame}${next?`；本段观察点 (${next.x.toFixed(2)}, ${next.y.toFixed(2)})`:''}`:'';
  const live=state?.connection==='connected'&&Date.now()/1000-(state?.local_received_unix||0)<3&&!state?.offline;
  get('unknown-execute').disabled=!live||!report.execution_supported||executor.active||pending||state?.task?.status!==5||!['GOAL_ROUTE','FRONTIER_ROUTE'].includes(report.status)||!get('unknown-approve').checked;
  get('unknown-probe').disabled=!live||!report.execution_supported||executor.active||pending||state?.task?.status!==5;
  get('unknown-scan').disabled=!live||!report.execution_supported||executor.active||pending||state?.task?.status!==5||!get('unknown-approve').checked;
  get('unknown-stop').disabled=!executor.active;
  get('unknown-clear').disabled=!goal||pending||executor.active;
 }
 window.unknownGoalEnabled=()=>get('unknown-mode').checked;
 window.selectUnknownGoal=target=>select({...target,relative:false});
 window.updateExploration=s=>{
  state=s;render(s.exploration);
  const valid=!!s.map&&!!s.pose&&(s.offline||(s.connection==='connected'&&Date.now()/1000-(s.local_received_unix||0)<3));
  get('unknown-set').disabled=!valid||pending||!!s.exploration?.executor?.active;
 };
 get('unknown-set').onclick=()=>{try{select({x:number('unknown-x'),y:number('unknown-y'),relative:get('unknown-relative').checked});}catch(e){get('unknown-status').textContent=e.message;}};
 get('unknown-clear').onclick=async()=>{try{await post('/exploration/clear',{});render({status:'IDLE',message:'目标已清除',execution_reason:'本模式只规划，不自动行走'});}catch(e){get('unknown-status').textContent=e.message;}};
 get('unknown-mode').onchange=()=>{if(get('unknown-mode').checked){get('clickmove').checked=false;get('clickmove').disabled=true;get('go').disabled=true;}};
 get('unknown-approve').onchange=()=>render(state?.exploration);
 for(const [id,path] of [['unknown-execute','/exploration/start'],['unknown-probe','/exploration/probe'],['unknown-scan','/exploration/scan']])get(id).onclick=async()=>{
  if(pending)return;pending=true;
  try{
   if(window.endManualControl)await window.endManualControl();
   const result=await post(path,{owner,approved:get('unknown-approve').checked});
   owned=true;get('unknown-status').textContent=result.message;
  }catch(e){get('unknown-status').textContent=e.message;}finally{pending=false;get('unknown-approve').checked=false;}
 };
 async function stop(){owned=false;try{const result=await post('/exploration/stop',{});get('unknown-status').textContent=result.message;}catch(e){get('unknown-status').textContent=e.message;}}
 get('unknown-stop').onclick=stop;
 setInterval(async()=>{if(!owned||document.hidden)return;try{await post('/exploration/heartbeat',{owner});}catch(e){owned=false;}},400);
 window.addEventListener('keydown',event=>{if(event.code==='Escape'&&state?.exploration?.executor?.active)stop();},true);
 document.addEventListener('visibilitychange',()=>{if(document.hidden&&owned)stop();});
 window.addEventListener('pagehide',()=>{if(owned){owned=false;fetch('/exploration/stop',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}',keepalive:true}).catch(()=>{});}});
 window.drawExploration=(context,grid,xy)=>{
  if(!get('unknown-mode').checked)return;
  const report=state?.exploration;if(!report?.target||report.target.frame!==grid.frame)return;
  context.save();
  const start=xy(report.start,grid),goal=xy(report.target,grid);
  context.setLineDash([2,6]);context.strokeStyle='#57a6e8';context.lineWidth=2;
  context.beginPath();context.moveTo(...start);context.lineTo(...goal);context.stroke();
  const route=report.preview;
  if(route?.frame===grid.frame&&route.points?.length){
   context.setLineDash([6,4]);context.strokeStyle='#e9c465';context.lineWidth=3;context.beginPath();
   route.points.forEach((point,index)=>{const p=xy(point,grid);if(index)context.lineTo(...p);else context.moveTo(...p);});context.stroke();
  }
  context.setLineDash([]);context.strokeStyle='#ffab38';context.beginPath();context.arc(...goal,10,0,Math.PI*2);context.stroke();
  if(report.waypoint){const next=xy(report.waypoint,grid);context.fillStyle='#e9c465';context.fillRect(next[0]-4,next[1]-4,8,8);}
  context.restore();
 };
})();
