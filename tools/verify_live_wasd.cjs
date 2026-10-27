// REAL ROBOT movement: operator must confirm four-way clearance and emergency stop.
'use strict';
const fs=require('fs'),path=require('path');
const root=path.resolve(__dirname,'..');
const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const scope=JSON.parse(fs.readFileSync(path.join(root,'config/scope.json'),'utf8'));
if(!process.argv.includes('--execute-short-motion')||!scope.manual_motion_enabled||scope.motion_enabled||scope.manual_sensor_mode!=='supervised_four_way_trial')throw Error('Explicit supervised short-motion mode required');
const evidence={started_unix:Date.now()/1000,kind:'REAL_MAIN_PAGE_WASD',duration_limit_s:1,results:[],physical_confirmation:'PENDING'};
let browser,page;
async function readState(){return page.evaluate(async()=>({web:await(await fetch('/state')).json(),manual:await(await fetch('/manual_state')).json()}));}
function compact(s){return {unix:Date.now()/1000,active:s.manual.active,direction:s.manual.direction,requested:s.manual.requested_direction,reason:s.manual.reason,velocity:s.manual.requested_velocity,motion_status:s.manual.motion_status,odom:s.web.odom,pose:s.web.pose,depth_min:s.manual.depth_min,depth_valid:s.manual.depth_valid};}
(async()=>{
 try{
  browser=await chromium.launch({channel:'msedge',headless:true});
  page=await browser.newPage();page.setDefaultTimeout(12000);
  await page.goto('http://127.0.0.1:8765/');
  await page.locator('#control-connect').click();
  await page.waitForFunction(()=>document.querySelector('#control-message')?.textContent.includes('松开停车'));
  // Connection acknowledgement can arrive before the first MotionStatus/TF.
  // Wait without motion for the same ready button the operator would see.
  await page.waitForFunction(()=>!document.querySelector('#manual-panel [data-direction="forward"]').disabled);
  for(const [key,direction] of [['w','forward'],['s','backward'],['a','shift_left'],['d','shift_right']]){
   const before=await readState();
   if(!before.manual.connected||!before.manual.enabled||before.manual.active||before.manual.direction_blocks?.[direction])throw Error('Not ready for '+direction+': '+JSON.stringify(before.manual.direction_blocks));
   if(before.web.navigation.active||before.manual.task!==5||!before.web.mapping.events.some(e=>e.event==='SCANNING'))throw Error('Expected computer-owned mapping trial');
   const row={key,direction,before:compact(before),samples:[]};evidence.results.push(row);
   // Fixed 1-second release timer is independent of slow HTTP sampling.
   let released=false;
   await page.keyboard.down(key);
   const timer=setTimeout(()=>{released=true;page.keyboard.up(key).catch(()=>{});},1000);
   try{
    for(let i=0;i<5&&!released;i++){
     await page.waitForTimeout(120);const current=await readState();row.samples.push(compact(current));
     if(current.manual.reason&&current.manual.active)throw Error('Active safety condition');
    }
    if(!released)await page.waitForTimeout(1000);
   }finally{clearTimeout(timer);await page.keyboard.up(key);}
   await page.waitForTimeout(700);const after=await readState();row.after=compact(after);
   if(after.manual.active||after.manual.requested_direction!=='stop')throw Error('Stop not confirmed; use physical emergency stop');
   row.sent=row.samples.some(s=>s.active&&s.direction===direction);
   if(!row.sent)throw Error('No active movement feedback for '+direction+'; no repeat');
   console.log(JSON.stringify({key,direction,sent:row.sent,stopped:!after.manual.active}));
  }
  evidence.result='REQUEST_AND_STOP_FEEDBACK_PASSED';
  await page.screenshot({path:path.join(root,'evidence/live_wasd_main_20261009.png'),fullPage:true});
 }catch(e){evidence.result='FAILED';evidence.error=e.message;console.error(e.message);process.exitCode=1;}
 finally{
  if(page){await page.keyboard.up('w').catch(()=>{});await page.keyboard.up('s').catch(()=>{});await page.keyboard.up('a').catch(()=>{});await page.keyboard.up('d').catch(()=>{});await page.keyboard.press('Escape').catch(()=>{});await page.locator('#control-disconnect').click({timeout:3000}).catch(()=>{});await page.waitForTimeout(500).catch(()=>{});}
  evidence.ended_unix=Date.now()/1000;
  fs.writeFileSync(path.join(root,'evidence/live_wasd_20261009.json'),JSON.stringify(evidence,null,2));
  if(browser)await browser.close();
 }
})();
