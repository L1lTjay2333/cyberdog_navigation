// Executes ONE already-previewed real-robot route, never retries.
'use strict';
const fs=require('fs'),path=require('path');
const root=path.resolve(__dirname,'..');
const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const scope=JSON.parse(fs.readFileSync(path.join(root,'config/scope.json'),'utf8'));
if(!process.argv.includes('--execute-approved-route')||scope.motion_enabled!==true)throw Error('Explicit supervised navigation approval required');
const record={kind:'REAL_MAIN_PAGE_NAVIGATION',started_unix:Date.now()/1000,samples:[],physical_confirmation:'PENDING'};
let browser,page,started=false;
async function state(){return page.evaluate(async()=>await(await fetch('/state')).json());}
function compact(s){return {unix:Date.now()/1000,task:s.task,pose:s.pose,decision:s.decision,navigation:s.navigation,cloud:s.cloud?{received_unix:s.cloud.received_unix,frame:s.cloud.frame,points:s.cloud.points}:null};}
async function cancel(){await page.locator('#cancel').click({timeout:2000}).catch(()=>{});}
(async()=>{
 try{
  browser=await chromium.launch({channel:'msedge',headless:true});page=await browser.newPage();page.setDefaultTimeout(15000);
  await page.goto('http://127.0.0.1:8765/');await page.waitForFunction(()=>!document.querySelector('#go').disabled);
  let s=await state();
  if(s.navigation.active||s.task?.status!==8||!s.selection||!s.decision?.can_start)throw Error('Preview/localization not ready');
  if(Date.now()/1000-(s.cloud?.received_unix||0)>2||!(s.cloud?.points>0))throw Error('No fresh depth cloud; no navigation submitted');
  record.goal=s.selection.goal;record.preview=s.selection.preview;record.samples.push(compact(s));
  await page.locator('#go').click();
  // Wait for the single navigation POST preparation, without clicking again.
  await page.waitForFunction(()=>document.querySelector('#message').textContent.includes('目标已提交')||!document.querySelector('#message').textContent.includes('目标已提交')&&document.querySelector('#go').disabled===false,{},{timeout:20000});
  s=await state();started=s.navigation.active;
  if(!started)throw Error('Navigation was not started: '+await page.locator('#message').innerText());
  record.samples.push(compact(s));let lastStage='';const deadline=Date.now()+60000;
  while(Date.now()<deadline){
   await page.waitForTimeout(500);s=await state();record.samples.push(compact(s));
   if(s.decision?.stage!==lastStage){lastStage=s.decision?.stage;console.log(JSON.stringify({stage:lastStage,task:s.task?.status,distance:s.decision?.distance_m}));}
   if(!s.navigation.active){
    record.result=s.decision?.stage||'UNKNOWN';record.final=compact(s);break;
   }
   const sent=s.navigation.events.some(e=>e.event==='SEND_ONCE');
   if(sent&&Date.now()/1000-(s.cloud?.received_unix||0)>2){await cancel();throw Error('Depth cloud became stale after goal submission; cancellation requested');}
  }
  if(!record.result){await cancel();throw Error('60-second supervised test limit reached; cancellation requested');}
  if(record.result!=='ARRIVED')throw Error('Navigation did not verify arrival: '+record.result);
  await page.screenshot({path:path.join(root,'evidence/live_navigation_main_20261009.png'),fullPage:true});
  console.log(JSON.stringify({result:record.result,goal:record.goal,distance:record.final.decision.distance_m}));
 }catch(e){record.result=record.result||'FAILED';record.error=e.message;console.error(e.message);process.exitCode=1;}
 finally{
  if(page&&started){try{const s=await state();if(s.navigation.active){await cancel();for(let i=0;i<16;i++){await page.waitForTimeout(500);const latest=await state();record.samples.push(compact(latest));if(!latest.navigation.active)break;}}record.final=compact(await state());}catch(e){record.stop_check_error=e.message;}}
  record.ended_unix=Date.now()/1000;fs.writeFileSync(path.join(root,'evidence/live_navigation_20261009.json'),JSON.stringify(record,null,2));
  if(browser)await browser.close();
 }
})();
