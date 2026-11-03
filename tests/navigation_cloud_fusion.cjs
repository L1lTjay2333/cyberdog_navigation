const fs=require('fs'),assert=require('assert/strict');
const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
(async()=>{
 const b=await chromium.launch({channel:'msedge',headless:true});
 try{
  const p=await b.newPage();await p.setContent('<div id="scene"><div class="cloudbuttons"></div></div><p id="cloudstatus"></p><canvas id="cloud" width="900" height="600"></canvas>'+['clearcloud','fitcloud','levelcloud','topcloud','exportcloud'].map(id=>`<button id="${id}"></button>`).join(''));
  await p.addScriptTag({content:fs.readFileSync('cloud_view.js','utf8')});
  const send=async(stamp,pts,opts={})=>p.evaluate(({stamp,pts,opts})=>{
   const now=Date.now()/1000;window.updateCloud({started_unix:1,connection:'connected',local_received_unix:now,emitted_unix:stamp+.1,task:{status:8},cloud:{stamp,received_unix:stamp,frame:'map',source_frame:'camera_depth_optical_frame',aligned:true,points_xyz:pts,...opts}});
  },{stamp,pts,opts});
  const text=()=>p.locator('#cloudstatus').textContent();
  await send(100,[[1,0,1],[1.01,0,1],[2,0,1]]);assert((await text()).includes('2 个体素'));
  await send(101,[[1.02,0,1],[3,0,1]]);assert((await text()).includes('3 个体素'));
  // Repeated sensor timestamp must not add a new point.
  await send(101,[[9,0,1]]);assert((await text()).includes('3 个体素'));
  await send(102,[[9,0,1]],{aligned:false,frame:'camera_depth_optical_frame'});assert((await text()).includes('已保留 3'));
  await send(100,[[9,0,1]],{stamp:90});assert((await text()).includes('过期'));
  await p.getByText('迷雾：开',{exact:true}).click();assert(await p.getByText('迷雾：关',{exact:true}).count());
  await p.click('#clearcloud');await send(104,[[1,0,1]],{aligned:false,frame:'camera_depth_optical_frame'});assert((await text()).includes('实时相机点云'));
  console.log('PASS voxel deduplication, accumulation, duplicate/stale frame rejection, alignment loss retention, fog toggle, local preview');
 }finally{await b.close();}
})().catch(e=>{console.error(e);process.exit(1)});
