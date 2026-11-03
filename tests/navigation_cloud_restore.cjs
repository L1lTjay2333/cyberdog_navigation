const fs=require('fs'),assert=require('assert/strict');
const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
(async()=>{
 const b=await chromium.launch({channel:'msedge',headless:true});
 try{
  const p=await b.newPage();const html='<div id="scene"><div class="cloudbuttons"></div></div><p id="cloudstatus"></p><canvas id="cloud" width="900" height="600"></canvas>'+['clearcloud','fitcloud','levelcloud','topcloud','exportcloud'].map(id=>`<button id="${id}"></button>`).join('');
  await p.route('http://127.0.0.1:8765/**',route=>route.fulfill({contentType:route.request().url().endsWith('/cloud/saved')?'application/json':'text/html',body:route.request().url().endsWith('/cloud/saved')?JSON.stringify({key:'room:laser_odom',frame:'laser_odom',points_xyz:[[1,2,3],[4,5,6]]}):html}));
  for(let i=0;i<2;i++){
   await p.goto('http://127.0.0.1:8765/');await p.addScriptTag({content:fs.readFileSync('cloud_view.js','utf8')});
   await p.evaluate(()=>window.updateCloud({started_unix:1,task:{status:5},mapping:{map_name:'room'},cloud_archive:{key:'room:laser_odom',points:2}}));
   await p.waitForTimeout(100);
   await p.evaluate(()=>window.updateCloud({started_unix:1,task:{status:5},connection:'connected',local_received_unix:Date.now()/1000,emitted_unix:100.1,mapping:{map_name:'room'},cloud_archive:{key:'room:laser_odom',points:2},cloud:{aligned:false,frame:'camera',source_frame:'camera',stamp:100,points_xyz:[[0,0,1]]}}));
   assert((await p.locator('#cloudstatus').textContent()).includes('已保留 2'));
  }
  console.log('PASS saved map restores after page navigation and survives unaligned incoming frames');
 }finally{await b.close();}
})().catch(e=>{console.error(e);process.exit(1)});
