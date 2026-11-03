const fs=require('fs'),assert=require('assert/strict');
const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
(async()=>{
 const b=await chromium.launch({channel:'msedge',headless:true});const p=await b.newPage({viewport:{width:1100,height:800}});
 const depth=JSON.parse(fs.readFileSync('evidence/navigation_depth_replay.json','utf8'));
 await p.setContent('<h1>导航深度显示 · 真实保存帧回放</h1><p id="depth-live-status"></p><canvas id="depth-live" width="160" height="120" style="width:640px;height:480px;image-rendering:pixelated"></canvas>');
 await p.addScriptTag({content:fs.readFileSync('depth_live.js','utf8')});
 await p.evaluate(d=>window.updateDepthLive({offline:true,connection:'offline',depth_preview:d}),depth);
 assert((await p.locator('#depth-live-status').textContent()).includes('离线回放'));
 const colors=await p.locator('canvas').evaluate(c=>new Set(c.getContext('2d').getImageData(0,0,c.width,c.height).data).size);assert(colors>10);
 await p.screenshot({path:'evidence/navigation_depth_replay.png'});
 await p.evaluate(d=>window.updateDepthLive({connection:'connected',depth_preview:{...d,received_unix:Date.now()/1000}}),depth);
 await p.waitForTimeout(2600);assert((await p.locator('#depth-live-status').textContent()).includes('已过期'));
 await p.evaluate(d=>window.updateDepthLive({connection:'connected',depth_preview:{...d,received_unix:Date.now()/1000,data_mm_b64:'AA=='}}),depth);
 assert((await p.locator('#depth-live-status').textContent()).includes('损坏'));
 // Robot clock offset must not blank fresh imagery. Missing returns remain gray.
 await p.evaluate(()=>{
  const packet=(stamp,z)=>({connection:'connected',emitted_unix:stamp+.1,local_received_unix:Date.now()/1000,
   depth_preview:{width:1,height:1,encoding:'uint16_le_mm',stamp,received_unix:stamp,source_frame:'optical',valid_fraction:1,
    data_mm_b64:btoa(String.fromCharCode(z&255,z>>8))}});
  window.updateDepthLive(packet(100,1000));window.updateDepthLive(packet(100.25,1010));
 });
 assert(!(await p.locator('#depth-live-status').textContent()).includes('过期'));
 await p.evaluate(()=>window.updateDepthLive({offline:true,depth_preview:{width:1,height:1,encoding:'uint16_le_mm',stamp:100.5,source_frame:'optical',valid_fraction:0,data_mm_b64:'AAA='}}));
 assert.deepEqual(await p.locator('canvas').evaluate(c=>Array.from(c.getContext('2d').getImageData(0,0,1,1).data)),[64,73,84,255]);
 console.log('PASS real frame rendering, fresh/stale transition, malformed packet handling');await b.close();
})().catch(e=>{console.error(e);process.exit(1)});
