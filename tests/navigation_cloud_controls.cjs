const fs=require('fs'),assert=require('assert/strict');
const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
(async()=>{
 const b=await chromium.launch({channel:'msedge',headless:true});
 try{
  const p=await b.newPage();await p.setContent('<div id="scene"><div class="cloudbuttons"></div></div><p id="cloudstatus"></p><canvas id="cloud" width="900" height="600" style="width:450px;height:300px"></canvas>'+['clearcloud','fitcloud','levelcloud','topcloud','exportcloud'].map(id=>`<button id="${id}"></button>`).join(''));
  await p.addScriptTag({content:fs.readFileSync('cloud_view.js','utf8')});
  await p.evaluate(()=>{const now=Date.now()/1000;window.fixture={started_unix:1,connection:'connected',local_received_unix:now,emitted_unix:100.1,task:{status:5},map:{frame:'laser_odom'},pose:{frame:'laser_odom',x:1,y:2,yaw:0,received_unix:100},cloud:{stamp:100,received_unix:100,frame:'laser_odom',source_frame:'camera_depth_optical_frame',aligned:true,points_xyz:[[1,2,1],[2,3,1]]}};window.updateCloud(window.fixture);});
  assert((await p.locator('#cloud-robot-status').textContent()).includes('机器狗实时位置'));
  const pic=()=>p.locator('canvas').evaluate(c=>c.toDataURL());const before=await pic();
  const box=await p.locator('canvas').boundingBox();await p.mouse.move(box.x+200,box.y+120);await p.mouse.down();await p.mouse.move(box.x+250,box.y+150);await p.mouse.up();assert.notEqual(await pic(),before);
  await p.getByText('拖动：平移',{exact:true}).click();assert(await p.getByText('拖动：旋转',{exact:true}).count());
  await p.getByText('跟随机器狗：关',{exact:true}).click();assert(await p.getByText('跟随机器狗：开',{exact:true}).count());
  await p.evaluate(()=>window.updateCloud({...window.fixture,pose:{...window.fixture.pose,frame:'map'}}));assert((await p.locator('#cloud-robot-status').textContent()).includes('尚无'));
  await p.evaluate(()=>window.updateCloud({...window.fixture,local_received_unix:Date.now()/1000}));await p.waitForTimeout(2700);assert((await p.locator('#cloud-robot-status').textContent()).includes('过期'));
  console.log('PASS CSS-scaled panning, mode switch, follow toggle, mapping pose display, frame mismatch rejection, stale marker');
 }finally{await b.close();}
})().catch(e=>{console.error(e);process.exit(1)});
