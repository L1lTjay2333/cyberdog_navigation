// Intercept every request. Verify controls without connecting or moving a robot.
const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const root=path.resolve(__dirname,'..');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true});
 try{
  const page=await browser.newPage({viewport:{width:1440,height:1100}}),calls=[],errors=[];
  page.on('pageerror',e=>errors.push(e.message));
  const now=Date.now()/1000;
  const grid={width:40,height:30,resolution:.1,frame:'laser_odom',origin:{x:0,y:0,qz:0,qw:1},received_unix:now,data:Array.from({length:1200},(_,i)=>i%40>=25?-1:0)};
  let exploration={status:'IDLE',message:'等待选点',execution_reason:'本模式只规划',execution_supported:false};
  let supported=false,executor={active:false,events:[]};
  await page.route('http://127.0.0.1:8765/**',async route=>{
   const req=route.request(),pathname=new URL(req.url()).pathname;
   if(req.method()==='POST'){
    const body=req.postDataJSON();calls.push({path:pathname,...body});
    if(pathname==='/exploration/target'){
     const target={x:body.x+(body.relative?.85:0),y:body.y+(body.relative?1.55:0),frame:'laser_odom'};
     exploration={status:'FRONTIER_ROUTE',target,start:{x:.85,y:1.55,frame:'laser_odom'},start_goal_distance_m:Math.hypot(target.x-.85,target.y-1.55),execution_supported:false,execution_reason:'建图导航互斥；只规划',message:'目标尚未知，先到观察边界',waypoint:{x:2.05,y:1.55,frame:'laser_odom'},preview:{frame:'laser_odom',points:[{x:.85,y:1.55},{x:2.05,y:1.55}]}};
    }else if(pathname==='/exploration/clear')exploration={status:'IDLE',message:'目标已清除',execution_reason:'只规划'};
    else if(pathname==='/exploration/start'||pathname==='/exploration/scan')executor={active:true,owner:body.owner,events:[{event:'READY',execute:true}]};
    else if(pathname==='/exploration/stop')executor={active:false,events:[{event:'STOP_SENT'}]};
    return route.fulfill({json:{ok:true,exploration}});
   }
   if(pathname==='/state')return route.fulfill({json:{connection:'connected',local_received_unix:Date.now()/1000,emitted_unix:Date.now()/1000,pose:{x:.85,y:1.55,frame:'laser_odom',received_unix:now},task:{status:5},map:grid,mapping:{active:false,map_name:'offline_mock',events:[]},navigation:{enabled:false,active:false,events:[]},exploration:{...exploration,execution_supported:supported,executor}}});
   if(pathname==='/manual_state')return route.fulfill({json:{connected:false,enabled:false}});
   const file=pathname==='/'?'navigation.html':pathname.slice(1);
   if(!['navigation.html','manual.js','cloud_view.js','unknown_goal.js'].includes(file))throw Error('Unmocked request: '+pathname);
   return route.fulfill({body:fs.readFileSync(path.join(root,file)),contentType:file.endsWith('.js')?'text/javascript':'text/html'});
  });
  await page.goto('http://127.0.0.1:8765/');
  await page.waitForFunction(()=>!document.querySelector('#unknown-set').disabled);
  await page.locator('#unknown-mode').check();
  await page.locator('#unknown-x').fill('3');await page.locator('#unknown-y').fill('0');
  await page.locator('#unknown-set').click();
  await page.waitForFunction(()=>document.querySelector('#unknown-distance').textContent.includes('3.00'));
  assert.ok(calls.some(c=>c.path==='/exploration/target'&&c.x===3&&c.relative===true));
  assert.equal(await page.locator('#unknown-execute').isDisabled(),true);
  assert.equal(await page.locator('#go').isDisabled(),true);
  await page.locator('#map').scrollIntoViewIfNeeded();
  const box=await page.locator('#map').boundingBox();
  await page.mouse.click(box.x+box.width*.8,box.y+box.height*.45);
  await page.waitForFunction(()=>!document.querySelector('#unknown-detail').textContent.includes('3.85'));
  assert.ok(calls.filter(c=>c.path==='/exploration/target').at(-1).relative===false,'Gray-map click creates an absolute unknown goal');
  assert.equal(calls.some(c=>c.path==='/goal'||c.path==='/navigate'||c.path==='/manual/control'),false);
  await page.locator('#unknown-clear').click();
  await page.waitForFunction(()=>document.querySelector('#unknown-distance').textContent.includes('未设置'));
  await page.locator('#unknown-x').fill('');await page.locator('#unknown-set').click();
  await page.waitForFunction(()=>document.querySelector('#unknown-status').textContent.includes('有效'));
  supported=true;
  await page.locator('#unknown-x').fill('2');await page.locator('#unknown-set').click();
  await page.waitForFunction(()=>!document.querySelector('#unknown-probe').disabled);
  assert.equal(await page.locator('#unknown-execute').isDisabled(),true,'Deployment alone must not auto-authorize motion');
  assert.equal(calls.some(c=>c.path==='/exploration/start'),false);
  await page.locator('#unknown-approve').check();
  await page.waitForFunction(()=>!document.querySelector('#unknown-execute').disabled);
  await page.locator('#unknown-execute').click();
  await page.waitForFunction(()=>!document.querySelector('#unknown-stop').disabled);
  await page.waitForTimeout(700);
  const start=calls.find(c=>c.path==='/exploration/start'),beat=calls.find(c=>c.path==='/exploration/heartbeat');
  assert.ok(start?.approved&&typeof start.owner==='string');assert.equal(beat?.owner,start.owner);
  assert.equal(await page.locator('#unknown-approve').isChecked(),false,'Each run consumes its confirmation');
  assert.equal(await page.locator('#unknown-set').isDisabled(),true,'Cannot retarget an active executor');
  await page.keyboard.press('Escape');
  await page.waitForFunction(()=>document.querySelector('#unknown-stop').disabled);
  assert.ok(calls.some(c=>c.path==='/exploration/stop'),'Escape cancels independent execution');
  await page.locator('#unknown-clear').click();
  await page.waitForFunction(()=>document.querySelector('#unknown-distance').textContent.includes('未设置'));
  assert.equal(await page.locator('#unknown-scan').isDisabled(),true,'Scan requires its own confirmation');
  await page.locator('#unknown-approve').check();
  await page.waitForFunction(()=>!document.querySelector('#unknown-scan').disabled);
  await page.locator('#unknown-scan').click();
  await page.waitForFunction(()=>!document.querySelector('#unknown-stop').disabled);
  assert.ok(calls.some(c=>c.path==='/exploration/scan'&&c.approved),'Scan needs no target or blocked-start bypass');
  await page.keyboard.press('Escape');
  await page.waitForFunction(()=>document.querySelector('#unknown-stop').disabled);
  assert.equal(calls.some(c=>c.path==='/navigate'||c.path==='/manual/control'),false);
  assert.deepEqual(errors,[]);
  await page.screenshot({path:path.join(root,'evidence/unknown_goal_browser_20261009.png'),fullPage:true});
  console.log(JSON.stringify({result:'PASS',relative_coordinates:true,gray_target:true,clear_target:true,explicit_execution_confirmation:true,owned_heartbeat:true,escape_cancels:true,approved_scan_without_target:true,page_errors:errors,robot_connections:0}));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1);});
