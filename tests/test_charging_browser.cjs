// Entire browser transport mocked. No robot commands or connections.
const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const root=path.resolve(__dirname,'..');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true});
 try{
  const page=await browser.newPage({viewport:{width:1440,height:1100}}),calls=[],errors=[];
  let charging=true,staleManual=false,nativeLatch=false;
  page.on('pageerror',e=>errors.push(e.message));
  await page.route('http://127.0.0.1:8765/**',async route=>{
   const req=route.request(),url=new URL(req.url()),now=Date.now()/1000;
   const battery={percent:45,wired_charging:charging,wireless_charging:false,received_unix:now};
   if(req.method()==='POST'){
    calls.push({path:url.pathname,...req.postDataJSON()});
    return route.fulfill({json:{ok:true,message:'mock'}});
   }
   if(url.pathname==='/manual_state')return route.fulfill({json:{connected:true,owns_session:true,enabled:true,
    battery:staleManual?{...battery,wired_charging:true,received_unix:now-10}:battery,
    battery_source:'passive_bms',motion_status:{motion_id:112,switch_status:nativeLatch?14:0},
    sensor_mode:'supervised_four_way_trial',depth_valid:true,depth_min:3,
    direction_blocks:charging?{forward:'机器狗正在充电，不发送运动'}:nativeLatch?{forward:'机器狗反馈充电状态，请等待'}:{}}});
   if(url.pathname==='/state')return route.fulfill({json:{connection:'connected',local_received_unix:now,emitted_unix:now,
    battery,task:{status:101},navigation:{enabled:true,active:false},mapping:{active:false,events:[]},map:null}});
   const file=url.pathname==='/'?'navigation.html':url.pathname.slice(1);
   if(!['navigation.html','manual.js','cloud_view.js','unknown_goal.js'].includes(file))return route.abort();
   return route.fulfill({body:fs.readFileSync(path.join(root,file)),contentType:file.endsWith('.js')?'text/javascript':'text/html'});
  });
  await page.goto('http://127.0.0.1:8765/');
  await page.waitForFunction(()=>document.querySelector('#control-battery').textContent.includes('充电中'));
  await page.locator('#control-keyboard').focus();await page.keyboard.press('w');
  await page.waitForFunction(()=>document.querySelector('#control-message').textContent.includes('充电'));
  charging=false;staleManual=true;
  await page.waitForFunction(()=>document.querySelector('#control-battery').textContent.includes('未充电')&&
   !document.querySelector('#control-message').textContent.includes('充电'));
  assert.ok((await page.locator('#nav-battery').innerText()).includes('未充电'));
  assert.equal(calls.some(c=>c.path==='/manual/control'&&c.armed),false,'Unplugging does not resume held movement');
  await page.locator('#control-keyboard').focus();await page.keyboard.down('w');
  await page.waitForTimeout(220);await page.keyboard.up('w');
  assert.ok(calls.some(c=>c.path==='/manual/control'&&c.armed&&c.direction==='forward'));
  staleManual=false;nativeLatch=true;
  await page.waitForFunction(()=>document.querySelector('#control-message').textContent.includes('仍为14'));
  assert.equal(await page.locator('[data-direction="forward"]').isDisabled(),true,'Never bypass actual native charging state');
  assert.deepEqual(errors,[]);
  console.log(JSON.stringify({result:'PASS',fresh_passive_bms_over_stale_manual:true,charging_error_cleared:true,
   unplug_without_reload:true,no_auto_movement:true,native_14_preserved:true,robot_connections:0}));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1);});
