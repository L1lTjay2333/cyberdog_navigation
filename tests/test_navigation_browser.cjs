// All URLs intercepted; real browser but ZERO robot connections.
const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const root=path.resolve(__dirname,'..');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true});
 const page=await browser.newPage({viewport:{width:1400,height:1100}});
 let selection=null,previewPending=false,active=false,events=[],stateInflight=0,stateMax=0,charging=false,distanceLimit=2.8;
 const calls=[],errors=[];page.on('pageerror',e=>errors.push(e.message));
 const grid={width:200,height:200,resolution:.05,frame:'map',origin:{x:-5,y:-5,qz:0,qw:1},received_unix:1,data:Array(40000).fill(0)};
 await page.route('http://127.0.0.1:8765/**',async route=>{
  const req=route.request(),url=new URL(req.url());
  if(req.method()==='POST'){
   const body=req.postDataJSON();calls.push({path:url.pathname,...body});
   if(url.pathname==='/goal'){
    previewPending=true;await new Promise(r=>setTimeout(r,650));previewPending=false;
    selection={goal:{...body,frame:'map'},preview:{frame:'map',kind:'stock_nav2_preview',length_m:1,points:[{x:0,y:0},body]}};
    return route.fulfill({json:{ok:true,...selection}});
   }
   if(url.pathname==='/navigate')active=true;
   return route.fulfill({json:{ok:true,message:'mocked'}});
  }
  if(url.pathname==='/state'){
   stateInflight++;stateMax=Math.max(stateMax,stateInflight);
   await new Promise(r=>setTimeout(r,180));stateInflight--;
   const now=Date.now()/1000;
   return route.fulfill({json:{connection:'connected',local_received_unix:now,emitted_unix:now-10,
    task:{status:8,received_unix:now-10},pose:{x:0,y:0,yaw:0,frame:'map',received_unix:now-10},
    trajectory:{frame:'map',distance_m:1,points:[{x:-1,y:0,break:true},{x:0,y:0}]},
    marker:{x:300,y:300,angle:0},display_map_revision:'test:1',map:url.searchParams.get('map_revision')==='test:1'?{...grid,data:undefined}:grid,selection,
    battery:{percent:9,wired_charging:charging,received_unix:now},
    navigation:{enabled:true,active,max_goal_distance_m:distanceLimit,events,preparing_navigation:active},
    decision:{can_start:!!selection&&!active&&!charging,message:charging?'机器狗正在充电':active?'沿用当前定位，正在核对并规划路线':'定位就绪'},mapping:{active:false,events:[]}}});
  }
  if(url.pathname==='/manual_state')return route.fulfill({json:{connected:false,enabled:true,active:false}});
  const file=url.pathname==='/'?'navigation.html':url.pathname.slice(1);
  if(!['navigation.html','manual.js','cloud_view.js','unknown_goal.js'].includes(file))return route.abort();
  return route.fulfill({body:fs.readFileSync(path.join(root,file)),contentType:file.endsWith('.js')?'text/javascript':'text/html'});
 });
 await page.goto('http://127.0.0.1:8765/');
 await page.waitForFunction(()=>document.querySelector('#info').textContent.includes('静态地图'));
 assert.match(await page.locator('#trajectory-status').textContent(),/1.00 米/);
 assert.equal(await page.locator('#status').textContent(),'机器人已连接');
 const clickWorld=async(x,y)=>{
  await page.locator('#map').scrollIntoViewIfNeeded();
  const rect=await page.locator('#map').boundingBox();
  await page.mouse.click(rect.x+((x+5)/.05*3)/900*rect.width,rect.y+(200-(y+5)/.05)*3/600*rect.height);
 };
 await clickWorld(3.5,0);
 assert.ok((await page.locator('#message').innerText()).includes('2.8'));
 assert.equal(calls.some(c=>c.path==='/goal'),false,'Far target rejected immediately');
 await clickWorld(1,0);
 await page.waitForFunction(()=>document.querySelector('#message').textContent.includes('已收到目标'));
 assert.equal(selection,null,'Click confirmation precedes remote planning response');
 await page.waitForFunction(()=>document.querySelector('#message').textContent.includes('当前为预览'));
 assert.equal(calls.some(c=>c.path==='/navigate'),false,'Selecting target is not implicit motion');
 await page.waitForFunction(()=>!document.querySelector('#go').disabled);
 assert.ok((await page.locator('#nav-battery').innerText()).includes('不限制导航'));
 charging=true;
 await page.waitForFunction(()=>document.querySelector('#go').disabled&&document.querySelector('#go-feedback').textContent.includes('充电'));
 assert.ok((await page.locator('#go').getAttribute('title')).includes('充电'));
 charging=false;await page.waitForFunction(()=>!document.querySelector('#go').disabled);
 await page.locator('#go').click();
 await page.waitForFunction(()=>document.querySelector('#message').textContent.includes('等待导航启动'));
 assert.equal(calls.filter(c=>c.path==='/navigate').length,1);
 await page.waitForFunction(()=>document.querySelector('#navstatus').textContent.includes('沿用当前定位'));
 assert.equal(calls.some(c=>c.path==='/relocalize'),false,'Navigation must not reset localization');
 active=false;selection=null;distanceLimit=null;
 await page.waitForTimeout(900);
 const wholeMapStart=calls.length;await clickWorld(3.5,0);
 await page.waitForFunction(()=>document.querySelector('#message').textContent.includes('当前为预览'));
 assert.ok(calls.slice(wholeMapStart).some(c=>c.path==='/goal'&&Math.abs(c.x-3.5)<.02),'Whole-map mode accepts a target beyond 2.8m');
 assert.equal(calls.slice(wholeMapStart).some(c=>c.path==='/navigate'),false,'No implicit motion');
 assert.equal(stateMax,1,'No overlapping state polling');assert.deepEqual(errors,[]);
 console.log(JSON.stringify({result:'PASS',immediate_target_feedback:true,distance_feedback:true,preview_not_motion:true,navigation_once:true,no_relocalization:true,serialized_polling:true,low_battery_not_locked:true,disabled_reason_visible:true,robot_connections:0}));
 await browser.close();
})().catch(e=>{console.error(e);process.exit(1);});
