// Real Edge keyboard test with every HTTP request intercepted; never reaches the robot.
const { chromium } = require('../evidence/browser_test_runtime/node_modules/playwright');
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const root=path.resolve(__dirname,'..');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true});
 const page=await browser.newPage({viewport:{width:1440,height:1100}});
 const calls=[],errors=[];page.on('pageerror',e=>errors.push(e.message));
 let connected=true,owned=false,streamConnected=true; // Abandoned old tab initially owns the session.
 let taskStatus=101,mappingEvents=[],saveFails=false;
 await page.route('http://127.0.0.1:8765/**',async route=>{
  const req=route.request(),pathname=new URL(req.url()).pathname;
  if(req.method()==='POST'){
   const data=req.postDataJSON();calls.push({path:pathname,...data});
   if(pathname==='/manual/connect'){
    if(connected&&!owned)return route.fulfill({json:{ok:false,message:'已有另一个网页持有遥控会话'}});
    connected=true;owned=true;
   }
   if(pathname==='/manual/disconnect')connected=false;
   if(pathname==='/mapping/start'){taskStatus=5;mappingEvents=[{event:'SCANNING'}];}
   if(pathname==='/mapping/save'){
    mappingEvents=saveFails?[{event:'ERROR',message:'Mock save rejected'}]:[{event:'SAVED'}];
    if(!saveFails)taskStatus=101;
   }
   if(pathname==='/relocalize')taskStatus=8;
   return route.fulfill({json:{ok:true,message:'Mock operation'}});
  }
  if(pathname==='/manual_state')return route.fulfill({json:{connected,owns_session:connected&&owned,enabled:true,sensor_mode:'supervised_four_way_trial',depth_valid:true,depth_min:2,task:101,direction_blocks:{},requested_direction:'stop'}});
  if(pathname==='/state')return route.fulfill({json:{connection:streamConnected?'connected':'disconnected',local_received_unix:Date.now()/1000-(streamConnected?0:10),emitted_unix:Date.now()/1000,task:{status:taskStatus},navigation:{enabled:false,active:false,events:[]},mapping:{active:false,map_name:'offline_test_map',events:mappingEvents},map:null}});
  const file=pathname==='/'?'navigation.html':pathname.slice(1);
  if(!['navigation.html','manual.html','manual.js','cloud_view.js','unknown_goal.js'].includes(file))return route.abort();
  await route.fulfill({body:fs.readFileSync(path.join(root,file)),contentType:file.endsWith('.js')?'text/javascript':'text/html'});
 });
 await page.goto('http://127.0.0.1:8765/');
 await page.waitForFunction(()=>document.querySelector('#control-connect').textContent.includes('接管本页'));
 await page.locator('#control-connect').click();
 assert.equal(calls[0].path,'/manual/disconnect','Takeover must stop old session first');
 await page.locator('[data-direction=forward]').waitFor({state:'visible'});
 await page.waitForFunction(()=>!document.querySelector('[data-direction=forward]').disabled);
 assert.equal(await page.evaluate(()=>document.activeElement.id),'control-keyboard','Connect must focus keyboard input');
 for(const [key,direction] of [['w','forward'],['s','backward'],['a','left'],['d','right'],['j','shift_left'],['k','shift_right']]){
  const before=calls.length;
  await page.keyboard.down(key);await page.waitForTimeout(180);await page.keyboard.up(key);await page.waitForTimeout(60);
  assert.ok(calls.slice(before).some(c=>c.path==='/manual/control'&&c.direction===direction&&c.armed),key+' must send movement');
  assert.ok(calls.slice(before).some(c=>c.path==='/manual/stop'),key+' release must stop');
 }
 await page.locator('#control-keyboard').focus();
 for(const [first,second,direction,sx,sy] of [['w','a','forward_left',1,1],['w','d','forward_right',1,-1],['s','a','backward_left',-1,1],['s','d','backward_right',-1,-1]]){
  const start=calls.length;
  await page.keyboard.down(first);await page.keyboard.down(second);await page.waitForTimeout(220);
  const command=calls.slice(start).find(c=>c.path==='/manual/control'&&c.direction===direction&&c.armed);
  assert.ok(command,first+'+'+second+' must send diagonal');
  assert.ok(command.axes[0]*sx>0&&command.axes[1]*sy>0);
  assert.ok(Math.abs(Math.hypot(...command.axes)-1)<0.00001,'Diagonal vector must be normalized');
  await page.keyboard.up(second);await page.waitForTimeout(150);
  assert.ok(calls.slice(start).some(c=>c.path==='/manual/control'&&c.direction===(first==='w'?'forward':'backward')),'Partial release retains held direction');
  await page.keyboard.up(first);await page.waitForTimeout(60);
  assert.ok(calls.slice(start).some(c=>c.path==='/manual/stop'),'Full release stops');
 }
 let comboStart=calls.length;
 await page.keyboard.down('w');await page.keyboard.down('a');await page.waitForTimeout(250);
 assert.ok(calls.slice(comboStart).some(c=>c.path==='/manual/control'&&c.direction==='forward_left'&&c.axes[0]>0&&c.axes[1]>0));
 await page.keyboard.up('a');await page.waitForTimeout(150);
 assert.ok(calls.slice(comboStart).some(c=>c.path==='/manual/control'&&c.direction==='forward'));
 await page.keyboard.up('w');
 comboStart=calls.length;
 await page.keyboard.down('w');await page.waitForTimeout(2300);await page.keyboard.up('w');
 assert.ok(calls.slice(comboStart).filter(c=>c.path==='/manual/control'&&c.armed).length>15,'Holding no longer stops after two seconds');
 const sitBefore=calls.length;await page.keyboard.press('Space');await page.waitForTimeout(80);
 assert.equal(calls.slice(sitBefore).filter(c=>c.path==='/manual/posture').length,1);
 const stick=page.locator('canvas[aria-label="全方向遥控摇杆"]');await stick.scrollIntoViewIfNeeded();const box=await stick.boundingBox();
 comboStart=calls.length;await page.mouse.move(box.x+90,box.y+90);await page.mouse.down();
 await page.mouse.move(box.x+50,box.y+40);await page.waitForTimeout(250);await page.mouse.up();
 assert.ok(calls.slice(comboStart).some(c=>c.path==='/manual/control'&&c.axes?.[0]>0&&c.axes?.[1]>0),'Joystick continuous vector');
 comboStart=calls.length;await page.mouse.move(box.x+90,box.y+90);await page.mouse.down();
 await page.mouse.move(box.x+25,box.y+90);await page.waitForTimeout(250);await page.mouse.up();
 assert.ok(calls.slice(comboStart).some(c=>c.path==='/manual/control'&&c.direction==='left'&&c.axes[0]===0),'Horizontal joystick rotates, never strafes');
 await page.locator('#clickmove').evaluate(el=>{el.disabled=false;el.focus();});
 const before=calls.length;
 await page.keyboard.down('w');await page.waitForTimeout(180);await page.keyboard.up('w');
 assert.ok(calls.slice(before).some(c=>c.path==='/manual/control'&&c.direction==='forward'),'W with checkbox focus');
 const navMessage=await page.locator('#message').innerText();
 assert.equal(errors.length,0,errors.join('\n'));
 assert.ok(!navMessage.includes('按住 W'),'manual UI must not overwrite navigation messages');
 const mapStart=calls.length;
 page.once('dialog',dialog=>dialog.accept());
 await page.locator('#mappingstart').click();
 await page.waitForFunction(()=>document.querySelector('#message').textContent.includes('新场地扫描已开启'));
 await page.waitForFunction(()=>!document.querySelector('[data-direction=left]').disabled);
 const startCalls=calls.slice(mapStart).map(c=>c.path);
 assert.ok(startCalls.indexOf('/manual/disconnect')<startCalls.indexOf('/mapping/start'));
 assert.ok(startCalls.indexOf('/mapping/start')<startCalls.indexOf('/manual/connect'));
 assert.equal(startCalls.includes('/manual/control'),false,'Connecting mapping must not move');
 assert.equal(await page.evaluate(()=>document.activeElement.id),'control-keyboard');
 await page.keyboard.down('a');await page.waitForTimeout(200);await page.keyboard.up('a');
 assert.ok(calls.slice(mapStart).some(c=>c.path==='/manual/control'&&c.direction==='left'),'Manual rotation usable during mapping');
 await page.waitForFunction(()=>!document.querySelector('#savelocalize').disabled);
 assert.ok((await page.locator('#workflowfeedback').innerText()).includes('正在扫描'));
 const saveStart=calls.length;
 await page.locator('#savelocalize').click();
 await page.waitForFunction(()=>document.querySelector('#workflowfeedback').textContent.includes('定位完成'));
 const workflowCalls=calls.slice(saveStart).map(c=>c.path);
 assert.ok(workflowCalls.indexOf('/manual/disconnect')<workflowCalls.indexOf('/mapping/save'),'Stop manual before saving');
 assert.ok(workflowCalls.indexOf('/mapping/save')<workflowCalls.indexOf('/relocalize'),'Save before localization');
 assert.equal(await page.locator('#go').isDisabled(),true,'Localization alone must not unlock movement');
 taskStatus=5;mappingEvents=[{event:'SCANNING'}];saveFails=true;
 await page.waitForFunction(()=>!document.querySelector('#savelocalize').disabled);
 const failStart=calls.length;await page.locator('#savelocalize').click();
 await page.waitForFunction(()=>document.querySelector('#message').textContent.includes('Mock save rejected'));
 assert.equal(calls.slice(failStart).some(c=>c.path==='/relocalize'),false,'Failed save must not localize');
 assert.equal(calls.some(c=>c.path==='/navigate'),false,'Workflow test must not request movement');
 streamConnected=false;
 await page.waitForFunction(()=>document.querySelector('#connection-detail').textContent.includes('后台自动重连'));
 assert.ok((await page.locator('#connection-detail').innerText()).includes('最后更新'));
 streamConnected=true;
 await page.waitForFunction(()=>document.querySelector('#connection-detail').textContent.includes('实时数据已连接'));
 await page.screenshot({path:path.join(root,'evidence/main_keyboard_browser_20261009.png'),fullPage:true});
 console.log(JSON.stringify({result:'PASS',browser:'Edge',directions:6,diagonal_combinations:4,mapping_connect_no_auto_motion:true,save_then_localize:true,failed_save_no_localize:true,key_release:true,checkbox_focus:true,page_errors:errors,robot_connections:0}));
 await browser.close();
})().catch(e=>{console.error(e);process.exit(1);});
