const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const assert=require('assert/strict');
(async()=>{const b=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});try{
 const p=await b.newPage({viewport:{width:1500,height:1000}}),errors=[],logs=[];
 p.on('pageerror',e=>errors.push(e.message));p.on('console',m=>{if(m.type()==='error')logs.push(m.text())});
 let stopped=0;
 await p.route('**/*',route=>{
  if(route.request().method()==='POST'){
   if(new URL(route.request().url()).pathname==='/cancel')stopped++;
   return route.fulfill({contentType:'application/json',body:JSON.stringify({ok:true,message:'停止按钮测试通过'})});
  }
  return route.continue();
 });
 await p.goto('http://127.0.0.1:8765/navigation');
 await p.locator('#navigation-background canvas').waitFor();
 await p.waitForTimeout(2500);
 assert(await p.locator('#cancel').isVisible());
 assert(await p.locator('#cancel').evaluate(e=>{const b=e.getBoundingClientRect();return document.elementFromPoint(b.x+b.width/2,b.y+b.height/2)===e}));
 await p.locator('#cancel').click();
 await p.waitForFunction(()=>document.getElementById('end-task-feedback').textContent.includes('停止按钮测试通过'));
 assert.equal(stopped,1);
 await p.mouse.move(1100,450);await p.waitForTimeout(500);
 await p.screenshot({path:'evidence/navigation_topography_desktop.png',fullPage:true});
 const mapTop=await p.locator('#map').evaluate(e=>{const b=e.getBoundingClientRect();return document.elementFromPoint(b.x+b.width/2,b.y+b.height/2)===e});
 assert(mapTop);
 await p.setViewportSize({width:390,height:844});await p.waitForTimeout(700);
 await p.screenshot({path:'evidence/navigation_topography_mobile.png',fullPage:true});
 assert.deepEqual(errors,[]);console.log(JSON.stringify({result:'PASS background canvas, stop control, map hit testing, mobile rendering',errors:logs}));
 }finally{await b.close()}})().catch(e=>{console.error(e);process.exit(1)});
