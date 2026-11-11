const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');const assert=require('assert/strict');
(async()=>{let b=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});try{
let p=await b.newPage({viewport:{width:1366,height:768}}),errors=[];p.on('pageerror',e=>errors.push(e.message));
await p.route('**/*',r=>r.request().method()==='POST'?r.fulfill({contentType:'application/json',body:JSON.stringify({ok:true,message:'玻璃面板测试'})}):r.continue());
for(const path of ['/navigation','/model/']){
 await p.goto('http://127.0.0.1:8765'+path);
 await p.locator('.glass-surface').first().waitFor();await p.waitForTimeout(1500);
 const geometry=await p.evaluate(()=>({scroll:document.documentElement.scrollHeight,height:innerHeight,panels:[...document.querySelectorAll('.react-glass-panel')].map(e=>({width:e.offsetWidth,height:e.offsetHeight,glass:e.querySelector('.glass-surface').getBoundingClientRect().height,filter:getComputedStyle(e.querySelector('.glass-surface')).backdropFilter}))}));
 assert(geometry.scroll<=geometry.height+1,JSON.stringify(geometry));
 assert(geometry.panels.every(x=>x.glass>0),JSON.stringify(geometry));
 if(path==='/navigation'){
  await p.locator('#cancel').click();await p.waitForFunction(()=>document.getElementById('end-task-feedback').textContent.includes('玻璃面板测试'));
  await p.locator('#nav-settings>summary').click();
  assert(await p.locator('#nav-settings>.card').isVisible());
  const c=p.locator('#cloud');const box=await c.boundingBox();await p.mouse.move(box.x+80,box.y+80);await p.mouse.down();await p.mouse.move(box.x+110,box.y+100);await p.mouse.up();
 }else{
  await p.locator('#robot-connection-check').click();await p.waitForFunction(()=>document.getElementById('robot-connection-feedback').textContent==='玻璃面板测试');
 }
 await p.screenshot({path:'evidence/glass_surface_'+(path==='/navigation'?'map':'model')+'.png'});
 console.log(path,JSON.stringify(geometry));
}
assert.deepEqual(errors,[]);console.log('PASS real GlassSurface layers, controls, details, drag, one-screen layout; POST mocked');
}finally{await b.close()}})().catch(e=>{console.error(e);process.exit(1)});

