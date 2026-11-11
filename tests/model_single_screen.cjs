const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');const assert=require('assert/strict');
(async()=>{const b=await chromium.launch({channel:'msedge',headless:true});try{const p=await b.newPage();const errors=[];p.on('pageerror',e=>errors.push(e.message));
await p.route('**/*',r=>r.request().method()==='POST'?r.fulfill({contentType:'application/json',body:JSON.stringify({ok:true,message:'模拟连接检查'})}):r.continue());
for(const [width,height] of [[1366,768],[1280,720],[1920,1080]]){
await p.setViewportSize({width,height});await p.goto('http://127.0.0.1:8765/model/');await p.waitForTimeout(700);
const x=await p.evaluate(()=>({scroll:document.documentElement.scrollHeight,height:innerHeight,canvas:document.getElementById('view').getBoundingClientRect().height,actions:['camera-connect','start','moving-start','stop','native-photo'].map(id=>{const r=document.getElementById(id).getBoundingClientRect();return[id,r.y,r.bottom]})}));
assert(x.scroll<=height+1,JSON.stringify(x));assert(x.canvas>150,JSON.stringify(x));assert(x.actions.every(a=>a[1]>=0&&a[2]<=height));
if(width===1366)await p.screenshot({path:'evidence/model_single_screen.png'});
console.log(width+'x'+height,JSON.stringify(x));
}
await p.locator('#robot-connection-check').click();await p.waitForFunction(()=>document.getElementById('robot-connection-feedback').textContent==='模拟连接检查');
await p.locator('#model-more summary').click();assert(await p.locator('#stats').isVisible());assert.deepEqual(errors,[]);console.log('PASS visible capture controls, connection callback, advanced diagnostics; no real commands');
}finally{await b.close()}})().catch(e=>{console.error(e);process.exit(1)});
