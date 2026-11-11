const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');const assert=require('assert/strict');
(async()=>{const b=await chromium.launch({channel:'msedge',headless:true});try{const p=await b.newPage({viewport:{width:1366,height:768}});const errors=[];p.on('pageerror',e=>errors.push(e.message));
await p.route('**/*',r=>r.request().method()==='POST'?r.fulfill({contentType:'application/json',body:JSON.stringify({ok:true,message:'模拟操作'})}):r.continue());
for(const [path,count] of [['/navigation',7],['/model/',2]]){
await p.goto('http://127.0.0.1:8765'+path);await p.waitForFunction(n=>document.querySelectorAll('.decrypted-heading').length===n,count);await p.waitForTimeout(3500);
assert.equal(await p.locator('.decrypted-heading').count(),count);
assert.equal(await p.locator('button .decrypted-heading').count(),0);
assert(await p.evaluate(()=>document.documentElement.scrollHeight<=innerHeight+1));
const host=p.locator('.decrypted-heading').first();const before=await host.getAttribute('class');await host.hover();await p.waitForTimeout(250);
assert(await host.locator('.encrypted').count()>0);
await p.mouse.move(1300,740);await p.waitForTimeout(3000);
assert(await host.locator('.revealed').count()>0);
if(path==='/navigation'){await p.locator('#cancel').click();await p.waitForFunction(()=>document.getElementById('end-task-feedback').textContent.includes('模拟操作'));}
else{await p.locator('#robot-connection-check').click();await p.waitForFunction(()=>document.getElementById('robot-connection-feedback').textContent==='模拟操作');}
}
assert.deepEqual(errors,[]);console.log('PASS 9 animated headings, hover scrambling and restored text, stable buttons/status and one-screen layouts; POST mocked');
}finally{await b.close()}})().catch(e=>{console.error(e);process.exit(1)});
