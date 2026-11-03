const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const assert=require('node:assert/strict');
(async()=>{const b=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});try{
const p=await b.newPage({viewport:{width:1280,height:960}}),errors=[];p.on('pageerror',e=>errors.push(e.message));
await p.goto('http://127.0.0.1:8768/');
await p.waitForFunction(()=>document.querySelector('#session').options.length>0);
await p.selectOption('#session','20261029_162923_107730');await p.selectOption('#mode','local');
if(!process.env.LOCAL_REUSE){
const started=p.waitForResponse(r=>r.url().endsWith('/api/start')&&r.request().method()==='POST');await p.locator('#first-frame').fill(process.env.LOCAL_FIRST_FRAME||'1');await p.locator('#start').click();const job=await (await started).json();assert.equal(job.ok,true);
await p.waitForFunction(async run=>{const s=await(await fetch('/api/state')).json();return s.run===run&&s.status==='done'},job.run,{timeout:180000});
}
await p.waitForFunction(()=>document.querySelector('#status').textContent.includes('10帧局部处理完成'),{},{timeout:180000});
await p.waitForFunction(()=>document.querySelector('#metrics').textContent.includes('局部第'));
assert.match(await p.locator('#result-scope').textContent(),/局部第.*共 10 帧.*原始记录共 331 帧/);
for(const key of ['observed','mls','supported','points']){assert.equal(await p.locator(`[data-layer="${key}"]`).isEnabled(),true);await p.locator(`[data-layer="${key}"]`).click();await p.waitForFunction(key=>document.getElementById('view').dataset.displayLayer===key,key);assert.equal(await p.locator('#error').textContent(),'');await p.screenshot({path:`evidence/local_${key}.png`});}
const state=await(await p.request.get('http://127.0.0.1:8768/api/state')).json();
assert.equal(await p.locator('[data-layer="aligned"]').isEnabled(),state.results.alignment_accepted);
if(state.results.alignment_accepted){await p.locator('[data-layer="aligned"]').click();await p.waitForFunction(()=>document.getElementById('view').dataset.displayLayer==='aligned');assert.equal(await p.locator('#error').textContent(),'');}
for(const a of await p.locator('#downloads a').all()){const r=await p.request.get(new URL(await a.getAttribute('href'),'http://127.0.0.1:8768/').href);assert.equal(r.status(),200);}
assert.deepEqual(errors,[]);console.log('PASS: UI launches local processing, reports actual alignment status, renders layers and downloads');
}finally{await b.close()}})().catch(e=>{console.error(e);process.exitCode=1});
