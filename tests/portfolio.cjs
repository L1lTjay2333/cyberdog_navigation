const{chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');const assert=require('assert/strict');
(async()=>{const b=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});try{
const p=await b.newPage({viewport:{width:1400,height:950}}),errors=[];p.on('pageerror',e=>errors.push(e.message));
await p.goto('http://127.0.0.1:8765/portfolio/');await p.waitForFunction(()=>window.portfolioProbe?.vertices>0);
assert.equal(await p.locator('.work').count(),5);
for(const id of ['chair44','chair331','roomgray','map1640','map1711']){
 await p.locator('[data-id="'+id+'"]').click();await p.waitForFunction(id=>window.portfolioProbe?.id===id,id);
 assert(await p.evaluate(()=>portfolioProbe.vertices>0));
}
await p.screenshot({path:'evidence/portfolio_showcase.png',fullPage:true});
const reply=await p.request.get('http://127.0.0.1:8765/portfolio/portfolio.zip');assert(reply.ok());
await p.goto('http://127.0.0.1:8765/');await p.locator('.folder-root').waitFor();assert.equal(await p.locator('.folder-item').count(),3);
assert.deepEqual(errors,[]);console.log('PASS five real works load, gray mesh and point clouds switch, ZIP downloads, home folder has three entries');
}finally{await b.close()}})().catch(e=>{console.error(e);process.exit(1)});
