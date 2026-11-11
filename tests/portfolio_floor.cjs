const{chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');const assert=require('assert/strict');
(async()=>{let b=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});try{let p=await b.newPage({viewport:{width:1400,height:950}});let errors=[];p.on('pageerror',e=>errors.push(e.message));await p.goto('http://127.0.0.1:8765/portfolio/');await p.waitForFunction(()=>window.portfolioProbe?.vertices>0);
assert.equal(await p.locator('.work').count(),4);assert.equal(await p.locator('[data-id="roomgray"]').count(),0);
for(const id of ['map1640','map1711']){
await p.locator('[data-id="'+id+'"]').click();await p.waitForFunction(id=>portfolioProbe.id===id&&portfolioProbe.floorTriangles>0,id);
assert(await p.locator('#floor-note').isVisible());await p.locator('#floor-toggle').uncheck();assert.equal(await p.evaluate(()=>portfolioProbe.floorVisible),false);
await p.locator('#floor-toggle').check();assert.equal(await p.evaluate(()=>portfolioProbe.floorVisible),true);
assert((await p.request.get(await p.locator('#floor-download').evaluate(e=>e.href))).ok());
}
await p.screenshot({path:'evidence/portfolio_floor_display.png',fullPage:true});assert.deepEqual(errors,[]);console.log('PASS selected work removed, four retained, two floors load, toggles and downloads work');
}finally{await b.close()}})().catch(e=>{console.error(e);process.exit(1)});

