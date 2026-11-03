const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');const assert=require('assert/strict');
(async()=>{const b=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});try{
 const p=await b.newPage({viewport:{width:1500,height:1000}}),errors=[];p.on('pageerror',e=>errors.push(e.message));
 await p.goto('http://127.0.0.1:8772/');await p.waitForFunction(()=>window.roomProbe?.positions>0);await p.waitForTimeout(800);
 await p.screenshot({path:'evidence/live_room_gray.png'});console.log(JSON.stringify(await p.evaluate(()=>({mode:roomProbe.mode,positions:roomProbe.positions,frames:roomProbe.report.frames}))));
 await p.locator('[data-mode="processed"]').click();await p.waitForFunction(()=>window.roomProbe?.mode==='processed');await p.screenshot({path:'evidence/live_room_processed.png'});
 await p.locator('[data-mode="raw"]').click();await p.waitForFunction(()=>window.roomProbe?.mode==='raw');await p.screenshot({path:'evidence/live_room_raw.png'});
 await p.locator('[data-mode="replayed"]').click();await p.waitForFunction(()=>window.roomProbe?.mode==='replayed');assert(await p.evaluate(()=>roomProbe.positions>0));
 assert.deepEqual(errors,[]);console.log('PASS gray mesh / processed / raw switching, real geometry loaded, no browser errors');
}finally{await b.close();}})().catch(e=>{console.error(e);process.exit(1)});
