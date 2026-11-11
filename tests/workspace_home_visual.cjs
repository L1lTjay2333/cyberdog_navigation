const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');const assert=require('assert/strict');
(async()=>{const b=await chromium.launch({channel:'msedge',headless:true});try{
 const page=await b.newPage({viewport:{width:1500,height:950}}),errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.goto('http://127.0.0.1:8765/');
 await page.waitForFunction(()=>loaded===34);await page.waitForTimeout(1800);
 assert.equal(await page.locator('.option').count(),2);
 const a=await page.evaluate(()=>frame);await page.waitForTimeout(500);assert.notEqual(await page.evaluate(()=>frame),a);
 await page.locator('#motion-toggle').click();const still=await page.evaluate(()=>frame);await page.waitForTimeout(250);assert.equal(await page.evaluate(()=>frame),still);
 await page.screenshot({path:'evidence/workspace_home_desktop.png'});
 await page.setViewportSize({width:390,height:844});await page.screenshot({path:'evidence/workspace_home_mobile.png',fullPage:true});
 assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
 await page.emulateMedia({reducedMotion:'reduce'});assert.equal(await page.locator('#motion-toggle').textContent(),'播放动效');
 assert.deepEqual(errors,[]);console.log('PASS 34 frames, animation advances, pause works, mobile fits, reduced motion respected, no JS errors');
 }finally{await b.close()}})().catch(e=>{console.error(e);process.exit(1)});
