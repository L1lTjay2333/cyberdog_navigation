const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const assert=require('assert/strict');
(async()=>{const browser=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});try{
 const page=await browser.newPage({viewport:{width:1500,height:950}}),errors=[],requests=[];
 page.on('pageerror',e=>errors.push(e.message));page.on('requestfailed',r=>requests.push({url:r.url(),error:r.failure()}));
 await page.goto('http://127.0.0.1:8765/');
 await page.locator('#dither-background canvas').waitFor();
 await page.waitForTimeout(6500);
 await page.screenshot({path:'evidence/dither_home_desktop.png'});
 await page.mouse.move(1120,440);await page.waitForTimeout(350);
 await page.screenshot({path:'evidence/dither_home_reveal.png'});
 assert.equal(await page.locator('.option').count(),2);
 assert.equal(await page.locator('.option[href="/model/"]').evaluate(e=>document.elementFromPoint(e.getBoundingClientRect().x+25,e.getBoundingClientRect().y+25).closest('a')===e),true);
 await page.setViewportSize({width:390,height:844});await page.waitForTimeout(800);
 assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
 await page.screenshot({path:'evidence/dither_home_mobile.png',fullPage:true});
 assert.deepEqual(errors,[]);console.log(JSON.stringify({status:'PASS canvas, links, mobile layout and no JS errors',failed:requests}));
 }finally{await browser.close()}})().catch(e=>{console.error(e);process.exit(1)});
