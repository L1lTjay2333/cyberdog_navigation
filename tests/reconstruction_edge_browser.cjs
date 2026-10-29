const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const assert=require('node:assert/strict');
(async()=>{const browser=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});try{
 const page=await browser.newPage({viewport:{width:1280,height:900}}),errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.goto('http://127.0.0.1:8769/edge_support/');
 await page.waitForFunction(()=>document.getElementById('view').dataset.loaded==='confirmed');
 await page.locator('#baseline').click();await page.waitForFunction(()=>document.getElementById('view').dataset.loaded==='baseline');
 await page.locator('#confirmed').click();await page.waitForFunction(()=>document.getElementById('view').dataset.loaded==='confirmed');
 await page.locator('#pending').click();await page.waitForFunction(()=>document.getElementById('view').dataset.pending==='true');
 await page.locator('#pending').click();await page.waitForFunction(()=>document.getElementById('view').dataset.pending==='false');
 await page.screenshot({path:'evidence/reconstruction_experiments/20261029_edge_support/browser.png',fullPage:true});
 assert.deepEqual(errors,[]);console.log('PASS: baseline, confirmed surface, uncertainty overlay toggles');
}finally{await browser.close()}})().catch(e=>{console.error(e);process.exitCode=1});
