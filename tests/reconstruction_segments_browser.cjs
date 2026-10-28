const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const assert=require('node:assert/strict');
(async()=>{const browser=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
try{const page=await browser.newPage({viewport:{width:1280,height:900}}),errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.goto('http://127.0.0.1:8767/turn_segments/');
 await page.locator('#status').filter({hasText:'预览'}).waitFor();
 assert.equal(await page.locator('#segment option').count(),12);
 for(let index=0;index<12;index++){
  await page.selectOption('#segment',String(index));
  await page.waitForFunction(index=>document.getElementById('view').dataset.loaded===index+':refined',index);
  await page.getByRole('button',{name:'原位姿分段灰模',exact:true}).click();
  await page.waitForFunction(index=>document.getElementById('view').dataset.loaded===index+':baseline',index);
  await page.getByRole('button',{name:'局部对齐分段灰模',exact:true}).click();
  await page.waitForFunction(index=>document.getElementById('view').dataset.loaded===index+':refined',index);
 }
 await page.selectOption('#segment','5');await page.waitForFunction(()=>document.getElementById('view').dataset.loaded==='5:refined');
 await page.screenshot({path:'evidence/reconstruction_experiments/segments_browser.png',fullPage:true});
 assert.deepEqual(errors,[]);console.log('PASS: 12 segments and baseline/refined switching; no script errors');
}finally{await browser.close()}})().catch(e=>{console.error(e);process.exitCode=1});
