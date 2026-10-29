const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const assert=require('node:assert/strict');
(async()=>{const browser=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});try{
 const page=await browser.newPage({viewport:{width:1280,height:900}}),errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.goto('http://127.0.0.1:8769/');
 for(const index of ['0','1']){await page.selectOption('#case',index);for(const layer of ['baseline','interior']){
  await page.locator('#'+layer).click();await page.waitForFunction(({index,layer})=>document.getElementById('view').dataset.loaded===index+':'+layer,{index,layer});
  const href=await page.locator('#download').getAttribute('href');const response=await page.request.get('http://127.0.0.1:8769/'+href);assert.equal(response.status(),200);
 }}
 await page.screenshot({path:'evidence/reconstruction_experiments/20261029_noise_audit_disjoint/browser.png',fullPage:true});
 assert.deepEqual(errors,[]);console.log('PASS: 4 offline layers and downloads, no script errors');
}finally{await browser.close()}})().catch(e=>{console.error(e);process.exitCode=1});
