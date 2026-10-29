const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const assert=require('node:assert/strict');
(async()=>{const browser=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});try{
 const page=await browser.newPage({viewport:{width:1280,height:900}}),errors=[];
 page.on('pageerror',e=>errors.push(e.message));
 await page.goto('http://127.0.0.1:8769/near_range/');
 await page.waitForFunction(()=>document.getElementById('view').dataset.loaded==='near');
 for(const key of ['near','baseline','supported']){
  await page.locator('#'+key).click();
  await page.waitForFunction(key=>document.getElementById('view').dataset.loaded===key,key);
  await page.screenshot({path:'evidence/reconstruction_experiments/20261029_noise_audit_disjoint/near_range/'+key+'.png',fullPage:true});
 }
 for(const name of ['near.ply','supported.ply','range_report.json']){
  const response=await page.request.get('http://127.0.0.1:8769/near_range/'+name);
  assert.equal(response.status(),200);assert.ok((await response.body()).length>0);
 }
 for(const port of [8766,8768]){
  const response=await page.request.get('http://127.0.0.1:'+port+'/');
  assert.equal(response.status(),200);assert.ok((await response.text()).includes('8769/near_range/'));
 }
 assert.deepEqual(errors,[]);console.log('PASS: three model layers and downloads; no browser errors');
}finally{await browser.close()}})().catch(e=>{console.error(e);process.exitCode=1});
