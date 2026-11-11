const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const assert=require('assert/strict');
(async()=>{const browser=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});try{
 const page=await browser.newPage({viewport:{width:1500,height:1000}}),errors=[];
 page.on('pageerror',e=>errors.push(e.message));
 for(const [index,destination] of [[0,'/model/'],[1,'/navigation']]){
  await page.goto('http://127.0.0.1:8765/');
  await page.locator('.folder-root').waitFor();
  assert.equal(await page.locator('.folder-root').count(),1);
  assert.equal(await page.locator('.folder-entry').count(),0);
  assert.equal(await page.locator('.folder-item').count(),2);
  const stage=page.locator('.folder-stage');
  await stage.focus();await page.keyboard.press('Enter');await page.waitForTimeout(900);
  assert.equal(await stage.getAttribute('aria-expanded'),'true');
  await page.screenshot({path:'evidence/folder_home_open.png'});
  await page.route('http://127.0.0.1:8765'+destination,route=>route.fulfill({contentType:'text/html; charset=utf-8',body:'<h1 id="target-test">ready</h1>'}),{times:1});
  await page.locator('[data-folder-item="'+index+'"]').focus();await page.keyboard.press('Enter');
  await page.locator('#target-test').waitFor();
  assert.equal(new URL(page.url()).pathname,destination);
 }
 await page.goto('http://127.0.0.1:8765/');
 await page.setViewportSize({width:390,height:844});await page.waitForTimeout(1500);
 assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
 await page.screenshot({path:'evidence/folder_home_mobile.png',fullPage:true});
 assert.deepEqual(errors,[]);console.log('PASS single folder, two working cards, no direct links, keyboard navigation and mobile layout; targets mocked');
 }finally{await browser.close()}})().catch(e=>{console.error(e);process.exit(1)});
