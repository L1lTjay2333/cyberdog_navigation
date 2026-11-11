const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const assert=require('assert/strict');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true});
 try {
 const page=await browser.newPage({viewport:{width:1500,height:1000}});
 const errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.route('**/*',route=>route.request().method()==='POST'
  ?route.fulfill({contentType:'application/json',body:JSON.stringify({ok:true,message:'测试连接反馈'})})
  :route.continue());
 await page.goto('http://127.0.0.1:8765/');
 assert.equal(await page.locator('.options .option').count(),2);
 await page.screenshot({path:'evidence/workspace_home.png'});
 await page.locator('a[href="/model/"]').click();
 await page.waitForTimeout(1200);
 assert(await page.locator('#camera-connect').isVisible());
 await page.locator('#robot-connection-check').click();
 await page.waitForFunction(()=>document.getElementById('robot-connection-feedback').textContent==='测试连接反馈');
 const remote=page.frameLocator('iframe');
 await remote.locator('#connect').waitFor({timeout:10000});
 assert((await page.locator('iframe').getAttribute('src')).startsWith('/model/'));
 await page.goto('http://127.0.0.1:8765/navigation');
 await page.waitForTimeout(1000);
 assert(await page.locator('#cancel').isVisible());
 assert(await page.locator('#sshcheck').count());
 assert.deepEqual(errors,[]);
 console.log('PASS home choices, model connection button, embedded remote, map controls; POST mocked');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
