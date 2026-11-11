const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const assert=require('assert/strict');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true});
 try{
 const page=await browser.newPage({viewport:{width:1400,height:900}});
 const errors=[];page.on('pageerror',e=>errors.push(e.message));
 let cancellations=0;
 await page.route('**/*',async route=>{
   const req=route.request();
   if(req.method()==='POST'){
     if(new URL(req.url()).pathname==='/cancel')cancellations++;
     return route.fulfill({contentType:'application/json',body:JSON.stringify({ok:true,message:'已请求取消，等待现场停车确认'})});
   }
   return route.continue();
 });
 await page.goto('http://127.0.0.1:8765/');
 await page.waitForTimeout(1200);
 assert(await page.locator('#cancel').isVisible());
 assert(await page.locator('#cancel').isEnabled());
 assert.equal(await page.locator('#nav-settings #cancel').count(),0);
 await page.locator('#cancel').click();
 await page.waitForFunction(()=>document.querySelector('#end-task-feedback').textContent.includes('重新连接遥控'));
 assert.equal(cancellations,1);
 assert.deepEqual(errors,[]);
 await page.screenshot({path:'evidence/navigation_end_button_20261105.png'});
 console.log('PASS visible outside folded settings, enabled while idle, cancel request and feedback; all POST requests mocked');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
