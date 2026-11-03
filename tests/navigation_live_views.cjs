const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
(async()=>{
 const b=await chromium.launch({channel:'msedge',headless:true});
 try {
  const p=await b.newPage({viewport:{width:1600,height:1000}}),errors=[];
  p.on('pageerror',e=>errors.push(e.message));
  await p.goto('http://127.0.0.1:8765/');await p.waitForTimeout(4000);
  console.log(JSON.stringify({depth:await p.locator('#depth-live-status').textContent(),cloud:await p.locator('#cloudstatus').textContent(),errors}));
  await p.screenshot({path:'evidence/navigation_live_views_fixed.png',fullPage:true});
 }finally{await b.close();}
})().catch(e=>{console.error(e);process.exit(1)});
