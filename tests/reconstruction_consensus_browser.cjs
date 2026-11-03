// Offline UI verification; all requests intercepted, no robot or server access.
const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const fs=require('node:fs');
const assert=require('node:assert/strict');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true});
 try{
  const page=await browser.newPage();const errors=[];
  page.on('pageerror',e=>errors.push(e.message));
  await page.route('**/*',route=>{
   assert.equal(route.request().method(),'GET');
   if(new URL(route.request().url()).pathname==='/state')return route.fulfill({json:{
    mode:'static_local_point_cloud',session:'offline-ui-test',status:'离线界面验证',
    accepted_frames:7,stable_points:0,points:[],voxels:0,max_voxels:200000,
    voxel_m:.01,temporal_filter:'static_reprojected_majority_3_of_5',
    temporal_window_frames:5,consensus_updates:3,consensus_pixels:120,
    consensus_valid_pixels:150,control_requests:0,calibration_status:'未验证',camera:{}}});
   return route.fulfill({contentType:'text/html',body:fs.readFileSync('reconstruction.html','utf8')});
  });
  await page.goto('http://127.0.0.1:8766/');
  await page.waitForFunction(()=>document.querySelector('#stats').textContent.includes('一致像素120/150'));
  assert((await page.locator('#stats').textContent()).includes('有效接收帧：7'));
  assert.deepEqual(errors,[]);
  await page.screenshot({path:'evidence/static_pipeline_20261031/ui.png',fullPage:true});
  console.log('PASS: static consensus counters, zero page errors, no real network requests');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
