// Simulated HTTP fixtures only. Every request is intercepted; zero robot access.
const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true});
 try{
  const page=await browser.newPage({viewport:{width:1360,height:1100}}),calls=[],errors=[];
  page.on('pageerror',e=>errors.push(e.message));let active=false,preparing=false,cameraReady=false;
  const state={session:'fixture-moving',frame:'vodom',mode:'moving_local_point_cloud',moving_supported:true,
    accepted_frames:10,stable_points:3,voxels:3,max_voxels:200000,voxel_m:.01,travel_m:.5,
    calibration_status:'fixture only',control_requests:0,points:[[1,0,0],[1,.1,0],[1,0,.1]],
    streaming_submaps:true,sealed_parts:3,active_voxels:12000,max_storage_bytes:536870912,
    trajectory:[{position:[0,0,0]},{position:[.5,0,0]}],
    capture_reference:{session:'fixture-moving',frame:'vodom',kind:'captured_camera_origin',position:[.5,0,0],forward:[1,0,0],latest_fused_pose:true}};
  await page.route('**/*',route=>{
   const request=route.request(),url=new URL(request.url());
   if(url.origin!=='http://127.0.0.1:8766')return route.abort();
   if(request.method()==='POST'){
    const data=request.postDataJSON();calls.push({path:url.pathname,...data});
    if(url.pathname==='/camera/connect'){preparing=true;active=false;}
    else {active=url.pathname==='/start';preparing=false;}
    return route.fulfill({json:{ok:true}});
   }
   if(url.pathname==='/state')return route.fulfill({json:{...state,active,preparing,busy:active||preparing,camera:{ready:cameraReady,message:preparing?'正在检查输入':cameraReady?'深度图与同时间位姿已就绪':'尚未检查'},status:'fixture only'}});
   if(url.pathname==='/')return route.fulfill({contentType:'text/html',body:fs.readFileSync(path.resolve(__dirname,'../reconstruction.html'),'utf8')});
   return route.abort();
  });
  await page.goto('http://127.0.0.1:8766/',{waitUntil:'networkidle'});
  await page.waitForFunction(()=>document.getElementById('stats').textContent.includes('0.500米'));
  assert((await page.locator('#head-reference').textContent()).includes('最新已融合帧'));
  assert((await page.locator('#stats').textContent()).includes('已落盘3块'));
  assert((await page.locator('#stats').textContent()).includes('边界可重复'));
  await page.getByRole('button',{name:'连接/检查深度相机'}).click();
  await page.waitForFunction(()=>document.getElementById('start').disabled);
  assert.equal(active,false);assert.deepEqual(calls[0],{path:'/camera/connect'});
  assert((await page.locator('#camera-status').textContent()).includes('正在检查'));
  preparing=false;cameraReady=true;
  await page.waitForFunction(()=>!document.getElementById('start').disabled);
  assert((await page.locator('#camera-status').textContent()).includes('已就绪'));
  await page.getByRole('button',{name:'开始60秒移动采集'}).click();
  await page.waitForFunction(()=>document.getElementById('moving-start').disabled);
  assert.deepEqual(calls[1],{path:'/start',seconds:60,mode:'moving'});
  await page.getByRole('button',{name:'停止本次采集'}).click();
  await page.waitForFunction(()=>!document.getElementById('moving-start').disabled);
  assert.deepEqual(calls[2],{path:'/stop'});
  await page.selectOption('#moving-duration','300');
  await page.getByRole('button',{name:'开始300秒移动采集'}).click();
  assert.deepEqual(calls[3],{path:'/start',seconds:300,mode:'moving'});
  assert.deepEqual(errors,[]);
  console.log(JSON.stringify({moving_start_stop:'PASS',trajectory_and_latest_head:'PASS',robot_connections:0,real_posts:0}));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
