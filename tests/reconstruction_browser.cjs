// Development verification: GET-only access to the local static laboratory.
// Every POST and every non-local URL is blocked; no capture or motion starts.
const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true});
 try{
  const page=await browser.newPage({viewport:{width:1360,height:1100}});
  const errors=[],posts=[];
  page.on('pageerror',e=>errors.push(e.message));
  await page.route('**/*',route=>{
   const r=route.request(),u=new URL(r.url());
   if(r.method()!=='GET'){posts.push(r.url());return route.abort();}
   if(u.origin!=='http://127.0.0.1:8766')return route.abort();
   return route.continue();
  });
  await page.goto('http://127.0.0.1:8766/',{waitUntil:'networkidle'});
  await page.waitForFunction(()=>document.getElementById('stats').textContent.includes('有效融合帧'));
  const response=await page.request.get('http://127.0.0.1:8766/state');
  const state=await response.json();
  assert(state.stable_points>0,'Live capture must exist before visual verification');
  assert(!state.active,'Do not interfere with an active capture');
  await page.getByRole('button',{name:'适配模型'}).click();
  await page.getByRole('button',{name:'XY俯视'}).click();
  await page.getByRole('button',{name:'水平视角'}).click();
  await page.screenshot({path:path.resolve(__dirname,'../evidence/reconstruction_model_live_20261027.png'),fullPage:true});
  const [download]=await Promise.all([
   page.waitForEvent('download'),page.getByRole('link',{name:'导出PLY模型'}).click()
  ]);
  const target=path.resolve(__dirname,'../evidence/reconstruction_browser_export_20261027.ply');
  await download.saveAs(target);
  const ply=fs.readFileSync(target,'utf8');
  assert(ply.includes(`element vertex ${state.stable_points}\n`));
  assert.equal(posts.length,0);assert.deepEqual(errors,[]);
  console.log(JSON.stringify({live_model_points:state.stable_points,frames:state.accepted_frames,export_verified:true,browser_errors:errors.length,post_requests:posts.length}));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
