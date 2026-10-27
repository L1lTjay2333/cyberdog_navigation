// Offline package smoke check. All browser requests are fulfilled from disk.
const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const runtime=path.resolve(process.argv[2]||'D:/cyberdog/tools/reconstruction_runtime/web/node_modules/three');
const html=`<!doctype html><html><head><script type="importmap">{"imports":{"three":"/build/three.module.js"}}</script></head><body><script type="module">
import * as THREE from 'three';
import {OrbitControls} from '/examples/jsm/controls/OrbitControls.js';
const scene=new THREE.Scene();
const camera=new THREE.PerspectiveCamera(50,1,.01,100);camera.position.z=3;
const renderer=new THREE.WebGLRenderer();renderer.setSize(128,128);document.body.append(renderer.domElement);
const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute([0,0,0,.2,.2,0],3));
scene.add(new THREE.Points(geometry,new THREE.PointsMaterial({size:.1})));
const controls=new OrbitControls(camera,renderer.domElement);controls.update();renderer.render(scene,camera);
const gl=renderer.getContext();window.probe={revision:THREE.REVISION,calls:renderer.info.render.calls,points:renderer.info.render.points,contextLost:gl.isContextLost()};
controls.dispose();geometry.dispose();renderer.dispose();
</script></body></html>`;
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 try{
  const page=await browser.newPage(),errors=[];
  page.on('pageerror',e=>errors.push(e.message));
  await page.route('**/*',route=>{
   const request=route.request(),url=new URL(request.url());
   if(request.method()!=='GET'||url.origin!=='http://runtime-check.local')return route.abort();
   if(url.pathname==='/')return route.fulfill({contentType:'text/html',body:html});
   const file=path.resolve(runtime,'.'+decodeURIComponent(url.pathname));
   if(!file.startsWith(runtime+path.sep)||!file.endsWith('.js'))return route.abort();
   return route.fulfill({contentType:'text/javascript',body:fs.readFileSync(file)});
  });
  await page.goto('http://runtime-check.local/');
  await page.waitForFunction(()=>window.probe,{timeout:15000});
  const result=await page.evaluate(()=>window.probe);
  assert.deepEqual(errors,[]);assert.equal(result.revision,'186');
  assert.equal(result.points,2);assert.equal(result.contextLost,false);assert(result.calls>0);
  result.mode='headless software WebGL smoke check, not GPU performance measurement';
  const target=path.resolve(__dirname,'../evidence/runtime_installation_20261027');
  fs.mkdirSync(target,{recursive:true});fs.writeFileSync(path.join(target,'three_check.json'),JSON.stringify(result,null,2));
  console.log(JSON.stringify(result));
 }finally{await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
