const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');const assert=require('assert/strict');
(async()=>{let b=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});try{let p=await b.newPage();
for(const [width,height] of [[1366,768],[1920,1080],[1280,720]]){
await p.setViewportSize({width,height});await p.goto('http://127.0.0.1:8765/navigation');await p.waitForTimeout(500);
const result=await p.evaluate(()=>({height:innerHeight,scroll:document.documentElement.scrollHeight,remoteBottom:document.getElementById('control-disconnect').getBoundingClientRect().bottom,views:['depth-live','map','cloud'].map(id=>{const r=document.getElementById(id).getBoundingClientRect();return[r.width,r.height]})}));
assert(result.scroll<=height+1,JSON.stringify(result));assert(result.remoteBottom<height,JSON.stringify(result));assert(Math.max(...result.views.map(r=>r[1]))-Math.min(...result.views.map(r=>r[1]))<1);
console.log(width+'x'+height,JSON.stringify(result));if(width===1366)await p.screenshot({path:'evidence/navigation_single_screen.png'});
} }finally{await b.close()}})().catch(e=>{console.error(e);process.exit(1)});
