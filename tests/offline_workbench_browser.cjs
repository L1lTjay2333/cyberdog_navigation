// Local-only browser verification; never invokes robot or capture endpoints.
const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const assert=require('node:assert/strict');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 try{
  const page=await browser.newPage({viewport:{width:1280,height:900}}),errors=[],requests=[];
  page.on('pageerror',error=>errors.push(error.message));
  page.on('request',request=>requests.push({url:request.url(),method:request.method()}));
  await page.goto('http://127.0.0.1:8768/');
  await page.locator('#layer-note').filter({hasText:'122,942'}).waitFor();
  assert.equal(await page.locator('canvas').count(),1);
  await page.getByRole('button',{name:'较高支持灰模',exact:true}).click();
  await page.locator('#layer-note').filter({hasText:'53,326'}).waitFor();
  await page.getByRole('button',{name:'处理后点云',exact:true}).click();
  await page.locator('#layer-note').filter({hasText:'30,414'}).waitFor();
  const links=await page.locator('#downloads a').evaluateAll(nodes=>nodes.map(node=>node.href));
  assert.equal(links.length,6);
  for(const url of links){const response=await page.request.get(url);assert.equal(response.status(),200);assert.ok((await response.body()).length>0)}
  await page.screenshot({path:'evidence/offline_runs/workbench_browser.png',fullPage:true});
  assert.deepEqual(errors,[]);assert.ok(requests.every(item=>item.url.startsWith('http://127.0.0.1:8768/')));
  assert.ok(requests.every(item=>item.method==='GET'));
  console.log('PASS: actual 44-frame result, 3 layers, 6 downloads, local GET only, no browser errors');
 }finally{await browser.close()}
})().catch(error=>{console.error(error);process.exitCode=1});
