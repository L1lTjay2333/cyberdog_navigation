const {chromium}=require('../evidence/browser_test_runtime/node_modules/playwright');
const assert=require('assert/strict');
(async()=>{const browser=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});try{
 const page=await browser.newPage({viewport:{width:1400,height:900}});const errors=[];page.on('pageerror',e=>errors.push(e.message));
 const catalog=await(await page.request.get('http://127.0.0.1:8765/portfolio/manifest.json')).json();let posts=0;
 await page.route('**/portfolio/delete',async route=>{posts++;assert.equal(route.request().postDataJSON().id,catalog[0].id);await route.fulfill({json:{ok:true,works:catalog.slice(1)}})});
 await page.goto('http://127.0.0.1:8765/portfolio/');await page.waitForFunction(()=>window.portfolioProbe?.vertices>0);
 const card=page.locator('[data-work-id]').first();await card.locator('.flip-card').click();await page.waitForTimeout(800);
 await card.getByRole('button',{name:'删除此作品',exact:true}).click();await card.getByRole('button',{name:'取消',exact:true}).click();assert.equal(posts,0);
 await card.getByRole('button',{name:'查看点云',exact:true}).click();
 await card.getByRole('button',{name:'删除此作品',exact:true}).click();await card.getByRole('button',{name:'确认删除',exact:true}).click();
 await page.waitForFunction(id=>window.portfolioProbe?.id===id,catalog[1].id);assert.equal(await page.locator('[data-work-id]').count(),catalog.length-1);assert.equal(posts,1);assert.deepEqual(errors,[]);
 await page.screenshot({path:'evidence/portfolio_flipcard.png',fullPage:true});console.log('PASS flip, view, cancel, mocked delete, next selection, no JS errors');
}finally{await browser.close()}})().catch(e=>{console.error(e);process.exit(1)});
