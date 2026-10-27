const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const events={},docEvents={},buttons=[],elements={},timers={};let bodies=[];
for(const id of ['message','feedback','distance','connect','disconnect','stop'])elements[id]={};
elements.connect.insertAdjacentElement=(_,element)=>{elements[element.id]=element;};
const context={console,crypto:{randomUUID:()=> 'keyboard_test_owner'},
 document:{hidden:false,getElementById:id=>elements[id],querySelectorAll:()=>buttons,
           createElement:()=>({style:{},setAttribute(){},addEventListener(){},focus(){},insertAdjacentElement(){},getContext:()=>({clearRect(){},beginPath(){},arc(){},stroke(){},fill(){}})}),
           addEventListener:(name,fn)=>docEvents[name]=fn},
 window:{addEventListener:(name,fn)=>events[name]=fn},
 setInterval:(fn,ms)=>{timers[ms]=fn;},fetch:async(path,opts)=>{
  if(opts)bodies.push({path,...JSON.parse(opts.body)});
  return {json:async()=>opts?{ok:true}:{connected:true,enabled:false}};
 }};
vm.createContext(context);vm.runInContext(fs.readFileSync('manual.js','utf8'),context);
function key(code,extra={}){return {code,target:{tagName:'BODY'},preventDefault(){},...extra};}
function read(expr){return vm.runInContext(expr,context);}
(async()=>{
 await new Promise(setImmediate);
 vm.runInContext('enabled=true',context);
 events.keydown(key('KeyW'));await timers[100]();
 assert.ok(bodies.some(b=>b.path==='/manual/control'&&b.direction==='forward'&&b.armed===true));
 events.keyup(key('KeyW'));assert.equal(bodies.at(-1).path,'/manual/stop');
 events.keydown(key('',{key:'w'}));assert.equal(read('held'),'forward');
 events.keyup(key('',{key:'w'}));assert.equal(read('held'),'stop');
 for(const [code,direction] of Object.entries({KeyW:'forward',KeyS:'backward',KeyA:'left',KeyD:'right',KeyJ:'shift_left',KeyK:'shift_right'})){
  events.keydown(key(code));assert.equal(read('held'),direction);
  events.keyup(key(code));assert.equal(read('held'),'stop');
 }
 events.keydown(key('KeyW'));events.keydown(key('KeyA'));assert.equal(read('held'),'forward_left');
 events.keyup(key('KeyW'));assert.equal(read('held'),'left');events.keyup(key('KeyA'));assert.equal(read('held'),'stop');
 events.keydown(key('KeyJ'));assert.equal(read('heldAxes'),null);events.keydown(key('KeyK'));assert.equal(read('held'),'stop');
 events.keyup(key('KeyJ'));assert.equal(read('held'),'shift_right');events.keyup(key('KeyK'));
 events.keydown(key('KeyW'));events.keydown(key('KeyJ'));assert.equal(read('held'),'stop');events.keyup(key('KeyJ'));events.keyup(key('KeyW'));
 events.keydown(key('KeyW',{repeat:true}));assert.equal(read('held'),'stop');
 events.keydown(key('KeyW',{target:{tagName:'INPUT'}}));assert.equal(read('held'),'stop');
 events.keydown(key('KeyW'));events.keydown(key('Space'));assert.equal(read('held'),'stop');
 assert.ok(bodies.some(b=>b.path==='/manual/posture'),'Space sends posture request once');
 events.keydown(key('KeyW'));events.blur();assert.equal(read('held'),'stop');
 events.keydown(key('KeyW'));context.document.hidden=true;docEvents.visibilitychange();assert.equal(read('held'),'stop');
 assert.ok(bodies.filter(x=>x.path==='/manual/stop').length>=8);
 console.log('PASS: WASD mapping, release, multi-key stop, repeat/input guard, space, blur/hidden');
})().catch(e=>{console.error(e);process.exitCode=1;});
