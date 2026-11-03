const vm=require('node:vm'),fs=require('node:fs'),assert=require('node:assert/strict');
(async()=>{
 const calls=[],messages=[],timers=[];const context={performance,Date,AbortSignal,setInterval:(f,t)=>{const id=setInterval(f,t);timers.push(id);return id;},postMessage:m=>messages.push(m),fetch:async(path,options)=>{calls.push({path,body:JSON.parse(options.body),at:performance.now()});await new Promise(r=>setTimeout(r,180));return {ok:true,json:async()=>({ok:true})};}};
 vm.createContext(context);vm.runInContext(fs.readFileSync('manual_transport.js','utf8'),context);
 const renew=()=>context.onmessage({data:{type:'intent',body:{owner:'test-owner',direction:'forward',armed:true}}});
 const renewal=setInterval(renew,70);renew();await new Promise(r=>setTimeout(r,1050));clearInterval(renewal);
 assert(calls.filter(c=>c.path==='/manual/control').length>=8,'slow responses must not serialize heartbeat');
 await context.onmessage({data:{id:1,path:'/manual/stop',body:{owner:'test-owner'}}});
 const controls=calls.filter(c=>c.path==='/manual/control').length;
 await new Promise(r=>setTimeout(r,300));assert.equal(calls.filter(c=>c.path==='/manual/control').length,controls,'stop must cancel held intent');
 renew();await new Promise(r=>setTimeout(r,850));assert(messages.some(m=>m.type==='expired'),'stale input must stop');
 for(let i=1;i<calls.length;i++)assert(calls[i].body.sequence>calls[i-1].body.sequence);
 timers.forEach(clearInterval);console.log('PASS: delayed response heartbeat, stop cancellation, input expiry, ordered sequences');
})().catch(e=>{console.error(e);process.exit(1)});
