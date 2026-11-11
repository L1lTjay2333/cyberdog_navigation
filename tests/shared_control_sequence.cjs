const fs=require('fs'),vm=require('vm'),assert=require('assert');
const source=fs.readFileSync('manual.js','utf8');
const part=source.slice(source.indexOf('const owner=sessionStorage'),source.indexOf('const controlWorker='));
const saved=new Map(),sessionStorage={getItem:k=>saved.get(k),setItem:(k,v)=>saved.set(k,v)};
function page(){const ctx={sessionStorage,crypto:require('crypto').webcrypto,Date:{now:()=>1000},Math};vm.createContext(ctx);vm.runInContext(part+';globalThis.get=()=>({owner,sequence:nextSequence()});',ctx);return ctx;}
const first=page(),a=first.get(),b=first.get(),next=page(),c=next.get();
assert.equal(a.owner,c.owner);assert(b.sequence>a.sequence);assert(c.sequence>b.sequence);
new vm.Script(source);
for(const file of ['navigation.html','reconstruction.html']){
 for(const m of fs.readFileSync(file,'utf8').matchAll(/<script(?:\s[^>]*)?>([\s\S]*?)<\/script>/g))if(m[1].trim())new vm.Script(m[1]);
}
console.log('PASS: same-millisecond commands and page reload remain ordered; all edited JS parses.');
