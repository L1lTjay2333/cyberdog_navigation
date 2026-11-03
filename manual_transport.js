// Scan-console transport worker: independent of point-cloud drawing.
let sequence=0,intent=null,renewed=0,version=0,inFlight=0;
const stamp=()=>sequence=Math.max(sequence+1,Date.now()*1000);
async function post(path,body,timeout=800){
 const response=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({...body,sequence:stamp()}),signal:AbortSignal.timeout(timeout)});
 const result=await response.json();if(!response.ok||!result.ok)throw Error(result.message||result.error||'遥控请求失败');return result;
}
onmessage=async({data})=>{
 if(data.type==='intent'){intent=data.body;renewed=performance.now();return;}
 if(data.path!=='/manual/control'){intent=null;version++;}
 try{const result=await post(data.path,data.body,data.path==='/manual/connect'?45000:1000);postMessage({id:data.id,result});}
 catch(e){postMessage({id:data.id,error:e.message});}
};
setInterval(()=>{
 if(!intent)return;
 if(performance.now()-renewed>500){const owner=intent.owner;intent=null;version++;post('/manual/stop',{owner}).catch(()=>{});postMessage({type:'expired',error:'页面输入心跳中断，已停车，请重新按键'});return;}
 if(inFlight>=3)return;
 const current=version;inFlight++;
 post('/manual/control',intent,500).catch(e=>{if(current===version){intent=null;version++;postMessage({type:'expired',error:e.message});}}).finally(()=>inFlight--);
},100);
