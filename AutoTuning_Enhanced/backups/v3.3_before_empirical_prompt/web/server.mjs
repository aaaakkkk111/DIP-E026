import http from 'node:http';
import https from 'node:https';
import {readFile} from 'node:fs/promises';
import {timingSafeEqual} from 'node:crypto';
const host=process.env.HOST||'127.0.0.1',port=Number(process.env.PORT||8787);
const endpoint=process.env.LLM_ENDPOINT||'https://api.deepseek.com/chat/completions';
const allowed=(process.env.LLM_ALLOWED_HOSTS||'api.deepseek.com,api.openai.com').split(',').map(s=>s.trim());
const upstream=new URL(endpoint);
if(upstream.protocol!=='https:'||!allowed.includes(upstream.hostname)||upstream.username||upstream.password)throw Error('LLM_ENDPOINT must be HTTPS on LLM_ALLOWED_HOSTS');
const tls=!!(process.env.TLS_CERT&&process.env.TLS_KEY);
const publicOrigin=process.env.PUBLIC_ORIGIN ? new URL(process.env.PUBLIC_ORIGIN).origin : null;
if(publicOrigin && !publicOrigin.startsWith('https://'))throw Error('PUBLIC_ORIGIN must be your HTTPS origin');
if(!['127.0.0.1','localhost','::1'].includes(host)&&(!tls||!process.env.PROXY_TOKEN))throw Error('LAN/public binding requires TLS_CERT, TLS_KEY and PROXY_TOKEN');
const authEqual=(a,b)=>{const aa=Buffer.from(a),bb=Buffer.from(b);return aa.length===bb.length&&timingSafeEqual(aa,bb);};
const json=(res,status,value)=>{res.writeHead(status,{'Content-Type':'application/json; charset=utf-8','Cache-Control':'no-store'});res.end(JSON.stringify(value));};
const handler=async(req,res)=>{
  try{
    const path=(req.url||'').split('?')[0];
    if(req.method==='GET'&&['/','/routea_bluefy.html'].includes(path)){
      res.writeHead(200,{'Content-Type':'text/html; charset=utf-8','Cache-Control':'no-store','X-Content-Type-Options':'nosniff'});
      res.end(await readFile(new URL('../routea_bluefy.html',import.meta.url)));return;
    }
    if(req.method==='GET'&&path==='/health'){json(res,200,{ok:true,proxyConfigured:!!process.env.LLM_API_KEY,tls});return;}
    if(req.method!=='POST'||path!=='/api/chat/completions'){json(res,404,{error:{message:'Not found'}});return;}
    // Same-origin only. No permissive CORS and no client-controlled upstream URL.
    if(req.headers.origin){const expected=publicOrigin||(tls?'https':'http')+'://'+req.headers.host;if(req.headers.origin!==expected){json(res,403,{error:{message:'Cross-origin proxy requests are disabled'}});return;}}
    const token=(req.headers.authorization||'').replace(/^Bearer\s+/i,'');
    if(process.env.PROXY_TOKEN&&!authEqual(token,process.env.PROXY_TOKEN)){json(res,401,{error:{message:'Proxy token required; enter PROXY_TOKEN in the page API Key field'}});return;}
    const key=process.env.LLM_API_KEY||(process.env.PROXY_TOKEN?'':token);
    if(!key){json(res,401,{error:{message:'Set LLM_API_KEY on server or enter a session API Key in the page'}});return;}
    let body='';for await(const chunk of req){body+=chunk;if(Buffer.byteLength(body)>256000){json(res,413,{error:{message:'Request too large'}});return;}}
    let input;try{input=JSON.parse(body);}catch(e){json(res,400,{error:{message:'Invalid JSON'}});return;}
    if(typeof input.model!=='string'||!input.model.trim()||!Array.isArray(input.messages)||!input.messages.length||input.messages.length>8||input.messages.some(x=>!['system','user','assistant'].includes(x.role)||typeof x.content!=='string')){json(res,400,{error:{message:'Invalid model/messages'}});return;}
    const controller=new AbortController(),timeout=setTimeout(()=>controller.abort(),40000);
    const abort=()=>controller.abort();res.on('close',abort);
    try{
      const response=await fetch(upstream,{method:'POST',signal:controller.signal,headers:{Authorization:'Bearer '+key,'Content-Type':'application/json'},body:JSON.stringify({model:input.model,messages:input.messages,stream:false,response_format:{type:'json_object'},max_tokens:1600})});
      const reply=await response.text();
      res.writeHead(response.status,{'Content-Type':'application/json; charset=utf-8','Cache-Control':'no-store'});
      res.end(reply.split(key).join('[REDACTED]'));
    }finally{clearTimeout(timeout);res.off('close',abort);}
  }catch(e){if(!res.headersSent)json(res,502,{error:{message:e.name==='AbortError'?'Upstream timed out or cancelled':'Upstream request failed; check server network and endpoint'}});else res.end();}
};
const server=tls?https.createServer({cert:await readFile(process.env.TLS_CERT),key:await readFile(process.env.TLS_KEY)},handler):http.createServer(handler);
server.listen(port,host,()=>console.log(`Route A console: ${tls?'https':'http'}://${host}:${port}/ (API keys are never logged)`));
