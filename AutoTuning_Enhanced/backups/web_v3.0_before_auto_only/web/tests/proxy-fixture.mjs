// Test-process-only upstream substitute. No real key or paid API request is used.
globalThis.fetch=async(url,options)=>{
  const body=JSON.parse(options.body);
  if(body.model==='fail')return new Response(JSON.stringify({error:{message:'Rejected '+process.env.LLM_API_KEY}}),{status:401});
  return new Response(JSON.stringify({choices:[{message:{content:JSON.stringify({ok:true})}}],testOnly:{url:String(url),model:body.model,stream:body.stream}}),{status:200});
};
