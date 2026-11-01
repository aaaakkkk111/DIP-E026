(function(root,factory){
  const api=factory();
  if(typeof module==='object'&&module.exports)module.exports=api;
  root.RouteAPlot=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
  'use strict';
  function niceLimit(value,minimum){
    const target=Math.max(minimum,Math.abs(value)*1.1);
    const power=10**Math.floor(Math.log10(target));
    return [1,2,2.5,5,10].find(n=>n*power>=target)*power;
  }
  function series(samples,key,now,width,height){
    const minimum=key==='angle'?5:100;
    const visible=samples.filter(p=>Number.isFinite(p.at)&&Number.isFinite(p[key])&&p.at<=now&&now-p.at<=30000);
    const peak=visible.reduce((m,p)=>Math.max(m,Math.abs(p[key])),0);
    const limit=niceLimit(peak,minimum),left=52,right=Math.max(left+1,width-12),top=12,bottom=Math.max(top+1,height-24);
    const points=visible.map(p=>({at:p.at,value:p[key],x:left+(p.at-(now-30000))/30000*(right-left),y:top+(limit-p[key])/(2*limit)*(bottom-top)}));
    return {key,unit:key==='angle'?'°':'raw',limit,peak,points,left,right,top,bottom,ticks:[-limit,-limit/2,0,limit/2,limit]};
  }
  return {niceLimit,series};
});
