(function(root,factory){
  if(typeof module === "object" && module.exports) module.exports=factory(require("./core.js"));
  else root.RouteASequence=factory(root.RouteACore);
})(typeof globalThis!=="undefined"?globalThis:this,function(C){
  "use strict";
  class Writer {
    constructor({send,onProgress=()=>{},gapMs=1000,timeoutMs=5000}){
      this.send=send;this.onProgress=onProgress;this.gapMs=Math.max(500,gapMs);this.timeoutMs=timeoutMs;this.context=null;
    }
    get busy(){return !!this.context;}
    progress(text){this.message=text;this.onProgress(text);}
    check(ctx){if(ctx!==this.context||ctx.cancelled)throw new Error(ctx.reason||"顺序更新已取消");}
    cancel(reason="顺序更新已取消"){
      const ctx=this.context;if(!ctx)return;
      ctx.cancelled=true;ctx.reason=reason;
      if(ctx.waiter)ctx.waiter.reject(new Error(reason));
    }
    feed(raw){
      const ctx=this.context;if(!ctx||!C.verify(raw))return;
      const parts=raw.slice(1,raw.lastIndexOf(",C")).split(",");
      if(parts[0]!=="P3")return;
      if(parts[1]==="ERR" && (Number(parts[3])===ctx.id||Number(parts[3])===0)){
        if(ctx.waiter)ctx.waiter.reject(new Error(`固件拒绝：${parts[2]}`));return;
      }
      if(parts[1]==="ACK"&&parts[2]==="ROLLED_BACK"){
        if(ctx.waiter)ctx.waiter.reject(new Error("固件已回滚，序列终止"));return;
      }
      if(!ctx.waiter)return;
      try{if(ctx.waiter.test(parts,raw))ctx.waiter.resolve(raw);}catch(error){ctx.waiter.reject(error);}
    }
    async exchange(ctx,body,test){
      this.check(ctx);
      let resolve,reject;const response=new Promise((ok,bad)=>{resolve=ok;reject=bad;});
      response.catch(()=>{});
      const waiter={test,resolve,reject};ctx.waiter=waiter;
      const timer=setTimeout(()=>reject(new Error("等待固件确认超时，停止后续参数")),this.timeoutMs);
      try{await this.send(C.frame(body));return await response;}
      finally{clearTimeout(timer);if(ctx.waiter===waiter)ctx.waiter=null;}
    }
    async readback(ctx,step,pid,profile,state){
      await this.exchange(ctx,`P3,SGET,${ctx.id},${step}`,(parts,raw)=>{
        if(parts[1]!=="STATUS")return false;
        const status=C.parseStatus(raw);
        if(!status||status.queryId!==ctx.id||status.queryStep!==step)return false;
        C.validatePid(status.pid);
        if(status.profile!==profile||status.state!==state||!C.pidMatches(status.pid,pid))throw new Error("实际PID读回不一致，停止并请求回滚");
        return true;
      });
    }
    async run({id,mode,profile,initialProfile,initialPid,targetPid,mask=7}){
      if(this.busy)throw new Error("已有参数更新序列");
      if(!Number.isInteger(id)||id<1||id>999999||!["M","T"].includes(mode)||!["N","L"].includes(profile))throw new Error("顺序事务参数无效");
      let actual=C.validatePid(initialPid);
      const target=C.mergePid(actual,targetPid,mask);
      for(const key of C.PID_KEYS)target[key]=Number(target[key].toFixed(2));
      const keys=C.PID_KEYS.filter(key=>Math.round(actual[key]*100)!==Math.round(target[key]*100));
      const ctx={id,mode,cancelled:false,waiter:null};this.context=ctx;let hb=null;
      try{
        this.progress(`建立逐项更新：${keys.length}项待写入`);
        await this.exchange(ctx,`P3,BSEQ,${id},30000,${mode},${profile}`,p=>p[1]==="ACK"&&p[2]==="SEQ_BEGIN"&&Number(p[3])===id);
        hb=setInterval(()=>{if(!ctx.cancelled)this.send(C.command("HB",id)).catch(error=>this.cancel(error.message));},500);
        for(let i=0;i<keys.length;i++){
          this.check(ctx);const key=keys[i],step=i+1,value=target[key];
          this.progress(`${step}/${keys.length} 写入 ${key}=${value.toFixed(2)}，等待完成确认`);
          await this.exchange(ctx,`P3,ONE,${id},${step},${key},${value.toFixed(2)}`,p=>
            p[1]==="SEQACK"&&p[2]==="ONE_APPLIED"&&Number(p[3])===id&&Number(p[4])===step&&p[5]===key&&Math.abs(Number(p[6])-value)<0.0049);
          actual={...actual,[key]:value};
          await this.readback(ctx,step,actual,initialProfile,2);
          this.progress(`${step}/${keys.length} ${key}读回一致；等待${this.gapMs} ms`);
          await new Promise(resolve=>setTimeout(resolve,this.gapMs));this.check(ctx);
        }
        this.progress("参数逐项完成；确认最终配置");
        await this.exchange(ctx,`P3,FSEQ,${id}`,p=>p[1]==="ACK"&&p[2]==="SEQ_DONE"&&Number(p[3])===id);
        clearInterval(hb);hb=null;
        await this.readback(ctx,7,target,profile,mode==="M"?0:2);
        this.check(ctx);this.progress("逐项更新完成，全部实际PID已确认");
        return {pid:target,profile,changed:keys};
      }catch(error){
        this.progress(`更新中止：${error.message}`);
        // The STM32 holds the pre-sequence champion; aborting a partial vector restores it.
        try{await this.send(C.command("ROLLBACK",id));}catch(_){}
        throw error;
      }finally{if(hb)clearInterval(hb);this.context=null;}
    }
  }
  return {Writer};
});
