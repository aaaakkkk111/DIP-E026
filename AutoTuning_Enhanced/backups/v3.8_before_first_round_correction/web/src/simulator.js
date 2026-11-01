(function(root,factory){
  const C = root.RouteACore || (typeof require === 'function' ? require('./core.js') : null);
  const api = factory(C);
  if(typeof module === 'object' && module.exports) module.exports = api;
  root.RouteASimulator = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function(C){
  'use strict';
  // Independent firmware model: the browser's candidate/state are never consulted.
  class Simulator {
    constructor(emit, options={}) {
      this.emit=emit; this.pid={...C.DEFAULT_PID}; this.champion={...this.pid}; this.candidate=null;
      this.state=0; this.lock=false; this.balance=options.balance ?? true; this.ms=0; this.id=0;this.closedId=0;
      this.ttl=30000; this.lastHb=0; this.appliedAt=0; this.preparedAt=0; this.motion=[0,0]; this.options=options;
    }
    reply(body) { if(!this.options.silent) this.emit(C.frame(body)); }
    ack(name,id=0) { if(this.options.dropAck===name) return; this.reply(`P1,ACK,${name},${id},${this.state},${this.lock?1:0}`); }
    error(code) { this.reply(`P1,ERR,${code}`); }
    rollback() { if(this.id)this.closedId=this.id;this.pid={...this.champion}; this.candidate=null; this.state=0; this.id=0; this.motion=[0,0]; }
    receive(raw) {
      if(!raw.startsWith('$P1,')) {
        if(/^\$[0-4],[0-2],0,0,0,0,0,0,0,0#$/.test(raw)) this.motion=raw.slice(1,-1).split(',').slice(0,2).map(Number);
        return;
      }
      if(!C.verify(raw)) return this.error('CRC');
      const x=raw.slice(1,raw.lastIndexOf(',C')).split(',');
      if(x[0]!=='P1') return this.error('PROTOCOL');
      const op=x[1];
      if(op==='GET') { if(!this.options.dropPid) this.reply(`P1,PID,${C.pidCsv(this.pid)},${this.state},${this.lock?1:0},${this.balance?1:0}`); return; }
      if(op==='MSET') {
        if(this.lock || this.state!==0) return this.error('LOCKED');
        try { this.pid=C.boundedPid(Object.fromEntries(C.PID_KEYS.map((k,i)=>[k,x[i+2]]))); this.champion={...this.pid}; this.ack('MSET'); }
        catch(e) { this.error('RANGE'); } return;
      }
      if(op==='TRAIN') {
        if(x[2]==='START') { if(!this.balance || this.state!==0) return this.error('NOT_READY'); this.lock=true; this.champion={...this.pid}; this.ack('TRAIN_START'); }
        else if(x[2]==='STOP') { const id=this.id;this.ack('TRAIN_STOP',id);this.rollback();this.lock=false; }
        else this.error('COMMAND'); return;
      }
      if(op==='DIAG') {
        if(this.balance || this.lock || this.state!==0) return this.error('DIAG_STATE');
        if(!['L+','L-','R+','R-','STOP'].includes(x[2])) return this.error('DIAG_ACTION');
        this.ack(x[2]); return;
      }
      const id=Number(x[2]);
      if(op==='PREP') {
        if(!this.lock || !this.balance || this.state!==0) return this.error('NOT_READY');
        if(!Number.isInteger(id) || id<=0) return this.error('ID');
        const p=Object.fromEntries(C.PID_KEYS.map((k,i)=>[k,Number(x[i+4])]));
        try { C.validateProposal({schema_version:1,decision:'propose',stage:'balance',candidate:p,expected_effect:'sim',requested_test:'quiet_balance',confidence:1},this.pid); }
        catch(e) { return this.error('CANDIDATE'); }
        this.ttl=Number(x[3]); if(this.ttl<1000 || this.ttl>30000) return this.error('TTL');
        this.candidate=p; this.id=id; this.state=1;this.preparedAt=this.ms;this.lastHb=this.ms; this.ack('PREP',id); return;
      }
      if(op==='STOP') { this.rollback(); this.ack('STOP',id); return; }
      if(op==='HB'&&id&&id===this.closedId&&id!==this.id)return;
      if(id!==this.id || !id) return this.error('TRIAL,'+this.id);
      if(op==='APPLY' && this.state===1) { this.ack('APPLY',id);this.pid={...this.candidate}; this.state=2; this.appliedAt=this.ms; this.lastHb=this.ms; }
      else if(op==='HB' && (this.state===1||this.state===2)) { this.lastHb=this.ms;this.ack('HB',id); }
      else if(op==='ACCEPT' && this.state===2) { this.closedId=this.id;this.champion={...this.pid}; this.candidate=null; this.state=0; this.id=0; this.ack('ACCEPT',id); }
      else if(op==='ROLLBACK') { this.rollback(); this.ack('ROLLBACK',id); }
      else this.error('STATE');
    }
    tick(step=200) {
      this.ms+=step;
      if(this.state!==0 && (this.ms-this.lastHb>2000 || this.ms-this.preparedAt>=this.ttl)) this.rollback();
      const a=this.options.unsafe ? 30 : Math.sin(this.ms/820)*2.4+Math.sin(this.ms/210)*.35;
      const g=Math.cos(this.ms/820)*13;
      const f=`T1,F,${this.ms},A${a.toFixed(2)},G${g.toFixed(1)},EL${Math.round(g)},ER${Math.round(g+1)},ML${Math.round(-a*120)},MR${Math.round(-a*118)},S${this.state},L${this.lock?1:0},R${this.balance?1:0},C1${Math.max(0,Math.round(a*20))},C2${Math.max(0,Math.round(-a*20))},C3${Math.max(0,Math.round(a*19))},C4${Math.max(0,Math.round(-a*19))}`;
      this.reply(f);
      if(this.ms%400===0)this.reply(`T1,E,${this.ms},O${(Math.abs(g)*.12).toFixed(3)},L${(Math.abs(a)*.45).toFixed(3)},B${(a*.5).toFixed(3)},W${this.ms>=3000?1:0}`);
      if(this.ms%1000===0) this.reply(`T1,S,${this.ms},V11.82,B0,V0,T0,AP${this.pid.AP.toFixed(2)},AD${this.pid.AD.toFixed(2)},VP${this.pid.VP.toFixed(2)},VI${this.pid.VI.toFixed(2)},TP${this.pid.TP.toFixed(2)},TD${this.pid.TD.toFixed(2)},D0,L${this.lock?1:0},R${this.balance?1:0},RA1.9-AUTO6-TXFIX-VP100-NOGUARD-SIM`);
    }
  }
  return { Simulator };
});
