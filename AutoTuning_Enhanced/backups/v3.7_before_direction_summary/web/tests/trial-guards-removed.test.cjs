const {test}=require('node:test'),assert=require('node:assert/strict');
const {make}=require('./harness.cjs'),C=require('../src/core.js');
const baseline={AP:50,AD:45,VP:62,VI:31,TP:17,TD:20};
const candidate={...baseline,AP:75,AD:70};
const fetch=async()=>({ok:true,json:async()=>({choices:[{message:{content:JSON.stringify({schema_version:1,decision:'propose',stage:'balance',candidate,expected_effect:'模拟用户27.09度场景，只验证应用事务',requested_test:'quiet_balance',confidence:.5})}}]})});

function readings(h,options){
  const Sim=h.context.RouteASimulator.Simulator,original=Sim.prototype.tick;
  Sim.prototype.tick=function(ms){
    const emit=this.emit;
    this.emit=raw=>{
      let body=raw.slice(1,raw.lastIndexOf(',C'));
      if(body.startsWith('T1,F,'))body=body.replace(/,A-?[\d.]+/,',A'+(options.angle??2)).replace(/,G-?[\d.]+/,',G100').replace(/,ML-?\d+/,',ML'+(options.output??1400)).replace(/,MR-?\d+/,',MR'+(options.output??1400));
      if(body.startsWith('T1,E,'))body=body.replace(/,O-?[\d.]+/,',O5.000').replace(/,L-?[\d.]+/,',L1.000').replace(/,B-?[\d.]+/,',B0.200');
      if(body.startsWith('T1,S,'))body=body.replace(/,V[\d.]+/,',V'+(options.battery??12));
      emit(C.frame(body));
    };
    original.call(this,ms);this.emit=emit;
  };
}

for(const [name,options] of [
  ['the exact 27.09-degree user APPLY gate',{angle:27.09}],
  ['saturated calculated outputs',{output:2600}],
  ['battery below both former tuning thresholds',{battery:9.2}],
  ['all three old conditions together',{angle:27.09,output:2600,battery:9.2}]
]) test(name+' no longer blocks or aborts automatic PID application',async()=>{
  const h=make({fetch});await h.action('mock');await h.advance(4000);
  for(const k of C.PID_KEYS)h.set('edit'+k).value=String(baseline[k]);
  await h.action('manual-pid');
  readings(h,options);await h.advance(1000);
  h.set('llmKey').value='test-key';h.set('loopCooldown').value='60';
  const pending=h.action('continuous-start');await h.flush();await h.advance(8500);
  const rows=h.snapshot().history;
  assert.ok(rows.length,JSON.stringify(h.snapshot()));
  assert.equal(rows[0].appliedConfirmed,true);
  assert.equal(rows[0].outcome,'accepted');
  assert.deepEqual(JSON.parse(JSON.stringify(rows[0].candidate)),candidate);
  assert.equal(h.snapshot().continuous.running,true);
  assert.match(h.set('transactionLog').value,/TX \$P1,APPLY,/);
  assert.match(h.set('transactionLog').value,/RX \$P1,PID,75.00,70.00,62.00,31.00,17.00,20.00,2,1,1,/);
  assert.doesNotMatch(h.set('transactionLog').value,/超过 25°|接近饱和，不能 APPLY|电压低于.*阈值/);
  await h.action('continuous-stop');await h.advance(300);await pending;
  assert.equal(h.snapshot().locked,false);assert.deepEqual(h.errors,[]);
});

test('old persisted battery threshold cannot resurrect a removed tuning gate',async()=>{
  const h=make({saved:{'routea-bluefy-v2':JSON.stringify({minBattery:'13.5',windowPolicy:2,baselineSeconds:'3',trialSeconds:'1'})}});
  assert.equal(h.doc.elements.some(e=>e.id==='minBattery'),false);
  await h.action('save-settings');
  assert.equal(Object.hasOwn(JSON.parse(h.saved['routea-bluefy-v2']),'minBattery'),false);
  assert.deepEqual(h.errors,[]);
});

test('firmware simulator keeps a trial at 30 degrees but still expires missing heartbeats',()=>{
  const rx=[],sim=new (require('../src/simulator.js').Simulator)(r=>rx.push(r),{unsafe:true});
  sim.receive(C.training('START'));sim.receive(C.prepare(71,30000,candidate));sim.receive(C.command('APPLY',71));
  sim.tick(200);assert.equal(sim.state,2);assert.equal(sim.pid.AP,75);
  assert.match(rx.find(r=>r.startsWith('$T1,F,')),/,A30.00,/);
  sim.tick(2000);assert.equal(sim.state,0);assert.deepEqual(sim.pid,C.DEFAULT_PID);
});
