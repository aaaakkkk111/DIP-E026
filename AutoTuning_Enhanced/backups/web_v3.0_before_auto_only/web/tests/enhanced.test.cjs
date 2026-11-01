const {test}=require('node:test'),assert=require('node:assert/strict');
const {make}=require('./harness.cjs');
const C=require('../src/core.js');
const proposal=p=>({schema_version:1,decision:'propose',stage:'balance',candidate:{...p,AP:p.AP+1,AD:p.AD+1,VP:p.VP+1,VI:p.VI+.5,TP:p.TP+.5,TD:p.TD+.5},expected_effect:'验证六参数事务，模拟数据不能证明实车效果',requested_test:'quiet_balance',confidence:.8});
const response=p=>({ok:true,json:async()=>({choices:[{message:{content:JSON.stringify(p)}}]})});
async function start(h){h.set('llmKey').value='test-only-key';await h.action('mock');await h.advance(4000);h.set('baselineSeconds').value='5';h.set('trialSeconds').value='5';h.set('loopCooldown').value='2';h.set('loopReview').value='rules-auto';h.set('continuousConsent').checked=true;const pending=h.action('continuous-start');await h.flush();return {pending};}
test('feature frames are CRC checked, finite, strictly prefixed; full six-parameter candidates allowed',()=>{
 const f=C.parseFeatures(C.frame('T1,E,200,O12.300,L1.250,B-2.500,W1'));assert.equal(f.osc,12.3);assert.equal(f.leanBand,-2.5);assert.equal(f.warm,true);
 for(const body of ['T1,E,0,O-1,L1,B1,W1','T1,E,0,ONaN,L1,B1,W1','T1,E,0,O1,L1,B1,W2','T1,E,0,X1,L1,B1,W1'])assert.equal(C.parseFeatures(C.frame(body)),null);
 const p=proposal(C.DEFAULT_PID);assert.deepEqual(C.validateProposal(p,C.DEFAULT_PID).candidate,p.candidate);assert.ok(C.prepare(1999999999,30000,{AP:288,AD:200,VP:72,VI:100,TP:100,TD:100}).length<80);
});
test('continuous automatic session runs two accepted rounds with six fields and feature evidence',async()=>{
 const requests=[],h=make({fetch:async(url,init)=>{const d=JSON.parse(JSON.parse(init.body).messages[1].content);requests.push(d);return response(proposal(d.current_pid));}});
 const Sim=h.context.RouteASimulator.Simulator,original=Sim.prototype.tick;Sim.prototype.tick=function(ms){
   const emit=this.emit;this.emit=raw=>{const p=h.context.RouteACore.bodyParts; // Keep wire CRC correct while modeling improvement after each application.
     let body=raw.slice(1,raw.lastIndexOf(',C'));const stage=this.state===2?this.pid.AP-96:Math.max(0,this.champion.AP-96);const scale=Math.pow(.65,stage);
     if(body.startsWith('T1,F,'))body=body.replace(/,A(-?[\d.]+)/,(_,v)=>',A'+(Number(v)*scale).toFixed(2)).replace(/,G(-?[\d.]+)/,(_,v)=>',G'+(Number(v)*scale).toFixed(1));
     if(body.startsWith('T1,E,'))body=body.replace(/,([OLB])(-?[\d.]+)/g,(_,k,v)=>','+k+(Number(v)*scale).toFixed(3));emit(h.context.RouteACore.frame(body));};
   original.call(this,ms);this.emit=emit;
 };
 const {pending}=await start(h);await h.advance(30000);assert.ok(h.snapshot().history.filter(x=>x.outcome==='accepted').length>=2,JSON.stringify(h.snapshot()));
 assert.ok(requests.length>=2);assert.deepEqual(requests[0].harness_limits.allowed_parameters,C.PID_KEYS);assert.equal(requests[0].harness_limits.feature_definition.sample_hz,200);assert.ok(requests[0].metrics.featureSamples>=20);assert.ok(requests[0].telemetry_samples.some(x=>x.osc!==null));
 assert.equal(h.set('manualPidButton').disabled,true);await h.action('continuous-stop');await h.advance(200);await pending;assert.equal(h.snapshot().continuous.running,false);assert.equal(h.snapshot().locked,false);assert.deepEqual(h.errors,[]);
});
test('unchanged simulated performance causes rollback and three unsuccessful rounds stop the session',async()=>{
 const h=make({fetch:async(url,init)=>response(proposal(JSON.parse(JSON.parse(init.body).messages[1].content).current_pid))});const {pending}=await start(h);await h.advance(47000);await pending;
 assert.equal(h.snapshot().continuous.running,false);assert.ok(h.snapshot().history.filter(x=>x.outcome==='rolled_back').length>=3);assert.equal(h.snapshot().pid.AP,96);assert.equal(h.snapshot().locked,false);assert.deepEqual(h.errors,[]);
});
test('hold never creates a firmware trial; stopping releases the lock and prevents new LLM calls',async()=>{
 let calls=0;const h=make({fetch:async(url,init)=>{calls++;const p=proposal(JSON.parse(JSON.parse(init.body).messages[1].content).current_pid);p.decision='hold';p.candidate=JSON.parse(JSON.parse(init.body).messages[1].content).current_pid;return response(p);}});const {pending}=await start(h);await h.advance(14000);assert.ok(calls>=1);assert.doesNotMatch(h.set('transactionLog').value,/TX \$P1,PREP/);await h.action('continuous-stop');await h.advance(300);await pending;const old=calls;await h.advance(10000);assert.equal(calls,old);assert.equal(h.snapshot().locked,false);assert.deepEqual(h.errors,[]);
});
test('version mismatch is readable but cannot write or start continuous tuning',async()=>{
 const h=make(),ble=h.installBle();await h.action('connect');ble.emit(h.context.RouteACore.frame('T1,S,1000,V12,B0,V0,T0,AP96,AD48,VP62,VI31,TP17,TD20,D0,L0,R1,RA1.8-PID-NO-PCT-CAP'));assert.equal(h.set('manualPidButton').disabled,true);assert.equal(h.set('continuousStart').disabled,true);
});
test('backgrounding during continuous sampling cancels the session before any PREP',async()=>{
 let calls=0;const h=make({fetch:async()=>{calls++;return response({});}});const {pending}=await start(h);await h.advance(1000);h.doc.hidden=true;await h.doc.dispatch('visibilitychange');await h.advance(500);await pending;assert.equal(h.snapshot().continuous.running,false);assert.equal(calls,0);assert.doesNotMatch(h.set('transactionLog').value,/TX \$P1,PREP/);
});

test('stopping a continuous active candidate rolls back all six and unlocks without another PREP',async()=>{const h=make({fetch:async(url,init)=>response(proposal(JSON.parse(JSON.parse(init.body).messages[1].content).current_pid))});const {pending}=await start(h);await h.advance(6500);assert.equal(h.snapshot().phase,'TRIAL');assert.equal(h.snapshot().pid.VP,63);await h.action('continuous-stop');await h.advance(300);await pending;assert.equal(h.snapshot().locked,false);assert.equal(h.snapshot().id,0);assert.equal(h.snapshot().pid.VP,62);assert.equal(h.snapshot().pid.AP,96);const count=h.snapshot().history.length;await h.advance(8000);assert.equal(h.snapshot().history.length,count);assert.deepEqual(h.errors,[]);});
test('missing LLM credentials cannot lock training or start a transaction',async()=>{const h=make();await h.action('mock');await h.advance(4000);h.set('continuousConsent').checked=true;await h.action('continuous-start');assert.equal(h.snapshot().locked,false);assert.equal(h.snapshot().id,0);assert.equal(h.snapshot().continuous.running,false);assert.deepEqual(h.errors,[]);});
test('offline simulated continuous demo works without an API key and labels the evidence',async()=>{const h=make();await h.action('mock');await h.advance(4000);h.set('loopSource').value='demo';h.set('loopReview').value='rules-auto';h.set('baselineSeconds').value='5';h.set('trialSeconds').value='5';h.set('continuousConsent').checked=true;const pending=h.action('continuous-start');await h.flush();await h.advance(16000);assert.ok(h.snapshot().history.length>=1);assert.equal(h.snapshot().history[0].source,'simulation');await h.action('continuous-stop');await h.advance(300);await pending;assert.equal(h.snapshot().locked,false);assert.deepEqual(h.errors,[]);});
