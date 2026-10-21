const assert=require("node:assert/strict"),fs=require("node:fs"),path=require("node:path"),vm=require("node:vm");
const C=require("../src/core.js");
const nodes=new Map();
function node(id){if(!nodes.has(id))nodes.set(id,{value:"",textContent:"",innerHTML:"",disabled:false,clientWidth:430,
classList:{add(){},remove(){},toggle(){}},dataset:{},addEventListener(){},querySelector(){return node(id+"-child");},getContext(){return new Proxy({}, {get:()=>()=>{}});}});return nodes.get(id);}
let clock=0,nextId=1;const timers=new Map();
function timer(fn,ms=0,repeat=0){const id=nextId++;timers.set(id,{fn,at:clock+Math.max(1,ms),repeat});return id;}
async function flush(){for(let i=0;i<35;i++)await Promise.resolve();}
async function advance(ms){const end=clock+ms;await flush();for(;;){
const due=[...timers.entries()].filter(([,x])=>x.at<=end).sort((a,b)=>a[1].at-b[1].at)[0];if(!due)break;
clock=due[1].at;if(due[1].repeat)due[1].at+=due[1].repeat;else timers.delete(due[0]);due[1].fn();await flush();}clock=end;await flush();}
class ClockDate extends Date{constructor(...x){super(...(x.length?x:[1792569600000+clock]));}static now(){return 1792569600000+clock;}}
const sandbox={console,TextEncoder,TextDecoder,Date:ClockDate,AbortController,Promise,RouteACore:C,navigator:{},
document:{hidden:false,getElementById:node,querySelectorAll:()=>[],addEventListener(){}},localStorage:{getItem:()=>null,setItem(){}},
setTimeout:(f,n)=>timer(f,n),clearTimeout:id=>timers.delete(id),setInterval:(f,n)=>timer(f,n,n),clearInterval:id=>timers.delete(id)};
sandbox.window={RouteACore:C,devicePixelRatio:1,isSecureContext:true,addEventListener(){},scrollTo(){}};
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(path.join(__dirname,"../src/sequence.js"),"utf8"),sandbox);
sandbox.window.RouteASequence=sandbox.RouteASequence;
vm.runInContext(fs.readFileSync(path.join(__dirname,"../src/app.js"),"utf8").replace("  boot();","  window.testOnly={state,writer,startMock,applyManual,applyPreset,startAuto,stopAuto,handleFrame};\n  boot();"),sandbox);
require("node:test")("sequential manual/LLM UI contract",async()=>{
const t=sandbox.window.testOnly,s=t.state;t.startMock();
let operation=t.applyManual("N",{...C.presetPid("N"),AP:95},1);
await advance(100);assert.ok(s.pendingManual);assert.equal(s.mockPid.AP,95);assert.equal(s.mockPid.AD,48);
await advance(1500);await operation;assert.equal(s.pendingManual,null);assert.equal(s.currentPid.AP,95);
let frames=s.logs.filter(x=>x.direction==="TX"&&x.text.startsWith("$P3,ONE,"));assert.equal(frames.length,1);assert.match(frames[0].text,/,AP,95.00,C/);
assert.equal(frames[0].text.split(",").length,7,"ONE carries one key/value, not six coefficients");
operation=t.applyManual("N",{...s.currentPid,TP:18},4);await advance(1700);await operation;assert.equal(s.currentPid.TP,18);assert.equal(s.currentPid.AP,95);
const sixStart=s.logs.length;
operation=t.applyManual("N",{AP:94,AD:49,VP:63,VI:32,TP:19,TD:21});await advance(9000);await operation;
assert.equal(s.logs.slice(sixStart).filter(x=>x.direction==="TX"&&x.text.startsWith("$P3,ONE,")).length,6);
// A multi-field target is sent as distinct, confirmed messages spaced >=1000ms after readback.
const oldCount=s.logs.length;operation=t.applyPreset("L");await advance(8000);await operation;
const one=s.logs.slice(oldCount).filter(x=>x.direction==="TX"&&x.text.startsWith("$P3,ONE,"));
assert.ok(one.length>=4);for(let i=1;i<one.length;i++)assert.ok(new Date(one[i].time)-new Date(one[i-1].time)>=1000);
assert.equal(s.profile,"L");assert.equal(s.currentPid.AD,75);
operation=t.applyPreset("N");await advance(8000);await operation;
await t.startAuto();await advance(55000);
assert.ok(s.auto.history.filter(x=>x.result==="接受").length>=2,"two full LLM Mock acceptance cycles required");
assert.ok(s.logs.every(x=>x.direction!=="TX"||!/^\$P3,(MSET|PREP|APPLY|PPRESET|PRESET),/.test(x.text)),"neither manual nor LLM may send bulk update frames");
const stopped=t.stopAuto(true);await advance(500);await stopped;assert.equal(s.auto.enabled,false);assert.equal(s.trainingLocked,false);
// Same-generation status without the query correlation must never unblock the next write.
let sent=[];const w=new sandbox.RouteASequence.Writer({send:async raw=>{sent.push(raw);},timeoutMs:200,gapMs:1000});
const run=w.run({id:888,mode:"M",profile:"N",initialProfile:"N",initialPid:C.presetPid("N"),targetPid:{...C.presetPid("N"),AP:95,AD:49}});run.catch(()=>{});
await flush();w.feed(C.frame("P3,ACK,SEQ_BEGIN,888,1,0,PN"));await flush();
w.feed(C.frame("P3,SEQACK,ONE_APPLIED,887,1,AP,95.00"));await flush();assert.ok(!sent.some(x=>x.includes("SGET")));
w.feed(C.frame("P3,SEQACK,ONE_APPLIED,888,1,AP,95.00"));await flush();
w.feed(C.frame("P3,STATUS,95,48,62,31,17,20,PN,S2,L0,B1"));await flush();assert.equal(sent.filter(x=>x.includes(",ONE,")).length,1);
await advance(250);await assert.rejects(run,/超时/);assert.equal(sent.filter(x=>x.includes(",ONE,")).length,1);assert.ok(sent.some(x=>x.includes(",ROLLBACK,")));
// Cancel after a confirmed first step, during the spacing delay: no second write may escape.
let cancelledFrames=[];const cancelWriter=new sandbox.RouteASequence.Writer({send:async raw=>{cancelledFrames.push(raw);},gapMs:1000});
const cancelling=cancelWriter.run({id:889,mode:"M",profile:"N",initialProfile:"N",initialPid:C.presetPid("N"),targetPid:{...C.presetPid("N"),AP:95,AD:49}});cancelling.catch(()=>{});
await flush();cancelWriter.feed(C.frame("P3,ACK,SEQ_BEGIN,889,1,0,PN"));await flush();
cancelWriter.feed(C.frame("P3,SEQACK,ONE_APPLIED,889,1,AP,95.00"));await flush();
cancelWriter.feed(C.frame("P3,STATUS,95,48,62,31,17,20,PN,S2,L0,B1,Q889,I1"));await flush();
cancelWriter.cancel("用户取消");await advance(1500);await assert.rejects(cancelling,/用户取消/);
assert.equal(cancelledFrames.filter(x=>x.includes(",ONE,")).length,1);
assert.ok(cancelledFrames.some(x=>x.includes(",ROLLBACK,")));
// A negative/huge PID report locks actual UI; it must not reach editing or queued transmissions.
t.handleFrame(C.frame("P3,STATUS,1.29,-6.8056469327705764e38,98897880,0.39,3.60,0,PN,S0,L0,B1"));
assert.equal(s.hardwareFault,true);assert.equal(s.currentPid,null);assert.equal(node("applyManual").disabled,true);
assert.ok(!node("liveAD").textContent.includes("e+38"));
console.log("Sequential UI PASS: AP95/TP18 single-key frames; ACK+correlated readback+1000ms; no bulk manual/LLM writes; two Mock accepted cycles; wrong ACK/stale readback/timeout stop remainder; corrupt PID locked");
});
