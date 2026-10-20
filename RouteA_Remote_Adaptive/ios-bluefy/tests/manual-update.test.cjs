const assert=require("node:assert/strict");
const fs=require("node:fs"),path=require("node:path"),vm=require("node:vm");
const C=require("../src/core.js");
const nodes=new Map();
function node(id) {
  if(!nodes.has(id)) nodes.set(id,{value:"",textContent:"",innerHTML:"",disabled:false,clientWidth:320,
    classList:{add(){},remove(){},toggle(){}},dataset:{},addEventListener(){},querySelector(){return node(id+"-child");},
    getContext(){return new Proxy({}, {get:()=>()=>{}});}});
  return nodes.get(id);
}
const sandbox={console,TextEncoder,TextDecoder,Date,AbortController,Promise,
  navigator:{},document:{hidden:false,getElementById:node,querySelectorAll:()=>[],addEventListener(){}},
  localStorage:{getItem:()=>null,setItem(){}},setTimeout:()=>1,clearTimeout(){},setInterval:()=>1,clearInterval(){}};
sandbox.window={RouteACore:C,devicePixelRatio:1,isSecureContext:true,addEventListener(){},scrollTo(){}};
vm.createContext(sandbox);
const app=fs.readFileSync(path.join(__dirname,"../src/app.js"),"utf8");
vm.runInContext(app.replace("  boot();","  window.testOnly = { state, handleFrame, applyManual, applyPreset, startAuto, decideNext, finishWindow };\n  boot();"),sandbox);
(async()=>{
  const t=sandbox.window.testOnly,s=t.state;
  s.mock=true;s.balanceStarted=true;s.currentPid=C.presetPid("N");
  const target={...C.presetPid("N"),AD:54};
  await t.applyManual("N",target);
  assert.ok(s.pendingManual);
  assert.equal(node("applyManual").disabled,true);
  await t.applyManual("N",{...target,AD:55});
  assert.equal(s.pendingManual.pid.AD,54,"second click must not overwrite in-flight target");
  t.handleFrame(C.frame("P3,ACK,MSET_QUEUED,0,0,0,PN"));
  assert.ok(s.pendingManual,"queue ACK is not application confirmation");
  t.handleFrame(C.frame("P3,STATUS,96,48,62,31,17,20,PN,S0,L0,B1"));
  assert.ok(s.pendingManual,"old telemetry must not confirm target");
  t.handleFrame(C.frame("P3,ACK,MSET_APPLIED,0,0,0,PN"));
  assert.ok(s.pendingManual,"completion ACK still requires exact PID readback");
  t.handleFrame(C.frame("P3,STATUS,96,53,62,31,17,20,PN,S0,L0,B1"));
  assert.ok(s.pendingManual,"wrong readback must not confirm target");
  t.handleFrame(C.frame("P3,STATUS,96,54,62,31,17,20,PN,S0,L0,B1"));
  assert.equal(s.pendingManual,null);assert.equal(node("applyManual").disabled,false);
  await t.applyManual("N",target);
  await t.startAuto();assert.equal(s.auto.enabled,false,"no auto start during pending manual update");
  t.handleFrame(C.frame("P3,ERR,MSET_ROLLBACK,0"));
  assert.equal(s.pendingManual,null);assert.equal(node("applyManual").disabled,false);
  const priorTx=s.logs.filter(x=>x.direction==="TX" && x.text.includes("P3,PREP")).length;
  s.auto.enabled=true;
  const decisionPromise=t.decideNext({absLean:0,motorEffort:200});
  s.auto.enabled=false;
  await decisionPromise;
  assert.equal(s.logs.filter(x=>x.direction==="TX" && x.text.includes("P3,PREP")).length,priorTx,
    "a pending LLM response must not apply after training is paused");
  s.auto.enabled=true;s.auto.phase="BASELINE";
  s.auto.fastSamples=[0,200,400].map(ms=>({ms,angle:25,gyro:1000,osc:10,lean:8,ml:2600,mr:2600,el:1,er:1}));
  s.auto.slowSamples=[];
  await t.finishWindow("BASELINE");
  assert.equal(s.auto.enabled,false,"dangerous baseline must stop before LLM application");
  assert.equal(s.logs.filter(x=>x.direction==="TX" && x.text.includes("P3,PREP")).length,priorTx);
  // The browser must wait for actual applied-state readback before evaluating.
  s.auto.enabled=true;s.auto.phase="APPLYING";s.auto.candidatePid={...target};s.auto.candidateProfile="N";
  t.handleFrame(C.frame("T3,F,200,A1,G0,O0.5,N0,EL0,ER0,ML20,MR20,S2,L1,PN,U0"));
  assert.equal(s.auto.phase,"APPLYING","TRIAL status alone is not PID confirmation");
  t.handleFrame(C.frame("P3,STATUS,96,48,62,31,17,20,PN,S2,L1,B1"));
  assert.equal(s.auto.phase,"APPLYING");
  t.handleFrame(C.frame("P3,STATUS,96,54,62,31,17,20,PN,S2,L1,B1"));
  assert.equal(s.auto.phase,"SETTLING");
  s.auto.enabled=false;s.trainingLocked=false;s.firmwareState=0;s.fast=null;
  await t.applyManual("N",{...target,VP:70},1);
  assert.equal(s.pendingManual.pid.VP,62,"balance-only update must preserve actual velocity parameters");
  assert.match(s.logs.filter(x=>x.direction==="TX").at(-1).text,/,M1,C/);
  s.pendingManual=null;
  await t.applyPreset("L");
  assert.match(s.logs.filter(x=>x.direction==="TX").at(-1).text,/P3,PRESET,L,/);
  s.pendingManual=null;s.mock=false;s.connected=true;s.firmware="RA3.2-STACKFIX";
  await t.applyManual("N",target);assert.equal(s.pendingManual,null,"old firmware must not enable writes");
  s.mock=true;s.connected=false;
  const input=fs.readFileSync(path.join(__dirname,"../../tests/fixtures/20261014_user_llm_fall.log"),"utf8");
  const replayFrames=[...input.matchAll(/RX\s+(\$.*?#)/g)].map(x=>x[1]);
  const corrupt=replayFrames.find(raw=>{const v=C.parseSlow(raw);if(!v)return false;try{C.validatePid(v.pid);return false;}catch(_){return true;}});
  assert.ok(corrupt && C.verify(corrupt),"fixture must contain CRC-valid corrupt PID");
  t.handleFrame(corrupt);
  assert.equal(s.hardwareFault,true);assert.equal(s.currentPid,null);
  assert.equal(s.auto.enabled,false);assert.equal(node("applyManual").disabled,true);
  const stops=s.logs.filter(x=>x.direction==="TX" && x.text.startsWith("$0,"));
  assert.ok(stops.length);assert.ok(stops.every(x=>x.text.length<80));
  console.log("Real fall log replay: PASS (corrupt PID locks UI, stops training, STOP stays within RX buffer)");
  console.log("Manual update UI regression tests: PASS (queued/applied/readback, duplicate prevention, rollback, auto interlock)");
})().catch(e=>{console.error(e);process.exit(1);});
