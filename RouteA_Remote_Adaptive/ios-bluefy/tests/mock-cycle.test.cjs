const assert=require("node:assert/strict"),fs=require("node:fs"),path=require("node:path"),vm=require("node:vm");
const C=require("../src/core.js");
const nodes=new Map();
function node(id) {
  if(!nodes.has(id)) nodes.set(id,{value:"",textContent:"",innerHTML:"",disabled:false,clientWidth:430,
    classList:{add(){},remove(){},toggle(){}},dataset:{},addEventListener(){},querySelector(){return node(id+"-child");},
    getContext(){return new Proxy({}, {get:()=>()=>{}});}});
  return nodes.get(id);
}
let clock=0,nextId=1;const timers=new Map();
function timer(fn,ms=0,repeat=0) {const id=nextId++;timers.set(id,{fn,at:clock+Math.max(1,ms),repeat});return id;}
async function flush(){for(let n=0;n<15;n++)await Promise.resolve();}
async function advance(ms){const end=clock+ms;await flush();for(;;){
  const due=[...timers.entries()].filter(([,x])=>x.at<=end).sort((a,b)=>a[1].at-b[1].at)[0];
  if(!due)break;clock=due[1].at;if(due[1].repeat)due[1].at+=due[1].repeat;else timers.delete(due[0]);
  due[1].fn();await flush();
}clock=end;await flush();}
class ClockDate extends Date { constructor(...x){super(...(x.length?x:[1792483200000+clock]));}static now(){return 1792483200000+clock;} }
const sandbox={console,TextEncoder,TextDecoder,Date:ClockDate,AbortController,Promise,
 navigator:{},document:{hidden:false,getElementById:node,querySelectorAll:()=>[],addEventListener(){}},
 localStorage:{getItem:()=>null,setItem(){}},
 setTimeout:(fn,ms)=>timer(fn,ms),clearTimeout:id=>timers.delete(id),
 setInterval:(fn,ms)=>timer(fn,ms,ms),clearInterval:id=>timers.delete(id)};
sandbox.window={RouteACore:C,devicePixelRatio:1,isSecureContext:true,addEventListener(){},scrollTo(){}};
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(path.join(__dirname,"../src/app.js"),"utf8").replace("  boot();",
 "  window.testOnly={state,startMock,startAuto,stopAuto,applyManual,applyPreset};\n  boot();"),sandbox);
(async()=>{
 const t=sandbox.window.testOnly,s=t.state;t.startMock();
 await t.applyManual("N",{...C.presetPid("N"),AD:50,VP:70},1);
 assert.equal(s.mockPid.AD,48,"queued write must await a control boundary");
 await advance(100);assert.equal(s.pendingManual,null);assert.equal(s.mockPid.AD,50);assert.equal(s.mockPid.VP,62);
 await t.applyPreset("L");await advance(100);assert.equal(s.profile,"L");assert.equal(s.currentPid.AD,75);
 await t.applyPreset("N");await advance(100);assert.equal(s.profile,"N");
 await t.startAuto();await advance(42000);
 assert.ok(s.auto.enabled,"automatic loop must continue after acceptance");
 const accepted=s.auto.history.filter(x=>x.result==="接受");
 assert.ok(accepted.length>=2,"two actual mock PREP/APPLY/readback/evaluation/ACCEPT cycles required");
 assert.ok(s.logs.some(x=>x.direction==="TX"&&x.text.includes("P3,PREP,")&&x.text.includes(",M1,")));
 assert.ok(s.logs.some(x=>x.direction==="TX"&&x.text.includes("P3,GET,")));
 assert.ok(!s.hardwareFault);assert.ok(s.currentPid.AD>48);
 const stopping=t.stopAuto(true);await advance(500);await stopping;
 assert.equal(s.auto.enabled,false);assert.equal(s.trainingLocked,false);
 console.log(`Mock full-loop PASS: ${accepted.length} accepted cycles; boundary commit -> readback -> settle -> two windows -> ACCEPT; pause unlocks`);
})().catch(e=>{console.error(e);process.exit(1);});
