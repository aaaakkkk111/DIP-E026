const {test}=require('node:test'),assert=require('node:assert/strict');
const {make}=require('./harness.cjs'),C=require('../src/core.js');
const candidate={AP:110,AD:60,VP:80,VI:20,TP:18,TD:21};
const proposal={schema_version:1,decision:'propose',stage:'balance',candidate,expected_effect:'Simulated candidate for workflow testing; not a real-vehicle result.',requested_test:'quiet_balance',confidence:.7};
const reply=p=>({ok:true,json:async()=>({choices:[{message:{content:JSON.stringify(p)}}]})});
async function language(h,value){h.set('languageSelect').value=value;await h.set('languageSelect').dispatch('change');}
function improveTrial(h){
  const Sim=h.context.RouteASimulator.Simulator,tick=Sim.prototype.tick;
  Sim.prototype.tick=function(ms){const emit=this.emit;this.emit=raw=>{let body=raw.slice(1,raw.lastIndexOf(',C'));if(this.state===2){if(body.startsWith('T1,F,'))body=body.replace(/,([AG])(-?[\d.]+)/g,(_,k,v)=>','+k+(Number(v)*.2).toFixed(2));if(body.startsWith('T1,E,'))body=body.replace(/,([OLB])(-?[\d.]+)/g,(_,k,v)=>','+k+(Number(v)*.2).toFixed(3));}emit(C.frame(body));};tick.call(this,ms);this.emit=emit;};
}
async function start(h,ble=false){
  h.set('llmKey').value='i18n-test-key';if(!ble)await h.action('mock');
  await h.advance(4000);const pending=h.action('continuous-start');await h.flush();return {pending};
}
async function stop(h,pending){await h.action('continuous-stop');await h.advance(300);await pending;assert.equal(h.snapshot().locked,false);assert.deepEqual(h.errors,[]);}

test('Chinese is the default; English covers all static and authored dynamic phrases',async()=>{
  const h=make(),I=h.context.RouteAI18n;assert.equal(h.snapshot().language,'zh');
  await language(h,'en');assert.equal(h.doc.documentElement.lang,'en');
  for(const key of Object.keys(I.catalog))assert.doesNotMatch(I.t(key),/[\u3400-\u9fff]/,key);
  for(const key of Object.values(I.staticTexts))assert.doesNotMatch(I.t(key),/[\u3400-\u9fff]/,key);
  assert.equal(h.set('continuousState').textContent,'Stopped · Not started');
  assert.equal(h.doc.querySelector('[data-action="stop"]').disabled,false);
  assert.deepEqual(h.errors,[]);
});

test('language preference persists and explicit query language is respected',async()=>{
  const h=make();await language(h,'en');
  assert.equal(h.saved['routea-ui-language'],'en');
  const next=make({saved:h.saved});assert.equal(next.snapshot().language,'en');
  const zh=make({saved:h.saved,search:'?lang=zh'});assert.equal(zh.snapshot().language,'zh');
  const en=make({search:'?lang=en'});assert.equal(en.snapshot().language,'en');
  assert.equal(en.context.RouteAI18n.setLanguage('invalid'),false);assert.equal(en.snapshot().language,'en');
});

test('English manual confirmation, six-PID writes, validation errors and CSV/JSON export work',async()=>{
  const h=make({search:'?lang=en'}),confirmed=[];h.context.confirm=message=>{confirmed.push(message);return true;};
  await h.action('mock');await h.advance(4000);
  for(const k of C.PID_KEYS)h.set('edit'+k).value=String(candidate[k]);
  await h.action('manual-pid');assert.deepEqual(JSON.parse(JSON.stringify(h.snapshot().pid)),candidate);
  assert.match(confirmed[0],/Write all six PID/);assert.match(h.set('operationStatus').textContent,/read back correctly/);
  h.set('editAP').value='NaN';await h.action('manual-pid');assert.match(h.set('operationStatus').textContent,/must be a finite number/);
  assert.doesNotMatch(h.set('operationStatus').textContent,/[\u3400-\u9fff]/);
  await h.action('export-csv');await h.action('export-json');
  assert.equal(h.downloads.length,2);const data=JSON.parse(await h.downloads[1].blob.text());
  assert.equal(data.pid.AP,110);assert.ok(data.telemetry.some(r=>r.kind==='fast'));
  assert.doesNotMatch(await h.downloads[1].blob.text(),/i18n-test-key/);assert.deepEqual(h.errors,[]);
});

test('switching language during an applied BLE trial sends no command and preserves connection, PID, lock, input and key',async()=>{
  const h=make({fetch:async()=>reply(proposal)}),b=h.installBle();improveTrial(h);
  h.context.setInterval(()=>b.tick(200),200);await h.action('connect');
  const {pending}=await start(h,true);await h.advance(4500);assert.equal(h.snapshot().phase,'TRIAL');
  const before=h.snapshot(),writes=b.writes.length;h.set('editAP').value='123.45';
  await language(h,'en');const after=h.snapshot();
  for(const key of ['connected','verified','locked','fw','phase','id'])assert.equal(after[key],before[key],key);
  assert.deepEqual(after.pid,before.pid);assert.equal(b.writes.length,writes);
  assert.equal(h.set('editAP').value,'123.45');assert.equal(h.set('llmKey').value,'i18n-test-key');
  assert.equal(after.continuous.running,true);assert.match(h.set('continuousState').textContent,/Trial acquisition/);
  assert.match(h.set('transactionLog').value,/TX \$P1,APPLY,/);
  await language(h,'zh');assert.equal(h.snapshot().id,before.id);await stop(h,pending);
});

test('Chinese and English run identical fragmented BLE command flows through automatic ACCEPT and unlock',async()=>{
  const results={};
  for(const lang of ['zh','en']){
    const requests=[],h=make({search:'?lang='+lang,fetch:async(url,init)=>{requests.push(JSON.parse(init.body));return reply(proposal);}}),b=h.installBle();
    improveTrial(h);h.context.setInterval(()=>b.tick(200),200);await h.action('connect');
    const {pending}=await start(h,true);await h.advance(8500);
    assert.ok(h.snapshot().history.some(r=>r.outcome==='accepted'));assert.equal(h.snapshot().pid.AP,110);
    await stop(h,pending);await h.action('export-json');
    const data=JSON.parse(await h.downloads.at(-1).blob.text());
    const commands=data.logs.filter(r=>r.direction==='TX').map(r=>r.text);
    for(const raw of commands)if(raw.startsWith('$P1,'))assert.equal(C.verify(raw),true);
    assert.ok(b.writes.every(chunk=>chunk.length<=20));
    results[lang]={requests,commands:commands.map(raw=>{const fields=raw.slice(1,raw.lastIndexOf(',C')).split(',');if(['PREP','APPLY','HB','ACCEPT','ROLLBACK'].includes(fields[1]))fields[2]='ID';return fields.join(',');})};
  }
  assert.deepEqual(results.en.commands,results.zh.commands);
  assert.equal(results.en.requests[0].messages[0].content,results.zh.requests[0].messages[0].content.replace('Write expected_effect in Simplified Chinese.','Write expected_effect in English.'));
  const en=JSON.parse(results.en.requests[0].messages[1].content),zh=JSON.parse(results.zh.requests[0].messages[1].content);
  assert.equal(en.ui_language,'en');delete en.ui_language;delete zh.ui_language;assert.deepEqual(en,zh);
});

test('English independent LLM review requests English reason without changing schema or empirical priors',async()=>{
  const requests=[],h=make({search:'?lang=en',fetch:async(url,init)=>{const q=JSON.parse(init.body);requests.push(q);return reply(JSON.parse(q.messages[1].content).proposal?{approved:true,reason:'Simulation review passed'}:proposal);}});
  h.set('loopReview').value='llm-auto';const {pending}=await start(h);await h.advance(4500);
  assert.equal(requests.length,2);assert.match(requests[0].messages[0].content,/Write expected_effect in English/);
  assert.match(requests[1].messages[0].content,/Write reason in English/);
  assert.match(requests[1].messages[0].content,/NO PAYLOAD on LEVEL GROUND/);
  assert.equal(h.snapshot().phase,'TRIAL');await stop(h,pending);
});

test('translation preserves raw frames, model JSON and verbatim previous model explanations',async()=>{
  const h=make({search:'?lang=en'}),I=h.context.RouteAI18n,frame=C.getPid(),raw='{"expected_effect":"原始模型解释"}';
  assert.equal(I.logText({direction:'TX',text:frame}),frame);
  assert.equal(I.logText({direction:'LLM',text:raw}),raw);
  assert.equal(I.t('本轮建议 · '+I.verbatim('原始模型解释')),"This round's proposal · 原始模型解释");
  I.text(h.set('llmResponse'),raw);await language(h,'zh');await language(h,'en');
  assert.equal(h.set('llmResponse').textContent,raw);
});

test('English demo explanations are localized without calling an external LLM',async()=>{
  const h=make({search:'?lang=en',fetch:async()=>{throw Error('No API call in demo');}});
  h.set('loopSource').value='demo';const {pending}=await start(h);await h.advance(8500);
  assert.ok(h.snapshot().history.length);
  assert.doesNotMatch(h.set('candidateBox').textContent,/[\u3400-\u9fff]/);
  assert.match(h.set('candidateBox').textContent,/Simulated candidate/);
  await stop(h,pending);
});
