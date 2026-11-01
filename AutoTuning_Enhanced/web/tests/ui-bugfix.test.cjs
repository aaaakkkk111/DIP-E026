const {test}=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const {make}=require('./harness.cjs'),C=require('../src/core.js'),P=require('../src/plot.js');
const actual=JSON.parse(fs.readFileSync(path.join(__dirname,'../../tests/fixtures/english_user_candidate_20261025.json'),'utf8'));
const reply=(value,finish_reason='stop')=>({ok:true,json:async()=>({choices:[{finish_reason,message:{content:typeof value==='string'?value:JSON.stringify(value)}}]})});
async function start(h){await h.action('mock');await h.advance(4000);h.set('llmKey').value='bugfix-test-key';const pending=h.action('continuous-start');await h.flush();return {pending};}
async function stop(h,pending){await h.action('continuous-stop');await h.advance(300);await pending;assert.equal(h.snapshot().locked,false);assert.deepEqual(h.errors,[]);}

test('replays the actual English candidate over 500 chars and confirms AP75/AD55/VI10 by APPLY readback',async()=>{
  assert.ok(actual.expected_effect.length>500&&actual.expected_effect.length<C.MAX_EXPLANATION_CHARS);
  const h=make({search:'?lang=en',fetch:async()=>reply(actual)});
  await h.action('mock');await h.advance(4000);
  h.set('editAP').value='50';h.set('editAD').value='45';await h.action('manual-pid');h.set('llmKey').value='bugfix-test-key';
  const pending=h.action('continuous-start');await h.flush();await h.advance(4500);
  assert.equal(h.snapshot().phase,'TRIAL');assert.equal(h.snapshot().lastFailure,null);
  assert.deepEqual(JSON.parse(JSON.stringify(h.snapshot().pid)),actual.candidate);
  assert.match(h.set('transactionLog').value,/TX \$P1,APPLY,/);
  assert.match(h.set('transactionLog').value,/RX \$P1,PID,75.00,55.00,62.00,10.00,17.00,20.00,2,1,1,/);
  assert.equal(h.set('llmResponse').textContent,JSON.stringify(actual));
  await stop(h,pending);
});

for(const lang of ['zh','en'])test(lang+' accepts long explanations equally while PID/schema validation remains strict',()=>{
  const h=make({search:'?lang='+lang});
  const q={...actual,expected_effect:'Explaining the observed telemetry. '.repeat(70)};
  assert.equal(h.context.RouteACore.validateProposal(q,C.DEFAULT_PID).expected_effect,q.expected_effect);
  assert.throws(()=>C.validateProposal({...q,candidate:{...q.candidate,AP:999}},C.DEFAULT_PID));
  assert.throws(()=>C.validateProposal({...q,confidence:'0.5'},C.DEFAULT_PID));
  assert.throws(()=>C.validateProposal({...q,unexpected:true},C.DEFAULT_PID));
});

for(const [name,fetch,pattern]of [
  ['invalid JSON',async()=>reply('{"schema_version":1'),/not valid JSON/],
  ['oversized explanation',async()=>reply({...actual,expected_effect:'x'.repeat(3001)}),/explanation is too long.*3001.*3000/],
  ['truncated response',async()=>reply(actual,'length'),/truncated.*finish_reason=length/],
  ['HTTP rejection',async()=>({ok:false,status:401,text:async()=>'Bearer bugfix-test-key denied'}),/HTTP 401/]
])test(name+' keeps a persistent English failure reason after training unlock, without sending a candidate',async()=>{
  const h=make({search:'?lang=en',fetch}),{pending}=await start(h);await h.advance(6000);await pending;
  assert.equal(h.snapshot().continuous.running,false);assert.equal(h.snapshot().locked,false);
  assert.equal(h.set('autoError').hidden,false);assert.match(h.set('autoErrorReason').textContent,pattern);
  assert.match(h.set('autoErrorEffect').textContent,/No PID candidate was sent/);
  assert.match(h.set('autoErrorStage').textContent,/LLM generating/);
  assert.match(h.set('llmStatus').textContent,/LLM processing failed/);
  assert.doesNotMatch(h.set('transactionLog').value,/TX \$P1,(PREP|APPLY|ROLLBACK),/);
  const saved=h.set('autoErrorReason').textContent;await h.advance(10000);assert.equal(h.set('autoErrorReason').textContent,saved);
  await h.action('export-json');const exported=await h.downloads.at(-1).blob.text(),data=JSON.parse(exported);
  assert.ok(data.lastFailure);assert.equal(data.lastFailure.noCandidateSent,true);
  assert.doesNotMatch(exported,/bugfix-test-key/);assert.deepEqual(h.errors,[]);
});

test('persistent failure remains readable across language changes and a successful restart clears it',async()=>{
  let bad=true;const h=make({search:'?lang=en',fetch:async()=>bad?reply('{'):reply(actual)}),{pending}=await start(h);
  await h.advance(5000);await pending;h.set('languageSelect').value='zh';await h.set('languageSelect').dispatch('change');
  assert.match(h.set('autoErrorTitle').textContent,/自动调参已停止/);assert.match(h.set('autoErrorReason').textContent,/LLM返回不是有效JSON/);
  bad=false;const restarted=h.action('continuous-start');await h.flush();assert.equal(h.snapshot().lastFailure,null);assert.equal(h.set('autoError').hidden,true);
  await h.advance(4500);assert.equal(h.snapshot().phase,'TRIAL');await stop(h,restarted);
});

test('user cancellation of a pending LLM request does not create a spurious failure alert',async()=>{
  let release;const h=make({search:'?lang=en',fetch:()=>new Promise(r=>release=r)}),{pending}=await start(h);
  await h.advance(4000);assert.equal(typeof release,'function');await h.action('continuous-stop');release(reply(actual));await h.flush();await h.advance(300);await pending;
  assert.equal(h.snapshot().lastFailure,null);assert.equal(h.snapshot().locked,false);assert.deepEqual(h.errors,[]);
});

test('long English independent-review reasons do not reject a valid candidate merely due to language length',async()=>{
  const reason='Reviewing telemetry, parameter bounds and the candidate explanation. '.repeat(25);
  assert.ok(reason.length>1000&&reason.length<C.MAX_EXPLANATION_CHARS);
  const h=make({search:'?lang=en',fetch:async(url,init)=>reply(JSON.parse(JSON.parse(init.body).messages[1].content).proposal?{approved:true,reason}:actual)});
  h.set('loopReview').value='llm-auto';const {pending}=await start(h);await h.advance(4500);
  assert.equal(h.snapshot().phase,'TRIAL');assert.equal(h.snapshot().lastFailure,null);await stop(h,pending);
});

test('plot uses independent physical-angle/raw-gyro axes and contains the actual 2006-count user sample',()=>{
  const now=50000,samples=[{at:49000,angle:-14.1,gyro:-1900},{at:49500,angle:17.07,gyro:2006},{at:50000,angle:4.98,gyro:1479}],copy=JSON.stringify(samples);
  for(const key of ['angle','gyro']){
    const p=P.series(samples,key,now,430,180);assert.equal(p.points.length,3);
    assert.ok(p.limit>Math.max(...samples.map(s=>Math.abs(s[key]))));
    for(const point of p.points){assert.ok(point.y>=p.top&&point.y<=p.bottom);assert.ok(point.x>=p.left&&point.x<=p.right);}
    assert.deepEqual(p.points.map(x=>x.value),samples.map(x=>x[key]));
  }
  assert.equal(P.series(samples,'gyro',now,430,180).unit,'raw');assert.equal(JSON.stringify(samples),copy);
  assert.notEqual(P.series(samples,'angle',now,430,180).limit,P.series(samples,'gyro',now,430,180).limit);
});

test('plot includes raw extremes, excludes expired/nonfinite data, and never changes measurement metrics',()=>{
  const now=100000,samples=[{at:50000,angle:100,gyro:32767},{at:99000,angle:0,gyro:-32768},{at:99500,angle:1,gyro:32767},{at:100000,angle:2,gyro:NaN}];
  const p=P.series(samples,'gyro',now,430,180);assert.equal(p.points.length,2);assert.ok(p.limit>=32768);
  assert.equal(P.series([],'angle',now,430,180).limit,5);assert.equal(P.series([],'gyro',now,430,180).limit,100);
  const data=[{at:1,angle:2,gyro:2006,ml:1500,mr:1500},{at:201,angle:-2,gyro:-1900,ml:1600,mr:1600}];
  const metrics=C.metrics(data);P.series(data,'gyro',201,430,180);assert.deepEqual(C.metrics(data),metrics);
});

test('existing acceptance policy and PID range limits are byte-for-byte unchanged as data',()=>{
  const old=require('../../backups/v3.6_before_web_error_plot_fix/web/src/core.js');
  assert.deepEqual(C.ACCEPTANCE,old.ACCEPTANCE);assert.deepEqual(C.RANGES,old.RANGES);
  for(const p of [C.DEFAULT_PID,actual.candidate]){assert.equal(C.manualSet(p),old.manualSet(p));assert.equal(C.prepare(71,30000,p),old.prepare(71,30000,p));}
});
