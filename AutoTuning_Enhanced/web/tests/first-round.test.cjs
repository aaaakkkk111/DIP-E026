const {test}=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const {make}=require('./harness.cjs'),C=require('../src/core.js');
const baseline={AP:150,AD:120,VP:62,VI:31,TP:17,TD:20},candidate={AP:165,AD:55,VP:50,VI:10,TP:17,TD:20};
const root=path.resolve(__dirname,'../..'),read=p=>fs.readFileSync(path.join(root,p),'utf8');
const reply=value=>({ok:true,json:async()=>({choices:[{finish_reason:'stop',message:{content:JSON.stringify(value)}}]})});
const proposal={schema_version:1,decision:'propose',stage:'balance',candidate,expected_effect:'Simulation only: first-round substantial AD correction and coordinated AP/velocity hypothesis; no physical improvement claimed.',requested_test:'quiet_balance',confidence:.45};
async function setup(h){await h.action('mock');await h.advance(4000);for(const [k,v]of Object.entries(baseline))h.set('edit'+k).value=String(v);await h.action('manual-pid');h.set('llmKey').value='first-round-test-only';}
async function stop(h,pending){await h.action('continuous-stop');await h.advance(300);await pending;assert.equal(h.snapshot().locked,false);assert.deepEqual(h.errors,[]);}
for(const lang of ['zh','en'])for(const independent of [false,true])test(lang+' large first-round corrective candidate reaches confirmed APPLY'+(independent?' through optional independent review':''),async()=>{
  const requests=[],h=make({search:'?lang='+lang,fetch:async(url,init)=>{const q=JSON.parse(init.body),d=JSON.parse(q.messages[1].content);requests.push({q,d});return reply(d.proposal?{approved:true,reason:'Mock review: magnitude alone does not require rejection'}:proposal);}});
  await setup(h);if(independent)h.set('loopReview').value='llm-auto';
  const pending=h.action('continuous-start');await h.flush();await h.advance(4500);
  assert.equal(h.snapshot().phase,'TRIAL');assert.deepEqual(JSON.parse(JSON.stringify(h.snapshot().pid)),candidate);
  assert.equal(requests.length,independent?2:1);
  for(const {q,d}of requests){assert.equal(d.tuning_round.number,1);assert.equal(d.tuning_round.first_round,true);assert.equal(d.local_direction_summary.comparisons.length,0);assert.match(q.messages[0].content,/FIRST-ROUND DECISIVE CORRECTION/);assert.match(q.messages[0].content,/Do not require a preliminary tiny probe/);assert.equal(d.harness_limits.relative_change_limit,null);assert.deepEqual(d.harness_limits.allowed_parameters,C.PID_KEYS);}
  assert.match(h.set('transactionLog').value,/TX \$P1,PREP,\d+,30000,165.00,55.00,50.00,10.00,17.00,20.00,/);
  await stop(h,pending);assert.deepEqual(JSON.parse(JSON.stringify(h.snapshot().pid)),baseline,'normal stop still restores the confirmed baseline');
});
test('round metadata distinguishes the next round and resets to first round after a deliberate restart',async()=>{
  const requests=[],h=make({fetch:async(url,init)=>{const q=JSON.parse(init.body),d=JSON.parse(q.messages[1].content);requests.push(d);return reply(requests.length===1?proposal:{...proposal,decision:'hold',candidate:d.current_pid});}});
  await setup(h);h.set('loopCooldown').value='2';const pending=h.action('continuous-start');await h.flush();await h.advance(13000);
  assert.ok(requests.length>=2);assert.equal(requests[0].tuning_round.first_round,true);assert.equal(requests[1].tuning_round.number,2);assert.equal(requests[1].tuning_round.first_round,false);assert.equal(requests[1].local_direction_summary.comparisons.length,1);
  await stop(h,pending);const n=requests.length,restarted=h.action('continuous-start');await h.flush();await h.advance(4500);
  assert.ok(requests.length>n);assert.equal(requests.at(-1).tuning_round.number,1);assert.equal(requests.at(-1).tuning_round.first_round,true);await stop(h,restarted);
});
test('first-round permission does not introduce a forced large step or weaken numeric validation',()=>{
  const policy=read('docs/FIRST_ROUND_POLICY_V3_9.txt');
  for(const phrase of ['may make a decisive large correction immediately','not a command to reset to AD48','no required minimum or maximum percentage step','does not force an aggressive move','measured acceptance/rollback rules remain unchanged','actual response and local directional evidence'])assert.ok(policy.includes(phrase),phrase);
  assert.equal(C.validateProposal({...proposal,decision:'hold',candidate:baseline},baseline).decision,'hold');
  assert.throws(()=>C.validateProposal({...proposal,candidate:{...candidate,AD:201}},baseline));
});
test('v3.9 only adds first-round prompt/context and version labels to v3.8',()=>{
  const old='backups/v3.8_before_first_round_correction/',normalize=s=>s.replace(/\r\n/g,'\n').replace(/v3\.[89]/g,'vVERSION').replace(/WEB 3\.[89]/g,'WEB VERSION');
  for(const file of ['core.js','plot.js','simulator.js'])assert.equal(read('web/src/'+file),read(old+'web/src/'+file));
  for(const file of ['template.html','i18n.js'])assert.equal(normalize(read('web/src/'+file)),normalize(read(old+'web/src/'+file)));
  const strip=s=>normalize(s).split('\n').filter(l=>!l.includes('const FIRST_ROUND_POLICY=')).join('\n').replace(/\+FIRST_ROUND_POLICY/g,'').replace(/,tuning_round:\{number:A\.round,first_round:A\.round===1\}/g,'');
  assert.equal(strip(read('web/src/app.js')),strip(read(old+'web/src/app.js')));
  const first=read('docs/FIRST_ROUND_POLICY_V3_9.txt');for(const kind of ['tuning','review'])assert.equal(read('docs/LLM_SYSTEM_'+kind+'.txt'),read(old+'docs/LLM_SYSTEM_'+kind+'.txt')+first);
});
