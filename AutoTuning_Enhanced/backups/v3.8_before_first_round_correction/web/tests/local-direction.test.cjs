const {test}=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const C=require('../src/core.js'),{make}=require('./harness.cjs');
const base={AP:150,AD:120,VP:62,VI:31,TP:17,TD:20};
const metric=(osc,lean)=>({meanOsc:osc,meanLean:lean,featureSamples:15,rmsAngle:lean,peakAngle:lean*2,rmsGyro:osc*16.4,saturationRate:0});
function row(id,from,to,before,after,outcome='accepted',extra={}){return {id,time:'2026-10-25T10:00:00Z',source:'ble',appliedConfirmed:true,base:from,candidate:to,baseline_metrics:before,trial_metrics:after,evaluation:C.evaluateCost(before,after),outcome,...extra};}
const summary=(pid,history,m=metric(30,2))=>C.localDirectionSummary(pid,m,history,'ble');
test('an observed AD increase worsening osc prioritizes a lower AD in accepted and rolled-back trials',()=>{
  for(const outcome of ['accepted','rolled_back']){
    const from={...base,AD:100},r=row(11,from,base,metric(20,1),metric(50,3),outcome);
    const s=summary(outcome==='accepted'?base:from,[r]);
    assert.equal(s.preferred_direction.action,'prefer_lower_AD_if_osc_remains_problematic');
    assert.deepEqual(s.preferred_direction.evidence_ids,[11]);assert.equal(s.comparisons[0].osc_delta,30);
    assert.equal(s.comparisons[0].outcome,outcome);
  }
});
test('lowering AD and improving both metrics supports continuing or observing',()=>{
  const next={...base,AD:90},r=row(12,base,next,metric(50,3),metric(20,1));
  const s=summary(next,[r]);assert.equal(s.preferred_direction.action,'continue_lower_AD_or_hold_to_observe');
  assert.equal(s.comparisons[0].pid_delta.AD,-30);assert.equal(s.preferred_direction.coordinated,false);
});
test('a later unsuccessful AD reduction supersedes the earlier successful downward direction',()=>{
  const next={...base,AD:90},bad={...base,AD:60};
  const history=[row(12,base,next,metric(50,3),metric(20,1)),row(13,next,bad,metric(20,1),metric(30,2),'rolled_back')];
  const s=summary(next,history);assert.equal(s.comparisons.length,2);
  assert.equal(s.preferred_direction.action,'stop_blind_AD_decrease_and_reassess');
  assert.deepEqual(s.preferred_direction.evidence_ids,[13]);
});
test('coordinated changes are identified and never presented as an isolated AD effect',()=>{
  const next={...base,AP:180,AD:90,VP:70,VI:10,TP:25,TD:30};
  const s=summary(next,[row(14,base,next,metric(50,3),metric(20,1))]);
  assert.equal(s.preferred_direction.coordinated,true);assert.equal(s.comparisons[0].changed_parameters.length,6);
  assert.match(s.comparisons[0].attribution,/not_isolated/);
});
test('subsequent gain changes mark older AD evidence as context-dependent',()=>{
  const next={...base,AD:90},next2={...next,VP:85};
  const s=summary(next2,[row(15,base,next,metric(50,3),metric(20,1)),row(16,next,next2,metric(20,1),metric(25,1))]);
  assert.equal(s.preferred_direction.context_changed_since,true);
});
test('missing, unconfirmed, different-source and unrelated PID-path evidence cannot produce a direction',()=>{
  const next={...base,AD:90};
  for(const r of [row(1,base,next,metric(50,3),metric(20,1),'accepted',{evaluation:null}),row(1,base,next,metric(50,3),metric(20,1),'accepted',{source:'simulation'}),row(1,base,next,metric(50,3),metric(20,1),'accepted',{appliedConfirmed:false}),row(1,base,next,metric(50,3),metric(20,1),'firmware_rollback'),row(1,base,next,metric(50,3),metric(20,1),'accepted',{trial_metrics:{meanOsc:1,meanLean:1,featureSamples:0}})]){
    assert.equal(summary(next,[r]).comparisons.length,0);
  }
  assert.equal(summary({...base,AP:100},[row(1,base,next,metric(50,3),metric(20,1))]).comparisons.length,0);
});
test('local summary is bounded to five actual comparisons and does not invent stability targets or mutate history',()=>{
  const history=[];let p=base;
  for(let id=1;id<=7;id++){const n={...p,AD:p.AD-1};history.push(row(id,p,n,metric(50,3),metric(20,1)));p=n;}
  const before=JSON.stringify(history),s=summary(p,history);
  assert.equal(s.comparisons.length,5);assert.equal(s.comparisons[0].id,3);
  assert.equal(s.experiment_target.specified,false);assert.equal(JSON.stringify(history),before);
  const empty=summary(base,[]);assert.equal(empty.comparisons.length,0);
  assert.equal(empty.preferred_direction.action,'assess_live_pattern_and_conditional_priors');
});
test('prompt policy uses local directions conditionally, requires new reversal evidence and keeps all six gains available',()=>{
  const policy=fs.readFileSync(path.join(__dirname,'../../docs/LOCAL_DIRECTION_POLICY_V3_8.txt'),'utf8');
  for(const phrase of ['AP150/AD120','lowering AD is the leading initial hypothesis','NEW relevant measured evidence','not permanent prohibitions','All six gains remain available for large, coordinated changes','No numeric stability target','AP and the velocity loop','lean is not fore-aft travel distance'])assert.ok(policy.includes(phrase),phrase);
  const candidate={AP:250,AD:190,VP:100,VI:100,TP:100,TD:100};
  assert.deepEqual(C.validateProposal({schema_version:1,decision:'propose',stage:'balance',candidate,expected_effect:'A decisive coordinated test, not proven improvement',requested_test:'quiet_balance',confidence:.6},base).candidate,candidate);
});

for(const lang of ['zh','en'])test(lang+' actual next-round LLM payload contains the completed local before/after experiment',async()=>{
  const requests=[];let count=0;
  const h=make({search:'?lang='+lang,fetch:async(url,init)=>{
    const q=JSON.parse(init.body),d=JSON.parse(q.messages[1].content);requests.push({q,d});count++;
    const value={schema_version:1,decision:count===1?'propose':'hold',stage:'balance',candidate:count===1?{...d.current_pid,AD:75}:d.current_pid,expected_effect:'Simulation-only directional-context test',requested_test:'quiet_balance',confidence:.6};
    return {ok:true,json:async()=>({choices:[{message:{content:JSON.stringify(value)}}]})};
  }});
  await h.action('mock');await h.advance(4000);h.set('llmKey').value='test-only';h.set('loopCooldown').value='2';
  const pending=h.action('continuous-start');await h.flush();await h.advance(13000);
  assert.ok(requests.length>=2,'second model request must occur');
  assert.equal(requests[0].d.local_direction_summary.comparisons.length,0);
  const data=requests[1].d,s=data.local_direction_summary;
  assert.equal(s.comparisons.length,1);assert.equal(s.comparisons[0].tested_pid.AD,75);
  assert.equal(s.comparisons[0].id,data.recent_history.at(-1).id);
  assert.equal(s.comparisons[0].before.meanOsc,data.recent_history.at(-1).baseline_metrics.meanOsc);
  assert.equal(s.comparisons[0].after.meanLean,data.recent_history.at(-1).trial_metrics.meanLean);
  assert.equal(data.harness_limits.relative_change_limit,null);assert.equal(data.harness_limits.acceptance_policy.maxWeightedCost,1.1);
  assert.equal(h.set('baselineSeconds').value,'3');assert.equal(h.set('trialSeconds').value,'1');
  assert.match(requests[1].q.messages[0].content,/LOCAL COMPARISON FIRST/);
  await h.action('continuous-stop');await h.advance(300);await pending;
  const restarted=h.action('continuous-start');await h.flush();const countBefore=requests.length;await h.advance(4000);
  assert.ok(requests.length>countBefore);assert.equal(requests.at(-1).d.local_direction_summary.comparisons.length,0,'new run does not reuse an old local direction');
  await h.action('continuous-stop');await h.advance(300);await restarted;assert.deepEqual(h.errors,[]);
});

test('existing controller flow, protocol, plots and UI layout are unchanged except version and prompt context',()=>{
  const root=path.resolve(__dirname,'../..'),old=path.join(root,'backups/v3.7_before_direction_summary'),read=p=>fs.readFileSync(p,'utf8');
  for(const f of ['plot.js','simulator.js'])assert.equal(read(path.join(root,'web/src',f)),read(path.join(old,'web/src',f)));
  const normalize=s=>s.replace(/v3\.[78]/g,'vVERSION').replace(/WEB 3\.[78]/g,'WEB VERSION');
  for(const f of ['template.html','i18n.js'])assert.equal(normalize(read(path.join(root,'web/src',f))),normalize(read(path.join(old,'web/src',f))));
  const strip=s=>normalize(s).split('\n').filter(l=>!l.includes('const LOCAL_DIRECTION_POLICY=')).join('\n').replace(/\+LOCAL_DIRECTION_POLICY/g,'').replace(/,local_direction_summary:C\.localDirectionSummary\(S\.baseline\.pid,S\.metrics,S\.history\.slice\(A\.historyStart\),S\.baseline\.source\)/g,'');
  assert.equal(strip(read(path.join(root,'web/src/app.js'))),strip(read(path.join(old,'web/src/app.js'))));
  const original=require(path.join(old,'web/src/core.js'));
  assert.deepEqual(C.ACCEPTANCE,original.ACCEPTANCE);assert.deepEqual(C.RANGES,original.RANGES);
  for(const [k,v]of Object.entries(original))if(typeof v==='function')assert.equal(C[k].toString(),v.toString(),k);
});
