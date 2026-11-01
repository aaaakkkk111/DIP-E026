const fs=require('node:fs');
const assert=require('node:assert/strict');
const C=require('../src/core.js');
const {make}=require('../tests/harness.cjs');
const lines=fs.readFileSync(__dirname+'/connection-2026-10-03.log','utf8').trim().split(/\r?\n/);
const stamp=s=>{const [h,m,sec]=s.split(':').map(Number);return (h*3600+m*60+sec)*1000;};
const rows=lines.map(s=>{const m=s.match(/^(\S+) (SYS|TX|RX) (.*)$/);return {at:stamp(m[1]),kind:m[2],text:m[3]};});
const start=rows.find(x=>x.kind==='TX'),rx=rows.filter(x=>x.kind==='RX');
const parsed=rx.map(x=>{assert.equal(C.verify(x.text),true);const f=C.parseFast(x.text),s=C.parseSlow(x.text);assert.ok(f||s);return f?{kind:'fast',frame:f}:{kind:'slow',frame:s};});
(async()=>{
  const h=make(),ble=h.installBle({dropPid:true});const pending=h.action('connect');await h.flush();
  let elapsed=0;
  for(const row of rx){const target=row.at-start.at;await h.advance(target-elapsed);elapsed=target;ble.emit(row.text);}
  assert.equal(h.snapshot().connected,true);assert.equal(h.snapshot().verified,false);
  assert.equal(h.snapshot().pid.AP,96);assert.equal(ble.dev.gatt.connected,true);
  await h.advance(4000-elapsed);await pending;assert.equal(ble.dev.gatt.connected,true);
  assert.equal(h.snapshot().verified,false);assert.equal(h.set('manualPidButton').disabled,true);
  assert.match(h.snapshot().op,/GET\/PID/);assert.deepEqual(h.errors,[]);
  console.log(JSON.stringify({rxFrames:rx.length,fast:parsed.filter(x=>x.kind==='fast').length,slow:parsed.filter(x=>x.kind==='slow').length,allCRCValid:true,allFramesParsed:true,pidReplies:rx.filter(x=>x.text.startsWith('$P1,PID,')).length,firmware:parsed.find(x=>x.kind==='slow').frame.firmware,realObservedTimeoutMs:rows.at(-1).at-start.at,softwareReproducedTimeoutMs:4000,result:'updated HTML preserves GATT and telemetry; unverified controls stay locked'},null,2));
})().catch(e=>{console.error(e);process.exitCode=1;});
