const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const {make}=require('../tests/harness.cjs');
const C=require('../src/core.js');
const pid='$P1,PID,96.00,48.00,62.00,31.00,17.00,20.00,0,0,1,C78BD#';
const fast='$T1,F,15420,A1.70,G15.0,EL0,ER0,ML1437,MR1437,S0,L0,R1,C10,C21437,C31437,C40,CA9FA#';
const slow='$T1,S,16070,V11.68,B-163,V-38,T0,AP96.00,AD48.00,VP62.00,VI31.00,TP17.00,TD20.00,D0,L0,R1,RA1.5-TEST-MODE,CC764#';
function setup(options={}){
  const h=make(),ble=h.installBle({dropPid:true});
  h.doc.parse(fs.readFileSync(__dirname+'/template.html','utf8'));
  const writes=[],times=[];let buffer='';
  for(const method of ['writeValue','writeValueWithResponse','writeValueWithoutResponse']){
    ble.ch[method]=async bytes=>{const text=new TextDecoder().decode(bytes);writes.push({method,text});times.push(h.context.Date.now());if(options.fail)throw Error('Native write rejected');if(options.hang)return new Promise(()=>{});buffer+=text;if(!buffer.endsWith('#'))return;assert.equal(buffer,C.getPid());buffer='';if(options.legacyError)ble.emit('$ReceivePackError#');else if(options.err)ble.emit(C.frame('P1,ERR,CRC,0'));else if(options.reply!==false)ble.emit(pid);};
  }
  if(options.request)h.context.navigator.bluetooth.requestDevice=options.request;
  vm.runInContext(fs.readFileSync(__dirname+'/get.js','utf8'),h.context,{filename:'diagnostic-get.js'});
  return {...h,ble,writes,times,click:id=>h.set(id).click()};
}
for(const [id,method]of Object.entries({legacy:'writeValue',withResponse:'writeValueWithResponse',withoutResponse:'writeValueWithoutResponse'})){
  test(method+' sends only GET and verifies fragmented firmware-format PID',async()=>{
    const h=setup();await h.click('connect');assert.equal(h.writes.length,0);
    await h.click(id);assert.deepEqual(h.writes,[{method,text:'$P1,GET,CD032#'}]);
    assert.match(h.set('result').textContent,/已确认 GET → PID 双向通信/);
    assert.match(h.set('log').value,/WRITE_DONE/);assert.match(h.set('log').value,/RX_BYTES/);
    assert.equal(h.ble.dev.gatt.connected,true);assert.equal(h.set('legacy').disabled,true);
    assert.deepEqual(h.errors,[]);
  });
}
test('valid user telemetry with no PID retains connection and reports absent command response',async()=>{
  const h=setup({reply:false});await h.click('connect');const pending=h.click('legacy');await h.flush();
  h.ble.emit(fast);h.ble.emit(slow);await h.advance(6100);await pending;
  assert.equal(C.verify(fast),true);assert.equal(C.verify(slow),true);
  assert.equal(h.ble.dev.gatt.connected,true);assert.match(h.set('result').textContent,/未收到有效 P1,PID/);
  assert.match(h.set('telemetry').textContent,/有效快遥测 1 帧 \/ 慢遥测 1 帧 \/ PID 0 帧/);
  assert.match(h.set('telemetry').textContent,/RA1.5-TEST-MODE/);assert.match(h.set('log').value,/"writeFinished":true/);
  assert.equal(h.set('withResponse').disabled,true);
  await h.click('disconnect');await h.click('connect');assert.equal(h.set('withResponse').disabled,false);
  assert.deepEqual(h.errors,[]);
});
test('rejected write remains distinct from absent PID response',async()=>{
  const h=setup({fail:true});await h.click('connect');await h.click('withResponse');
  assert.match(h.set('result').textContent,/Native write rejected/);assert.match(h.set('log').value,/"writeFinished":false/);
  assert.equal(h.ble.dev.gatt.connected,true);assert.deepEqual(h.errors,[]);
});
test('hanging native write is bounded and requires reconnect before another probe',async()=>{
  const h=setup({hang:true});await h.click('connect');const pending=h.click('legacy');await h.flush();
  await h.advance(5100);await pending;assert.match(h.set('result').textContent,/写入接口 5 秒未返回/);
  assert.equal(h.set('withoutResponse').disabled,true);assert.equal(h.ble.dev.gatt.connected,true);
  assert.deepEqual(h.errors,[]);
});
test('firmware error is recorded as a command response',async()=>{
  const h=setup({err:true});await h.click('connect');await h.click('legacy');
  assert.match(h.set('result').textContent,/固件返回错误：CRC,0/);assert.match(h.set('log').value,/FIRMWARE_ERR/);
  assert.deepEqual(h.errors,[]);
});
test('manual disconnect cancels a pending probe',async()=>{
  const h=setup({reply:false});await h.click('connect');const pending=h.click('legacy');await h.flush();
  await h.click('disconnect');await pending;assert.equal(h.ble.dev.gatt.connected,false);
  assert.match(h.set('link').textContent,/已手动断开/);await h.advance(6500);
  assert.match(h.set('result').textContent,/可以重新连接/);assert.deepEqual(h.errors,[]);
});
test('cancelled device selection does not open a stale GATT connection',async()=>{
  let resolve;const h=setup({request:()=>new Promise(r=>{resolve=r;})});const pending=h.click('connect');
  await h.flush();await h.click('disconnect');resolve(h.ble.dev);await pending;
  assert.equal(h.ble.dev.gatt.connected,false);assert.equal(h.set('connect').disabled,false);
  assert.deepEqual(h.errors,[]);
});
test('paced GET sends the identical 14-byte frame at 60 ms gaps',async()=>{
  const h=setup();await h.click('connect');const pending=h.click('paced');await h.flush();
  await h.advance(1000);await pending;
  assert.equal(h.writes.length,14);assert.ok(h.writes.every(x=>x.method==='writeValueWithResponse'&&x.text.length===1));
  assert.equal(h.writes.map(x=>x.text).join(''),C.getPid());
  for(let i=1;i<h.times.length;i++)assert.equal(h.times[i]-h.times[i-1],60);
  assert.match(h.set('result').textContent,/已确认 GET → PID 双向通信.*慢速 60 ms/);
  assert.match(h.set('log').value,/"chunkSize":1,"chunkDelayMs":60/);
  assert.deepEqual(h.errors,[]);
});
test('manual disconnect during paced GET prevents all later byte writes',async()=>{
  const h=setup({reply:false});await h.click('connect');const pending=h.click('paced');await h.flush();
  assert.equal(h.writes.length,1);await h.click('disconnect');await h.advance(6500);await pending;
  assert.equal(h.writes.length,1);assert.equal(h.ble.dev.gatt.connected,false);assert.deepEqual(h.errors,[]);
});
test('observed legacy ReceivePackError is classified as firmware routing failure, not CRC failure',async()=>{
  const h=setup({legacyError:true});await h.click('connect');await h.click('withResponse');
  assert.match(h.set('result').textContent,/ReceivePackError：Route A 包头未识别/);
  assert.match(h.set('log').value,/LEGACY_ERR/);assert.doesNotMatch(h.set('log').value,/PARSE CRC 无效/);
  assert.match(h.set('telemetry').textContent,/无效帧 0/);assert.equal(h.ble.dev.gatt.connected,true);
  assert.deepEqual(h.errors,[]);
});
test('critical test events survive heavy telemetry and remain in copied/downloaded text',async()=>{
  const h=setup({reply:false});await h.click('connect');const pending=h.click('legacy');await h.flush();
  for(let i=0;i<300;i++)h.ble.emit(fast);
  await h.advance(6100);await pending;const text=h.set('log').value;
  assert.match(text,/GATT /);assert.match(text,/TX_ATTEMPT /);assert.match(text,/WRITE_DONE /);assert.match(text,/RESULT /);
  await h.click('export');const exported=await h.downloads.at(-1).blob.text();
  assert.match(exported,/GATT /);assert.match(exported,/RESULT /);assert.deepEqual(h.errors,[]);
});
