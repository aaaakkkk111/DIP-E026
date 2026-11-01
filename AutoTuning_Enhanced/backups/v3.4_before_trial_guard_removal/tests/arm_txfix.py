from pathlib import Path
import sys,struct,json,importlib.util,hashlib,re
sys.dont_write_bytecode=True
R=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(R/'tests/deps'))
spec=importlib.util.spec_from_file_location('arm',R/'routea-pid-investigation-20261021/emulate_binary.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
NEW=R/'firmware/build/stm32_Balance_Car_RA1_9_AUTO6_TXFIX_VP100.axf'
OLD=R/'baseline/RA1.9_AUTO6_before_TXFIX.axf'
def data(m,name,fmt='<I'):return struct.unpack(fmt,m.uc.mem_read(m.symbols[name][0],struct.calcsize(fmt)))[0]
def put(m,name,v,fmt='<I'):m.uc.mem_write(m.symbols[name][0],struct.pack(fmt,v))
rows=[]
def passed(name,**kw):print('PASS',name,flush=True);rows.append(dict(name=name,passed=True,**kw))
m=mod.Machine(NEW);m.call('RouteA_Init');m.call('RouteA_BalanceStarted');m.command('P1,TRAIN,START');m.command('P1,PREP,113782012,30000,72,68,62,31,17,20');m.command('P1,APPLY,113782012');m.call('RouteA_ControlBegin');m.command('P1,ACCEPT,113782012');assert ',ACK,ACCEPT,113782012,0,1,' in m.tx[-1];assert data(m,'last_closed_trial_id')==113782012;n=len(m.tx);m.command('P1,HB,113782012');assert len(m.tx)==n;m.command('P1,GET');assert ',PID,72.00,68.00,62.00,31.00,17.00,20.00,0,1,1,' in m.tx[-1];passed('actual user ACCEPT then late HB preserves candidate without TRIAL,0')
m.command('P1,PREP,222,30000,73,69,63,32,18,21');m.command('P1,APPLY,222');m.call('RouteA_ControlBegin');before=data(m,'heartbeat_ms');n=len(m.tx);m.command('P1,HB,113782012');assert len(m.tx)==n and data(m,'heartbeat_ms')==before;m.command('P1,HB,999');assert ',ERR,TRIAL,222,' in m.tx[-1];passed('old HB cannot disrupt or keep alive a newer transaction; unknown ID still rejected')
m.command('P1,ROLLBACK,222');m.call('RouteA_ControlBegin');assert data(m,'last_closed_trial_id')==222;n=len(m.tx);m.command('P1,HB,222');assert len(m.tx)==n;assert list(m.gains().values())==[7200,68,6200,31,1700,20];passed('rollback records completed ID and preserves full baseline restoration')
m.command('P1,PREP,333,30000,74,70,64,33,19,22');m.command('P1,APPLY,333');m.call('RouteA_ControlBegin')
for i in range(201):m.call('RouteA_Tick10ms')
m.call('RouteA_ControlBegin');assert data(m,'state','<B')==0;assert list(m.gains().values())==[7200,68,6200,31,1700,20];passed('2-second heartbeat protection remains active')
q=mod.Machine(NEW);q.call('RouteA_Init')
for i in range(100):q.call('RouteA_Tick10ms');q.call('RouteA_Task')
assert len([f for f in q.tx if f.startswith('$T1,F,')])==5;assert len([f for f in q.tx if f.startswith('$T1,E,')])==2;assert len([f for f in q.tx if f.startswith('$T1,S,')])==1;passed('unchanged 5 Hz fast / 2.5 Hz feature cadence / 1 Hz slow scheduling')
for mode in [0,3,20]:
 a=mod.Machine(OLD);b=mod.Machine(NEW)
 for x in [a,b]:
  put(x,'mode',mode,'<B');x.call('Set_PID')
  if mode==20:x.call('RouteA_Init')
 for i in range(50):
  for x in [a,b]:x.call('EXTI15_10_IRQHandler')
  assert data(a,'Motor_Left','<i')==data(b,'Motor_Left','<i');assert data(a,'Motor_Right','<i')==data(b,'Motor_Right','<i')
passed('150 baseline control cycles unchanged across Standard Load TEST')
(R/'tests/arm_txfix_vp100_results.json').write_text(json.dumps({'axf_sha256':hashlib.sha256(NEW.read_bytes()).hexdigest(),'tests':rows},indent=2),encoding='utf8')
