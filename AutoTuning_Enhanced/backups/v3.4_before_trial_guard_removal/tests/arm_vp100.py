from pathlib import Path
import sys,struct,json,hashlib,importlib.util
sys.dont_write_bytecode=True
R=Path(__file__).resolve().parent.parent;sys.path.insert(0,str(R/'tests/deps'))
spec=importlib.util.spec_from_file_location('arm',R/'routea-pid-investigation-20261021/emulate_binary.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
NEW=R/'firmware/build/stm32_Balance_Car_RA1_9_AUTO6_TXFIX_VP100.axf';rows=[]
def ok(name):rows.append({'name':name,'passed':True});print('PASS',name,flush=True)
for v in [72,76,80,84,100]:
 m=mod.Machine(NEW);m.call('RouteA_Init');m.call('RouteA_BalanceStarted');m.command('P1,MSET,96,48,%s,31,17,20'%v);assert ',ACK,MSET,' in m.tx[-1];m.call('RouteA_ControlBegin');assert list(m.gains().values())==[9600,48,v*100,31,1700,20];m.command('P1,GET');assert ',%.2f,31.00,17.00,20.00,' %v in m.tx[-1]
ok('real ARM MSET/APPLY boundary and GET preserve VP72/76/80/84/100 and scaling')
m=mod.Machine(NEW);m.call('RouteA_Init');m.call('RouteA_BalanceStarted')
for v in ['100.01','101','NaN','1e39','9.99']:
 m.command('P1,MSET,96,48,%s,31,17,20'%v);assert ',ERR,' in m.tx[-1], (v,m.tx[-1]);m.call('RouteA_ControlBegin');assert m.gains()['Velocity_Kp']==6200
ok('real ARM rejects >100 nonfinite and below10 without parameter change')
m.command('P1,TRAIN,START');m.command('P1,PREP,501,30000,96,48,100,31,17,20');assert ',ACK,PREP,' in m.tx[-1];m.command('P1,APPLY,501');m.call('RouteA_ControlBegin');assert m.gains()['Velocity_Kp']==10000;m.command('P1,ROLLBACK,501');m.call('RouteA_ControlBegin');assert m.gains()['Velocity_Kp']==6200
m.command('P1,PREP,502,30000,96,48,101,31,17,20');assert ',ERR,RANGE,' in m.tx[-1]
ok('real ARM automatic PREP VP100 applies and rolls back; VP101 rejected')
m=mod.Machine(NEW);m.call('RouteA_Init');m.call('RouteA_BalanceStarted');m.command('P1,MSET,288,200,100,100,100,100');m.call('RouteA_ControlBegin')
for i in range(100):m.call('RouteA_Tick10ms');m.call('RouteA_Task')
slow=[f for f in m.tx if f.startswith('$T1,S,')];assert len(slow)==1;assert 'RA1.9-AUTO6-TXFIX-VP100' in slow[0];assert ',VP100.00,' in slow[0]
ok('max-range slow telemetry fits formatter with VP100 firmware tag')
(R/'tests/arm_vp100_results.json').write_text(json.dumps({'axf_sha256':hashlib.sha256(NEW.read_bytes()).hexdigest(),'tests':rows},indent=2),encoding='utf8')
