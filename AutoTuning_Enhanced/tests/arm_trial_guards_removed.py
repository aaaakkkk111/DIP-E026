"""Exercise compiled ARM instructions, not a reimplementation of C guard logic."""
from pathlib import Path
import sys,struct,json,hashlib,importlib.util
sys.dont_write_bytecode=True
R=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(R/'tests/deps'))
spec=importlib.util.spec_from_file_location('arm',R/'routea-pid-investigation-20261021/emulate_binary.py')
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
from unicorn.arm_const import UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3
NEW=R/'firmware/build/stm32_Balance_Car_RA1_9_AUTO6_TXFIX_VP100_NOGUARD.axf'
rows=[]
def get(m,n,fmt='<B'):return struct.unpack(fmt,m.uc.mem_read(m.symbols[n][0],struct.calcsize(fmt)))[0]
def put(m,n,v,fmt='<f'):m.uc.mem_write(m.symbols[n][0],struct.pack(fmt,v))
def capture(m,output):
    for reg,v in [(UC_ARM_REG_R1,19),(UC_ARM_REG_R2,100),(UC_ARM_REG_R3,200)]:m.uc.reg_write(reg,v)
    m.uc.mem_write(m.stack,struct.pack('<3i',0,output,output))
    m.call('RouteA_ControlCapture',19)
def ok(name):print('PASS',name,flush=True);rows.append({'name':name,'passed':True})
def prepared():
    m=mod.Machine(NEW);m.call('RouteA_Init');m.call('RouteA_BalanceStarted')
    m.command('P1,MSET,50,45,62,31,17,20');m.call('RouteA_ControlBegin')
    m.command('P1,TRAIN,START');m.command('P1,PREP,1109635223,30000,75,70,62,31,17,20')
    assert ',ACK,PREP,' in m.tx[-1] and get(m,'state')==1
    return m

for name,angle,output,battery in [
    ('angle-only',27.09,1400,12),('saturation-only',2,2600,12),
    ('battery-only',2,1400,9.2),('combined',27.09,2600,9.2)]:
    m=prepared();put(m,'Angle_Balance',angle);put(m,'battery',battery)
    m.command('P1,APPLY,1109635223');m.call('RouteA_ControlBegin')
    assert ',ACK,APPLY,' in m.tx[-1] and get(m,'state')==2
    for _ in range(100):capture(m,output);m.call('RouteA_ControlBegin')
    assert get(m,'state')==2 and get(m,'rollback_request')==0
    assert list(m.gains().values())==[7500,70,6200,31,1700,20]
    assert get(m,'snap_ml','<i')==output and get(m,'snap_mr','<i')==output
    m.command('P1,GET');assert ',PID,75.00,70.00,62.00,31.00,17.00,20.00,2,1,1,' in m.tx[-1]
    if name=='combined':
        m.command('P1,ROLLBACK,1109635223');m.call('RouteA_ControlBegin')
        assert get(m,'state')==0 and list(m.gains().values())==[5000,45,6200,31,1700,20]
    else:
        m.command('P1,ACCEPT,1109635223');assert get(m,'state')==0
    ok(name+': 100 capture cycles retain candidate; explicit completion works')

m=prepared();m.command('P1,APPLY,1109635223');m.call('RouteA_ControlBegin')
for _ in range(201):m.call('RouteA_Tick10ms')
m.call('RouteA_ControlBegin');assert get(m,'state')==0 and m.gains()['Balance_Kp']==5000
ok('missing heartbeat still restores the baseline after 2 seconds')

m=mod.Machine(NEW);m.call('RouteA_Init');m.call('RouteA_BalanceStarted')
m.command('P1,TRAIN,START');m.command('P1,PREP,9,1000,75,70,62,31,17,20');m.command('P1,APPLY,9');m.call('RouteA_ControlBegin')
for i in range(101):
    m.call('RouteA_Tick10ms')
    if i%20==0:m.command('P1,HB,9')
m.call('RouteA_ControlBegin');assert get(m,'state')==0
ok('transaction TTL still closes an expired trial despite heartbeats')

OLD=R/'firmware/build/stm32_Balance_Car_RA1_9_AUTO6_TXFIX_VP100.axf'
for mode in [0,3,20]:
    a=mod.Machine(OLD);b=mod.Machine(NEW)
    for m in [a,b]:
        put(m,'mode',mode,'<B');m.call('Set_PID')
        if mode==20:m.call('RouteA_Init')
    for i in range(50):
        for m in [a,b]:m.call('EXTI15_10_IRQHandler')
        for n in ['Motor_Left','Motor_Right']:assert get(a,n,'<i')==get(b,n,'<i')
        assert a.gains()==b.gains()
ok('Standard/Load/idle TEST control outputs match previous firmware for 150 cycles')

m=mod.Machine(NEW);m.call('RouteA_Init');m.call('RouteA_BalanceStarted');m.command('P1,MSET,288,200,100,100,100,100');m.call('RouteA_ControlBegin')
for i in range(100):m.call('RouteA_Tick10ms');m.call('RouteA_Task')
slow=[x for x in m.tx if x.startswith('$T1,S,')]
assert len(slow)==1 and 'RA1.9-AUTO6-TXFIX-VP100-NOGUARD' in slow[0]
assert len([x for x in m.tx if x.startswith('$T1,F,')])==5
assert len([x for x in m.tx if x.startswith('$T1,E,')])==2
ok('5Hz fast, 2.5Hz features and 1Hz slow telemetry retain valid extended version frames')

(R/'tests/arm_trial_guards_removed_results.json').write_text(json.dumps({'axf':str(NEW),'sha256':hashlib.sha256(NEW.read_bytes()).hexdigest(),'hardware_verified':False,'tests':rows},indent=2),encoding='utf8')
