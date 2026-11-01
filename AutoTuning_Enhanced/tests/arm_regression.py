from pathlib import Path
import sys,struct,math,json,hashlib,importlib.util
sys.dont_write_bytecode=True
R=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(R/'tests/deps'))
spec=importlib.util.spec_from_file_location('arm_model',R/'routea-pid-investigation-20261021/emulate_binary.py');model=importlib.util.module_from_spec(spec);spec.loader.exec_module(model)
from unicorn.arm_const import *
NEW=R/'firmware/build/stm32_Balance_Car_RA1_9_AUTO6.axf'
OLD=R/'baseline/RA1.8_original.axf'
def put(m,name,value,fmt='<f'):m.uc.mem_write(m.symbols[name][0],struct.pack(fmt,value))
def get(m,name,fmt='<f'):return struct.unpack(fmt,m.uc.mem_read(m.symbols[name][0],struct.calcsize(fmt)))[0]
def bits(v):return struct.unpack('<I',struct.pack('<f',v))[0]
def step(m,a,g):m.uc.reg_write(UC_ARM_REG_R1,bits(g));m.call('RouteA_FeaturesStep',bits(a))
report=[]
def result(name,**data):report.append(dict(name=name,passed=True,**data));print('PASS',name)
m=model.Machine(NEW);m.call('RouteA_Init');m.call('RouteA_BalanceStarted');m.command('P1,MSET,95,49,63,32,18,21');assert m.gains()['Balance_Kp']==9600;m.call('RouteA_ControlBegin');assert list(m.gains().values())==[9500,49,6300,32,1800,21];result('manual six coefficients apply only on control boundary')
m.command('P1,TRAIN,START');m.command('P1,PREP,123,30000,97,50,64,33,19,22');assert ',ACK,PREP,' in m.tx[-1];m.command('P1,APPLY,123');m.call('RouteA_ControlBegin');assert list(m.gains().values())==[9700,50,6400,33,1900,22];m.command('P1,ROLLBACK,123');m.call('RouteA_ControlBegin');assert list(m.gains().values())==[9500,49,6300,32,1800,21];result('LLM all six update and full champion rollback')
for body in ['P1,MSET,NaN,48,62,31,17,20','P1,PREP,2,30000,289,48,62,31,17,20','P1,PREP,2,30000,96,48,73,31,17,20']:
 m.command(body);assert ',ERR,' in m.tx[-1]
result('nonfinite out-of-range and manual-lock rejection')
for field in ['AP','AD','VP','VI','TP','TD']:
 q=model.Machine(NEW);q.call('RouteA_Init');q.call('RouteA_BalanceStarted');q.inject=(q.symbols['ra_send'][0]&~1)+4
 vals=[96,48,62,31,17,20];vals[['AP','AD','VP','VI','TP','TD'].index(field)]+=1;q.command('P1,MSET,'+','.join(map(str,vals)));q.call('RouteA_ControlBegin');expected=[vals[0]*100,vals[1],vals[2]*100,vals[3],vals[4]*100,vals[5]];assert list(q.gains().values())==expected;assert q.stack-q.lowest<4096
result('six real ARM parser paths survive balance-IRQ preemption',max_observed_stack=q.stack-q.lowest)
class StimulusMachine(model.Machine):
 def trace(self,cpu,address,size,user):
  if address in getattr(self,'hooks',{}):
   name=self.hooks[address]
   if name=='I2C_ReadOneByte':
    reg=cpu.reg_read(UC_ARM_REG_R1);values={0x43:self.rawgyro>>8&255,0x44:self.rawgyro&255,0x3d:self.accy>>8&255,0x3e:self.accy&255,0x3f:self.accz>>8&255,0x40:self.accz&255}
    cpu.reg_write(UC_ARM_REG_R0,values.get(reg,0));cpu.reg_write(UC_ARM_REG_PC,cpu.reg_read(UC_ARM_REG_LR));return
   if name=='Read_Encoder':
    motor=cpu.reg_read(UC_ARM_REG_R0);cpu.reg_write(UC_ARM_REG_R0,self.encoder if motor==0 else (-self.encoder)&0xffffffff);cpu.reg_write(UC_ARM_REG_PC,cpu.reg_read(UC_ARM_REG_LR));return
  super().trace(cpu,address,size,user)
for mode,vals in [(0,[9600,48,6200,31,1700,20]),(3,[9600,75,7000,35,1400,20]),(20,[9600,48,6200,31,1700,20])]:
 a=StimulusMachine(OLD);b=StimulusMachine(NEW)
 for x in [a,b]:
  put(x,'mode',mode,'<B');x.call('Set_PID');assert list(x.gains().values())==vals
  if mode==20:x.call('RouteA_Init')
 for i in range(100):
  for x in [a,b]:
   x.rawgyro=int(400*math.sin(i*.2));x.accy=int(16384*math.sin(.04*math.sin(i*.08)));x.accz=16384;x.encoder=int(4*math.sin(i*.17));x.call('EXTI15_10_IRQHandler')
  assert get(a,'Motor_Left','<i')==get(b,'Motor_Left','<i');assert get(a,'Motor_Right','<i')==get(b,'Motor_Right','<i')
result('300 baseline control cycles match RA1.8 across all three supported modes')
z=model.Machine(NEW);put(z,'mode',0,'<B')
for count,want in [(300,3),(600,20),(900,0),(600,20),(300,3),(0,0)]:
 z.call('car_mode',count);assert get(z,'mode','<B')==want
result('three-mode menu wraps forwards and backwards with original internal IDs')
q=model.Machine(NEW);q.call('RouteA_Init');q.call('RouteA_BalanceStarted');q.command('P1,TRAIN,START');q.command('P1,PREP,42,30000,97,49,63,32,18,21');q.command('P1,APPLY,42');q.call('RouteA_ControlBegin')
for i in range(201):q.call('RouteA_Tick10ms')
q.call('RouteA_ControlBegin');assert list(q.gains().values())==[9600,48,6200,31,1700,20];result('real ARM heartbeat timeout restores complete accepted vector')
for i in range(100):step(q,0,0)
for i in range(100):q.call('RouteA_Tick10ms');q.call('RouteA_Task')
frames=[v for v in q.tx if v.startswith('$T1,E,')];assert 2<=len(frames)<=3;assert ',W0,' in frames[-1];result('feature reporting stays separate at 2.5 Hz and exposes warmup',frames_in_one_second=len(frames))
for hz in [0.1,1,4,12,35]:
 x=model.Machine(NEW);x.call('RouteA_Init');ref=[0.,0.,0.,0.,0.,0.]
 for i in range(1600):
  a=2*math.sin(2*math.pi*hz*i*.005);g=164*math.sin(2*math.pi*hz*i*.005);step(x,a,g)
  if i==0:ref[0]=g;ref[3]=a
  ref[0]+=.200849*(g-ref[0]);ref[1]+=.334511*(g-ref[0]-ref[1]);ref[2]+=.016393*(ref[1]**2-ref[2]);ref[3]+=.009337*(a-ref[3]);ref[4]+=.059120*(a-ref[3]-ref[4]);ref[5]+=.016393*(ref[4]**2-ref[5])
 assert abs(get(x,'feature_osc')-math.sqrt(ref[2])/16.4)<.002;assert abs(get(x,'feature_lean')-math.sqrt(ref[5]))<.002;assert get(x,'feature_samples','<H')==600
 result('actual ARM bandpass matches double-precision reference',hz=hz,osc=get(x,'feature_osc'),lean=get(x,'feature_lean'))
step(x,float('nan'),0);assert get(x,'feature_samples','<H')==0;result('invalid feature input resets prewarm state')
(R/'tests/arm_results.json').write_text(json.dumps({'axf_sha256':hashlib.sha256(NEW.read_bytes()).hexdigest(),'tests':report},indent=2),encoding='utf8')
