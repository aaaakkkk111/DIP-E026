"""Execute actual ARMCC firmware instructions and C library, with only hardware TX/PWM stubbed."""
from pathlib import Path
import sys, struct, json, binascii, hashlib
sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent/'baseline-v4-migration-20261014/tests/testdeps'))
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE, UC_HOOK_MEM_WRITE
from unicorn.arm_const import *

def load_elf(path):
    data=Path(path).read_bytes()
    assert data[:7]==b'\x7fELF\x01\x01\x01'
    phoff,shoff=struct.unpack_from('<II',data,28)
    phsize,phnum,shsize,shnum,shnames=struct.unpack_from('<5H',data,42)
    sections=[struct.unpack_from('<10I',data,shoff+i*shsize) for i in range(shnum)]
    symbols={}
    for section in sections:
        if section[1]!=2:continue
        strings=sections[section[6]]
        strings=data[strings[4]:strings[4]+strings[5]]
        for i in range(0,section[5],section[9]):
            name,value,size,info,other,index=struct.unpack_from('<IIIBBH',data,section[4]+i)
            name=strings[name:strings.find(b'\0',name)].decode('ascii','replace')
            if name and index and value:symbols[name]=(value,size)
    segments=[]
    for i in range(phnum):
        kind,offset,vaddr,paddr,filesz,memsz,flags,align=struct.unpack_from('<8I',data,phoff+i*phsize)
        if kind==1 and filesz:segments.append((vaddr,data[offset:offset+filesz]))
    return symbols,segments

class Machine:
    def __init__(self,path):
        self.symbols,segments=load_elf(path)
        self.uc=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
        self.uc.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M3)
        self.uc.mem_map(0x08000000,0x40000)
        self.uc.mem_map(0x20000000,0x20000)
        self.uc.mem_map(0x40000000,0x30000)
        self.uc.mem_map(0x42000000,0x2000000)
        for address,raw in segments:self.uc.mem_write(address,raw)
        self.stack=self.symbols['__initial_sp'][0]
        self.stack_size=self.symbols['STACK'][1]
        self.bottom=self.stack-self.stack_size
        self.lowest=self.stack
        self.tx=[]
        self.in_irq=False;self.pending_irq=None;self.inject=None;self.irq_records=[]
        self.irq_candidate_writes=[]
        self.hooks={self.symbols[n][0]&~1:n for n in ['UART5_TxEnqueue','Set_Pwm','I2C_ReadOneByte','Read_Encoder'] if n in self.symbols}
        self.uc.hook_add(UC_HOOK_CODE,self.trace)
        self.uc.hook_add(UC_HOOK_MEM_WRITE,self.memory_write)
        for name,value in zip(['Balance_Kp','Balance_Kd','Velocity_Kp','Velocity_Ki','Turn_Kp','Turn_Kd'],[9600,48,6200,31,1700,20]):
            self.uc.mem_write(self.symbols[name][0],struct.pack('<f',value))
        self.uc.mem_write(self.symbols['mode'][0],b'\x14')
        self.uc.mem_write(self.symbols['GET_Angle_Way'][0],b'\x02')
        self.uc.mem_write(self.symbols['battery'][0],struct.pack('<f',12.0))
    def memory_write(self,cpu,access,address,size,value,user):
        candidate=self.symbols['candidate'][0]
        if self.in_irq and address<candidate+24 and address+size>candidate and len(self.irq_candidate_writes)<32:
            self.irq_candidate_writes.append({'pc':hex(cpu.reg_read(UC_ARM_REG_PC)),
               'sp':hex(cpu.reg_read(UC_ARM_REG_SP)),'destination':hex(address),
               'size':size,'value':hex(value)})
    def string(self,address):
        raw=bytes(self.uc.mem_read(address,256));return raw[:raw.index(0)].decode('ascii','replace')
    def trace(self,cpu,address,size,user):
        if cpu.reg_read(UC_ARM_REG_SP)<self.lowest:
            self.lowest=cpu.reg_read(UC_ARM_REG_SP);self.lowest_pc=address
        if address==0x0803f000:cpu.emu_stop();return
        if not self.in_irq and self.inject is not None and address==self.inject:
            self.inject=None;self.pending_irq=cpu.context_save();cpu.emu_stop();return
        if address in self.hooks:
            if self.hooks[address]=='UART5_TxEnqueue':
                self.tx.append(self.string(cpu.reg_read(UC_ARM_REG_R0)))
                cpu.reg_write(UC_ARM_REG_R0,1)
            elif self.hooks[address]=='I2C_ReadOneByte':
                register=cpu.reg_read(UC_ARM_REG_R1)
                cpu.reg_write(UC_ARM_REG_R0,{0x3d:0x04,0x3e:0x00,0x3f:0x40,0x40:0x00}.get(register,0))
            elif self.hooks[address]=='Read_Encoder':cpu.reg_write(UC_ARM_REG_R0,0)
            cpu.reg_write(UC_ARM_REG_PC,cpu.reg_read(UC_ARM_REG_LR))
    def call(self,name,arg=0):
        self.uc.reg_write(UC_ARM_REG_SP,self.stack)
        self.uc.reg_write(UC_ARM_REG_LR,0x0803f001)
        self.uc.reg_write(UC_ARM_REG_R0,arg)
        self.uc.emu_start(self.symbols[name][0]|1,0,timeout=10_000_000,count=2_000_000)
        while self.pending_irq is not None:
            saved=self.pending_irq;self.pending_irq=None
            resume=self.uc.reg_read(UC_ARM_REG_PC)
            top=self.uc.reg_read(UC_ARM_REG_SP)
            candidate_before=bytes(self.uc.mem_read(self.symbols['candidate'][0],24))
            regs=[UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3,UC_ARM_REG_R12,UC_ARM_REG_LR,UC_ARM_REG_PC,UC_ARM_REG_XPSR]
            self.uc.mem_write(top-32,struct.pack('<8I',*(self.uc.reg_read(r) for r in regs)))
            self.uc.reg_write(UC_ARM_REG_SP,top-32)
            self.uc.reg_write(UC_ARM_REG_LR,0x0803f001)
            self.in_irq=True
            self.uc.emu_start(self.symbols['EXTI15_10_IRQHandler'][0]|1,0,timeout=10_000_000,count=2_000_000)
            assert self.uc.reg_read(UC_ARM_REG_PC)==0x0803f000,hex(self.uc.reg_read(UC_ARM_REG_PC))
            self.in_irq=False
            candidate_after=bytes(self.uc.mem_read(self.symbols['candidate'][0],24))
            self.irq_records.append({'preempted_at':hex(resume),'stack_at_preemption':self.stack-top,
                                    'combined_stack_used':self.stack-self.lowest,
                                    'candidate_changed_by_irq':candidate_before!=candidate_after,
                                    'candidate_before':struct.unpack('<6f',candidate_before),
                                    'candidate_after':struct.unpack('<6f',candidate_after)})
            self.uc.context_restore(saved)
            self.uc.emu_start(resume|1,0,timeout=10_000_000,count=2_000_000)
        assert self.uc.reg_read(UC_ARM_REG_PC)==0x0803f000,(name,hex(self.uc.reg_read(UC_ARM_REG_PC)))
    def command(self,body):
        raw=f'${body},C{binascii.crc_hqx(body.encode(),0xffff):04X}#'
        self.uc.mem_write(0x20010000,raw.encode()+b'\0')
        self.call('RouteA_HandleFrame',0x20010000)
        return raw
    def gains(self):
        return {n:struct.unpack('<f',self.uc.mem_read(self.symbols[n][0],4))[0]
                for n in ['Balance_Kp','Balance_Kd','Velocity_Kp','Velocity_Ki','Turn_Kp','Turn_Kd']}

def run(path,inject=False):
    runs=[]
    for body in ['P1,MSET,95.00,48.00,62.00,31.00,17.00,20.00','P1,MSET,96.00,48.00,62.00,31.00,18.00,20.00']:
        m=Machine(path);m.call('RouteA_Init');m.call('RouteA_BalanceStarted');before=m.gains();m.command('P1,GET');initial=m.tx[-1]
        if inject:m.inject=(m.symbols['ra_send'][0]&~1)+4
        sent=m.command(body);after_parse=m.gains();parsed_tx=m.tx[-1]
        candidate=struct.unpack('<6f',m.uc.mem_read(m.symbols['candidate'][0],24))
        m.call('RouteA_ControlBegin');after_apply=m.gains();m.command('P1,GET')
        runs.append({'sent':sent,'before':before,'initial_get':initial,'after_parse':after_parse,
          'candidate':candidate,'parse_reply':parsed_tx,'after_apply':after_apply,'readback':m.tx[-1],
          'max_stack_used':m.stack-m.lowest,'configured_stack':m.stack_size,'interrupts':m.irq_records,
          'irq_candidate_writes':m.irq_candidate_writes})
    result={'axf':str(path),'sha256':hashlib.sha256(Path(path).read_bytes()).hexdigest(),'runs':runs}
    print(json.dumps(result,indent=2))
    return result

if __name__=='__main__':
    path=Path(sys.argv[1]) if len(sys.argv)>1 else ROOT.parent/'routea-firmware-v1.6/build/stm32_Balance_Car_RouteA_1_6.axf'
    result=run(path,'--irq' in sys.argv)
    (ROOT/'binary-reproduction.json').write_text(json.dumps(result,indent=2),encoding='utf8')

