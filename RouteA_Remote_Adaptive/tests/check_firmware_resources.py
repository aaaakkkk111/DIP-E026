from pathlib import Path
import re
r=Path(__file__).resolve().parents[1]
def depths(file):
    result={}
    for block in file.read_text(encoding='utf-8',errors='replace').split('<P><STRONG>'):
        n=re.match(r'<a name="[^"\n]+"></a>([^<]+)</STRONG>',block)
        d=re.search(r'Max Depth = (\d+)',block)
        if n and d: result[n.group(1)]=int(d.group(1))
    return result
new=depths(r/'firmware/OBJ/stm32_Balance_Car_L.htm')
old_path=r/'backups/20261014_before_stack_parser_fix/firmware/OBJ/stm32_Balance_Car_L.htm'
old=depths(old_path) if old_path.exists() else {}
asm=(r/'firmware/CMSIS/startup_stm32f10x_hd.s').read_text()
stack=int(re.search(r'Stack_Size\s+EQU\s+(0x[0-9a-fA-F]+)',asm).group(1),16)
irq=['EXTI15_10_IRQHandler','UART5_IRQHandler','TIM6_IRQHandler','TIM2_IRQHandler','USART1_IRQHandler']
# Conservative allowance: sum enabled ISR paths, hardware entry frames, extra 512 bytes.
budget=new['main']+sum(new.get(k,0) for k in irq)+32*len(irq)+512
assert stack>=budget,(stack,budget)
assert '__stack_limit' in asm and 'Stack_Fill' in asm
source=(r/'firmware/APP/RouteA/route_a.c').read_text()
assert 'sscanf(' not in source
assert 'PID_ResetDynamicState(' not in source
assert source.index('__set_PRIMASK(mask);',source.index('static void ra_read')) < source.index('pid->ap *=',source.index('static void ra_read'))
maptext=(r/'firmware/USER/Listings/stm32_Balance_Car_L.map').read_text(encoding='utf-8',errors='replace')
load=int(re.search(r'Load Region LR_IROM1 .*?Size: (0x[0-9a-fA-F]+)',maptext).group(1),16)
ram=int(re.search(r'Execution Region RW_IRAM1 .*?Size: (0x[0-9a-fA-F]+)',maptext).group(1),16)
assert load<0x10000
assert ram<0xC000
print(f'Resource gate PASS: stack={stack}, conservative budget={budget}, ROM={load:#x}, RAM={ram:#x}')
if old: print(f"Before: main={old['main']}, control ISR={old['EXTI15_10_IRQHandler']}, minimum combined with hardware frame={old['main']+old['EXTI15_10_IRQHandler']+32} > old stack=1024")
print(f"After: main={new['main']}, PID frame handler={new['RouteA_HandleFrame']}, control ISR={new['EXTI15_10_IRQHandler']}")
