from pathlib import Path
import subprocess,hashlib,re,json,xml.etree.ElementTree as ET
r=Path(__file__).resolve().parents[1]
out=r/'tests/bin';out.mkdir(exist_ok=True)
base=['gcc','-std=c99','-O1','-Wall','-Wextra','-Wno-format','-Wno-misleading-indentation','-I',str(r/'tests/native_stubs'),'-I',str(r/'firmware/APP/PID'),'-I',str(r/'firmware/APP/mode'),'-I',str(r/'firmware/APP/OLED_Show')]
def build(name,source,defs=()):
    exe=out/(name+'.exe')
    subprocess.run(base+list(defs)+[str(r/'tests'/source),'-lm','-o',str(exe)],check=True,capture_output=True)
    return exe
subprocess.run([str(build('sequential','sequential_native.c'))],check=True)
subprocess.run([str(build('menu','mode_menu.c'))],check=True)
new=subprocess.check_output([str(build('new_trace','stability_trace.c'))])
ref=r/'tests/ra30_reference'
defs=['-DTRACE_PID="'+str(ref/'pid_control.c').replace('\\','/')+'"','-DTRACE_ROUTE="'+str(ref/'route_a.c').replace('\\','/')+'"']
old=subprocess.check_output([str(build('original_ra30_trace','stability_trace.c',defs))])
assert new==old,'Idle Standard/Load/TEST control output differs from original RA3.0'
assert len(new.splitlines())==4200
project=r/'firmware/USER/stm32_Balance_Car.uvprojx'
for node in ET.parse(project).findall('.//FilePath'):assert (project.parent/node.text.replace('\\','/')).is_file(),node.text
text=(r/'firmware/APP/RouteA/route_a.c').read_text()
assert 'sscanf(' not in text and 'PID_ResetDynamicState(' not in text
assert 'memory_fault' not in text and 'RA_TRANSITION_STEPS' not in text
maptext=(r/'firmware/USER/Listings/stm32_Balance_Car_L.map').read_text(errors='replace')
load=int(re.search(r'Load Region LR_IROM1 .*?Size: (0x[0-9a-fA-F]+)',maptext).group(1),16)
ram=int(re.search(r'Execution Region RW_IRAM1 .*?Size: (0x[0-9a-fA-F]+)',maptext).group(1),16)
assert load<0x10000 and ram<0xC000
print('PASS: original RA3.0 versus recovery: all 4200 Standard/Load/TEST control cycles identical before tuning')
print('PASS: bounded single-field parser, no live state reset or new startup latch; ROM',hex(load),'RAM',hex(ram))
subprocess.run(['node','--test',str(r/'ios-bluefy/tests/core.test.cjs'),str(r/'ios-bluefy/tests/sequential-ui.test.cjs')],check=True)
