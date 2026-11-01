"""Full ARMCC 5.06u7 rebuild of the isolated copied firmware, with a 64 KiB linker guard."""
from pathlib import Path
import subprocess,xml.etree.ElementTree as ET,hashlib,json
ROOT=Path(__file__).resolve().parent
PROJECT=ROOT/'firmware/USER/stm32_Balance_Car.uvprojx'
OUT=ROOT/'build';OUT.mkdir(exist_ok=True)
BIN=Path(r'D:/DIP/MDK-ARM/core/ARM/ARMCC/Bin')
tree=ET.parse(PROJECT);user=PROJECT.parent
vc=tree.find('.//Cads/VariousControls')
includes=[(user/p.replace('\\','/')).resolve() for p in vc.findtext('IncludePath').split(';')]
defines=vc.findtext('Define').split(',')
objects=[];log=[]
def run(args):
 p=subprocess.run([str(x) for x in args],cwd=OUT,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,errors='replace')
 log.append(p.stdout)
 if p.returncode:
  (OUT/'build.log').write_text('\n'.join(log),encoding='utf8');raise RuntimeError(p.stdout)
 return p.stdout
for item in tree.findall('.//File'):
 name=item.findtext('FilePath');source=(user/name.replace('\\','/')).resolve()
 if source.suffix.lower() not in {'.c','.s'}:continue
 obj=OUT/(source.stem+'.o');objects.append(obj)
 if source.suffix.lower()=='.s':run([BIN/'armasm.exe','--cpu','Cortex-M3','-g','--pd','__MICROLIB SETA 1','-o',obj,source])
 else:
  args=[BIN/'armcc.exe','--cpu','Cortex-M3','--library_type=microlib','-g','-O1','--c99','--split_sections','-c',source,'-o',obj]
  args += ['-D'+x for x in defines]+['-I'+str(p) for p in includes]
  run(args)
scatter=OUT/'guard.sct'
scatter.write_text('LR_IROM1 0x08000000 0x00010000 {\n ER_IROM1 0x08000000 0x00010000 {\n *.o (RESET, +First)\n *(InRoot$$Sections)\n .ANY (+RO)\n }\n RW_IRAM1 0x20000000 0x0000C000 { .ANY (+RW +ZI) }\n}\n')
axf=OUT/'stm32_Balance_Car_RA1_9_AUTO6_TXFIX_VP100.axf'
response=OUT/'link.rsp'
response.write_text('\n'.join('"'+str(p)+'"' for p in objects))
run([BIN/'armlink.exe','--library_type=microlib','--via',response,'--scatter',scatter,'--map','--symbols','--callgraph','--callgraph_file',OUT/'callgraph.htm','--info','sizes,totals','--list',OUT/'firmware.map','-o',axf])
hexfile=OUT/'stm32_Balance_Car_RA1_9_AUTO6_TXFIX_VP100.hex'
run([BIN/'fromelf.exe','--i32combined','--output',hexfile,axf])
upper=0;maximum=0;load=0
for row in hexfile.read_text().splitlines():
 data=bytes.fromhex(row[1:]);assert sum(data)&255==0,'Intel HEX checksum failure'
 if data[3]==4:upper=int.from_bytes(data[4:6],'big')<<16
 if data[3]==0:maximum=max(maximum,upper+int.from_bytes(data[1:3],'big')+data[0]-1);load+=data[0]
assert maximum<0x08010000,'Flash guard violated'
(OUT/'build.log').write_text('\n'.join(log),encoding='utf8')
result={'compiler':run([BIN/'armcc.exe','--vsn']).strip(),'source_files':len(objects),'max_address':hex(maximum),'load_bytes':load,'hex_sha256':hashlib.sha256(hexfile.read_bytes()).hexdigest(),'warnings':sum(s.lower().count('warning:') for s in log),'errors':0}
(OUT/'validation.json').write_text(json.dumps(result,indent=2),encoding='utf8')
print(json.dumps(result,indent=2))
