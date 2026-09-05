"""Build an isolated Keil source project using the extracted vendor libraries.
Does not invoke a compiler or flash any hardware. Existing destination is refused.
"""
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "reference" / "6.LQR" / "STM32_code"
DEST = ROOT / "firmware_uart_bench"


def main():
    if DEST.exists():
        raise SystemExit(f"Already exists; use the project in {DEST}. No files overwritten.")
    for directory in ("CMSIS", "FWLib"):
        shutil.copytree(SOURCE / directory, DEST / directory)
    user = DEST / "USER"
    user.mkdir()
    (DEST / "OBJ").mkdir()
    (DEST / "Listings").mkdir()
    for name in ("stm32f10x_conf.h", "stm32f10x_it.h", "stm32f10x_it.c"):
        shutil.copy2(SOURCE / "USER" / name, user / name)
    interrupts = (user / "stm32f10x_it.c").read_text(encoding="gb18030")
    interrupts = interrupts.replace('#include "stm32f10x_it.h"', '#include "stm32f10x_it.h"\n#include "pc_link.h"')
    import re
    interrupts, count = re.subn(r"void SysTick_Handler\(void\)\s*\{\s*\}",
                                "void SysTick_Handler(void)\n{\n  pc_link_ms++;\n}", interrupts)
    if count != 1:
        raise RuntimeError("Vendor SysTick layout differs; inspect before building")
    (user / "stm32f10x_it.c").write_text(interrupts, encoding="utf-8")
    for name in ("main.c", "pc_link.c", "pc_link.h"):
        shutil.copy2(ROOT / "bench_src" / name, user / name)
    tree = ET.parse(SOURCE / "USER" / "LQR.uvprojx")
    project = tree.getroot()
    for node in project.iter("TargetName"):
        node.text = "UART_BENCH"
    for node in project.iter("OutputName"):
        node.text = "UART_BENCH"
    for node in project.iter("IncludePath"):
        if node.text:
            node.text = r"..\USER;..\CMSIS;..\FWLib\inc"
    for node in project.iter("UpdateFlashBeforeDebugging"):
        node.text = "0"
    keep = {"main.c", "stm32f10x_it.c", "system_stm32f10x.c", "core_cm3.c",
            "startup_stm32f10x_hd.s", "misc.c", "stm32f10x_gpio.c", "stm32f10x_rcc.c",
            "stm32f10x_usart.c", "stm32f10x_dma.c"}
    for groups in project.iter("Groups"):
        for group in list(groups):
            files = group.find("Files")
            for file in list(files):
                if file.findtext("FileName") not in keep:
                    files.remove(file)
            if not list(files):
                groups.remove(group)
        files = groups[0].find("Files")
        file = ET.SubElement(files, "File")
        for key, value in (("FileName", "pc_link.c"), ("FileType", "1"), ("FilePath", r".\pc_link.c")):
            ET.SubElement(file, key).text = value
    tree.write(user / "UART_BENCH.uvprojx", encoding="utf-8", xml_declaration=True)
    print(user / "UART_BENCH.uvprojx")


if __name__ == "__main__":
    main()
