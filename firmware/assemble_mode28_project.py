"""Build the mode-28 Keil project without the team's own project folder.

    stock Yahboom project (stm32_Balance_Car_L, from the Yahboom download)
  + the team's mode-28 sources (rl_mode28_20261003/src)
  + the patches the team's README lists but the zip does not contain,
    re-created from that README: myenum.h (modes 21-28), app_mode.c (menu
    offers only modes 1 and 28; RL_Reset on selection), oled_show.c (mode
    name), AllHeader.h (includes), main.c (CAL wait, key wait, mode-28 loop
    with Bluetooth, console and OLED status), plus the console hooks the
    team's earlier tune_io patch had made: bsp.c (USART1 at 230400,
    Bluetooth and TUNE_Init in mode 28) and usart.c (received bytes to
    TUNE_RxByte)
  + this repo's policy files (README-STM32-DEPLOYMENT.md Step 1: the four
    files, the policy_build_obs_rp edit and POLICY_NO_FLOAT_INFER)
  -> DEST, never compiled. Every change is marked with a "mode 28" comment.

    python firmware/assemble_mode28_project.py --stock <...>/stm32_Balance_Car_L         --team-src <...>/rl_mode28_20261003/src --dest <new folder>/stm32_Balance_Car_L

Built and flashed 2026-10-07: 0 errors (4 stock warnings), image end
0x0800FA78, 1416 bytes under the 64 KB this board loads reliably; timing on
the car matched the team's v6 (0 late ticks, worst ISR 4.5 ms). For a new
policy only APP/RL/policy_weights.h and policy_weights_q.h change.

Stock files are GBK-encoded: every edit is done on bytes (latin-1 round trip),
and every anchor must match exactly once or the script stops.
"""
import argparse
import os
import re
import shutil
import sys

_ap = argparse.ArgumentParser(description="Assemble the mode-28 Keil project.")
_ap.add_argument("--stock", required=True, help="the stock stm32_Balance_Car_L folder")
_ap.add_argument("--team-src", required=True, help="rl_mode28_20261003/src from the team")
_ap.add_argument("--dest", required=True, help="new stm32_Balance_Car_L folder to create")
_args = _ap.parse_args()
STOCK = _args.stock
TEAM = _args.team_src
REPO_FW = os.path.dirname(os.path.abspath(__file__))
DEST = _args.dest

def read(path):
    with open(path, "rb") as f:
        return f.read().decode("latin-1")


def write(path, text):
    with open(path, "wb") as f:
        f.write(text.encode("latin-1"))


def nl_of(text):
    return "\r\n" if "\r\n" in text else "\n"


def once(text, anchor, what):
    n = text.count(anchor)
    if n != 1:
        sys.exit("anchor for %s found %d times: %r" % (what, n, anchor))
    return text.index(anchor)


def insert_before(text, anchor, block, what):
    i = once(text, anchor, what)
    return text[:i] + block + text[i:]


def insert_after_line(text, anchor, block, what):
    i = once(text, anchor, what)
    j = text.index("\n", i) + 1
    return text[:j] + block + text[j:]


def replace_once(text, old, new, what):
    once(text, old, what)
    return text.replace(old, new)


def lines(nl, *ls):
    return nl.join(ls) + nl


# ---------------------------------------------------------------- 1. copy stock
if os.path.exists(DEST):
    sys.exit("DEST already exists, not overwriting: " + DEST)
shutil.copytree(STOCK, DEST, ignore=shutil.ignore_patterns("*.uvguix.*"))
# "never compiled": drop every build output the stock copy carries
for sub in ("OBJ", r"USER\Objects", r"USER\Listings"):
    p = os.path.join(DEST, sub)
    if os.path.isdir(p):
        shutil.rmtree(p)
os.makedirs(os.path.join(DEST, "OBJ"))

# ------------------------------------------------------- 2. team mode-28 files
rl = os.path.join(DEST, "APP", "RL")
os.makedirs(rl)
for f in ("rl_mode.c", "rl_mode.h", "removed_modes_stubs.c", "removed_modes21_27_stubs.c",
          "policy_q.c", "policy_q.h", "odom.c", "odom.h"):
    shutil.copy2(os.path.join(TEAM, f), rl)
shutil.copy2(os.path.join(TEAM, "app_control.c"), os.path.join(DEST, "APP", "app_control.c"))
shutil.copy2(os.path.join(TEAM, "startup_stm32f10x_hd.s"), os.path.join(DEST, "CMSIS", "startup_stm32f10x_hd.s"))
shutil.copy2(os.path.join(TEAM, "stm32_Balance_Car_modes_1_28.uvprojx"),
             os.path.join(DEST, "USER", "stm32_Balance_Car.uvprojx"))

# ------------------------------------------- 3. guide Step 1.1: run-8 files in
for f in ("policy.c", "policy.h", "policy_weights.h", "policy_weights_q.h"):
    shutil.copy2(os.path.join(REPO_FW, f), rl)

# ---------------------------------------- 4. guide Step 1.2: drop wheel angles
p = os.path.join(rl, "rl_mode.c")
t = read(p)
t = replace_once(
    t,
    "policy_build_obs_rp(obs, roll, pitch, od.wheel_angle_l, od.wheel_angle_r, od.v_forward,",
    "policy_build_obs_rp(obs, roll, pitch, od.v_forward,",
    "rl_mode.c policy_build_obs_rp")
write(p, t)

# --------------------------------------- 5. guide Step 1.3: POLICY_NO_FLOAT_INFER
p = os.path.join(DEST, "USER", "stm32_Balance_Car.uvprojx")
t = read(p)
t = replace_once(t, "<Define>STM32F10X_HD,USE_STDPERIPH_DRIVER</Define>",
                 "<Define>STM32F10X_HD,USE_STDPERIPH_DRIVER,POLICY_NO_FLOAT_INFER</Define>",
                 "uvprojx Define")
write(p, t)

# ----------------------------------------------- 6. header for the stub files
write(os.path.join(rl, "removed_modes21_27.h"), "\r\n".join([
    "/* removed_modes21_27.h -- declarations for removed_modes21_27_stubs.c.",
    " * The team's modes 21-27 and their serial test console are not in this",
    " * build; app_control.c still names them and links to the stubs.",
    " * TUNE_Init / TUNE_Poll / TUNE_RxByte are mode 28's console (rl_mode.c). */",
    "#ifndef __REMOVED_MODES21_27_H",
    "#define __REMOVED_MODES21_27_H",
    "",
    "#define IS_LOAD_MODE(m) ((m) >= Load_22 && (m) <= Load_25)",
    "",
    "void LD_Tick(void);",
    "void V4_Tick(void);",
    "void LA_Tick(int ml, int mr, int el, int er);",
    "void TUNE_Sample(float gyro, float angle, int ml, int mr, int el, int er);",
    "int  TUNE_Inject(void);",
    "int  TUNE_Injecting(void);",
    "void TUNE_Fell(void);",
    "int  TUNE_Logging(void);",
    "",
    "void TUNE_Init(void);",
    "void TUNE_Poll(void);",
    "void TUNE_RxByte(unsigned char c);",
    "",
    "#endif",
    "",
]))

# ------------------------------------------------- 7. myenum.h: modes 21-28
p = os.path.join(DEST, "USER", "myenum.h")
t = read(p)
nl = nl_of(t)
t = insert_before(t, "\tMode_Max ", lines(
    nl,
    "\t/* 21-27: the team's experimental modes, not in this build",
    "\t * (APP\\RL\\removed_modes21_27_stubs.c); app_control.c still names them */",
    "\tMode21_Removed,",
    "\tLoad_22, Load_23, Load_24, Load_25,",
    "\tLoad_Adapt,      //26",
    "\tAdapt_V4,        //27",
    "\tRL_Policy,       //28: the RL network (APP\\RL\\rl_mode.c)",
    "\t",
), "myenum.h Mode_Max")
write(p, t)

# -------------------------------------------------- 8. AllHeader.h: includes
p = os.path.join(DEST, "USER", "AllHeader.h")
t = read(p)
nl = nl_of(t)
t = insert_after_line(t, '#include "KF.h"', lines(
    nl,
    "",
    "//RL policy, mode 28 (APP\\RL)",
    '#include "rl_mode.h"',
    '#include "removed_modes21_27.h"',
), "AllHeader.h KF.h")
write(p, t)

# ------------------------------------- 9. app_mode.c: only modes 1 and 28
p = os.path.join(DEST, "APP", "mode", "app_mode.c")
t = read(p)
nl = nl_of(t)
m = re.search(r"\t\tif\(cnt < cnt_old\).*?(?=\t\tcnt_old = cnt;)", t, re.S)
if not m or t.count("if(cnt < cnt_old)") != 1:
    sys.exit("app_mode.c car_mode block not found")
t = t[:m.start()] + lines(
    nl,
    "\t\t//this build has only mode 1 and mode 28: any step switches between them",
    "\t\tmode = (mode == RL_Policy) ? Normal : RL_Policy;",
    "\t\t",
) + t[m.end():]
t = insert_after_line(t, "\tSet_PID();", lines(
    nl,
    "",
    "\tif(mode == RL_Policy) RL_Reset(); //mode 28: start its calibration (rl_mode.c)",
), "app_mode.c Set_PID call")
write(p, t)

# ------------------------------------------------- 10. oled_show.c: mode name
p = os.path.join(DEST, "APP", "OLED_Show", "oled_show.c")
t = read(p)
nl = nl_of(t)
t = insert_after_line(t, '"20.EM Track"', lines(
    nl,
    '\t\tcase RL_Policy: OLED_Draw_Line("28.RL Policy", 1, true, true);  \t\t\t\t \t\t\tbreak;',
), "oled_show.c EM Track")
write(p, t)

# ------------------------------- 11. bsp.c: 230400 console, Bluetooth, TUNE_Init
p = os.path.join(DEST, "BSP", "bsp.c")
t = read(p)
nl = nl_of(t)
t = replace_once(t, "uart_init(115200);", "uart_init(230400); //230400: mode 28's serial console (rl_trim_helper.py)",
                 "bsp.c uart_init")
t = replace_once(t, "\tif(mode == Normal || mode == Weight_M)\r\n" if nl == "\r\n" else "\tif(mode == Normal || mode == Weight_M)\n",
                 "\tif(mode == Normal || mode == Weight_M || mode == RL_Policy) //mode 28 drives from the Bluetooth app" + nl,
                 "bsp.c bluetooth if")
t = insert_before(t, "\tif(mode < LiDar_avoid || mode > LiDar_wall_Line )", lines(
    nl,
    "\tif(mode == RL_Policy) TUNE_Init(); //mode 28: serial console on USART1 (rl_mode.c)",
    "",
), "bsp.c TIM6 if")
write(p, t)

# ------------------------------ 12. usart.c: received bytes go to the console
p = os.path.join(DEST, "BSP", "Usart1", "usart.c")
t = read(p)
nl = nl_of(t)
t = insert_before(t, "void USART1_IRQHandler(void)", lines(
    nl,
    "void TUNE_RxByte(unsigned char c); //mode 28 serial console (rl_mode.c)",
), "usart.c IRQ handler")
t = replace_once(t, "\t\tUSART1_Send_U8(Rx1_Temp);",
                 "\t\tTUNE_RxByte(Rx1_Temp); //mode 28 console; the receive interrupt is only enabled in mode 28",
                 "usart.c echo")
write(p, t)

# ------------------------------------------ 13. main.c: CAL, key wait, loop
p = os.path.join(DEST, "USER", "main.c")
t = read(p)
nl = nl_of(t)
t = insert_after_line(t, "\tMPU6050_EXTI_Init();", lines(
    nl,
    "\t",
    "\tif(mode == RL_Policy) //mode 28: hold the car upright and still until its calibration is done",
    "\t{",
    "\t\twhile(!rl_calibrated)",
    "\t\t{",
    '\t\t\tsprintf(showbuf,"CAL hold %3ld%%   ", rl_cal_n / 10);',
    "\t\t\tOLED_Draw_Line(showbuf, 2, false, true);",
    "\t\t\tTUNE_Poll();",
    "\t\t}",
    "\t}",
), "main.c MPU6050_EXTI_Init")
t = replace_once(t, "\twhile(!Key1_State(1) && Stop_Flag ==1 );",
                 "\twhile(!Key1_State(1) && Stop_Flag ==1 )" + nl +
                 "\t{" + nl +
                 "\t\tif(mode == RL_Policy) TUNE_Poll(); //mode 28: serial console while waiting" + nl +
                 "\t}",
                 "main.c key wait")
t = insert_before(t, "\t\telse if(mode == U_Follow)", lines(
    nl,
    "\t\telse if(mode == RL_Policy) //mode 28: Bluetooth commands, serial console, OLED status",
    "\t\t{",
    "\t\t\tif (newLineReceived)",
    "\t\t\t{",
    "\t\t\t\tProtocolCpyData();",
    "\t\t\t\tProtocol();",
    "\t\t\t}",
    "\t\t\tTUNE_Poll();",
    "\t\t\tif(rl_state == RL_CAL)      sprintf(showbuf,\"CAL hold %3ld%%   \", rl_cal_n / 10);",
    "\t\t\telse if(rl_state == RL_RUN) sprintf(showbuf,\"L%5d R%5d   \", rl_pwm_l, rl_pwm_r);",
    "\t\t\telse                        sprintf(showbuf,\"ARM p%6.1f    \", rl_pitch_deg);",
    "\t\t\tOLED_Draw_Line(showbuf, 3, false, true);",
    "\t\t}",
    "\t\t",
), "main.c U_Follow branch")
write(p, t)

print("assembled:", DEST)
print("open", os.path.join(DEST, "USER", "stm32_Balance_Car.uvprojx"), "in Keil and build (F7)")
