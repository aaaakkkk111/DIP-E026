# -*- coding: utf-8 -*-
"""导出可以直接加进 Keil 工程的 load_ctrl.c / load_ctrl.h。

和 export_load_gains.py 的区别：
  - 拆成 .c/.h 两个文件，能被工程里其他 .c 调用
  - **不定义** Balance_K/Velocity_K/Turn_K —— 05.weight_control 里已经有了，
    再定义一次是重复符号、链接报错
  - 六个增益用 extern 引用 pid_control.c 里的那几个全局，不另立门户
  - 注释全部 ASCII —— 原厂那些 .c 是 GB2312，Keil 按系统代码页读，混进
    UTF-8 中文会乱码甚至报错。中文说明单独放 load_ctrl_README.md
  - C89：不用 //、不用块中声明、不用变长数组

用法：  python scripts/export_keil.py [输出目录]
"""
import os
import sys
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import balance_bot.firmware.load_detect as D                      # noqa: E402
from balance_bot.firmware.load_sched import NORMAL, HEAVY, FACTORY  # noqa: E402

FS = 200.0

# 按**电机实际死区**分的版本。固件的 MOTOR_IGNORE_PULSE 固定 1300，变的是
# 电机自己的死区。三个版本都在 MuJoCo 里扫过（scratchpad/dbrate.py，
# 含原地转向的行驶剧本，每格 8 种子）：
#
#   死区    NORMAL 空车存活            HEAVY 4kg 存活
#           kp192 kp288 kp384          kp192 kp288 kp384
#    650     5/8   8/8   8/8            0/8   7/8   8/8
#    900     1/8   8/8   8/8            0/8   7/8   8/8
#   1300     0/8   6/8   8/8            0/8   0/8   6/8
#   1700     0/8   0/8   0/8            0/8   0/8   0/8   <- 任何 kp 都不行
#   2100     0/8   0/8   0/8            0/8   0/8   0/8
#
# **死区 >1300 时调 PID 没用**，那是欠补偿导致的结构性失效：固件补 1300、
# 电机实际要 1700，小指令出不来力矩，同时 1700+信号被 2600 上限截断。
# 实测：死区 1700 配补偿 1300 是 0/6，把补偿也改成 1700 立刻变 5/6。
# 所以那种情况要改的是 MOTOR_IGNORE_PULSE，不是增益。
#
# 每个 (kp, kd, vkp, vki) x2 + 两个阈值，阈值由该死区下实测的抖动几何中点定。
VARIANTS = {
    650:  dict(db=650, comp=1300, normal=(288.0, 0.48,  82.0, 0.69),
               heavy=(288.0, 1.20, 116.0, 0.90),
               loaded=19.4, empty=36.7, margin=1.65,
               chat="NORMAL empty 46.8 / 2kg 8.1,  HEAVY empty 60.5 / 2kg 22.3",
               chat_cn="NORMAL 空车 46.8 / 2kg 8.1    HEAVY 空车 60.5 / 2kg 22.3"),
    900:  dict(db=900, comp=1300, normal=(288.0, 0.48,  82.0, 0.69),
               heavy=(288.0, 1.20, 116.0, 0.90),
               loaded=6.2, empty=33.1, margin=1.69,
               chat="NORMAL empty 32.8 / 2kg 1.2,  HEAVY empty 56.0 / 2kg 19.6",
               chat_cn="NORMAL 空车 32.8 / 2kg 1.2    HEAVY 空车 56.0 / 2kg 19.6"),
    1300: dict(db=1300, comp=1300, normal=(384.0, 0.48, 110.0, 0.92),
               heavy=(384.0, 1.20, 155.0, 1.20),
               loaded=7.5, empty=21.9, margin=2.62,
               chat="NORMAL empty 20.9 / 2kg 2.7,  HEAVY empty 57.4 / 2kg 8.4",
               chat_cn="NORMAL 空车 20.9 / 2kg 2.7    HEAVY 空车 57.4 / 2kg 8.4"),
    # 死区 1500 > 固件补偿 1300，**必须把 MOTOR_IGNORE_PULSE 改成 1500**。
    # 阳性对照（4kg 行驶，6 种子）：补偿1500 是 6/6，补偿1300 是 0/6。
    # 补偿改对之后死区被完全吸收，PID 和 db1300 一模一样，只有阈值变。
    # 全载重实测（8 种子，0/1/2/3/4 kg）：
    #     NORMAL  7 8 7 4 0      HEAVY  7 8 8 8 8
    # kp 细扫（HEAVY 4kg）：320=4/8 352=6/8 384=8/8 416=8/8 448=6/8 480=6/8
    # kp400 每一行都不如 384（6 8 6 4 0 / 6 8 8 8 8），余量也差（1.95 vs 2.23）。
    1500: dict(db=1500, comp=1500, normal=(384.0, 0.48, 110.0, 0.92),
               heavy=(384.0, 1.20, 155.0, 1.20),
               loaded=6.8, empty=23.0, margin=2.23,
               chat="NORMAL empty 18.9 / 2kg 2.5,  HEAVY empty 51.2 / 2kg 10.3",
               chat_cn="NORMAL 空车 18.9 / 2kg 2.5    HEAVY 空车 51.2 / 2kg 10.3"),
    # --- 真车反馈逼出来的这一版 -----------------------------------------
    # 真车实测死区 1400~1500，但 db650 那套（kp 288）效果最好：停振最快、
    # 自稳最好。孪生排不了这件事 —— 它在暂态上差 3 倍（上升时间）到 85%
    # （超调），而「停振快慢」正是暂态。孪生只能给上下界。
    #
    # 用户要求再减 kp。扫下来约束是**单边**的（死区1450 补偿1500，8 种子）：
    #          N kp  H kp | NORMAL 0/1/2 | HEAVY 2/3/4 | 空车行程  osc  rev/s | 余量
    #   原样    288   288 | 7/8 8/8 8/8  | 8/8 8/8 1/8 |  139mm   43.0  1.4  | 1.77x
    #   这一版  240   288 | 8/8 8/8 8/8  | 8/8 8/8 1/8 |  120mm   43.2  1.3  | 1.77x
    #   再低    224   288 | 8/8 8/8 8/8  | 8/8 8/8 1/8 |  117mm   42.3  1.6  | 1.77x
    # NORMAL 一路降到 208 都是满分；掉的是 HEAVY 的 3 kg（kp240 还是 6/6，
    # kp224 就 0/6）。**两档共用 kp 只是历史原因**，这里拆开：NORMAL 降到
    # 240，HEAVY 留 288。
    #
    # 诚实说明：孪生**测不到**减 kp 带来的好处 —— 低频晃 43.0 vs 43.2，
    # 31 Hz 抖动在 208~288 区间跟 kp 没有系统性关系（换种子重测不复现，
    # 第一轮那个 -35% 是噪声）。选 240 的理由是「每一格都不比原样差，而且
    # 是用户在真车上要的方向」，不是「孪生说它更好」。最终值请用
    # scripts/capture_uart.py --tune 在真车上走阶梯选。
    #
    # 4 kg 是 1/8：db650 这一家做不了 4 kg，那要 db1500 的 kp 384。
    "650soft": dict(db=650, comp=1300,
                    normal=(240.0, 0.48,  82.0, 0.69),
                    heavy=(288.0, 1.20, 116.0, 0.90),
                    loaded=6.7, empty=28.9, margin=1.77,
                    chat="NORMAL empty 22.3 / 2kg 2.0,  HEAVY empty 51.0 / 2kg 16.3",
                    chat_cn="NORMAL 空车 22.3 / 2kg 2.0    HEAVY 空车 51.0 / 2kg 16.3"),
}
ORDER = ("balance_kp", "balance_kd", "velocity_kp",
         "velocity_ki", "turn_kp", "turn_kd")


def lit(mode):
    g = mode.gains()
    return "{ " + ", ".join("%.1ff" % (g[k] * 100.0) for k in ORDER) + " }"


def tuple_literal(t):
    """(kp, kd, vkp, vki) -> C 初始化式。转向环不随载重调，用原厂值。"""
    kp, kd, vkp, vki = t
    return ("{ %.1ff, %.1ff, %.1ff, %.1ff, 1400.0f, 20.0f }"
            % (kp * 100, kd * 100, vkp * 100, vki * 100))


def alpha(tau):
    dt = 1.0 / FS
    return dt / (tau + dt)


# 低频前后晃那条链的时间常数。0.3 Hz 高通放掉稳态漂移（用户明确说位置可以
# 变），3 Hz 低通放掉 31 Hz 抖（那条另有 ld_rms），2 s 平滑出读数。
P_HP_TAU = 0.53
P_LP_TAU = 0.053
P_EN_TAU = 2.0

HZ_LO = 1.0 / (2 * 3.14159265 * D.HP_TAU)
HZ_HI = 1.0 / (2 * 3.14159265 * D.LP_TAU)

HEADER = """/* load_ctrl.h -- automatic load-mode switching for the Yahboom balance car.
 * Generated %(when)s by scripts/export_keil.py -- do not edit by hand.
 * See load_ctrl_README.md for the Chinese explanation and the bring-up list.
 */
#ifndef __LOAD_CTRL_H
#define __LOAD_CTRL_H

/* One call per 200 Hz control tick, placed in app_control.c just before
 * Balance_PD().  It runs the detector and rewrites the six PID gains. */
void  LD_Tick(void);

/* Lower level, if you want to drive it yourself.
 *   gyro_lsb : Gyro_Balance, the raw MPU6050 register value
 *   moving   : non-zero while a drive command is active
 *   returns  : 1 = use the HEAVY gain set */
void  LD_Reset(void);
int   LD_Update(float gyro_lsb, int moving);

/* 1 = HEAVY is active.  Boots at 1 on purpose; see the README. */
extern int   ld_heavy;

/* Band-limited gyro RMS in deg/s.  Read this during threshold calibration. */
extern float ld_rms;

/* ---- the two gain sets, in the firmware's x100 integer units ------------
 * WRITABLE ON PURPOSE.  LD_Tick() copies the selected set into Balance_Kp
 * and friends on EVERY tick, so assigning those globals directly -- from a
 * debug console, a test harness, anywhere -- is undone 5 ms later and looks
 * like the write silently did nothing.  Edit the table, not the global. */
typedef struct { float kp, kd, vkp, vki, tkp, tkd; } LD_Gains;
extern LD_Gains ld_g_normal;
extern LD_Gains ld_g_heavy;

/* 0 = the detector decides, 1 = hold NORMAL, 2 = hold HEAVY.
 * Holding one set is what the staged bring-up list needs: it separates
 * "is this gain set any good" from "did the detector pick the right one". */
extern int ld_force;

/* Standstill fore/aft oscillation -- the slow rocking, NOT the 31 Hz buzz.
 *   ld_osc  0.3..3 Hz band RMS of wheel position, in encoder counts
 *   ld_rev  travel-direction reversals per second
 * ld_rms says nothing about these: it is band-limited to %(lo).0f..%(hi).0f Hz
 * and a car that rocks once a second is flat zero there.  Both stay 0
 * unless LD_Pos() is called. */
extern float ld_osc;
extern float ld_rev;

/* OPTIONAL, and it needs one line at the call site.  Pass the summed wheel
 * position each tick:   LD_Pos((float)(Encoder_Left + Encoder_Right));
 * Any monotone-in-distance counter will do; the metric is scale-dependent,
 * so only compare numbers taken from the same car.
 * It takes an argument instead of reading a global because in several
 * Yahboom releases the position accumulator is a function-static inside
 * Velocity(), which cannot be reached with extern. */
void LD_Pos(float wheel_counts);

/* Thresholds, in the same deg/s units as ld_rms.  These are simulation
 * numbers with only a %(margin).2fx margin -- they MUST be recalibrated on the
 * real car before the detector is trusted.  Four readings; see the README. */
extern float LD_LOADED_BELOW;   /* in NORMAL, below this -> car is loaded */
extern float LD_EMPTY_ABOVE;    /* in HEAVY,  above this -> car is empty  */

#endif /* __LOAD_CTRL_H */
"""

SOURCE = """/* load_ctrl.c -- automatic load-mode switching for the Yahboom balance car.
 * Generated %(when)s by scripts/export_keil.py -- do not edit by hand.
 *
 * THIS BUILD IS FOR A MOTOR DEAD BAND OF ABOUT %(db)d COUNTS.
 *   Measure yours first (see the README), then pick the matching folder.
 *
 *   >>> SET  MOTOR_IGNORE_PULSE  TO  %(comp)d  IN APP/app_motor.c  <<<
 *   Line 5 of that file.  The factory value is 1300.  If the motor's real
 *   dead band exceeds the compensation, no gain set works at all: measured
 *   0 out of 6 at dead band 1700 with compensation 1300, and 5 of 6 once the
 *   compensation matched.  Changing the gains cannot fix an under-compensated
 *   dead band -- small commands produce no torque, so the loop is open there.
 *   Measured chatter at this dead band: %(chat)s
 *   Detection margin: %(margin).2fx
 *
 * FIGURES BELOW MARKED [db1300] WERE MEASURED ON THE DEAD BAND 1300 BUILD
 * (Balance_Kp 38400).  They show the mechanism, not this build's numbers.
 * This build's own measured figures are the chatter and margin quoted above.
 *
 * WHY TWO GAIN SETS
 *   No single set covers 0..4 kg.  The conflict is in the D term:
 *     empty car   needs Balance_Kd <= 48,  or it chatters hard at ~31 Hz
 *     4 kg moving needs Balance_Kd >= 96,  or it falls when accelerating
 *   There is no safe value in between: from Kd 56 to 72 the empty-car
 *   chatter jumps 23.5 -> 107.3 deg/s, a 4.6x step in a 0.16 window. [db1300]
 *
 * HOW THE SWITCH DECIDES
 *   It does not estimate the payload.  Payload is essentially unobservable
 *   on this machine: mass added on top also raises the COM, and the axle
 *   recoil cancels most of the extra pitch inertia, so the measured signature
 *   spans only 1.29x over the whole 0..4 kg range. [db1300]
 *   Instead it measures a SYMPTOM -- gyro energy near 31 Hz:
 *       31 Hz chatter present = Kd is too large for the present load
 *                             = the NORMAL set is the correct one
 *   That is the decision criterion itself, not a proxy for it, which is why
 *   it works where a payload estimate does not: it only has to tell which
 *   side of the boundary we are on, not how heavy the load is.
 *
 * WHY A BAND-PASS AND NOT A HIGH-PASS
 *   Chatter has two sources with opposite causes:
 *     empty car  Kd too large -> ~31 Hz fast-mode limit cycle
 *     4 kg       Kd too small -> large low-frequency wallowing
 *   Wide-band energy merges them into one number that is NOT monotonic in
 *   load (NORMAL: 0kg 44.8 / 2kg 8.8 / 4kg 35.5, both ends high). [db1300]
 *   that statistic a 4 kg car reads as "empty", stays in NORMAL and falls --
 *   measured 0 out of 8 [db1300].  The %(lo).1f..%(hi).1f Hz band-pass fixes it.
 *
 * VALIDATION (MuJoCo twin, disturbance 0.4, 20 seeds at each end)
 *     empty  survived 20/20,  decided NORMAL 20/20
 *     4 kg   survived 19/20,  decided HEAVY  20/20
 *   Between 0.5 and 2 kg the decision varies seed to seed, but both sets are
 *   safe there (7..8 of 8), so the inconsistency has no consequence.
 *
 * BEFORE YOU FLASH
 *   Balance_Kp is %(kpx).1fx the factory value.  Follow the staged bring-up
 *   list in load_ctrl_README.md, and do the threshold calibration -- a
 *   %(margin).2fx margin will not survive the difference between this car and
 *   the simulation.
 */

#include <math.h>
#include "load_ctrl.h"

/* The six gains live in pid_control.c.  Referenced, never redefined. */
extern float Balance_Kp, Balance_Kd;
extern float Velocity_Kp, Velocity_Ki;
extern float Turn_Kp, Turn_Kd;

/* From AllHeader.h.  Declared here so this file does not have to drag the
 * whole project header in; the types match AllHeader.h exactly. */
extern float Gyro_Balance;
extern float Move_X, Move_Z;

/* NOTE for the 05.weight_control project: leave Balance_K / Velocity_K /
 * Turn_K at 1.0.  Both sets below already account for the payload, and
 * multiplying again scales the D term with everything else -- which is
 * exactly what makes the factory weight mode shake an empty car apart. */

/* [db1300] survival table -- the mechanism, not this build's numbers:
 *   load     NORMAL(Kd 48)        HEAVY(Kd 120)
 *   0 kg   5/5   22.5 deg/s    5/5  >120 deg/s   <- never HEAVY when empty
 *   1 kg   5/5    1.2          5/5    9.3
 *   2 kg   5/5    0.9          5/5    6.9
 *   4 kg   1/5   10.2          5/5   11.2        <- NORMAL runs out here
 *   (drive-script survivals out of 5, and standstill gyro RMS at that load)
 *
 * Velocity_Ki IS the position loop.  The firmware's "velocity loop" is
 * P(speed) + I(position):
 *     Encoder_Integral += Encoder_bias;   <- integrated counts = position
 *     velocity = -Encoder_bias*Vkp - Encoder_Integral*Vki;
 * Raising it from the factory 31 cut the standstill wander 74 -> 48 mm [db1300]
 * empty and 35 to 28 mm at 4 kg.  Do not push it further: more damping also
 * suppresses the chatter the detector needs. */
LD_Gains ld_g_normal = %(g_normal)s;
LD_Gains ld_g_heavy  = %(g_heavy)s;

/* Not const: see the note in load_ctrl.h.  Retuning on the real car means
 * changing these two lines (or writing the struct at run time); changing
 * Balance_Kp itself has no lasting effect. */

/* Factory values, for reference:
 * static const LD_Gains G_FACTORY = %(g_factory)s; */

/* ---- detector ---------------------------------------------------------- */

#define LD_HZ           %(fs).1ff
#define LD_A_HP         %(a_hp).6ff   /* high-pass corner %(lo).1f Hz */
#define LD_A_LP         %(a_lp).6ff   /* low-pass corner  %(hi).1f Hz */
#define LD_A_EN         %(a_en).6ff   /* energy smoothing */
#define LD_LSB_PER_DPS  %(lsb).1ff

#define LD_PRIME_S      %(prime).2ff    /* settle time before deciding */
#define LD_CONFIRM_S    %(confirm).1ff     /* a verdict must hold this long */
#define LD_LOCKOUT_S    %(lockout).1ff     /* no deciding right after a switch */

float LD_LOADED_BELOW = %(loaded).1ff;
float LD_EMPTY_ABOVE  = %(empty).1ff;

int   ld_heavy = 1;
int   ld_force = 0;
float ld_rms   = 0.0f;
float ld_osc   = 0.0f;
float ld_rev   = 0.0f;

static float ld_mean, ld_band, ld_energy, ld_hold, ld_lock, ld_prime;
static int   ld_rev_last;

/* Boot in HEAVY on purpose.  The risk is asymmetric:
 *     NORMAL on a 4 kg car -> falls as soon as it accelerates
 *     HEAVY  on an empty car -> chatters above 120 deg/s but does NOT fall
 *                               (20 of 20 survived) [db1300]
 * and that chatter is the loudest signal the detector has, so a detection
 * failure costs a couple of noisy seconds rather than a fall.  Booting in
 * NORMAL with the drive command gated was tried and rejected: it got both
 * safety-critical cases wrong. */
void LD_Reset(void)
{
    ld_mean = 0.0f;
    ld_band = 0.0f;
    ld_energy = 0.0f;
    ld_hold = 0.0f;
    ld_lock = 0.0f;
    ld_prime = 0.0f;
    ld_heavy = 1;
}

int LD_Update(float gyro_lsb, int moving)
{
    float dt = 1.0f / LD_HZ;
    float hp;
    int want;

    ld_mean   += LD_A_HP * (gyro_lsb - ld_mean);    /* drop below %(lo).0f Hz */
    hp         = gyro_lsb - ld_mean;
    ld_band   += LD_A_LP * (hp - ld_band);          /* drop above %(hi).0f Hz */
    ld_energy += LD_A_EN * (ld_band * ld_band - ld_energy);
    ld_rms     = (float)sqrt((double)ld_energy) / LD_LSB_PER_DPS;

    if (ld_prime < LD_PRIME_S)
    {
        ld_prime += dt;
        return ld_heavy;
    }

    /* Freeze the verdict while driving.  The statistic is only calibrated at
     * standstill; steering and acceleration push an empty car down into the
     * "loaded" band and cause spurious switches.  The payload only changes
     * when somebody stops and loads the car, so nothing is lost. */
    if (moving)
    {
        ld_hold = 0.0f;
        return ld_heavy;
    }

    if (ld_lock > 0.0f)
    {
        ld_lock -= dt;
        ld_hold = 0.0f;
        return ld_heavy;
    }

    want = ld_heavy;
    if (!ld_heavy && ld_rms < LD_LOADED_BELOW)
    {
        want = 1;
    }
    else if (ld_heavy && ld_rms > LD_EMPTY_ABOVE)
    {
        want = 0;
    }

    if (want != ld_heavy)
    {
        ld_hold += dt;
        if (ld_hold >= LD_CONFIRM_S)
        {
            ld_heavy = want;
            ld_hold  = 0.0f;
            ld_lock  = LD_LOCKOUT_S;
        }
    }
    else
    {
        ld_hold = 0.0f;
    }

    return ld_heavy;
}

/* ---- slow fore/aft oscillation ----------------------------------------
 * Same shape as the detector above, one band lower: high-pass at 0.3 Hz to
 * drop steady drift (the user is content to let the car wander), low-pass at
 * 3 Hz to drop the 31 Hz buzz, then RMS.  ld_rev counts how often the wheels
 * change direction, which separates "drifted 20 cm and stopped" from
 * "hunted back and forth 20 cm forty times" -- the two look identical in a
 * peak-to-peak number and only the second one is the complaint. */
#define LD_P_HP  %(p_hp).6ff       /* 0.3 Hz */
#define LD_P_LP  %(p_lp).6ff       /* 3 Hz   */
#define LD_P_EN  %(p_en).6ff       /* 2 s averaging */

void LD_Pos(float wheel_counts)
{
    static float mean, band, energy, rate, prev;
    static int   inited;
    float hp, d;

    if (!inited) { mean = wheel_counts; prev = wheel_counts; inited = 1; }

    mean   += LD_P_HP * (wheel_counts - mean);
    hp      = wheel_counts - mean;
    band   += LD_P_LP * (hp - band);
    energy += LD_P_EN * (band * band - energy);
    ld_osc  = (float)sqrt((double)energy);

    /* A reversal is a sign change of the per-tick increment.  The 1-count
     * dead zone keeps encoder quantisation from counting as motion. */
    d = wheel_counts - prev;
    if ((d > 1.0f && ld_rev_last < 0) || (d < -1.0f && ld_rev_last > 0))
    {
        rate += LD_P_EN * (LD_HZ - rate);
    }
    else
    {
        rate += LD_P_EN * (0.0f - rate);
    }
    if (d >  1.0f) ld_rev_last =  1;
    if (d < -1.0f) ld_rev_last = -1;
    prev   = wheel_counts;
    ld_rev = rate;
}

/* Call once per control tick, in app_control.c's EXTI15_10_IRQHandler,
 * after Get_Angle() and before Balance_PD(). */
void LD_Tick(void)
{
    const LD_Gains *g;

    LD_Update(Gyro_Balance,
              (Move_X != 0.0f) || (Move_Z != 0.0f));

    /* ld_force overrides the detector here rather than after the fact.
     * Patching Balance_Kd downstream of this function -- which is what the
     * first version of debug_uart did -- leaves Velocity_Kp and Velocity_Ki
     * at the OTHER set's values, producing a mixture that was never tested. */
    if (ld_force == 1)      g = &ld_g_normal;
    else if (ld_force == 2) g = &ld_g_heavy;
    else                    g = ld_heavy ? &ld_g_heavy : &ld_g_normal;

    Balance_Kp  = g->kp;
    Balance_Kd  = g->kd;
    Velocity_Kp = g->vkp;
    Velocity_Ki = g->vki;
    Turn_Kp     = g->tkp;
    Turn_Kd     = g->tkd;
}
"""

README = u"""# load_ctrl —— 负载自动切档（电机死区 {db} 版）

> **这一份是给电机实际死区 ≈ {db} counts 的车用的。**
> 先测你车的死区（方法见文末），再挑对应的目录：
> `db650/` `db900/` `db1300/`。测出来 >1300 的话见文末最后一节。


自动生成于 {when}，由 `scripts/export_keil.py`。别手改 .c/.h，改这个脚本。

## 加进工程

1. 把 `load_ctrl.c` 和 `load_ctrl.h` 放进 `APP/PID/`
2. Keil 里右键 `APP` 组 → Add Existing Files → 选 `load_ctrl.c`
3. `app_control.c` 顶部加 `#include "load_ctrl.h"`
4. 在 `EXTI15_10_IRQHandler` 里加**一行**，位置是 `Get_Angle()` 之后、
   `Balance_PD()` 之前：

```c
    Get_Angle(GET_Angle_Way);
    Encoder_Left  = Read_Encoder(MOTOR_ID_ML);
    Encoder_Right = -Read_Encoder(MOTOR_ID_MR);
    Get_Velocity_Form_Encoder(Encoder_Left, Encoder_Right);

    LD_Tick();                    /* <<< 加这一行 */

    Balance_Pwm = Balance_PD(Angle_Balance, Gyro_Balance);
```

5. `main()` 里初始化一次：`LD_Reset();`

`pid_control.c` 顶部那六个 `float` **不用动**——`LD_Tick()` 每拍会覆写它们。
文件本身也不重复定义任何全局，所以不会有重复符号。

**05.weight_control 工程注意**：`Balance_K` / `Velocity_K` / `Turn_K` 保持
1.0。这两套增益已经把负载算进去了，再乘一遍等于把 D 项也放大——那正是原厂
负载模式在空车上剧烈振荡的原因。

## 编译注意

- 注释全是 ASCII。原厂那些 .c 是 GB2312，Keil 按系统代码页读，混 UTF-8 中文
  会乱码。所以中文说明单独放在这个文件里。
- C89 写法：没有 `//`、没有块中声明。armcc 默认模式能过。
- 用了 `sqrt()`（double），不是 `sqrtf`——老版本 armcc 的 `sqrtf` 不一定有。
  每 5 ms 一次，开销可以忽略。

## 阈值标定（必做）

.c 里那两个阈值是**孪生里的数**，余量只有 1.8 倍。真车的陀螺噪声底、机械
间隙、电机响应都不一样。四次读数：

| 步骤 | 做什么 | 读 `ld_rms` |
|---|---|---|
| 1 | 空车站稳（此时在 NORMAL） | `R_empty_N` |
| 2 | 装 2 kg 站稳 | `R_load_N` |
| 3 | 手动置 `ld_heavy = 1`，仍装 2 kg | `R_load_H` |
| 4 | 卸货变空车，**扶住车**（会剧烈抖） | `R_empty_H` |

```
LD_LOADED_BELOW = sqrt(R_empty_N * R_load_N)
LD_EMPTY_ABOVE  = sqrt(R_empty_H * R_load_H)
```

两个变量是非 const 的全局，可以用 Keil 的 Watch 窗口在线改，不用重烧。

**不标的后果**：真车 rms 通常比孪生高，检测器会一直判「空车」、一直停在
NORMAL —— 而 NORMAL 在 4 kg 行驶时会摔。**这一步不能省。**

## 上车清单

每一步不过就停。

0. 先确认编译优化是 **-O1**。-O0 时 0x08010000 以上的 flash 存不住 RW 初值，
   全局变量开机就是乱的。烧完读回 `Balance_Kp`，对一下是不是 {kp:.0f}。
1. **先不接检测器**（不调 `LD_Tick`），手动把六个增益设成 G_NORMAL：
   空车扶着开机、放手、前后左右各开一遍。
2. 手动设成 G_HEAVY，**装 2 kg**，重复。这一步第一次把 Kd 提到 {hkd:.0f}，
   务必先扶着、确认不抖再放手。
3. G_HEAVY + 4 kg，重复。
4. 做上面的标定，填两个阈值。
5. 接上 `LD_Tick()`。空车开机：应该先抖一两秒（HEAVY 用在空车上），然后自己
   切到 NORMAL 安静下来。**这是正常的，不是故障。**
6. 装 2 kg，停稳几秒，应该自己切到 HEAVY。

## 现象对照

| 现象 | 原因 | 处理 |
|---|---|---|
| 30 Hz 嗡嗡声，NORMAL 下 | Kd 偏大 | `Balance_Kd` {nkd:.0f} → 34，别动 Kp |
| 30 Hz 嗡嗡声，HEAVY 下 | 货不够 2 kg，或真车这个模态比孪生早 | 先查载重；确实够就退回 NORMAL |
| 慢速前后晃，1 秒以上一个来回 | 速度环 | 调 `Velocity_Kp` |
| 起步就往前扑 | Kd 不够 | 该切 HEAVY 了 |
| 档位来回跳 | 阈值没标好 | 重做标定 |
| 停不住，游走几厘米 | 位置环带宽的物理极限 | 见下 |

## 已知没解决的

**车不会真正静止，一直在约 48 mm 范围内游走。**

固件没有位置反馈：`Encoder_Integral` 是速度指令的开环积分，两边都在累积，
误差无处消除。`Velocity_Ki` 只能把游走从 74 mm 压到 48 mm，再往下会压掉
检测器需要的抖动信号。

要根治得加外环：量位置误差 → 算速度指令 → 写 `Move_X`。那是另一件事。

## 怎么测你这台车的死区

源码里 `APP/app_motor.c` 第 5 行：

```c
#define MOTOR_IGNORE_PULSE (1300)//死区  1450 25Khz   此值需要看静止状态微调
```

**厂家自己写明这个数要一车一调**，而且 20 多个工程全用同一个默认值 1300，
没有一台是标定过的。1300/2880 = 45% 占空比，对普通减速电机来说偏大。

十分钟测法：

1. 车**离地**架起来，轮子悬空
2. 临时改成 `#define MOTOR_IGNORE_PULSE (0)`，并注释掉平衡环的调用
3. 直接 `Set_Pwm(n, n)`，n 从 0 每次加 25
4. 记下**轮子刚开始转**的那个 n —— 左右轮分开测，很可能不一样

## 测出来 >1300 怎么办

**改 `MOTOR_IGNORE_PULSE`，不要改 PID。** 死区超过固件补偿量之后，
任何增益都救不回来（实测死区 1700 配 kp 96/144/192/288/384 全部 0/8）。
机制是欠补偿：固件补 1300、电机实际要 1700，小指令出不来力矩，
同时 1700+信号被 2600 的上限截断，可用范围只剩 900 counts。

实测：死区 1700 + 补偿 1300 是 **0/6**；把补偿也改成 1700，立刻变 **5/6**。

所以：把 `MOTOR_IGNORE_PULSE` 改成你实测的值，然后用 `db1300/` 这一套。
"""


def main(outdir):
    """每个死区版本一个子目录：<outdir>/db650/ db900/ db1300/。"""
    now = "%s" % datetime.now().strftime("%Y-%m-%d %H:%M")
    for db, v in sorted(VARIANTS.items(),
                        key=lambda kv: (kv[1]["db"], str(kv[0]))):
        d = os.path.join(outdir, "db%s" % db)
        if not os.path.isdir(d):
            os.makedirs(d)
        db_n = v["db"]
        sub = dict(when="%s   (motor dead band %d)" % (now, db_n),
                   g_normal=tuple_literal(v["normal"]),
                   g_heavy=tuple_literal(v["heavy"]),
                   g_factory=lit(FACTORY), fs=FS,
                   a_hp=alpha(D.HP_TAU), a_lp=alpha(D.LP_TAU),
                   a_en=alpha(D.ENERGY_TAU), lo=HZ_LO, hi=HZ_HI,
                   lsb=D.GYRO_LSB_PER_DEG_S,
                   loaded=v["loaded"], empty=v["empty"],
                   prime=3.0 * D.ENERGY_TAU, confirm=D.CONFIRM_SECONDS,
                   lockout=D.LOCKOUT_SECONDS, db=db_n, margin=v["margin"],
                   comp=v["comp"],
                   kpx=v["normal"][0] / 96.0,
                   p_hp=alpha(P_HP_TAU), p_lp=alpha(P_LP_TAU),
                   p_en=alpha(P_EN_TAU),
                   chat=v["chat"])
        files = [("load_ctrl.h", HEADER % sub, "ascii"),
                 ("load_ctrl.c", SOURCE % sub, "ascii"),
                 ("load_ctrl_README.md",
                  README.format(when=now, db=db_n,
                                kp=v["normal"][0] * 100.0,
                                nkd=v["normal"][1] * 100.0,
                                hkd=v["heavy"][1] * 100.0,
                                loaded=v["loaded"], empty=v["empty"],
                                margin=v["margin"], chat=v["chat_cn"]), "utf-8")]
        for name, text, enc in files:
            if enc == "ascii":
                text.encode("ascii")       # 保证 Keil 不会碰到非 ASCII
            with open(os.path.join(d, name), "w",
                      encoding=enc, newline="\r\n") as f:
                f.write(text)
        n, h = v["normal"], v["heavy"]
        print("db%-7s NORMAL Kp=%-6.0f Kd=%-5.0f Vkp=%-6.0f Vki=%-5.0f"
              % (db, n[0] * 100, n[1] * 100, n[2] * 100, n[3] * 100))
        print("%7s HEAVY  Kp=%-6.0f Kd=%-5.0f Vkp=%-6.0f Vki=%-5.0f"
              % ("", h[0] * 100, h[1] * 100, h[2] * 100, h[3] * 100))
        print("%7s LOADED_BELOW=%.1f  EMPTY_ABOVE=%.1f  margin %.2fx"
              % ("", v["loaded"], v["empty"], v["margin"]))
    print("写入 %s" % os.path.abspath(outdir))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "keil_out")
