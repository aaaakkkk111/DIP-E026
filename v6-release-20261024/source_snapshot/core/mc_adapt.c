/* mc_adapt.c -- firmware glue for mc_core.
 *
 * In the 5 ms control ISR (app_control.c):
 *   Encoders read          -> MC_FwObserve(Encoder_Left, Encoder_Right)
 *   PID computed           -> MC_FwOutput(&Motor_Left, &Motor_Right)   (was PWM_Ignore x2)
 *   PWM limited, Set_Pwm   -> MC_FwApplied(motors_live, Motor_Left, Motor_Right)
 *
 * With mc.enable == 0 (the default) MC_FwOutput calls the factory PWM_Ignore, so the
 * firmware behaves exactly like the v4 release (and the serial 'd' command still sets
 * the factory compensation); the estimator keeps learning in the background ("shadow
 * mode") so its estimates can be logged and compared on the real car.
 *
 * Backlash test: only while the car has not been started (Stop_Flag == 1: mode chosen,
 * second KEY1 not pressed), i.e. while the firmware itself drives no motor.  The user
 * holds the chosen wheel still; the test drives that motor alternately forward and back,
 * BT_REV half-cycles of 0.3 s at each of two PWM levels just above the dead band, and
 * records the motor-encoder travel of each half-cycle (the free play plus the gear
 * spring's deflection).  Two levels let the spring part be extrapolated away.
 */
#pragma Ospace   /* image must stay below 64 KB on this board */
#include "mc_adapt.h"
#include "AllHeader.h"
#include <math.h>

MC_State mc;

volatile int bt_active = 0, bt_done = 0, bt_side = 0, bt_aborted = 0;
int bt_levels[BT_LEVELS];
int bt_travel[BT_LEVELS][BT_REV];

#define BT_HALF_TICKS 60          /* 0.3 s per direction */

static int live_prev = 0;
static int bt_lvl, bt_half, bt_tick, bt_acc, bt_out;
static int cal_l, cal_r;
volatile int cal_aborted = 0;

void MC_FwInit(void)
{
    MC_Init(&mc);
}

static void bt_step(int enc)
{
    bt_acc += enc;
    if (++bt_tick < BT_HALF_TICKS) return;
    /* half-cycle finished: the first one only seats the gears on one side */
    if (bt_half > 0 && bt_half <= BT_REV)
        bt_travel[bt_lvl][bt_half - 1] = bt_acc < 0 ? -bt_acc : bt_acc;
    bt_acc = 0; bt_tick = 0;
    if (++bt_half > BT_REV) {
        bt_half = 0;
        if (++bt_lvl >= BT_LEVELS) { bt_active = 0; bt_done = 1; bt_out = 0; return; }
    }
    bt_out = ((bt_half & 1) ? -1 : 1) * bt_levels[bt_lvl];
}

int MC_FwCalStart(void)
{
    if (Stop_Flag != 1 || bt_active || mc.cal_active) return -1;
    cal_aborted = 0;
    MC_CalStart(&mc);
    return 0;
}

int MC_FwBacklashStart(int side)
{
    if (Stop_Flag != 1 || bt_active || mc.cal_active) return -1;
    bt_side = side ? 1 : 0;
    bt_levels[0] = MC_Comp(&mc, bt_side, 1) + 80;
    bt_levels[1] = MC_Comp(&mc, bt_side, 1) + 200;
    bt_lvl = 0; bt_half = 0; bt_tick = 0; bt_acc = 0;
    bt_out = bt_levels[0];
    bt_done = 0; bt_aborted = 0;
    bt_active = 1;
    return 0;
}

void MC_FwObserve(int enc_l, int enc_r)
{
    int ok = live_prev && fabsf(Angle_Balance) < 15.0f && !bt_active;
    MC_Observe(&mc, enc_l, enc_r, battery, ok);
    if (bt_active) {
        if (Stop_Flag != 1) { bt_active = 0; bt_aborted = 1; bt_out = 0; }   /* car was started */
        else bt_step(bt_side ? enc_r : enc_l);
    }
    if (mc.cal_active) {
        if (Stop_Flag != 1) { mc.cal_active = 0; cal_aborted = 1; cal_l = cal_r = 0; }
        else MC_CalTick(&mc, enc_l, enc_r, &cal_l, &cal_r);
        if (!mc.cal_active) cal_l = cal_r = 0;
    }
}

void MC_FwOutput(int *ml, int *mr)
{
    if (!mc.enable) {
        *ml = PWM_Ignore(*ml);
        *mr = PWM_Ignore(*mr);
        return;
    }
    *ml = MC_Output(&mc, 0, *ml);
    *mr = MC_Output(&mc, 1, *mr);
}

void MC_FwApplied(int live, int ml, int mr)
{
    live_prev = live;
    if (bt_active || bt_aborted || bt_done == 1 || mc.cal_active || cal_l || cal_r) {
        int l = (bt_active && bt_side == 0) ? bt_out : cal_l;
        int r = (bt_active && bt_side == 1) ? bt_out : cal_r;
        if (!live) Set_Pwm(l, r);                    /* the firmware itself drives nothing now */
        MC_Applied(&mc, l, r);
        return;
    }
    MC_Applied(&mc, live ? ml : 0, live ? mr : 0);
}
