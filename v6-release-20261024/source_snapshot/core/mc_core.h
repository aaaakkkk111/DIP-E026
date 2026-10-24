/* mc_core.h -- motor compensation: dead band that follows the battery voltage (+ a
 * wheels-in-the-air self-calibration) and optional gear-backlash handling.
 *
 * Pure logic, no hardware access (same pattern as v4_core): the firmware calls it from
 * the 5 ms control ISR, and the host builds it into a DLL so the digital twin runs the
 * same code.
 *
 * Why this design (2026-10-12):
 *   - the breakaway PWM rises as the battery voltage falls (torque ~ Kt*Vb*duty/R, so the
 *     breakaway duty ~ 1/Vb): every compensation is scaled by MC_VREF / Vb each tick;
 *   - the breakaway itself (and its slow drift: temperature, wear, the two motors not
 *     matching) is measured by a SELF-CALIBRATION with the wheels in the air: each wheel
 *     and direction is ramped slowly from MC_CAL_START until the encoder moves.  With no
 *     ground and no body acting on the wheel this measures the motor + gearbox alone;
 *   - an ON-LINE estimator (pushes from rest while balancing) is kept for logging only:
 *     on the 2026-10-07 standing logs it looked plausible, but in the twin it ran the wrong
 *     way when the real dead band was lower (wheels that stop while balancing are often
 *     held by the body, so "did not move" is not about friction).  use_online = 0 by
 *     default;
 *   - backlash: optional take-up through the play on reversals (gap > 0) and an optional
 *     small-command hold (|u| < hold -> 0) that avoids crossing the play at all.
 *
 * Everything is OFF until enable = 1: with enable = 0 the output equals the factory
 * PWM_Ignore (1480 forward / 1455 reverse), bit for bit.
 */
#ifndef __MC_CORE_H
#define __MC_CORE_H

/* The on-line estimator (pushes from rest while balancing) was logging only and proved
 * unreliable in the twin; it is compiled out of the firmware (2026-10-21, flash for the
 * mode-27 get-up).  Build with -DMC_ONLINE=1 to get it back (the host DLL may). */
#ifndef MC_ONLINE
#define MC_ONLINE    0
#endif

#define MC_VREF      12.0f
#define MC_REST      3
#define MC_W         4
#define MC_BAND      30.0f
#define MC_STEP      2.0f
#define MC_COMP_MIN  1350
#define MC_COMP_MAX  1800
#define MC_CAL_START 1300
#define MC_CAL_RAMP  2        /* PWM counts per 5 ms tick */
#define MC_CAL_MAX   1900
#define MC_CAL_REST  60       /* ticks between ramps (0.3 s) */

typedef struct {
    /* configuration (serial-settable) */
    int   enable;          /* 0 = factory PWM_Ignore exactly */
    int   learn;           /* on-line estimator updates (logging) */
    int   use_online;      /* 1 = compensation from the on-line estimate (experimental) */
    float margin;          /* comp = calibrated breakaway - margin (default 0: twin 10-12, 20 hurt with backlash) */
    int   gap;             /* backlash, motor encoder counts (0 = off) */
    int   takeup;          /* PWM above comp used while crossing the play */
    int   hold;            /* |u| < hold -> 0 (0 = off) */
    /* breakaway at MC_VREF: [wheel 0 L / 1 R][dir 0 fwd / 1 rev] */
    float cal[2][2];       /* self-calibration (wheels in the air) */
    int   calibrated;
    float est[2][2];       /* on-line estimate (logging) */
    float vb;              /* last battery voltage */
    /* on-line push detection per wheel */
    int   rest[2];
    int   push_n[2], push_dir[2], push_moved[2];
    float push_sum[2];
    int   last_ccr[2];     /* PWM actually applied last tick */
    long  pushes, updates;
    /* backlash per wheel: motor position inside the play, 0 = teeth touching on the
       reverse side, gap = touching on the forward side (moved by the motor encoder,
       clamped at both ends) */
    int   play[2];
    /* self-calibration state */
    int   cal_active, cal_step, cal_pwm, cal_wait, cal_acc;
} MC_State;

void  MC_Init(MC_State *s);
/* Every tick, BEFORE computing the new PWM: this tick's encoder counts (forward
   positive), the battery voltage, and whether on-line learning is allowed now. */
void  MC_Observe(MC_State *s, int enc_l, int enc_r, float vbat, int learn_ok);
/* Pre-compensation command u (the PID output) -> PWM to write, for wheel side 0/1. */
int   MC_Output(MC_State *s, int side, int u);
/* After limiting / motor cut-off: the PWM really applied this tick. */
void  MC_Applied(MC_State *s, int ccr_l, int ccr_r);
/* Compensation for a wheel and direction (+1 / -1) right now. */
int   MC_Comp(const MC_State *s, int side, int dir);
/* Self-calibration (wheels in the air, nothing else driving the motors): start it, then
   every tick call MC_CalTick with this tick's encoders; it returns 1 while running and
   writes the PWM to apply into *pl / *pr. */
void  MC_CalStart(MC_State *s);
int   MC_CalTick(MC_State *s, int enc_l, int enc_r, int *pl, int *pr);

#endif
