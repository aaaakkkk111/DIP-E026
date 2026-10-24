/* rec_core.h -- mode 27 large-tilt recovery and get-up from lying on the floor.
 *
 * Pure logic, no firmware globals (same pattern as v4_core / mc_core): the firmware
 * calls it every 5 ms, the host builds it into a DLL so the twin runs this code.
 *
 * 1. Large-tilt recovery (twin 2026-10-20, runs_e2e/rec_final_checks.log):
 *   - the old 40 deg limit was the firmware cut-off, not the motors.  Cut-off is 45 deg;
 *   - the v4 balance loop lifts the car fine; what went wrong afterwards was the speed
 *     loop's integral (position) winding up over the 30 cm of the lift.  So while
 *     recovering (|tilt| > on_deg until upright, slow and calm) the integral is held at
 *     0; v4 keeps deciding (pausing it failed 0/4 when started at a tilt: v4 boots on
 *     the stiff loaded gains and must be allowed to relax them).
 *
 * 2. Get-up from lying (2026-10-21, geometry from the STEP model, runs_e2e/selfright2.log):
 *   - the chassis is boat-shaped; the body first touches flat ground at 78.5 deg and a
 *     fallen car rests at ~79 deg (not 44.5 deg as the old box model said);
 *   - pushing flat out saturates the wheel speed (~1 m/s) in 0.2 s before the body is
 *     up, and the body falls back: 0-2/4.  Push 0.2 s, COAST 0.15 s (the body rocks and
 *     the wheels slow down, which gives back speed margin), push again: 4/4 at 12 V and
 *     10.5 V, upright in ~0.9 s.  The balance controller takes over below sr_hand deg;
 *   - safety: only within SR_WINDOW after a fall from upright, only once the car lies
 *     still, at most SR_TRIES attempts with SR_REST off in between, then locked.
 *
 * Lock (both): |tilt| > REC_LOCK_DEG for REC_LOCK_TICKS outside a get-up attempt ->
 * motors off until |tilt| < REC_UNLOCK_DEG (the chassis rests at ~79 deg, beyond the
 * 45 deg cut-off, so normally the cut-off already stops a fallen car; the lock also
 * covers a car held or stuck between 40 and 45 deg).
 *
 * err = Angle_Balance - Mid_Angle (deg), rate from the raw gyro (16.4 LSB per deg/s),
 * speed = mean of the two encoders (counts per 5 ms, forward positive).
 */
#ifndef __REC_CORE_H
#define __REC_CORE_H

#define REC_USPAN       1320      /* PWM above the dead band (2800 - 1480) = full push */
#define REC_CALM        20        /* 0.1 s */
#define REC_LOCK_DEG    40.0f
#define REC_LOCK_TICKS  200       /* 1 s */
#define REC_UNLOCK_DEG  10.0f
#define REC_FALL_DEG    45.0f     /* = the mode-27 cut-off: crossing it from upright = a fall */
#define SR_LIE_DEG      60.0f
#define SR_STILL_DPS    30.0f
#define SR_STILL_TICKS  100       /* lying still 0.5 s before trying */
#define SR_WINDOW       1000      /* get up only within 5 s of the fall */
#define SR_TRIES        3
#define SR_REST         200       /* motors off 1 s between attempts */

enum { REC_NORMAL = 0, REC_RECOVER = 1, REC_DRIVE = 2 };

typedef struct {
    int   mode;            /* 0 = off (v4 only, lock still active), 1 = on */
    float on_deg;          /* enter recovery above this tilt */
    float exit_deg, exit_dps, exit_v;   /* back to v4 when all below for REC_CALM ticks */
    int   active, calm, over, locked;
    long  entries, locks, ticks;
    /* get-up from lying */
    int   sr_en;           /* 1 = allowed */
    int   sr_push1, sr_coast, sr_push2;   /* ticks */
    int   sr_rev;          /* 1 = first push AWAY from the lean ("flywheel" start).  Twin 10-21
                              (getup_dist.log): 1-4/8 and no shorter -- the wheels do not slip
                              enough, the car is dragged back and forth.  Keep 0. */
    float sr_hand;         /* hand over to the balance controller below this tilt */
    int   sr_state, sr_n, sr_dir, sr_tries, sr_still, sr_win, was_up;
    long  sr_attempts, sr_ok;
} REC_State;

void REC_Init(REC_State *s);
/* REC_NORMAL: v4 as usual.  REC_RECOVER: v4 as usual, speed integral held at 0.
   REC_DRIVE: get-up attempt -- write *u (before the dead band) to both motors, ignore
   the angle cut-off and the lock, speed integral held at 0.  s->locked: motors off. */
int  REC_Step(REC_State *s, float err_deg, float gyro_lsb, int enc_l, int enc_r, int *u);
/* Every tick while the motors are switched off by the user (Stop_Flag, before the 2nd
   KEY1): clears the state and counts the car as upright, so that starting it while it
   lies on the floor arms a get-up exactly like a fall does. */
void REC_Stopped(REC_State *s);

#endif
