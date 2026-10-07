/* v4_core.h -- mode 27 decision logic, free of firmware globals.
 *
 * Port of scripts/rl/switch_v3.py + switch_v4.py (2026-10-01).  Kept pure so
 * the same file compiles on the host (gcc) and is checked tick-for-tick
 * against the Python version: see scripts/rl/v4_port_check.py.
 *
 * Inputs per 5 ms tick:  Angle_Balance (deg), Gyro_Balance (raw LSB, 16.4 per
 * deg/s), and whether the speed command is non-zero.  Output: f, the
 * interpolation coefficient between the stock Normal (f=0) and Weight_M (f=1)
 * gain sets.  V4_GainsFor() turns f into the six firmware gains (x100 units).
 *
 * What it does, in one paragraph:
 *   boot on Weight_M (f=1, the point where chatter-vs-load is calibrated);
 *   while standing still, wait until the 8-16 Hz chatter reading has settled,
 *   then latch ONE decision: chatter >= 3 deg/s -> light (f=0.12), else heavy
 *   (f=0.40).  While driving, only a "light" decision is allowed, and only on
 *   an unmistakable reading (every sample of the last 0.5 s >= 20).  Any tilt
 *   that is too large for the present gains (predicted tilt theta + 0.1*omega
 *   above 3 deg standing / 16 deg moving) jumps back to Weight_M and re-probes;
 *   a re-probe after such an event judges "light" only on >= 20 as well.
 *
 * Round 2 (2026-10-01, tune_v4_r2.py): the standing threshold is
 * max(3 deg, 6 x RMS of this car's own standing theta_pred), so a car that
 * shakes more does not upshift on its own noise; the balance angle th_ref
 * follows fast while driving / just stopped (slopes) and slow once standing;
 * and 0.3 s after an upshift, if the low-passed predicted tilt never reached
 * 6 deg, the event was small and the previous gear is restored at once.
 */
#ifndef __V4_CORE_H
#define __V4_CORE_H

#define V4_FS        200.0f
#define V4_WIN_N     60      /* CONV_WIN_S 0.3 s */
#define V4_MWIN_N    100     /* MOVE_WIN_S 0.5 s */

typedef struct {
    /* chatter (same IIRs as load_adapt.c la_osc) */
    float hp, bp, en, osc;
    /* load decision (switch_v3) */
    int   latched, event_probe, move_latched;
    float f, f_latched, osc_at_latch;
    int   n_still, n_move;
    float win[V4_WIN_N];   int win_n, win_head;
    float mwin[V4_MWIN_N]; int mwin_n, mwin_head;
    long  n_tick, t_latch;
    /* slow escape (switch_v3) */
    float struggle, boost;
    long  escape_ticks;
    /* upshift on large tilt (switch_v4) */
    int   th_ref_ok;
    float th_ref, th_pred;
    int   n_cmd_still, arm_cnt, armed, was_latched;
    int   rec, calm;
    long  rec_ticks, rec_entries, rearms;
    /* round 2 (switch_v4 2026-10-01): low-passed gyro, noise-adaptive
       standing threshold, quick revert after a small event */
    float om_lp, noise, on_still;
    int   qr_active, qr_n;
    float qr_peak, qr_prev;
    long  qr_reverts;
    /* round 3: stop-calm gate, optional re-probe on stop */
    int   stop_calm, stop_calm_n;
    long  n_drive, stop_probes;
    /* fast light: readings since the last unlatch (boot or upshift), moving or not */
    float fwin[V4_WIN_N];  int fwin_n, fwin_head;
    long  n_unl, fast_latches;
    /* command-edge grace: ticks since the drive command last changed */
    long  edge_n, edge_skips;
    long  up_n, relatch_graces;   /* ticks since the last upshift; post-upshift relatch graces */
} V4_State;

typedef struct { float bkp, bkd, vkp, vki, tkp, tkd; } V4_Gains;  /* x100 */

void  V4_Init(V4_State *s);
float V4_Step(V4_State *s, float angle_deg, float gyro_lsb, int moving);
void  V4_CmdEdge(V4_State *s);   /* call when the drive command changes (key pressed / released) */
void  V4_GainsFor(float f, V4_Gains *g);

#endif
