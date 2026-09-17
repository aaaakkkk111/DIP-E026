/* load_ctrl.h -- automatic load-mode switching for the Yahboom balance car.
 * Generated 2026-09-16 14:55   (motor dead band 900) by scripts/export_keil.py -- do not edit by hand.
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
 * ld_rms says nothing about these: it is band-limited to 27..45 Hz
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
 * numbers with only a 1.69x margin -- they MUST be recalibrated on the
 * real car before the detector is trusted.  Four readings; see the README. */
extern float LD_LOADED_BELOW;   /* in NORMAL, below this -> car is loaded */
extern float LD_EMPTY_ABOVE;    /* in HEAVY,  above this -> car is empty  */

#endif /* __LOAD_CTRL_H */
