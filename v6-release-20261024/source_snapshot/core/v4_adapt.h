/* v4_adapt.h -- mode 27: firmware glue around v4_core (see v4_core.h). */
#ifndef __V4_ADAPT_H
#define __V4_ADAPT_H

void V4_Reset(void);
void V4_Tick(int enc_l, int enc_r);          /* call every 5 ms tick, BEFORE Balance_PD() */

extern float v4_f;           /* current gain interpolation, 0 = Normal, 1 = Weight_M */
extern float v4_osc;         /* 8-16 Hz chatter, deg/s (same as la_osc) */
extern int   v4_latched;     /* 1 = load decision latched, 0 = probing */
extern long  v4_rearms;      /* upshift-and-reprobe count since reset */
extern float v4_th_ref;      /* balance-angle reference, deg (log only) */
extern int   v4_rec;         /* 1 = in recovery after an upshift (log only) */

/* large-tilt recovery (rec_core.c) */
#include "rec_core.h"
extern REC_State v4_rs;
extern int   v4_rec_act;     /* 1 = rec_core is driving this tick */
extern int   v4_rec_lock;    /* 1 = motors locked off (stuck > 40 deg for 1 s, or get-up gave up) */
extern float v4_light_split, v4_mid_split;   /* v5 three-level load probe (v4_core.c) */
extern int   v4_opt;  extern float v4_wamp;   /* v5.1 switches (v4_core.c) */
extern float v4_swup;                          /* v5.2 sway upshift threshold, deg RMS */
extern float v4_dnsmall;                       /* v5.2 small-chatter downshift threshold, deg/s */
extern int   v4_sr_drive;    /* 1 = get-up attempt: Turn_Off ignores the tilt cut-off and the lock */
void V4_RecOverride(int *ml, int *mr);   /* after Velocity_PI: speed integral held at 0; get-up push */

#endif
