/* v4_adapt.h -- mode 27: firmware glue around v4_core (see v4_core.h). */
#ifndef __V4_ADAPT_H
#define __V4_ADAPT_H

void V4_Reset(void);
void V4_Tick(void);          /* call every 5 ms tick, BEFORE Balance_PD() */

extern float v4_f;           /* current gain interpolation, 0 = Normal, 1 = Weight_M */
extern float v4_osc;         /* 8-16 Hz chatter, deg/s (same as la_osc) */
extern int   v4_latched;     /* 1 = load decision latched, 0 = probing */
extern long  v4_rearms;      /* upshift-and-reprobe count since reset */
extern float v4_th_ref;      /* balance-angle reference, deg (log only) */
extern int   v4_rec;         /* 1 = in recovery after an upshift (log only) */

#endif
