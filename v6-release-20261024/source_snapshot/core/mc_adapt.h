/* mc_adapt.h -- firmware glue for mc_core (dead-band tracking + backlash take-up)
 * and the backlash measurement test.  Serial commands live in tune_io.c ('a...'). */
#ifndef __MC_ADAPT_H
#define __MC_ADAPT_H
#include "mc_core.h"

extern MC_State mc;

/* backlash test (motor-side encoder travel at each reversal against a held wheel) */
#define BT_LEVELS   2
#define BT_REV      6            /* reversals recorded per level */
extern volatile int bt_active, bt_done, bt_side, bt_aborted;
extern int bt_levels[BT_LEVELS];
extern int bt_travel[BT_LEVELS][BT_REV];

void MC_FwInit(void);
void MC_FwObserve(int enc_l, int enc_r);          /* right after the encoders are read */
void MC_FwOutput(int *ml, int *mr);               /* replaces the two PWM_Ignore calls */
void MC_FwApplied(int live, int ml, int mr);      /* after Set_Pwm */
int  MC_FwBacklashStart(int side);                /* 0 ok, -1 refused (car is balancing) */
int  MC_FwCalStart(void);                         /* wheels-in-the-air self-calibration */
extern volatile int cal_aborted;

#endif
