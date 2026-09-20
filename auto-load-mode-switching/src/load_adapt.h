/* load_adapt.h -- mode 26: symptom-driven PID gain ladder.
 *
 * No payload estimate.  Two measured symptoms move a 5-step gain ladder:
 *
 *   too much gain for the current load -> 8-16 Hz chatter -> step DOWN
 *   too little gain for the load       -> slow lean       -> step UP
 *
 * The bands come from this car, not from simulation (2026-09-16 recordings):
 *   empty + Kd 48 : 8-16 Hz gyro RMS  4.55 deg/s, angle std 0.25 deg
 *   empty + Kd 120: 8-16 Hz gyro RMS 94.93 deg/s, angle std 3.95 deg
 * a 20.9x separation, which is what makes the down-shift reliable.
 *
 * Boots at the TOP of the ladder on purpose: too much gain only shakes,
 * too little drops a loaded car.
 */
#ifndef __LOAD_ADAPT_H
#define __LOAD_ADAPT_H

#define LA_LEVELS 5

void  LA_Reset(void);
void  LA_Tick(int ml, int mr, int el, int er);

extern int   la_level;      /* 0 = lightest gains, LA_LEVELS-1 = heaviest */
extern int   la_force;      /* -1 = automatic, else hold this level */
extern int   la_panic;      /* express-lane jump counter, for logs  */
extern int   LA_BOOT_LEVEL; /* level chosen at mode select          */
extern float la_osc;        /* 8-16 Hz gyro RMS, deg/s -- down-shift signal */
extern float la_lean;       /* 0.3-2 Hz angle RMS, deg -- up-shift signal */
extern float la_sat;        /* smoothed PWM clipping fraction, 0..1     */
extern float LA_OSC_HI;     /* above this for LA_DWELL_S -> step down */
extern float LA_LEAN_HI;    /* 0.3-2 Hz angle RMS above this -> step up */
extern float LA_OSC_LO;     /* ...but only while osc is below this      */
extern float LA_ANG_PANIC;  /* |pitch| past this -> jump straight to top */
extern float la_k;          /* payload measurement: alpha per unit drive */
extern float LA_K_EMPTY;    /* la_k anchor with nothing aboard           */
extern float LA_K_FULL;     /* la_k anchor at the top of the load range  */
extern float LA_DWELL_S;    /* how long a symptom must persist        */
extern float LA_LOCK_S;     /* settle time after a shift              */

#endif /* __LOAD_ADAPT_H */
