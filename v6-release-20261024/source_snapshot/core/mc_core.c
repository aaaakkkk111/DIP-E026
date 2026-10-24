/* mc_core.c -- see mc_core.h */
#pragma Ospace   /* image must stay below 64 KB on this board */
#include "mc_core.h"

#define FACTORY_FWD 1480
#define FACTORY_REV 1455

static int iabs(int x) { return x < 0 ? -x : x; }
static int isgn(int x) { return x > 0 ? 1 : (x < 0 ? -1 : 0); }

void MC_Init(MC_State *s)
{
    int w;
    s->enable = 0; s->learn = 1; s->use_online = 0;
    s->margin = 0.0f;
    s->gap = 0; s->takeup = 150; s->hold = 0;
    s->calibrated = 0;
    for (w = 0; w < 2; w++) {
        s->cal[w][0] = s->est[w][0] = 1500.0f;
        s->cal[w][1] = s->est[w][1] = 1500.0f;
        s->rest[w] = 0; s->push_n[w] = 0; s->push_dir[w] = 0; s->push_moved[w] = 0;
        s->push_sum[w] = 0.0f; s->last_ccr[w] = 0;
        s->play[w] = 0;
    }
    s->vb = MC_VREF; s->pushes = 0; s->updates = 0;
    s->cal_active = 0; s->cal_step = 0; s->cal_pwm = 0; s->cal_wait = 0; s->cal_acc = 0;
}

/* breakaway duty ~ 1/Vb: PWM needed now = PWM at MC_VREF * vscale */
static float vscale(const MC_State *s)
{
    float vb = s->vb;
    if (vb < 7.0f || vb > 14.0f) vb = MC_VREF;   /* bad reading: no feed-forward */
    return MC_VREF / vb;
}

int MC_Comp(const MC_State *s, int side, int dir)
{
    int d = dir > 0 ? 0 : 1;
    float c;
    if (!s->enable) return dir > 0 ? FACTORY_FWD : FACTORY_REV;
#if MC_ONLINE
    if (s->use_online)      c = s->est[side][d] * vscale(s) - s->margin;
    else
#endif
    if (s->calibrated) c = s->cal[side][d] * vscale(s) - s->margin;
    else                    c = (float)(dir > 0 ? FACTORY_FWD : FACTORY_REV) * vscale(s);   /* voltage feed-forward only */
    if (c < MC_COMP_MIN) c = MC_COMP_MIN;
    if (c > MC_COMP_MAX) c = MC_COMP_MAX;
    return (int)(c + 0.5f);
}

#if MC_ONLINE
static void finish_push(MC_State *s, int w)
{
    int d = s->push_dir[w] > 0 ? 0 : 1;
    float lev_ref = (s->push_sum[w] / (float)MC_W) / vscale(s);
    float *e = &s->est[w][d];
    s->pushes++;
    if (s->learn && lev_ref > *e - MC_BAND && lev_ref < *e + MC_BAND) {
        *e += s->push_moved[w] ? -MC_STEP : MC_STEP;
        if (*e < 1250.0f) *e = 1250.0f;
        if (*e > 1850.0f) *e = 1850.0f;
        s->updates++;
    }
    s->push_n[w] = 0;
}
#endif

void MC_Observe(MC_State *s, int enc_l, int enc_r, float vbat, int learn_ok)
{
    int w, enc;
#if MC_ONLINE
    int ccr;
#else
    (void)learn_ok;
#endif
    s->vb = vbat;
    for (w = 0; w < 2; w++) {
        enc = w ? enc_r : enc_l;     /* produced by last_ccr, the PWM of the previous tick */
        if (s->gap > 0) {            /* backlash: motor position inside the play */
            s->play[w] += enc;
            if (s->play[w] < 0) s->play[w] = 0;
            if (s->play[w] > s->gap) s->play[w] = s->gap;
        }
#if MC_ONLINE
        ccr = s->last_ccr[w];
        /* on-line pushes from rest (logging): PWM ticks k..k+W-1 one way after
           >= MC_REST ticks at rest; outcome = an encoder step that way in the next W+1 reads */
        if (!learn_ok) { s->push_n[w] = 0; s->rest[w] = 0; continue; }
        if (s->push_n[w] > 0) {
            if (enc != 0 && isgn(enc) == s->push_dir[w]) s->push_moved[w] = 1;
            if (s->push_n[w] >= MC_W) {
                finish_push(s, w);
            } else if (ccr != 0 && isgn(ccr) == s->push_dir[w]) {
                s->push_sum[w] += (float)iabs(ccr);
                s->push_n[w]++;
            } else {
                s->push_n[w] = 0;
            }
        } else if (s->rest[w] >= MC_REST && ccr != 0) {
            s->push_n[w] = 1;
            s->push_dir[w] = isgn(ccr);
            s->push_sum[w] = (float)iabs(ccr);
            s->push_moved[w] = (enc != 0 && isgn(enc) == s->push_dir[w]);
        }
        s->rest[w] = (enc == 0) ? s->rest[w] + 1 : 0;
#endif
    }
}

int MC_Output(MC_State *s, int side, int u)
{
    int sg = isgn(u), comp, need;
    if (!s->enable) {                                /* factory PWM_Ignore, bit for bit */
        if (u > 0) return u + FACTORY_FWD;
        if (u < 0) return u - FACTORY_REV;
        return 0;
    }
    if (s->hold > 0 && iabs(u) < s->hold) return 0;  /* small-command hold */
    if (sg == 0) return 0;
    comp = MC_Comp(s, side, sg);
    if (s->gap > 0) {
        need = sg > 0 ? s->gap - s->play[side] : s->play[side];
        if (need > 0) {
            int extra = iabs(u) > s->takeup ? iabs(u) : s->takeup;
            return sg * (comp + extra);
        }
    }
    return u + sg * comp;
}

void MC_Applied(MC_State *s, int ccr_l, int ccr_r)
{
    s->last_ccr[0] = ccr_l;
    s->last_ccr[1] = ccr_r;
}

/* ---- self-calibration: steps 0..3 = L fwd, L rev, R fwd, R rev ---- */
void MC_CalStart(MC_State *s)
{
    s->cal_active = 1; s->cal_step = 0; s->cal_pwm = 0;
    s->cal_wait = MC_CAL_REST; s->cal_acc = 0;
}

int MC_CalTick(MC_State *s, int enc_l, int enc_r, int *pl, int *pr)
{
    int w, dir, enc;
    *pl = 0; *pr = 0;
    if (!s->cal_active) return 0;
    w = s->cal_step >> 1;
    dir = (s->cal_step & 1) ? -1 : 1;
    enc = w ? enc_r : enc_l;
    if (s->cal_wait > 0) {                           /* let the wheel stop first */
        if (enc != 0) s->cal_wait = MC_CAL_REST; else s->cal_wait--;
        if (s->cal_wait == 0) { s->cal_pwm = MC_CAL_START; s->cal_acc = 0; }
        return 1;
    }
    if (isgn(enc) == dir) s->cal_acc += iabs(enc);
    if (s->cal_acc >= 2 || s->cal_pwm >= MC_CAL_MAX) {
        /* breakaway: the PWM two ticks ago started it (encoder lag) */
        float p = (float)(s->cal_pwm - 2 * MC_CAL_RAMP);
        s->cal[w][dir > 0 ? 0 : 1] = p / vscale(s);  /* store at the reference voltage */
        s->cal_pwm = 0;
        if (++s->cal_step >= 4) { s->cal_active = 0; s->calibrated = 1; return 0; }
        s->cal_wait = MC_CAL_REST;
        return 1;
    }
    s->cal_pwm += MC_CAL_RAMP;
    if (w) *pr = dir * s->cal_pwm; else *pl = dir * s->cal_pwm;
    return 1;
}
