/* rec_core.c -- see rec_core.h */
#pragma Ospace   /* image must stay below 64 KB on this board */
#include "rec_core.h"

enum { SR_IDLE = 0, SR_WAIT, SR_PUSH1, SR_COAST, SR_PUSH2, SR_OFF };

static float fabs_(float x) { return x < 0.0f ? -x : x; }

void REC_Init(REC_State *s)
{
    s->mode = 1; s->on_deg = 20.0f;
    s->exit_deg = 2.0f; s->exit_dps = 30.0f; s->exit_v = 4.0f;
    s->active = 0; s->calm = 0; s->over = 0; s->locked = 0;
    s->entries = 0; s->locks = 0; s->ticks = 0;
    s->sr_en = 1; s->sr_push1 = 40; s->sr_coast = 30; s->sr_push2 = 120; s->sr_hand = 30.0f; s->sr_rev = 0;
    s->sr_state = SR_IDLE; s->sr_n = 0; s->sr_dir = 0; s->sr_tries = 0; s->sr_still = 0;
    s->sr_win = 0; s->was_up = 0; s->sr_attempts = 0; s->sr_ok = 0;
}

void REC_Stopped(REC_State *s)
{
    s->active = 0; s->calm = 0; s->over = 0; s->locked = 0;
    s->sr_state = SR_IDLE; s->sr_win = 0; s->was_up = 1;
}

static void lock(REC_State *s)
{
    if (!s->locked) { s->locked = 1; s->locks++; }
    s->sr_state = SR_IDLE; s->sr_win = 0; s->active = 0; s->calm = 0;
}

int REC_Step(REC_State *s, float err, float gyro_lsb, int enc_l, int enc_r, int *u)
{
    float om = gyro_lsb / 16.4f, v = 0.5f * (float)(enc_l + enc_r), a = fabs_(err);
    *u = 0;

    /* ---- get-up from lying ---- */
    if (a < REC_FALL_DEG) s->was_up = 1;
    else if (s->was_up) {                          /* just fell from upright */
        s->was_up = 0;
        if (s->sr_en && !s->locked) { s->sr_state = SR_WAIT; s->sr_win = SR_WINDOW; s->sr_tries = 0; s->sr_still = 0; }
    }
    if (s->sr_state != SR_IDLE) {
        if (--s->sr_win <= 0 && s->sr_state == SR_WAIT) { lock(s); return REC_NORMAL; }
        switch (s->sr_state) {
        case SR_WAIT:                              /* lying still? */
            if (a > SR_LIE_DEG && fabs_(om) < SR_STILL_DPS) {
                if (++s->sr_still >= SR_STILL_TICKS) {
                    s->sr_state = SR_PUSH1; s->sr_n = 0; s->sr_dir = err > 0.0f ? 1 : -1;
                    s->sr_tries++; s->sr_attempts++;
                }
            } else s->sr_still = 0;
            if (a < REC_FALL_DEG) s->sr_state = SR_IDLE;   /* got up / was put up by hand */
            return REC_NORMAL;
        case SR_PUSH1:
            if (++s->sr_n >= s->sr_push1) { s->sr_state = SR_COAST; s->sr_n = 0; }
            *u = (s->sr_rev ? -s->sr_dir : s->sr_dir) * REC_USPAN;
            return REC_DRIVE;
        case SR_COAST:
            if (++s->sr_n >= s->sr_coast) { s->sr_state = SR_PUSH2; s->sr_n = 0; }
            return REC_DRIVE;                      /* *u = 0 */
        case SR_PUSH2:
            if (a < s->sr_hand) {                  /* up: the balance controller takes over */
                s->sr_state = SR_IDLE; s->sr_ok++; s->was_up = 1;
                s->active = 1; s->calm = 0; s->entries++; s->over = 0;
                return REC_RECOVER;
            }
            if (++s->sr_n >= s->sr_push2) { s->sr_state = SR_OFF; s->sr_n = 0; return REC_NORMAL; }
            *u = s->sr_dir * REC_USPAN;
            return REC_DRIVE;
        default:                                   /* SR_OFF: motors off between attempts */
            if (++s->sr_n >= SR_REST) {
                if (s->sr_tries >= SR_TRIES) { lock(s); return REC_NORMAL; }
                s->sr_state = SR_WAIT; s->sr_still = 0; s->sr_win = SR_WINDOW;
            }
            return REC_NORMAL;
        }
    }

    /* ---- lock: stuck or held at a large tilt ---- */
    if (a > REC_LOCK_DEG) { if (++s->over >= REC_LOCK_TICKS) lock(s); }
    else s->over = 0;
    if (s->locked) {
        if (a < REC_UNLOCK_DEG) { s->locked = 0; s->was_up = 1; }
        s->active = 0; s->calm = 0;
        return REC_NORMAL;
    }

    /* ---- large-tilt recovery ---- */
    if (s->mode == 0) return REC_NORMAL;
    if (!s->active) {
        if (a <= s->on_deg) return REC_NORMAL;
        s->active = 1; s->calm = 0; s->entries++;
    }
    if (a < s->exit_deg && fabs_(om) < s->exit_dps && fabs_(v) < s->exit_v) {
        if (++s->calm >= REC_CALM) { s->active = 0; return REC_NORMAL; }
    } else s->calm = 0;
    s->ticks++;
    return REC_RECOVER;
}
