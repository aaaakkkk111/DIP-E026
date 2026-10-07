/* v4_core.c -- see v4_core.h.  Every constant below names its Python source.
 *
 * One deliberate difference from the Python: the slow escape measures "is the
 * car struggling" against the v4 balance-angle tracker th_ref.  The Python v3
 * used the simulator's gravity_pitch there, which the real car cannot know.
 */
#include "v4_core.h"
#include <math.h>

/* ---- switch_v3.py ---- */
#define HP_A          0.200849f   /* 8 Hz  (load_adapt.c LA_A_HP) */
#define LP_A          0.334511f   /* 16 Hz (LA_A_LP) */
#define EN_A          0.016393f   /* energy, ~0.3 s (LA_A_EN) */
#define LSB_DPS       16.4f
#define PROBE_MIN_N   240         /* PROBE_MIN_S 1.2 */
#define PROBE_MAX_N   600         /* PROBE_MAX_S 3.0 */
#ifndef FAST_LIGHT_N
#define FAST_LIGHT_N  100         /* FAST_LIGHT_S 0.5; 0 = off.  2026-10-03 real car waited the full 3 s */
#endif
#define CONV_TOL      0.15f
#define OSC_SPLIT     23.0f       /* 2026-10-01: "light only when sure" (switch_v3.py) */
#define F_LIGHT       0.0f        /* 2026-10-03 real car: 0.12 kept the empty car oscillating
                                     (mode-26 records: empty reads 4.5-7.4 at f=0 but 53.8-58.3
                                     at f=0.25-0.5; the twin under-reads this 30x) */
#define F_HEAVY       0.40f
#define MOVE_SPLIT    20.0f
#define MOVE_MIN_N    200         /* MOVE_MIN_S 1.0 */
#define EVENT_SPLIT   20.0f
#define RATE_UP       0.022f
#define STRUGGLE_DEG  7.1031f
#define STRUGGLE_LPF  0.01f
#define BOOST_DECAY   0.99667221f /* exp(-1/(BOOST_TAU 1.5 * 200)) */

/* ---- switch_v4.py ---- */
#define REC_S         0.10f
#ifndef REC_ON_STILL
#define REC_ON_STILL  3.0f        /* REC_LIGHT / REC_HEAVY threshold */
#endif
#define REC_ON_MOVE   16.0f
#define STILL_N       200         /* STILL_S 1.0 */
#define ARM_DEG       2.5f
#define ARM_N         40          /* ARM_S 0.2 */
#define REC_OFF_DEG   3.0f
#define REC_OFF_DPS   60.0f
#define REC_OFF_TICKS 4
#define REF_RATE      (1.0f / 400.0f)   /* 1/(REF_TAU 2.0 * 200) */
#define REF_CALM_DEG  4.0f
#define BIG           1e30f
/* round 2 */
#define REF_RATE_FAST (1.0f / 50.0f)    /* 1/(REF_TAU_FAST 0.25 * 200) */
#ifndef NOISE_K
#define NOISE_K       6.0f              /* 0 = off; tune_v4_r2.py 2026-10-01 */
#endif
#define NOISE_RATE    (1.0f / 400.0f)   /* 1/(NOISE_TAU 2.0 * 200) */
#define NOISE_CAP     8.0f              /* standing threshold never above this */
#define QR_N          60                /* QR_T 0.3 */
#ifndef QR_PEAK
#define QR_PEAK       6.0f              /* < 0 = off (Python None) */
#endif
#define OMLP_A        0.08994276f       /* 1-exp(-2*pi*OMLP_HZ 3.0/200) */
/* round 3 */
#ifndef STOP_CALM
#define STOP_CALM     1                 /* standing threshold only after the stop has settled */
#endif
#define STOP_CALM_N   120               /* STOP_CALM_S 0.6 */
#ifndef REPROBE_ON_STOP
#define REPROBE_ON_STOP 0               /* off: user to decide (cost: ~1.2 s stiff chatter per stop) */
#endif
#define REPROBE_DRIVE_N 200             /* REPROBE_DRIVE_S 1.0 */
#ifndef REC_ON_MOVE_HEAVY
#define REC_ON_MOVE_HEAVY (-1.0f)       /* < 0 = off (Python None) */
#endif
#define CLASS_SPLIT_F 0.26f
/* round 4 (2026-10-03 real car): command-edge grace, see V4_CmdEdge */
#ifndef STRUGGLE_IN_REC
#define STRUGGLE_IN_REC 0               /* 1 = old behaviour */
#endif
#ifndef RELATCH_GRACE
#define RELATCH_GRACE 1                 /* 0 = off */
#endif
#define RELATCH_GRACE_WITHIN_N 600      /* relatch within 3 s of the upshift */
#define RELATCH_GRACE_N 400             /* 2 s; re-upshifts seen up to 1.97 s after a relatch */
#ifndef EDGE_GRACE_N
#define EDGE_GRACE_N  300               /* 1.5 s; 0 = off.  Rate-term upshifts seen up to 1.42 s after a key */
#endif

/* stock gain sets, firmware units (x100); Python pid_gains("Normal"/"Weight_M") */
static const V4_Gains GN = {  9600.0f,  48.0f, 6200.0f, 31.00f, 1700.0f, 20.0f };
static const V4_Gains GH = { 19200.0f, 150.0f, 9450.0f, 47.25f, 1400.0f, 20.0f };

static float absf(float x) { return x < 0.0f ? -x : x; }

void V4_GainsFor(float f, V4_Gains *g)
{
    g->bkp = GN.bkp + (GH.bkp - GN.bkp) * f;
    g->bkd = GN.bkd + (GH.bkd - GN.bkd) * f;
    g->vkp = GN.vkp + (GH.vkp - GN.vkp) * f;
    g->vki = GN.vki + (GH.vki - GN.vki) * f;
    g->tkp = GN.tkp + (GH.tkp - GN.tkp) * f;
    g->tkd = GN.tkd + (GH.tkd - GN.tkd) * f;
}

void V4_Init(V4_State *s)
{
    int i;
    s->hp = s->bp = s->en = s->osc = 0.0f;
    s->latched = 0; s->event_probe = 0; s->move_latched = 0;
    s->f = 1.0f; s->f_latched = 1.0f; s->osc_at_latch = 0.0f;
    s->n_still = 0; s->n_move = 0;
    for (i = 0; i < V4_WIN_N; i++)  s->win[i] = 0.0f;
    for (i = 0; i < V4_MWIN_N; i++) s->mwin[i] = 0.0f;
    s->win_n = s->win_head = 0;
    s->mwin_n = s->mwin_head = 0;
    s->n_tick = 0; s->t_latch = -1;
    s->struggle = 0.0f; s->boost = 0.0f; s->escape_ticks = 0;
    s->th_ref_ok = 0; s->th_ref = 0.0f; s->th_pred = 0.0f;
    s->n_cmd_still = 0; s->arm_cnt = 0; s->armed = 0; s->was_latched = 0;
    s->rec = 0; s->calm = 0;
    s->rec_ticks = 0; s->rec_entries = 0; s->rearms = 0;
    s->om_lp = 0.0f; s->noise = 0.0f; s->on_still = 0.0f;
    s->qr_active = 0; s->qr_n = 0; s->qr_peak = 0.0f; s->qr_prev = 1.0f;
    s->qr_reverts = 0;
    s->stop_calm = 0; s->stop_calm_n = 0; s->n_drive = 0; s->stop_probes = 0;
    for (i = 0; i < V4_WIN_N; i++) s->fwin[i] = 0.0f;
    s->fwin_n = s->fwin_head = 0;
    s->n_unl = 0; s->fast_latches = 0;
    s->edge_n = EDGE_GRACE_N; s->edge_skips = 0;
    s->up_n = 1000000L; s->relatch_graces = 0;
}

/* The drive command changed.  For EDGE_GRACE_N ticks the tilt upshift looks at
   the lean alone: the commanded start / brake swings the body fast (up to
   265 deg/s on the real car) but leans it little (<= 11.5 deg), and the
   0.1 s rate prediction turned that swing into a false "push". */
void V4_CmdEdge(V4_State *s) { s->edge_n = 0; }

/* ring buffer: push, and statistics over what it holds */
static void push(float *buf, int cap, int *n, int *head, float x)
{
    buf[*head] = x;
    *head = (*head + 1) % cap;
    if (*n < cap) (*n)++;
}
static float buf_min(const float *b, int n) { int i; float m = b[0]; for (i = 1; i < n; i++) if (b[i] < m) m = b[i]; return m; }
static float buf_max(const float *b, int n) { int i; float m = b[0]; for (i = 1; i < n; i++) if (b[i] > m) m = b[i]; return m; }
static float buf_mean(const float *b, int n) { int i; float a = 0.0f; for (i = 0; i < n; i++) a += b[i]; return a / (float)n; }

/* Jump to Weight_M (f=1, the calibration point) and re-probe.  switch_v4._rearm */
static void rearm(V4_State *s)
{
    s->f = 1.0f; s->f_latched = 1.0f; s->latched = 0;
    s->armed = 0; s->arm_cnt = 0;
    s->boost = 0.0f;
    s->n_still = 0; s->win_n = 0; s->win_head = 0;
    s->event_probe = 1;
    s->rearms++;
}

/* Small event: restore the gear latched before the upshift, no re-probe
   (the load did not change).  switch_v4._revert */
static void revert(V4_State *s, float f_prev)
{
    s->f = f_prev; s->f_latched = f_prev; s->latched = 1;
    s->event_probe = 0;
    s->boost = 0.0f;
    s->rec = 0;
    s->n_still = 0; s->win_n = 0; s->win_head = 0;
    s->armed = 0; s->arm_cnt = 0;
    s->qr_reverts++;
}

float V4_Step(V4_State *s, float angle_deg, float gyro_lsb, int moving)
{
    float om = gyro_lsb / LSB_DPS;
    float err, on, en;
    int   can_up, still;

    /* ================= switch_v4: upshift on large tilt ================= */
    s->om_lp += OMLP_A * (om - s->om_lp);
    if (!s->th_ref_ok) { s->th_ref = angle_deg; s->th_ref_ok = 1; }
    err = angle_deg - s->th_ref;
    on = REC_ON_STILL;
    s->n_cmd_still = moving ? 0 : s->n_cmd_still + 1;
    still = s->n_cmd_still >= STILL_N;
    s->th_pred = err + REC_S * om;
    if (s->edge_n < EDGE_GRACE_N) s->edge_n++;
    /* stop-calm gate: after a stop, wait until the car has actually settled
       before the 3 deg standing threshold applies (slope stops, hard brakes) */
    if (STOP_CALM || REPROBE_ON_STOP) {
        if (moving) {
            s->n_drive++;
            s->stop_calm = 0; s->stop_calm_n = 0;
        } else if (!s->stop_calm) {
            s->stop_calm_n = absf(s->th_pred) < ARM_DEG ? s->stop_calm_n + 1 : 0;
            if (s->stop_calm_n >= STOP_CALM_N && still) {
                s->stop_calm = 1;
                if (REPROBE_ON_STOP && s->n_drive >= REPROBE_DRIVE_N
                        && s->latched && s->f_latched < CLASS_SPLIT_F && !s->rec) {
                    rearm(s);
                    s->stop_probes++;
                }
                s->n_drive = 0;
            }
        }
        if (STOP_CALM) still = still && s->stop_calm;
    }
    /* standing threshold follows this car's own noise (armed = last tick's) */
    if (NOISE_K > 0.0f) {
        /* learn noise only from samples inside the threshold, and cap it:
           a real lean must not be learned as "noise" (runaway to 118 deg) */
        float nk, cur;
        nk = NOISE_K * sqrtf(s->noise > 0.0f ? s->noise : 0.0f);
        if (nk > NOISE_CAP) nk = NOISE_CAP;
        cur = nk > on ? nk : on;
        if (s->latched && s->armed && still && !s->rec && absf(s->th_pred) < cur)
            s->noise += (s->th_pred * s->th_pred - s->noise) * NOISE_RATE;
        nk = NOISE_K * sqrtf(s->noise > 0.0f ? s->noise : 0.0f);
        if (nk > NOISE_CAP) nk = NOISE_CAP;
        if (nk > on) on = nk;
    }
    s->on_still = on;
    if (!still) {
        float mv = REC_ON_MOVE;
        if (REC_ON_MOVE_HEAVY > 0.0f && s->latched && s->f_latched >= CLASS_SPLIT_F)
            mv = REC_ON_MOVE_HEAVY;
        if (mv > on) on = mv;
    }
    /* quick revert: QR_T after an upshift, small low-passed peak -> undo */
    if (s->qr_active) {
        float p = absf(err + REC_S * s->om_lp);
        s->qr_n++;
        if (p > s->qr_peak) s->qr_peak = p;
        if (s->qr_n >= QR_N) {
            s->qr_active = 0;
            if (QR_PEAK > 0.0f && s->qr_peak < QR_PEAK && !s->latched)
                revert(s, s->qr_prev);
        }
    }

    can_up = s->latched && s->f_latched < 0.999f;
    if (s->latched && !s->was_latched) {
        s->armed = 0; s->arm_cnt = 0;
        /* round 4b (2026-10-03 real car): relatched shortly after an upshift.
           The car is still rolling back from the push and brakes by itself --
           no key, so no command-edge grace, and the "standing" 3-8 deg
           threshold applied: the brake swing re-triggered an upshift every
           ~1.4 s, 2-3 s of chatter per push.  Treat it like a key edge and make
           "standing" wait until the car has really settled again. */
        if (RELATCH_GRACE && s->up_n < RELATCH_GRACE_WITHIN_N) {
            s->edge_n = EDGE_GRACE_N - RELATCH_GRACE_N;   /* rate term ignored for RELATCH_GRACE_N */
            s->n_cmd_still = 0;
            s->stop_calm = 0; s->stop_calm_n = 0;
            s->relatch_graces++;
        }
    }
    if (s->up_n < 1000000L) s->up_n++;
    s->was_latched = s->latched;
    if (!s->armed) {
        if (s->latched && absf(s->th_pred) < ARM_DEG) {
            if (++s->arm_cnt >= ARM_N) s->armed = 1;
        } else {
            s->arm_cnt = 0;
        }
    }
    if (!(can_up && s->armed)) on = BIG;

    if (!s->rec) {
        /* balance angle: fast & ungated while driving / just stopped,
           slow & gated once standing (so a push is not absorbed) */
        if (!still)                         s->th_ref += err * REF_RATE_FAST;
        else if (absf(err) < REF_CALM_DEG)  s->th_ref += err * REF_RATE;
        float trig = s->edge_n < EDGE_GRACE_N ? err : s->th_pred;
        if (absf(trig) <= on && absf(s->th_pred) > on) s->edge_skips++;
        if (absf(trig) > on) {
            float prev = s->f_latched;
            s->rec = 1; s->rec_entries++; s->calm = 0;
            s->up_n = 0;
            rearm(s);
            s->qr_active = 1; s->qr_n = 0; s->qr_peak = 0.0f; s->qr_prev = prev;
        }
    } else {
        if (absf(err) < REC_OFF_DEG && absf(om) < REC_OFF_DPS) {
            if (++s->calm >= REC_OFF_TICKS) s->rec = 0;
        } else {
            s->calm = 0;
            if (absf(s->th_pred) > on) rearm(s);
        }
    }

    /* ================= switch_v3: chatter, probe, latch ================= */
    s->hp += HP_A * (gyro_lsb - s->hp);
    s->bp += LP_A * ((gyro_lsb - s->hp) - s->bp);
    s->en += EN_A * (s->bp * s->bp - s->en);
    en = s->en > 0.0f ? s->en : 0.0f;
    s->osc = sqrtf(en) / LSB_DPS;

    s->n_tick++;
    if (!moving) {
        s->n_still++;
        push(s->win, V4_WIN_N, &s->win_n, &s->win_head, s->osc);
    } else {
        s->n_still = 0; s->win_n = 0; s->win_head = 0;
    }
    if (moving) {
        s->n_move++;
        push(s->mwin, V4_MWIN_N, &s->mwin_n, &s->mwin_head, s->osc);
    } else {
        s->n_move = 0; s->mwin_n = 0; s->mwin_head = 0;
    }

    /* fast light, moving or not: since the last unlatch (boot or upshift), once
       every reading in the last 0.3 s is past the split, only an empty car can
       do that -- latch light now instead of waiting for the reading to settle
       (standing) or for 1 s of driving with stale pre-upshift readings (moving).
       2026-10-03 real car: the empty car shook for seconds after an upshift. */
    if (s->latched) {
        s->n_unl = 0; s->fwin_n = 0; s->fwin_head = 0;
    } else {
        s->n_unl++;
        push(s->fwin, V4_WIN_N, &s->fwin_n, &s->fwin_head, s->osc);
    }
    if (!s->latched && FAST_LIGHT_N > 0 && absf(s->f - 1.0f) < 1e-3f
            && s->n_unl >= FAST_LIGHT_N && s->fwin_n >= V4_WIN_N
            && buf_min(s->fwin, s->fwin_n) >= OSC_SPLIT) {
        s->t_latch = s->n_tick; s->osc_at_latch = s->osc;
        s->f_latched = F_LIGHT; s->f = F_LIGHT; s->latched = 1;
        s->event_probe = 0;
        s->fast_latches++;
    }

    /* driving: only an unmistakable "light" may latch, and only at f=1 */
    if (!s->latched && absf(s->f - 1.0f) < 1e-3f
            && s->n_move >= MOVE_MIN_N && s->mwin_n >= V4_MWIN_N
            && buf_min(s->mwin, s->mwin_n) >= MOVE_SPLIT) {
        s->t_latch = s->n_tick; s->osc_at_latch = s->osc;
        s->f_latched = F_LIGHT; s->f = F_LIGHT; s->latched = 1;
        s->move_latched = 1;
    }

    if (!s->latched) {
        int ready = 0;
        if (s->n_still >= PROBE_MAX_N) {
            ready = 1;
        } else if (s->n_still >= PROBE_MIN_N && s->win_n >= V4_WIN_N) {
            float lo = buf_min(s->win, s->win_n), hi = buf_max(s->win, s->win_n);
            float mean = buf_mean(s->win, s->win_n);
            if (mean < 1e-6f) mean = 1e-6f;
            ready = (hi - lo) / mean <= CONV_TOL;
        }
        if (ready) {
            int light;
            s->t_latch = s->n_tick; s->osc_at_latch = s->osc;
            if (s->event_probe) {
                float split = OSC_SPLIT > EVENT_SPLIT ? OSC_SPLIT : EVENT_SPLIT;
                light = s->win_n > 0 && buf_min(s->win, s->win_n) >= split;
            } else {
                light = s->osc >= OSC_SPLIT;
            }
            s->f_latched = light ? F_LIGHT : F_HEAVY;
            s->event_probe = 0;
            s->f = s->f_latched;
            s->latched = 1;
        }
    } else {
        /* slow escape: sustained lean -> temporary boost that decays back */
        float e2 = absf(angle_deg - s->th_ref);
        /* round 4: not while recovering from an upshift -- th_ref is frozen
           there and the post-event swing (up to 19 deg on the real car) read as
           a sustained lean, boosting the empty car to f 0.6-1.0 for 4-5 s */
        if (STRUGGLE_IN_REC || !s->rec)
            s->struggle += STRUGGLE_LPF * (e2 - s->struggle);
        if (s->struggle > STRUGGLE_DEG) {
            s->boost += RATE_UP; if (s->boost > 1.0f) s->boost = 1.0f;
            s->escape_ticks++;
        } else {
            s->boost *= BOOST_DECAY;
            if (s->boost < 1e-4f) s->boost = 0.0f;
        }
        s->f = s->f_latched + s->boost;
        if (s->f > 1.0f) s->f = 1.0f;
        if (s->f < 0.0f) s->f = 0.0f;
    }
    if (s->rec) s->rec_ticks++;
    return s->f;
}
