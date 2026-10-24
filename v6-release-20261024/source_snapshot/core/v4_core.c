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
/* ---- v5 (2026-10-21): chatter downshift ----
 * The latched gear never went DOWN: a light/medium load judged "heavy" (0.40) -- or
 * any gear that is too stiff for the load -- kept the car chattering, and the chatter's
 * rate term (th_pred = err + 0.1*omega) even triggered upshifts to f=1: positive
 * feedback.  8-16 Hz chatter is the direct sign of a gear that is too stiff (twin, fixed
 * gears: calm 0.3 deg/s, too stiff 20-34; real car: calm 4.5-7.4, too stiff 54-58).
 *   - latched and chatter >= DOWN_SPLIT for DOWN_N ticks -> f_latched -= DOWN_STEP,
 *     then wait DOWN_WAIT for the reading to settle; down to 0 at most;
 *   - while chattering, a tilt below UP_BLOCK_DEG does not upshift. */
#ifndef CHATTER_DOWN
#define CHATTER_DOWN  1
#endif
#define DOWN_SPLIT    15.0f
#define DOWN_N        30                /* 0.15 s -- 10-24 user: small high-frequency chatter must go FAST (was 0.3 s) */
#define DOWN_WAIT     40                /* 0.2 s between steps (was 0.5 s) */
#define DOWN_STEP     0.10f
#define UP_BLOCK_DEG  8.0f

/* v5: three-level probe (see v4_core.h).  The old single split (OSC_SPLIT 23) put a
 * 0.5 kg load (23.1 at f=1 in the twin) on the light gear at start-off: 2/3 falls. */
float v4_light_split = 25.0f;   /* twin sweep 23/25/27/30 -> 25-27 best (split_sweep.log) */
float v4_mid_split   = 20.0f;

/* ---- v5.1 (2026-10-21): small loads oscillated, grew, calmed, grew again -- and fell
 * when it grew too far.  The loop: any event -> f=1 and a 1.2-3 s probe on the stiffest
 * gear (a small load diverges there); the 'slow escape' read the oscillation's |error| as
 * a lean and pushed f up while the chatter downshift pushed it down; the 8-16 Hz meter
 * does not see the visible 2-5 Hz sway.  Each fix has a bit in v4_opt:
 *   1  graded upshift: an event EX deg over the threshold adds 0.2 per 5 deg as a
 *      temporary boost; only EX >= BIG_EV re-probes at f=1
 *   2  probe abort: oscillating for PA_N while probing at f=1 -> latch soft at once
 *   4  slow escape on the signed, low-passed lean (an oscillation averages out); held
 *      while oscillating
 *   8  wide-band oscillation: rate of sign changes of (rate - 1 Hz low-pass) >= W_HZ and
 *      mean |.| >= v4_wamp -> oscillating (as well as the 8-16 Hz meter)
 *   16 ceiling: after a chatter downshift from f, f stays <= f - 0.1 for CEIL_N, unless
 *      |tilt| > CEIL_ESC deg */
int   v4_opt  = 50;   /* v5.2 10-24: bits 2+16+32 (32 = low-frequency sway upshift).  bit 1 (graded upshift) off: twin 10-21, an empty car given 2-4 kg with a light
                         nudge never re-probed and fell (9/60 swaps); bit 2 already prevents the
                         divergence at f=1 that bit 1 was meant for (v51_score_opt30.log) */
float v4_wamp = 40.0f;   /* twin 10-21: 15 and 25 took a loaded car's start-off sway for
                            oscillation (1 kg / 2 kg fell); 40+ = never in the benchmark */
#define BIG_EV     15.0f
#define EV_STEP    0.20f
#define EV_DEG     5.0f
#define PA_N       40               /* 0.2 s */
#define GLP_A      0.030928f        /* 1 Hz low-pass at 200 Hz */
#define CR_A       0.01f            /* ~0.5 s averaging */
#define W_HZ       2.0f
#define W_HYST     2.0f             /* deg/s, sign-change hysteresis */
#define CEIL_N     2000             /* 10 s */
#define CEIL_ESC   12.0f

/* ---- v5.2 (2026-10-24): shift by FREQUENCY ----
 * Real car with a load: the body sways visibly back and forth a few times a second, the
 * swing grows, the car falls -- on the gear v5/v5.1 picked.  Twin, load CoM 0.15-0.25 m
 * above the axle (a real load sits ON the top plate, its CoM above it): f 0-0.4 sway and
 * fall, f 0.7-1 hold (sway_probe.log).  v5.1 bit 8 took that sway for 'too stiff' and
 * shifted DOWN; bit 4 averaged it away.  Rule now (user, 10-24):
 *   low-frequency sway (0.5-6 Hz tilt) above v4_swup and not dying out -> up 0.1, kept
 *   (latched, no decay), up to 1;
 *   8-16 Hz chatter -> down (CHATTER_DOWN), as before;
 *   calm -> no change;
 *   no conflict: no sway upshift while chattering; no chatter downshift while swaying
 *   hard (>= 2 x v4_swup); after a chatter downshift the ceiling holds unless the sway
 *   is >= 2 x v4_swup.
 * The real car's own 4.3 Hz idle limit cycle is ~0.67 deg amplitude (0.47 RMS): below
 * the 1.5 deg RMS default. */
float v4_swup = 1.5f;
#define SW_HP_A    0.015585f        /* 0.5 Hz high-pass at 200 Hz */
#define SW_LP_A    0.171865f        /* 6 Hz low-pass */
#define SW_EN_A    0.01f            /* ~0.5 s energy */
#define SU_N       20               /* 0.1 s above the threshold (10-24: 2 kg at 0.20 m grew 0.3->4 deg in 1.4 s) */
#define SU_WAIT    60               /* 0.3 s between steps */
#define PL_N       600              /* 3 s after a probe latch: a growing sway upshifts even while driving */
#define SU_STEP    0.10f
#define ENF_A      0.05f            /* fast chatter energy, ~0.1 s (slow one EN_A ~0.3 s) */
#define SW_HYST    0.3f             /* deg, reversal hysteresis of the sway */
#define SW_MIN_HZ  1.0f             /* a real sway reverses at least ~once a second */
/* v5.2b (10-24 twin, 2 kg at 0.20 m): the sway grew from 0.3 to 3 deg over ~4 s on f 0.4-0.5;
   waiting for 1.5 deg RMS was too late -- the tilt event re-probed at f=1, the probe said
   'heavy' (0.4) and the cycle repeated every ~5 s.  Now also: a GROWING sway from
   SW_GROW_K x v4_swup up; and an event that hits a car that was already swaying puts the
   floor above the gear that failed. */
#define SW_GROW_K  0.5f             /* growing sway counts from 0.5 x v4_swup (0.75 deg RMS; real idle
                                       4.3 Hz limit cycle 0.47) */
/* 10-24 real car, EMPTY: chatter would not go away, f hung at 0.1 or chattered at 0.0.  The real
   chatter (~54 deg/s at ~14 Hz = ~0.6 deg) leaked through the one-pole 6 Hz low-pass into the sway
   meter, on top of the idle limit cycle -> 'growing sway' -> up 0.1 -> chatter -> down -> up ...
   (the twin under-reads chatter ~30x, so it never showed).  Now: two-pole low-pass; sway upshift
   only with NO chatter on the fast meter and not within SU_AFTER_DN of a chatter downshift; the
   ceiling after a chatter downshift = the gear it went down to. */
#define SU_AFTER_DN 200             /* 1 s: no sway upshift after a chatter downshift */
/* 10-24 real car: load taken off -> gear stuck at 0.2, chattering.  Taking the load off is an
   event: rec=1, re-probe, latched 0.2.  rec ends only at |tilt - th_ref| < 3 deg and |rate| < 60
   deg/s -- but th_ref is frozen in rec while the balance angle moved with the load, and the
   chatter itself keeps the raw rate > 60.  rec never ended, and the chatter downshift was gated
   by !rec.  Now: the downshift runs in rec too; the exit uses the 3 Hz low-passed rate; a rec
   older than REC_STALE_N with the body no longer swinging ends and re-centres th_ref. */
#define REC_STALE_N 600             /* 3 s */
/* 10-24 user: small high-frequency chatter must go too.  Fast meter >= v4_dnsmall (between the
   real car's calm 4.5-7.4 at f=0 and the 15 split) for SM_N -> down 0.1 at any gear.  The sway
   upshift needs the fast meter below the same v4_dnsmall, so the two never fight. */
float v4_dnsmall = 7.5f;
#define SM_N       60               /* 0.3 s */
#define LOW_F      0.15f            /* 10-24 real car: f hung at 0.1 with chatter below the 15 split */
#define LOW_N      100              /* 0.5 s of any chatter (>= half the split) at f <= LOW_F -> 0 */
#define SW_GROW_R  1.2f             /* sw >= 1.2 x sw 0.5 s ago */
#define SW_FAIL_UP 0.30f            /* floor = failed gear + 0.3 */
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
    s->dn_hi = 0; s->dn_wait = 0; s->downs = 0; s->up_blocks = 0;
    s->osc_on = 0; s->pa_n = 0; s->ceil_n = 0; s->xsign = 0;
    s->glp = 0.0f; s->cr = 0.0f; s->am = 0.0f; s->sl = 0.0f; s->ceil = 1.0f;
    s->small_ups = 0; s->probe_aborts = 0;
    s->swh = 0.0f; s->swl = 0.0f; s->swl2 = 0.0f; s->swe = 0.0f; s->sw = 0.0f; s->sw_old = 0.0f;
    s->su_n = 0; s->su_wait = 0; s->sw_t = 0; s->sway_ups = 0;
    s->enf = 0.0f; s->oscf = 0.0f;
    s->swcr = 0.0f; s->floor_f = 0.0f; s->swsg = 0; s->sway_fails = 0; s->pl_n = 0; s->low_n = 0; s->rec_n = 0; s->sm_n = 0;
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
    s->ceil_n = 0; s->ceil = 1.0f;   /* v5.2: a new probe = maybe a new load; the old chatter ceiling (10-24 twin:
                                       1 kg -> 3 kg stayed capped at 0.1 and swayed for 8 s) no longer applies */
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
    int   can_up, still, swaying;

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
        if (absf(trig) > on && CHATTER_DOWN && s->osc_on && absf(err) < UP_BLOCK_DEG) {
            s->up_blocks++;                 /* chatter, not a push: no upshift */
        } else if (absf(trig) > on && (v4_opt & 1) && absf(trig) - on < BIG_EV) {
            /* v5.1 graded upshift: a temporary boost, no re-probe; re-arm needed before the next */
            float ex = absf(trig) - on, st = EV_STEP;
            while (ex > EV_DEG) { st += EV_STEP; ex -= EV_DEG; }
            s->boost += st; if (s->boost > 1.0f) s->boost = 1.0f;
            s->armed = 0; s->arm_cnt = 0;
            s->small_ups++;
        } else if (absf(trig) > on) {
            float prev = s->f_latched;
            if ((v4_opt & 32) && s->sw_old >= SW_GROW_K * v4_swup && s->swcr * V4_FS * 0.5f >= SW_MIN_HZ
                    && s->oscf < v4_dnsmall) {
                float fl = prev + SW_FAIL_UP;   /* swaying before the event: this gear was too soft */
                if (fl > 1.0f) fl = 1.0f;
                if (fl > s->floor_f) s->floor_f = fl;
                s->sway_fails++;
            }
            s->rec = 1; s->rec_entries++; s->calm = 0; s->rec_n = 0;
            s->up_n = 0;
            rearm(s);
            s->qr_active = 1; s->qr_n = 0; s->qr_peak = 0.0f; s->qr_prev = prev;
        }
    } else {
        s->rec_n++;
        if (absf(err) < REC_OFF_DEG && absf(s->om_lp) < REC_OFF_DPS) {
            if (++s->calm >= REC_OFF_TICKS) s->rec = 0;
        } else if (s->rec_n >= REC_STALE_N && absf(s->om_lp) < REC_OFF_DPS && s->sw < v4_swup) {
            s->rec = 0; s->th_ref = angle_deg;   /* stale: the balance angle moved (load changed) */
        } else {
            s->calm = 0;
            if (absf(s->th_pred) > on && !(CHATTER_DOWN && s->osc_on && absf(err) < UP_BLOCK_DEG))
                rearm(s);
        }
    }

    /* ================= switch_v3: chatter, probe, latch ================= */
    s->hp += HP_A * (gyro_lsb - s->hp);
    s->bp += LP_A * ((gyro_lsb - s->hp) - s->bp);
    s->en += EN_A * (s->bp * s->bp - s->en);
    en = s->en > 0.0f ? s->en : 0.0f;
    s->osc = sqrtf(en) / LSB_DPS;
    {   /* v5.1 wide-band oscillation: sign changes of (rate - 1 Hz low-pass) */
        float x;
        int sg, c = 0;
        s->glp += GLP_A * (om - s->glp);
        x = om - s->glp;
        sg = x > W_HYST ? 1 : (x < -W_HYST ? -1 : s->xsign);
        if (sg != 0 && s->xsign != 0 && sg != s->xsign) c = 1;
        s->xsign = sg;
        s->cr += CR_A * ((float)c - s->cr);                /* sign changes per tick */
        s->am += CR_A * (absf(x) - s->am);
        /* v5.2 sway meter: 0.5-6 Hz band of the tilt, RMS over ~0.5 s; sw_old = 0.5 s ago */
        s->swh += SW_HP_A * (angle_deg - s->swh);
        s->swl2 += SW_LP_A * ((angle_deg - s->swh) - s->swl2);   /* two-pole 6 Hz low-pass */
        s->swl += SW_LP_A * (s->swl2 - s->swl);
        s->swe += SW_EN_A * (s->swl * s->swl - s->swe);
        s->sw = sqrtf(s->swe > 0.0f ? s->swe : 0.0f);
        if (++s->sw_t >= 100) { s->sw_t = 0; s->sw_old = s->sw; }
        {   /* sway reversals per tick (EMA ~0.5 s): a push or a brake reverses once, a sway keeps doing it */
            int g = s->swl > SW_HYST ? 1 : (s->swl < -SW_HYST ? -1 : s->swsg), c2 = 0;
            if (g != 0 && s->swsg != 0 && g != s->swsg) c2 = 1;
            s->swsg = g;
            s->swcr += CR_A * ((float)c2 - s->swcr);
        }
        s->enf += ENF_A * (s->bp * s->bp - s->enf);
        s->oscf = sqrtf(s->enf > 0.0f ? s->enf : 0.0f) / LSB_DPS;
        s->osc_on = s->osc >= DOWN_SPLIT
                 || ((v4_opt & 8) && s->cr * V4_FS * 0.5f >= W_HZ && s->am >= v4_wamp);
    }

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
    /* v5.1 probe abort: oscillating on the stiffest gear while probing -> latch soft now */
    /* 8-16 Hz chatter only: a loaded car's start-off sway (wide band) must not end the probe */
    /* v5.2: a big low-frequency sway leaks into the 8-16 Hz meter (twin: sway 10 deg RMS read 15);
       it is not chatter -- no probe abort / light latch while swaying */
    swaying = (v4_opt & 32) && s->sw >= v4_swup;
    s->pa_n = (!s->latched && (v4_opt & 2) && !swaying && s->osc >= DOWN_SPLIT && absf(s->f - 1.0f) < 1e-3f) ? s->pa_n + 1 : 0;
    if (s->pa_n >= PA_N && s->n_unl >= PA_N) {
        s->t_latch = s->n_tick; s->osc_at_latch = s->osc;
        s->f_latched = s->osc >= v4_light_split ? F_LIGHT : V4_F_MID;
        if ((v4_opt & 32) && s->f_latched < s->floor_f) s->f_latched = s->floor_f;
        s->f = s->f_latched; s->latched = 1; s->event_probe = 0; s->pa_n = 0; s->dn_wait = DOWN_WAIT;
        if (s->f_latched > F_LIGHT) s->pl_n = PL_N;
        s->probe_aborts++;
    }
    if (!s->latched && FAST_LIGHT_N > 0 && !swaying && absf(s->f - 1.0f) < 1e-3f
            && s->n_unl >= FAST_LIGHT_N && s->fwin_n >= V4_WIN_N
            && buf_min(s->fwin, s->fwin_n) >= v4_light_split) {
        s->t_latch = s->n_tick; s->osc_at_latch = s->osc;
        s->f_latched = F_LIGHT; s->f = F_LIGHT; s->latched = 1;
        s->event_probe = 0;
        s->fast_latches++;
    }

    /* driving: only an unmistakable "light" may latch, and only at f=1 */
    if (!s->latched && !swaying && absf(s->f - 1.0f) < 1e-3f
            && s->n_move >= MOVE_MIN_N && s->mwin_n >= V4_MWIN_N
            && buf_min(s->mwin, s->mwin_n) >= (MOVE_SPLIT > v4_light_split ? MOVE_SPLIT : v4_light_split)) {
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
            float rd, split = v4_light_split > EVENT_SPLIT ? v4_light_split : EVENT_SPLIT;
            s->t_latch = s->n_tick; s->osc_at_latch = s->osc;
            if (s->event_probe) {                 /* after an event: the whole window must agree */
                rd = s->win_n > 0 ? buf_min(s->win, s->win_n) : 0.0f;
            } else {
                rd = s->osc; split = v4_light_split;
            }
            s->f_latched = rd >= split ? F_LIGHT : (rd >= v4_mid_split ? V4_F_MID : F_HEAVY);
            if (v4_opt & 32) {                 /* v5.2: keep the sway-upshifted gear unless judged empty */
                if (s->f_latched == F_LIGHT) s->floor_f = 0.0f;
                else if (s->f_latched < s->floor_f) s->f_latched = s->floor_f;
            }
            s->event_probe = 0;
            s->f = s->f_latched;
            s->latched = 1;
            s->dn_wait = DOWN_WAIT;
            if (s->f_latched > F_LIGHT) s->pl_n = PL_N;
        }
    } else {
        /* v5: chatter downshift (see CHATTER_DOWN) */
        if (CHATTER_DOWN) {                  /* 10-24: in rec too (chatter is chatter) */
            /* fast meter: react in ~0.15 s and stop stepping as soon as the chatter dies */
            s->dn_hi = ((s->oscf >= DOWN_SPLIT || s->osc_on) && !((v4_opt & 32) && s->sw >= 2.0f * v4_swup)) ? s->dn_hi + 1 : 0;
            if (s->oscf < v4_dnsmall) s->dn_hi = 0;
            s->sm_n = (s->oscf >= v4_dnsmall && !((v4_opt & 32) && s->sw >= 2.0f * v4_swup)) ? s->sm_n + 1 : 0;
            if (s->dn_wait > 0) s->dn_wait--;
            else if (s->dn_hi < DOWN_N && s->sm_n >= SM_N && s->f_latched > 0.0f) {
                s->f_latched -= DOWN_STEP;          /* small chatter: 0.1 per step */
                if (s->f_latched < 0.0f) s->f_latched = 0.0f;
                if (v4_opt & 16) { s->ceil = s->f_latched; s->ceil_n = CEIL_N; }
                s->su_wait = SU_AFTER_DN; s->su_n = 0;
                if (s->floor_f > s->f_latched) s->floor_f = s->f_latched;
                s->boost = 0.0f;
                s->dn_wait = DOWN_WAIT; s->sm_n = 0; s->downs++;
            }
            else if (s->dn_hi >= DOWN_N && (s->f_latched > 0.0f || s->boost > 0.0f)) {
                /* strong chatter steps further: 0.1 / 0.2 from 1.5x / 0.3 from 2x the split */
                s->f_latched -= s->oscf >= 2.0f * DOWN_SPLIT ? 3.0f * DOWN_STEP
                              : (s->oscf >= 1.5f * DOWN_SPLIT ? 2.0f * DOWN_STEP : DOWN_STEP);
                if (s->f_latched < 0.0f) s->f_latched = 0.0f;
                if (v4_opt & 16) { s->ceil = s->f_latched; s->ceil_n = CEIL_N; }   /* ceiling = the new gear */
                s->su_wait = SU_AFTER_DN; s->su_n = 0;
                if (s->floor_f > s->f_latched - DOWN_STEP) s->floor_f = s->f_latched - DOWN_STEP;
                if (s->floor_f < 0.0f) s->floor_f = 0.0f;
                s->boost = 0.0f;
                s->dn_wait = DOWN_WAIT; s->dn_hi = 0; s->downs++;
            }
        }
        /* v5.2 bottom sweep: near the softest gear, any lasting chatter -> straight to 0 */
        if (CHATTER_DOWN && s->f_latched > 0.0f && s->f_latched <= LOW_F) {
            s->low_n = (s->oscf >= v4_dnsmall && !((v4_opt & 32) && s->sw >= 2.0f * v4_swup)) ? s->low_n + 1 : 0;
            if (s->low_n >= LOW_N) {
                s->f_latched = 0.0f; s->boost = 0.0f; s->floor_f = 0.0f;
                if (v4_opt & 16) { s->ceil = 0.0f; s->ceil_n = CEIL_N; }
                s->su_wait = SU_AFTER_DN; s->su_n = 0; s->low_n = 0; s->downs++;
            }
        } else {
            s->low_n = 0;
        }
        /* v5.2 sway upshift (kept, no decay) */
        if (v4_opt & 32) {                     /* also during v4's event recovery (rec) */
            int up;
            if (s->pl_n > 0) s->pl_n--;
            up = (s->n_cmd_still >= STILL_N || s->pl_n > 0     /* standing, or just after a probe latch */
                      || (s->sw >= v4_swup && s->sw >= s->sw_old))  /* or driving with a big, growing sway */
                     && s->swcr * V4_FS * 0.5f >= SW_MIN_HZ         /* reversing: a sway, not a lean */
                     && ((s->sw >= v4_swup && s->sw >= 0.9f * s->sw_old)   /* swaying, not dying out */
                         || (s->sw >= SW_GROW_K * v4_swup && s->sw >= SW_GROW_R * s->sw_old))   /* small but growing */
                     && s->osc < DOWN_SPLIT && s->oscf < v4_dnsmall   /* no chatter at all */
                     && s->f_latched < 1.0f;
            if (up && s->ceil_n > 0 && s->f_latched + 0.5f * SU_STEP > s->ceil && s->sw < 2.0f * v4_swup)
                up = 0;                            /* respect a recent chatter downshift */
            s->su_n = up ? s->su_n + 1 : 0;
            if (s->su_wait > 0) s->su_wait--;
            else if (s->su_n >= SU_N) {
                s->f_latched += s->sw >= 2.0f * v4_swup ? 3.0f * SU_STEP       /* step by sway size: 0.1/0.2/0.3 */
                              : (s->sw >= v4_swup ? 2.0f * SU_STEP : SU_STEP);
                if (s->f_latched > 1.0f) s->f_latched = 1.0f;
                if (s->ceil_n > 0 && s->f_latched > s->ceil) s->ceil = s->f_latched;
                s->su_wait = SU_WAIT; s->su_n = 0; s->sway_ups++;
                s->floor_f = s->f_latched;         /* remembered across re-probes */
            }
        }
        /* slow escape: sustained lean -> temporary boost that decays back */
        float e2 = absf(angle_deg - s->th_ref);
        if (v4_opt & 4) {                  /* v5.1: signed low-pass; held while oscillating */
            if (!s->osc_on && !s->rec) s->sl += STRUGGLE_LPF * ((angle_deg - s->th_ref) - s->sl);
            e2 = absf(s->sl);
        }
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
        if (s->ceil_n > 0) {               /* v5.1 ceiling after a chatter downshift */
            s->ceil_n--;
            if (s->f > s->ceil && absf(err) < CEIL_ESC && !((v4_opt & 32) && s->sw >= 2.0f * v4_swup))
                s->f = s->ceil;
        }
        if (s->f > 1.0f) s->f = 1.0f;
        if (s->f < 0.0f) s->f = 0.0f;
    }
    if (s->rec) s->rec_ticks++;
    return s->f;
}
