/* load_adapt.c -- see load_adapt.h */

#include "load_adapt.h"
#include "AllHeader.h"
#include <math.h>

extern float Balance_Kp, Balance_Kd;
extern float Velocity_Kp, Velocity_Ki;
extern float Turn_Kp, Turn_Kd;
extern float Encoder_Integral;
extern int   motor_ignore_pulse;

#define LA_FS 200.0f

/* Ladder endpoints: stock Normal <-> stock Weight_M with its load factors
 * already multiplied in.  pid_control.c applies Balance_K / Velocity_K /
 * Turn_K only when mode == Weight_M, so in mode 26 they do not apply and
 * these values are used as written.  Never fold this into Weight_M. */
static const float LA_KP [LA_LEVELS] = {  9600.0f, 12000.0f, 14400.0f, 16800.0f, 19200.0f };
static const float LA_KD [LA_LEVELS] = {    48.0f,    73.5f,    99.0f,   124.5f,   150.0f };
static const float LA_VKP[LA_LEVELS] = {  6200.0f,  7012.0f,  7825.0f,  8637.0f,  9450.0f };
static const float LA_VKI[LA_LEVELS] = {    31.0f,    35.0f,    39.0f,    43.0f,    47.0f };
static const float LA_TKP[LA_LEVELS] = {  1700.0f,  1625.0f,  1550.0f,  1475.0f,  1400.0f };

/* ---- what decides the level -------------------------------------------
 * Earlier versions picked the level from SYMPTOMS: chatter meant too much
 * gain, wallowing meant too little.  That kept deadlocking, because BOTH
 * failures produce large slow swings and only one of them produces chatter,
 * so the two rules fought over the same measurement and left bands where
 * neither fired.  Patching that with a plain timer was worse -- a timer
 * decides with no information at all.
 *
 * This version measures the payload directly instead.
 *
 * The dead band compensation keeps the car in a permanent small limit cycle,
 * so the plant is ALWAYS excited; nothing has to be driven.  Inside that
 * excitation, angular acceleration follows applied torque as
 *
 *     alpha = tau / C        C = pitch inertia about the axle
 *
 * and C is exactly what a payload changes: 0.0023 empty, 0.0079 with 0.5 kg
 * on the top plate, 0.024 at 2 kg.  A running regression of band-passed
 * angular acceleration on band-passed drive therefore tracks the load, and
 * says nothing about the gains.
 *
 * Validated offline on shift_test.txt (2026-09-19), one recording:
 *     empty,  level 0        k = 1.97
 *     loaded, level 1-2      k = 1.05
 *     empty again, level 0   k = 2.36
 * about 2x separation, with 20 percent spread between the two empty samples. */
#define LA_A_EHP  0.158500f    /*  6 Hz */
#define LA_A_ELP  0.385800f    /* 20 Hz */
#define LA_A_FRG  0.999000f    /* regression forgetting, about 5 s */

/* Anchors in the units of la_k.  MEASURED ON THIS CAR; re-measure if the car
 * changes.  Run empty and read k, then load it and read k again.  Both are
 * tunable from the console: j = empty anchor, u = loaded anchor. */
float LA_K_EMPTY = 2.10f;
float LA_K_FULL  = 0.60f;

/* Chatter no longer chooses the level, but it is still the fastest evidence
 * that the present gains are too high, so it stays as an override. */
#define LA_A_HP      0.200849f  /*  8 Hz  */
#define LA_A_LP      0.334511f  /* 16 Hz  */
#define LA_A_EN      0.016393f  /* energy smoothing, about 0.3 s */
#define LA_A_LEAN_HP 0.009337f  /* 0.3 Hz */
#define LA_A_LEAN_LP 0.059120f  /* 2 Hz   */
#define LA_LSB       16.4f

float LA_OSC_HI    = 25.0f;
float LA_LEAN_HI   = 1.0f;
float LA_OSC_LO    = 10.0f;
float LA_ANG_PANIC = 4.0f;
float LA_DWELL_S   = 1.0f;
float LA_LOCK_S    = 2.0f;

int   LA_BOOT_LEVEL = 2;
int   la_level = 2;
int   la_force = -1;
int   la_panic = 0;
float la_osc   = 0.0f;
float la_lean  = 0.0f;
float la_sat   = 0.0f;
float la_k     = 0.0f;

static float mean_hp, band, energy;
static float lean_mean, lean_band, lean_energy;
static float e_umean, e_uband, e_amean, e_aband, e_suu, e_sua, gyro_prev;
static float hold_dn, hold_up, lock, no_down;
static int   la_applied = -1;

static void la_apply(void)
{
    int i = la_level;
    if (i < 0) i = 0;
    if (i >= LA_LEVELS) i = LA_LEVELS - 1;

    /* Bumpless transfer.  Velocity_PI() outputs -Encoder_Integral*Vki/100, so
     * changing Vki with the integral untouched steps the motor command by up
     * to 1280 PWM, right when the car is already in trouble. */
    if (la_applied >= 0 && i != la_applied && LA_VKI[i] > 0.0f)
        Encoder_Integral *= LA_VKI[la_applied] / LA_VKI[i];
    la_applied = i;

    la_level    = i;
    Balance_Kp  = LA_KP [i];
    Balance_Kd  = LA_KD [i];
    Velocity_Kp = LA_VKP[i];
    Velocity_Ki = LA_VKI[i];
    Turn_Kp     = LA_TKP[i];
    Turn_Kd     = 20.0f;
}

void LA_Reset(void)
{
    mean_hp = 0.0f; band = 0.0f; energy = 0.0f;
    lean_mean = 0.0f; lean_band = 0.0f; lean_energy = 0.0f;
    e_umean = 0.0f; e_uband = 0.0f; e_amean = 0.0f; e_aband = 0.0f;
    e_suu = 0.0f; e_sua = 0.0f; gyro_prev = 0.0f;
    hold_dn = 0.0f; hold_up = 0.0f; lock = 0.0f; no_down = 0.0f;
    la_osc = 0.0f; la_lean = 0.0f; la_sat = 0.0f; la_k = 0.0f;
    la_level = LA_BOOT_LEVEL;
    la_applied = -1;
    la_apply();
}

void LA_Tick(int ml, int mr, int el, int er)
{
    float dt = 1.0f / LA_FS;
    float hp, dev, sat, u, a, drive;

    (void)el; (void)er;

    /* ---- payload measurement: alpha against drive, inside the limit cycle -- */
    drive = (float)((ml + mr) / 2);
    u = (drive > 0.0f) ? (drive - (float)motor_ignore_pulse)
                       : (drive + (float)motor_ignore_pulse);
    if ((drive > 0.0f && u < 0.0f) || (drive < 0.0f && u > 0.0f)) u = 0.0f;
    a = (Gyro_Balance - gyro_prev) * LA_FS;
    gyro_prev = Gyro_Balance;

    e_umean += LA_A_EHP * (u - e_umean);
    e_uband += LA_A_ELP * ((u - e_umean) - e_uband);
    e_amean += LA_A_EHP * (a - e_amean);
    e_aband += LA_A_ELP * ((a - e_amean) - e_aband);

    e_suu = LA_A_FRG * e_suu + e_uband * e_uband;
    e_sua = LA_A_FRG * e_sua + e_uband * e_aband;
    if (e_suu > 1.0f)
    {
        la_k = e_sua / e_suu;
        if (la_k < 0.0f) la_k = -la_k;     /* sign is a wiring convention */
    }

    /* ---- chatter, kept as the fast "gains are too high" override ---- */
    mean_hp += LA_A_HP * (Gyro_Balance - mean_hp);
    hp       = Gyro_Balance - mean_hp;
    band    += LA_A_LP * (hp - band);
    energy  += LA_A_EN * (band * band - energy);
    la_osc   = (float)sqrt((double)energy) / LA_LSB;

    lean_mean   += LA_A_LEAN_HP * (Angle_Balance - lean_mean);
    lean_band   += LA_A_LEAN_LP * ((Angle_Balance - lean_mean) - lean_band);
    lean_energy += LA_A_EN * (lean_band * lean_band - lean_energy);
    la_lean      = (float)sqrt((double)lean_energy);

    sat = (ml >= 2800 || ml <= -2800 || mr >= 2800 || mr <= -2800) ? 1.0f : 0.0f;
    la_sat += LA_A_EN * (sat - la_sat);

    if (la_force >= 0) { la_level = la_force; la_apply(); return; }

    /* ---- express lane up: a QUIET car that is wallowing badly ---- */
    dev = (lean_band < 0.0f) ? -lean_band : lean_band;
    if (dev > LA_ANG_PANIC && la_osc < LA_OSC_HI && la_level < LA_LEVELS - 1)
    {
        la_level = LA_LEVELS - 1;
        energy   = 0.0f;
        no_down  = 3.0f;
        la_panic++;
        hold_dn = 0.0f; hold_up = 0.0f; lock = 0.0f;
        la_apply();
        return;
    }
    if (no_down > 0.0f) no_down -= dt;

    if (lock > 0.0f)
    {
        lock -= dt; hold_dn = 0.0f; hold_up = 0.0f;
        la_apply();
        return;
    }

    /* ---- override: chattering means the gains are too high, right now ---- */
    if (la_osc > LA_OSC_HI && la_level > 0 && no_down <= 0.0f)
    {
        int   jump  = 1;
        float dwell = LA_DWELL_S;
        if (la_osc > 2.0f * LA_OSC_HI) { jump = 2; dwell = LA_DWELL_S * 0.3f; }

        hold_up = 0.0f;
        hold_dn += dt;
        if (hold_dn >= dwell)
        {
            la_level -= jump;
            if (la_level < 0) la_level = 0;
            hold_dn = 0.0f;
            lock = LA_LOCK_S * 0.5f;
            energy = 0.0f;
        }
        la_apply();
        return;
    }
    hold_dn = 0.0f;

    /* ---- up: wallowing WHILE QUIET means the gains are too low ----
     * la_k is still computed and logged, but it does NOT drive the level.
     * On the car it swung between 3.6 and 27.4 on an unchanging empty
     * chassis and overlapped the loaded range completely (2026-09-19), so
     * it decided nothing reliable.  The offline check that made it look
     * usable had used an FFT brick-wall band-pass; two cascaded one-pole
     * IIRs, which is what fits in the tick, leave far too much of the
     * differentiated gyro noise in the regression.  The chatter measurement
     * below, by contrast, has never overlapped: 4.5-7.4 empty at level 0
     * against 53.8-58.3 empty at levels 1-2. */
    if (la_lean > LA_LEAN_HI && la_osc < LA_OSC_LO && la_level < LA_LEVELS - 1)
    {
        hold_up += dt;
        if (hold_up >= LA_DWELL_S * 0.6f)
        {
            la_level++;
            hold_up = 0.0f;
            lock = LA_LOCK_S;
        }
    }
    else { hold_up = 0.0f; }

    la_apply();
}
