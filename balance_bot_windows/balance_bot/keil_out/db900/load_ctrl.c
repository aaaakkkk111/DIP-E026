/* load_ctrl.c -- automatic load-mode switching for the Yahboom balance car.
 * Generated 2026-09-16 14:55   (motor dead band 900) by scripts/export_keil.py -- do not edit by hand.
 *
 * THIS BUILD IS FOR A MOTOR DEAD BAND OF ABOUT 900 COUNTS.
 *   Measure yours first (see the README), then pick the matching folder.
 *
 *   >>> SET  MOTOR_IGNORE_PULSE  TO  1300  IN APP/app_motor.c  <<<
 *   Line 5 of that file.  The factory value is 1300.  If the motor's real
 *   dead band exceeds the compensation, no gain set works at all: measured
 *   0 out of 6 at dead band 1700 with compensation 1300, and 5 of 6 once the
 *   compensation matched.  Changing the gains cannot fix an under-compensated
 *   dead band -- small commands produce no torque, so the loop is open there.
 *   Measured chatter at this dead band: NORMAL empty 32.8 / 2kg 1.2,  HEAVY empty 56.0 / 2kg 19.6
 *   Detection margin: 1.69x
 *
 * FIGURES BELOW MARKED [db1300] WERE MEASURED ON THE DEAD BAND 1300 BUILD
 * (Balance_Kp 38400).  They show the mechanism, not this build's numbers.
 * This build's own measured figures are the chatter and margin quoted above.
 *
 * WHY TWO GAIN SETS
 *   No single set covers 0..4 kg.  The conflict is in the D term:
 *     empty car   needs Balance_Kd <= 48,  or it chatters hard at ~31 Hz
 *     4 kg moving needs Balance_Kd >= 96,  or it falls when accelerating
 *   There is no safe value in between: from Kd 56 to 72 the empty-car
 *   chatter jumps 23.5 -> 107.3 deg/s, a 4.6x step in a 0.16 window. [db1300]
 *
 * HOW THE SWITCH DECIDES
 *   It does not estimate the payload.  Payload is essentially unobservable
 *   on this machine: mass added on top also raises the COM, and the axle
 *   recoil cancels most of the extra pitch inertia, so the measured signature
 *   spans only 1.29x over the whole 0..4 kg range. [db1300]
 *   Instead it measures a SYMPTOM -- gyro energy near 31 Hz:
 *       31 Hz chatter present = Kd is too large for the present load
 *                             = the NORMAL set is the correct one
 *   That is the decision criterion itself, not a proxy for it, which is why
 *   it works where a payload estimate does not: it only has to tell which
 *   side of the boundary we are on, not how heavy the load is.
 *
 * WHY A BAND-PASS AND NOT A HIGH-PASS
 *   Chatter has two sources with opposite causes:
 *     empty car  Kd too large -> ~31 Hz fast-mode limit cycle
 *     4 kg       Kd too small -> large low-frequency wallowing
 *   Wide-band energy merges them into one number that is NOT monotonic in
 *   load (NORMAL: 0kg 44.8 / 2kg 8.8 / 4kg 35.5, both ends high). [db1300]
 *   that statistic a 4 kg car reads as "empty", stays in NORMAL and falls --
 *   measured 0 out of 8 [db1300].  The 26.5..45.5 Hz band-pass fixes it.
 *
 * VALIDATION (MuJoCo twin, disturbance 0.4, 20 seeds at each end)
 *     empty  survived 20/20,  decided NORMAL 20/20
 *     4 kg   survived 19/20,  decided HEAVY  20/20
 *   Between 0.5 and 2 kg the decision varies seed to seed, but both sets are
 *   safe there (7..8 of 8), so the inconsistency has no consequence.
 *
 * BEFORE YOU FLASH
 *   Balance_Kp is 3.0x the factory value.  Follow the staged bring-up
 *   list in load_ctrl_README.md, and do the threshold calibration -- a
 *   1.69x margin will not survive the difference between this car and
 *   the simulation.
 */

#include <math.h>
#include "load_ctrl.h"

/* The six gains live in pid_control.c.  Referenced, never redefined. */
extern float Balance_Kp, Balance_Kd;
extern float Velocity_Kp, Velocity_Ki;
extern float Turn_Kp, Turn_Kd;

/* From AllHeader.h.  Declared here so this file does not have to drag the
 * whole project header in; the types match AllHeader.h exactly. */
extern float Gyro_Balance;
extern float Move_X, Move_Z;

/* NOTE for the 05.weight_control project: leave Balance_K / Velocity_K /
 * Turn_K at 1.0.  Both sets below already account for the payload, and
 * multiplying again scales the D term with everything else -- which is
 * exactly what makes the factory weight mode shake an empty car apart. */

/* [db1300] survival table -- the mechanism, not this build's numbers:
 *   load     NORMAL(Kd 48)        HEAVY(Kd 120)
 *   0 kg   5/5   22.5 deg/s    5/5  >120 deg/s   <- never HEAVY when empty
 *   1 kg   5/5    1.2          5/5    9.3
 *   2 kg   5/5    0.9          5/5    6.9
 *   4 kg   1/5   10.2          5/5   11.2        <- NORMAL runs out here
 *   (drive-script survivals out of 5, and standstill gyro RMS at that load)
 *
 * Velocity_Ki IS the position loop.  The firmware's "velocity loop" is
 * P(speed) + I(position):
 *     Encoder_Integral += Encoder_bias;   <- integrated counts = position
 *     velocity = -Encoder_bias*Vkp - Encoder_Integral*Vki;
 * Raising it from the factory 31 cut the standstill wander 74 -> 48 mm [db1300]
 * empty and 35 to 28 mm at 4 kg.  Do not push it further: more damping also
 * suppresses the chatter the detector needs. */
LD_Gains ld_g_normal = { 28800.0f, 48.0f, 8200.0f, 69.0f, 1400.0f, 20.0f };
LD_Gains ld_g_heavy  = { 28800.0f, 120.0f, 11600.0f, 90.0f, 1400.0f, 20.0f };

/* Not const: see the note in load_ctrl.h.  Retuning on the real car means
 * changing these two lines (or writing the struct at run time); changing
 * Balance_Kp itself has no lasting effect. */

/* Factory values, for reference:
 * static const LD_Gains G_FACTORY = { 9600.0f, 48.0f, 6200.0f, 31.0f, 1400.0f, 20.0f }; */

/* ---- detector ---------------------------------------------------------- */

#define LD_HZ           200.0f
#define LD_A_HP         0.454545f   /* high-pass corner 26.5 Hz */
#define LD_A_LP         0.588235f   /* low-pass corner  45.5 Hz */
#define LD_A_EN         0.016393f   /* energy smoothing */
#define LD_LSB_PER_DPS  16.4f

#define LD_PRIME_S      0.90f    /* settle time before deciding */
#define LD_CONFIRM_S    2.0f     /* a verdict must hold this long */
#define LD_LOCKOUT_S    3.0f     /* no deciding right after a switch */

float LD_LOADED_BELOW = 6.2f;
float LD_EMPTY_ABOVE  = 33.1f;

int   ld_heavy = 1;
int   ld_force = 0;
float ld_rms   = 0.0f;
float ld_osc   = 0.0f;
float ld_rev   = 0.0f;

static float ld_mean, ld_band, ld_energy, ld_hold, ld_lock, ld_prime;
static int   ld_rev_last;

/* Boot in HEAVY on purpose.  The risk is asymmetric:
 *     NORMAL on a 4 kg car -> falls as soon as it accelerates
 *     HEAVY  on an empty car -> chatters above 120 deg/s but does NOT fall
 *                               (20 of 20 survived) [db1300]
 * and that chatter is the loudest signal the detector has, so a detection
 * failure costs a couple of noisy seconds rather than a fall.  Booting in
 * NORMAL with the drive command gated was tried and rejected: it got both
 * safety-critical cases wrong. */
void LD_Reset(void)
{
    ld_mean = 0.0f;
    ld_band = 0.0f;
    ld_energy = 0.0f;
    ld_hold = 0.0f;
    ld_lock = 0.0f;
    ld_prime = 0.0f;
    ld_heavy = 1;
}

int LD_Update(float gyro_lsb, int moving)
{
    float dt = 1.0f / LD_HZ;
    float hp;
    int want;

    ld_mean   += LD_A_HP * (gyro_lsb - ld_mean);    /* drop below 27 Hz */
    hp         = gyro_lsb - ld_mean;
    ld_band   += LD_A_LP * (hp - ld_band);          /* drop above 45 Hz */
    ld_energy += LD_A_EN * (ld_band * ld_band - ld_energy);
    ld_rms     = (float)sqrt((double)ld_energy) / LD_LSB_PER_DPS;

    if (ld_prime < LD_PRIME_S)
    {
        ld_prime += dt;
        return ld_heavy;
    }

    /* Freeze the verdict while driving.  The statistic is only calibrated at
     * standstill; steering and acceleration push an empty car down into the
     * "loaded" band and cause spurious switches.  The payload only changes
     * when somebody stops and loads the car, so nothing is lost. */
    if (moving)
    {
        ld_hold = 0.0f;
        return ld_heavy;
    }

    if (ld_lock > 0.0f)
    {
        ld_lock -= dt;
        ld_hold = 0.0f;
        return ld_heavy;
    }

    want = ld_heavy;
    if (!ld_heavy && ld_rms < LD_LOADED_BELOW)
    {
        want = 1;
    }
    else if (ld_heavy && ld_rms > LD_EMPTY_ABOVE)
    {
        want = 0;
    }

    if (want != ld_heavy)
    {
        ld_hold += dt;
        if (ld_hold >= LD_CONFIRM_S)
        {
            ld_heavy = want;
            ld_hold  = 0.0f;
            ld_lock  = LD_LOCKOUT_S;
        }
    }
    else
    {
        ld_hold = 0.0f;
    }

    return ld_heavy;
}

/* ---- slow fore/aft oscillation ----------------------------------------
 * Same shape as the detector above, one band lower: high-pass at 0.3 Hz to
 * drop steady drift (the user is content to let the car wander), low-pass at
 * 3 Hz to drop the 31 Hz buzz, then RMS.  ld_rev counts how often the wheels
 * change direction, which separates "drifted 20 cm and stopped" from
 * "hunted back and forth 20 cm forty times" -- the two look identical in a
 * peak-to-peak number and only the second one is the complaint. */
#define LD_P_HP  0.009346f       /* 0.3 Hz */
#define LD_P_LP  0.086207f       /* 3 Hz   */
#define LD_P_EN  0.002494f       /* 2 s averaging */

void LD_Pos(float wheel_counts)
{
    static float mean, band, energy, rate, prev;
    static int   inited;
    float hp, d;

    if (!inited) { mean = wheel_counts; prev = wheel_counts; inited = 1; }

    mean   += LD_P_HP * (wheel_counts - mean);
    hp      = wheel_counts - mean;
    band   += LD_P_LP * (hp - band);
    energy += LD_P_EN * (band * band - energy);
    ld_osc  = (float)sqrt((double)energy);

    /* A reversal is a sign change of the per-tick increment.  The 1-count
     * dead zone keeps encoder quantisation from counting as motion. */
    d = wheel_counts - prev;
    if ((d > 1.0f && ld_rev_last < 0) || (d < -1.0f && ld_rev_last > 0))
    {
        rate += LD_P_EN * (LD_HZ - rate);
    }
    else
    {
        rate += LD_P_EN * (0.0f - rate);
    }
    if (d >  1.0f) ld_rev_last =  1;
    if (d < -1.0f) ld_rev_last = -1;
    prev   = wheel_counts;
    ld_rev = rate;
}

/* Call once per control tick, in app_control.c's EXTI15_10_IRQHandler,
 * after Get_Angle() and before Balance_PD(). */
void LD_Tick(void)
{
    const LD_Gains *g;

    LD_Update(Gyro_Balance,
              (Move_X != 0.0f) || (Move_Z != 0.0f));

    /* ld_force overrides the detector here rather than after the fact.
     * Patching Balance_Kd downstream of this function -- which is what the
     * first version of debug_uart did -- leaves Velocity_Kp and Velocity_Ki
     * at the OTHER set's values, producing a mixture that was never tested. */
    if (ld_force == 1)      g = &ld_g_normal;
    else if (ld_force == 2) g = &ld_g_heavy;
    else                    g = ld_heavy ? &ld_g_heavy : &ld_g_normal;

    Balance_Kp  = g->kp;
    Balance_Kd  = g->kd;
    Velocity_Kp = g->vkp;
    Velocity_Ki = g->vki;
    Turn_Kp     = g->tkp;
    Turn_Kd     = g->tkd;
}
