/* v4_adapt.c -- mode 27 glue: firmware globals in, six PID gains out.
 *
 * The decision logic lives in v4_core.c so it can be checked on the host
 * against the Python version tick for tick.  This file only:
 *   1. works out whether the speed command is non-zero, with exactly the rules
 *      Velocity_PI() uses to build Movement (pid_control.c:98-105);
 *   2. writes the gains, with the same bumpless transfer as la_apply():
 *      Velocity_PI() outputs -Encoder_Integral*Velocity_Ki/100, so changing
 *      Ki with the integral untouched steps the motor command by up to ~1280
 *      PWM at the moment the car is already in trouble.
 *
 * Called BEFORE Balance_PD() (like LD_Tick for modes 22-25), so a decision
 * taken this tick is already in this tick's PWM.  mode 26's LA_Tick runs
 * after the PWM and takes effect one tick late.
 */
#include "v4_adapt.h"
#include "v4_core.h"
#include "AllHeader.h"

extern float Balance_Kp, Balance_Kd;
extern float Velocity_Kp, Velocity_Ki;
extern float Turn_Kp, Turn_Kd;
extern float Encoder_Integral;

float v4_f = 1.0f;
float v4_osc = 0.0f;
int   v4_latched = 0;
long  v4_rearms = 0;
float v4_th_ref = 0.0f;
int   v4_rec = 0;

static V4_State st;

static int cmd_moving(void)
{
    float movement;
    if (g_newcarstate == enRUN || g_newcarstate == enps2Fleft || g_newcarstate == enps2Fright)
        movement = Car_Target_Velocity;
    else if (g_newcarstate == enBACK || g_newcarstate == enps2Bleft || g_newcarstate == enps2Bright)
        movement = -Car_Target_Velocity;
    else if (g_newcarstate == enAvoid)
        movement = -10.0f;
    else if (g_newcarstate == enFollow)
        movement = 10.0f;
    else
        movement = (float)Move_X;
    if (movement > 1e-6f || movement < -1e-6f) return 1;
    /* turning counts as a command too (Turn_PD's rules): an in-place turn
       shakes the body and must not be judged as standing still */
    if ((g_newcarstate == enLEFT || g_newcarstate == enps2Fleft || g_newcarstate == enps2Bleft ||
         g_newcarstate == enRIGHT || g_newcarstate == enps2Fright || g_newcarstate == enps2Bright)
            && (Car_Turn_Amplitude_speed > 1e-6f || Car_Turn_Amplitude_speed < -1e-6f))
        return 1;
    if (g_newcarstate == enTLEFT || g_newcarstate == enTRIGHT) return 1;
    return Move_Z > 1e-6f || Move_Z < -1e-6f;
}

static void apply(float f)
{
    V4_Gains g;
    V4_GainsFor(f, &g);
    if (g.vki > 0.0f && Velocity_Ki > 0.0f && g.vki != Velocity_Ki)
        Encoder_Integral *= Velocity_Ki / g.vki;      /* bumpless */
    Balance_Kp  = g.bkp;
    Balance_Kd  = g.bkd;
    Velocity_Kp = g.vkp;
    Velocity_Ki = g.vki;
    Turn_Kp     = g.tkp;
    Turn_Kd     = g.tkd;
}

void V4_Reset(void)
{
    V4_Init(&st);
    v4_f = st.f; v4_osc = 0.0f; v4_latched = 0; v4_rearms = 0;
    Velocity_Ki = 0.0f;            /* first apply() must not rescale the integral */
    apply(st.f);                   /* boot on Weight_M: the safe end */
}

void V4_Tick(void)
{
    /* Motors off -- not started yet (Stop_Flag before the second KEY1), picked
     * up, fallen past 40 deg, or low battery: hold v4 at its boot state.
     * This ISR runs from MPU6050_EXTI_Init() on, i.e. while the car is still
     * in the hand.  Without this, the probe counted that time as "standing",
     * saw no chatter (motors off) and latched HEAVY after 3 s; the empty car
     * then started on f=0.40, which on the real car shakes hard (2026-10-03).
     * Probing now starts when balancing starts, as it always did in the twin,
     * and a fall makes it re-judge the load from scratch. */
    if (Turn_Off(Angle_Balance, battery) == 1) {
        V4_Init(&st);
        v4_f = st.f; v4_osc = 0.0f; v4_latched = 0;
        v4_th_ref = Angle_Balance; v4_rec = 0;
        apply(st.f);
        return;
    }
    {   /* key pressed / released / changed: start the upshift grace window */
        static int last_cs = -1;
        if ((int)g_newcarstate != last_cs) { if (last_cs >= 0) V4_CmdEdge(&st); last_cs = (int)g_newcarstate; }
    }
    v4_f = V4_Step(&st, Angle_Balance, (float)Gyro_Balance, cmd_moving());
    v4_osc = st.osc;
    v4_latched = st.latched;
    v4_rearms = st.rearms;
    v4_th_ref = st.th_ref;
    v4_rec = st.rec;
    apply(v4_f);
}
