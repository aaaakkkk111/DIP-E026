/* Balance-and-drive policy inference for STM32.
 *
 * The network itself is three matrix-vector products with tanh in between, so
 * it is written out directly rather than pulled in through a framework. At
 * 6530 parameters (the pinned 34 -> 64 -> 64 -> 2 actor; see the header
 * comment in policy_weights.h) there is nothing for TFLite Micro or X-CUBE-AI
 * to optimise that would repay their code size and build complexity.
 *
 * Building the OBSERVATION is the larger job here, and it is stateful: the
 * run-7 policy conditions on a 235 ms history of (pitch, pitch rate, forward
 * speed) and on two leaky integrals of tracking error, all of which have to be
 * carried between calls exactly the way train_real_robot.py's Python wrapper
 * carries them between environment steps. See policy_reset_state() and
 * finish_obs() below.
 *
 * Original float weights live in flash as `static const`. The timing port
 * additionally uses 12,800 bytes of int16 weight tables and 520 bytes of
 * scales in RAM, plus activations and history. Actual linked RAM/stack usage
 * must be checked in the fresh Keil map; this is not a target-build result.
 */
#include "policy.h"
#include "policy_weights.h"
#include "policy_testvectors.h"

#include <math.h>
#include <stdint.h>
#include <stddef.h> /* size_t: ARMCC5 does not supply it through math.h */

/* Cortex-M3 has no FPU. Linear interpolation of tanh (step 1/16 on
 * [0,8]) avoids expf and the old biased rational approximation.
 * This is an EXPLICIT numerical approximation, not exact SB3 arithmetic.
 * verify_export.py measures the exported checkpoint's action errors.
 * No target execution time is assumed; stopped inference must qualify. */
#ifndef POLICY_FAST_TANH
#define POLICY_FAST_TANH 1
#endif
#ifndef POLICY_FIXED_DENSE
#define POLICY_FIXED_DENSE 1
#endif

static const float tanh_lut[129] = {
    0.00000000f, 0.0624187467f, 0.124353002f, 0.185333200f, 0.244918662f, 0.302709729f, 0.358357398f, 0.411570056f, 
    0.462117157f, 0.509829974f, 0.554599722f, 0.596373555f, 0.635148952f, 0.670967074f, 0.703905604f, 0.734071520f, 
    0.761594156f, 0.786618812f, 0.809301070f, 0.829801910f, 0.848283640f, 0.864906618f, 0.879826700f, 0.893193340f, 
    0.905148254f, 0.915824544f, 0.925346225f, 0.933828043f, 0.941375538f, 0.948085286f, 0.954045260f, 0.959335293f, 
    0.964027580f, 0.968187217f, 0.971872746f, 0.975136698f, 0.978026115f, 0.980583047f, 0.982845029f, 0.984845517f, 
    0.986614298f, 0.988177862f, 0.989559749f, 0.990780856f, 0.991859725f, 0.992812795f, 0.993654634f, 0.994398146f, 
    0.995054754f, 0.995634567f, 0.996146531f, 0.996598555f, 0.996997635f, 0.997349955f, 0.997660979f, 0.997935538f, 
    0.998177898f, 0.998391828f, 0.998580659f, 0.998747332f, 0.998894443f, 0.999024286f, 0.999138886f, 0.999240031f, 
    0.999329300f, 0.999408086f, 0.999477619f, 0.999538987f, 0.999593146f, 0.999640944f, 0.999683128f, 0.999720356f, 
    0.999753211f, 0.999782206f, 0.999807795f, 0.999830378f, 0.999850308f, 0.999867896f, 0.999883417f, 0.999897116f, 
    0.999909204f, 0.999919873f, 0.999929287f, 0.999937596f, 0.999944929f, 0.999951400f, 0.999957110f, 0.999962150f, 
    0.999966597f, 0.999970522f, 0.999973986f, 0.999977042f, 0.999979740f, 0.999982121f, 0.999984221f, 0.999986075f, 
    0.999987712f, 0.999989156f, 0.999990430f, 0.999991554f, 0.999992547f, 0.999993423f, 0.999994195f, 0.999994877f, 
    0.999995479f, 0.999996011f, 0.999996479f, 0.999996893f, 0.999997258f, 0.999997580f, 0.999997865f, 0.999998116f, 
    0.999998337f, 0.999998532f, 0.999998705f, 0.999998857f, 0.999998991f, 0.999999110f, 0.999999214f, 0.999999307f, 
    0.999999388f, 0.999999460f, 0.999999524f, 0.999999580f, 0.999999629f, 0.999999673f, 0.999999711f, 0.999999745f, 
    0.999999775f
};

static inline float policy_tanh(float x)
{
#if POLICY_FAST_TANH
    float ax=fabsf(x), pos, fraction, value;
    unsigned index;
    if (ax >= 8.0f) return x < 0.0f ? -1.0f : 1.0f;
    pos=ax*16.0f; index=(unsigned)pos; fraction=pos-(float)index;
    value=tanh_lut[index]+fraction*(tanh_lut[index+1U]-tanh_lut[index]);
    return x < 0.0f ? -value : value;
#else
    return tanhf(x);
#endif
}

/* Rowwise int16 weights and per-layer dynamic int16 inputs eliminate
 * thousands of software-float MACs. int64 accumulation cannot overflow:
 * max 64*32767^2 < 2^36. Quantization is recorded and tested, not hidden.
 * Original float weights remain immutable in flash; prepare once at boot. */
#if POLICY_FIXED_DENSE
static int16_t qw0[POLICY_N_H1*POLICY_N_OBS];
static int16_t qw1[POLICY_N_H2*POLICY_N_H1];
static int16_t qw2[POLICY_N_ACT*POLICY_N_H2];
static float qs0[POLICY_N_H1], qs1[POLICY_N_H2], qs2[POLICY_N_ACT];
static int quantized_ready=0;
static int16_t qround(float x) { return (int16_t)(x < 0.0f ? x-0.5f : x+0.5f); }
static int prepare_layer(const float *w,int16_t *qw,float *scale,int rows,int cols)
{
    int o,i;
    for(o=0;o<rows;++o) {
        float max_abs=0.0f;
        for(i=0;i<cols;++i) {
            float a=fabsf(w[o*cols+i]);
            if(!isfinite(a)) return 0;
            if(a>max_abs) max_abs=a;
        }
        scale[o]=max_abs>0.0f ? max_abs/32767.0f : 1.0f/32767.0f;
        for(i=0;i<cols;++i) qw[o*cols+i]=qround(w[o*cols+i]/scale[o]);
    }
    return 1;
}
static void dense_fixed(const int16_t *qw,const float *scale,const float *bias,
                        const float *x,float *y,int rows,int cols,int activate)
{
    int16_t qx[POLICY_N_H1];
    float max_abs=0.0f,input_scale,input_inverse;
    int i,o;
    for(i=0;i<cols;++i) if(fabsf(x[i])>max_abs) max_abs=fabsf(x[i]);
    input_scale=max_abs>0.0f ? max_abs/32767.0f : 1.0f/32767.0f;
    /* Cortex-M3 software division was repeated for every input. One
     * reciprocal per layer replaces 162 divisions/tick with multiplies.
     * Float rounding can change a quantized input by one LSB; the numerical
     * verifier tests this explicitly. Original checkpoint weights unchanged. */
    input_inverse=1.0f/input_scale;
    for(i=0;i<cols;++i) qx[i]=qround(x[i]*input_inverse);
    for(o=0;o<rows;++o) {
        const int16_t *row=qw+o*cols;
        int64_t sum=0,second=0;
        float value;
        /* Widen BEFORE multiplication: exposes signed long multiply-accumulate
         * to the ARM optimizer instead of 32-bit multiply/sign-extend/add.
         * Two independent accumulators halve loop branches; the integer dot
         * product is EXACTLY the same (no overflow: <2^36). No DSP/FPU needed. */
        for(i=0;i+1<cols;i+=2) {
            sum+=(int64_t)row[i]*(int64_t)qx[i];
            second+=(int64_t)row[i+1]*(int64_t)qx[i+1];
        }
        if(i<cols) sum+=(int64_t)row[i]*(int64_t)qx[i];
        sum+=second;
        value=bias[o]+((float)sum*input_scale)*scale[o];
        y[o]=activate ? policy_tanh(value) : value;
    }
}
#endif

int policy_prepare(void)
{
#if POLICY_FIXED_DENSE
    quantized_ready=prepare_layer(policy_w0,qw0,qs0,POLICY_N_H1,POLICY_N_OBS) &&
                    prepare_layer(policy_w1,qw1,qs1,POLICY_N_H2,POLICY_N_H1) &&
                    prepare_layer(policy_w2,qw2,qs2,POLICY_N_ACT,POLICY_N_H2);
    return quantized_ready;
#else
    return 1;
#endif
}

/* y = tanh(W.x + b), with W stored row-major as (n_out, n_in). */
static void dense_tanh(const float *w, const float *b, const float *x,
                       float *y, int n_out, int n_in)
{
    for (int o = 0; o < n_out; ++o) {
        const float *row = w + (size_t)o * n_in;
        float acc = b[o];
        for (int i = 0; i < n_in; ++i) {
            acc += row[i] * x[i];
        }
        y[o] = policy_tanh(acc);
    }
}

/* y = W.x + b, no activation (the action head is linear). */
static void dense_linear(const float *w, const float *b, const float *x,
                         float *y, int n_out, int n_in)
{
    for (int o = 0; o < n_out; ++o) {
        const float *row = w + (size_t)o * n_in;
        float acc = b[o];
        for (int i = 0; i < n_in; ++i) {
            acc += row[i] * x[i];
        }
        y[o] = acc;
    }
}

void policy_infer(const float obs[POLICY_N_OBS], float action[POLICY_N_ACT])
{
    float h1[POLICY_N_H1];
    float h2[POLICY_N_H2];

    for(int i=0;i<POLICY_N_OBS;++i) {
        if(!isfinite(obs[i])) { action[0]=action[1]=NAN; return; }
    }
#if POLICY_FIXED_DENSE
    if(!quantized_ready) { action[0]=action[1]=NAN; return; }
    dense_fixed(qw0,qs0,policy_b0,obs,h1,POLICY_N_H1,POLICY_N_OBS,1);
    dense_fixed(qw1,qs1,policy_b1,h1,h2,POLICY_N_H2,POLICY_N_H1,1);
    dense_fixed(qw2,qs2,policy_b2,h2,action,POLICY_N_ACT,POLICY_N_H2,0);
#else
    dense_tanh(policy_w0, policy_b0, obs, h1, POLICY_N_H1, POLICY_N_OBS);
    dense_tanh(policy_w1, policy_b1, h1, h2, POLICY_N_H2, POLICY_N_H1);
    dense_linear(policy_w2, policy_b2, h2, action, POLICY_N_ACT, POLICY_N_H2);
#endif

    /* Stable-Baselines3 clips to the action space inside predict(), so the
     * same clamp has to happen here or the motors will be commanded actions
     * the policy was never evaluated at. POLICY_ACTION_LIMIT is 1.0 for this
     * (PWM) checkpoint - see policy_action_to_pwm() for what happens next. */
    for (int i = 0; i < POLICY_N_ACT; ++i) {
        if (action[i] >  POLICY_ACTION_LIMIT) action[i] =  POLICY_ACTION_LIMIT;
        if (action[i] < -POLICY_ACTION_LIMIT) action[i] = -POLICY_ACTION_LIMIT;
    }
}

int policy_self_test(void)
{
    /* Run the ACTUAL compiled kernel, not merely its checkpoint fingerprint.
     * Called once before IWDG/IMU IRQ enable; never in the 5ms control loop.
     * These four exported checkpoint vectors exercise all three dense layers.
     * The explicit 0.002 action tolerance allows the documented int16/LUT
     * approximation (sampled host error <0.0005), not an arbitrary wrong port.
     * It does not establish sensor signs, motor calibration or balance. */
    float action[POLICY_N_ACT];
    int sample,channel;
    for(sample=0;sample<POLICY_N_TESTS;++sample) {
        policy_infer(policy_test_obs+sample*POLICY_N_OBS,action);
        for(channel=0;channel<POLICY_N_ACT;++channel) {
            if(!isfinite(action[channel]) ||
               fabsf(action[channel]-policy_test_expected[sample*POLICY_N_ACT+channel])>0.002f)
                return 0;
        }
    }
    return 1;
}

/* --- history buffer + leaky integrals --------------------------------------
 *
 * All state a pure C port of train_real_robot.py's PWMCommandWrapper needs
 * between control ticks. Reset with policy_reset_state(), advanced once per
 * tick inside finish_obs() below - never touch these directly.
 */
const int policy_history_taps[POLICY_N_HISTORY_TAPS] = {2, 5, 11, 23, 47};

static float g_hist[POLICY_HISTORY_LEN][POLICY_N_HISTORY_SIGNALS];
static int   g_hist_head = 0;
static float g_pos_err = 0.0f;
static float g_yaw_err = 0.0f;
static int g_obs_started = 0;

void policy_reset_state(float pitch, float pitch_rate, float v_forward)
{
    /* No real history exists yet at the moment of reset, so every slot is
     * primed with the current sample rather than zeros - feeding zeros would
     * look like a violent transient the policy has to learn to ignore, which
     * is exactly what train_real_robot.py avoids by doing the same thing. */
    for (int i = 0; i < POLICY_HISTORY_LEN; ++i) {
        g_hist[i][0] = pitch;
        g_hist[i][1] = pitch_rate;
        g_hist[i][2] = v_forward;
    }
    g_hist_head = 0;
    g_pos_err = 0.0f;
    g_yaw_err = 0.0f;
    g_obs_started = 0;
}

static void hist_push(float pitch, float pitch_rate, float v_forward)
{
    g_hist_head = (g_hist_head + 1) % POLICY_HISTORY_LEN;
    g_hist[g_hist_head][0] = pitch;
    g_hist[g_hist_head][1] = pitch_rate;
    g_hist[g_hist_head][2] = v_forward;
}

/* ticks_back=0 is the sample just pushed (this tick); ticks_back=N is N
 * ticks before that - matching HISTORY_TAPS' meaning in train_real_robot.py
 * exactly (Python's self._hist[0] is likewise the just-pushed sample). */
static const float *hist_at(int ticks_back)
{
    int idx = (g_hist_head - ticks_back + POLICY_HISTORY_LEN) % POLICY_HISTORY_LEN;
    return g_hist[idx];
}

/* Shared tail of both public obs-builders: the base 17 inputs (unchanged
 * since run 1), then the two pieces of state that make this a run-7
 * checkpoint rather than an earlier one.
 *
 * The leaky integrals are updated HERE, as a side effect of building the
 * observation - matching train_real_robot.py's own comment on why: this is
 * the one place called exactly once per control tick, so the value exists
 * before the observation containing it is built. */
static void finish_obs(float obs[POLICY_N_OBS],
                       const float quat_wxyz[4], float pitch,
                       float wheel_angle_l, float wheel_angle_r,
                       float v_forward,
                       const float gyro_xyz[3],
                       float wheel_vel_l, float wheel_vel_r,
                       float cmd_forward, float cmd_turn)
{
    /* Index 0, 8 and 9 are not measurable on the real robot: the simulator
     * supplied chassis height and the lateral/vertical components of body
     * velocity. Substituting constants was verified against the trained
     * policy in simulation. */
    obs[0]  = POLICY_NOMINAL_HEIGHT;
    obs[1]  = quat_wxyz[0];
    obs[2]  = quat_wxyz[1];
    obs[3]  = quat_wxyz[2];
    obs[4]  = quat_wxyz[3];
    obs[5]  = wheel_angle_l;
    obs[6]  = wheel_angle_r;
    obs[7]  = v_forward;
    obs[8]  = 0.0f;               /* lateral velocity, not sensed */
    obs[9]  = 0.0f;               /* vertical velocity, not sensed */
    obs[10] = gyro_xyz[0];
    obs[11] = gyro_xyz[1];
    obs[12] = gyro_xyz[2];
    obs[13] = wheel_vel_l;
    obs[14] = wheel_vel_r;
    obs[15] = cmd_forward;
    obs[16] = cmd_turn;

    /* Leaky integrals, both "decay * accumulator + error * dt". Order between
     * the two does not matter - neither depends on the other - only that both
     * use this tick's sensor values. gyro_xyz[2] is yaw rate (see
     * policy_build_obs()'s parameter doc), the same signal the reward and the
     * training-time wrapper both call "actual_v_turn". */
    if(g_obs_started) {
    g_pos_err = POLICY_POS_DECAY * g_pos_err
              + (v_forward - cmd_forward) * POLICY_DT;

    g_yaw_err = POLICY_YAW_DECAY * g_yaw_err
              + (gyro_xyz[2] - cmd_turn) * POLICY_DT;
    if (g_yaw_err >  POLICY_YAW_CLIP_RAD) g_yaw_err =  POLICY_YAW_CLIP_RAD;
    if (g_yaw_err < -POLICY_YAW_CLIP_RAD) g_yaw_err = -POLICY_YAW_CLIP_RAD;
    } else g_obs_started=1; /* Python wrapper's first observation has zero integrals */

    /* Push this tick's (pitch, pitch rate, v_forward) sample, then read out
     * the five taps in the same order train_real_robot.py concatenates them. */
    hist_push(pitch, gyro_xyz[1], v_forward);
    int o = POLICY_N_BASE_OBS;
    for (int t = 0; t < POLICY_N_HISTORY_TAPS; ++t) {
        const float *s = hist_at(policy_history_taps[t]);
        obs[o++] = s[0];
        obs[o++] = s[1];
        obs[o++] = s[2];
    }

    obs[o++] = g_pos_err * POLICY_POS_OBS_SCALE;
    obs[o++] = g_yaw_err * POLICY_YAW_OBS_SCALE;
    /* o is now POLICY_N_OBS (34); nothing left to fill. */
}

void policy_build_obs(float obs[POLICY_N_OBS],
                      const float quat_wxyz[4],
                      float wheel_angle_l, float wheel_angle_r,
                      float v_forward,
                      const float gyro_xyz[3],
                      float wheel_vel_l, float wheel_vel_r,
                      float cmd_forward, float cmd_turn)
{
    /* Pitch is the middle angle of the same 'xyz' Euler decomposition
     * train_real_robot.py uses for its history-tap "pitch" signal, extracted
     * here since this entry point is handed a quaternion with no separate
     * pitch scalar. The roll term cancels out of this formula exactly (see
     * policy_build_obs_rp()'s quaternion construction), so this is exact for
     * any roll, not a small-angle approximation. */
    const float w = quat_wxyz[0], x = quat_wxyz[1], y = quat_wxyz[2], z = quat_wxyz[3];
    float s = 2.0f * (w * y - z * x);
    if (s >  1.0f) s =  1.0f;   /* guard asinf() against float rounding at the poles */
    if (s < -1.0f) s = -1.0f;
    const float pitch = asinf(s);

    finish_obs(obs, quat_wxyz, pitch, wheel_angle_l, wheel_angle_r, v_forward,
              gyro_xyz, wheel_vel_l, wheel_vel_r, cmd_forward, cmd_turn);
}

void policy_build_obs_rp(float obs[POLICY_N_OBS],
                         float roll, float pitch,
                         float wheel_angle_l, float wheel_angle_r,
                         float v_forward,
                         const float gyro_xyz[3],
                         float wheel_vel_l, float wheel_vel_r,
                         float cmd_forward, float cmd_turn)
{
    /* Build the quaternion from roll and pitch alone, with yaw pinned to zero.
     *
     * The MPU6050 is a 6-axis part - gyro plus accelerometer, no magnetometer -
     * so roll and pitch are absolutely referenced against gravity but YAW IS
     * NOT OBSERVABLE and drifts without bound. Measured tolerance of the
     * trained policy to a yaw error: fine at 10, 30 and 90 degrees, but it
     * FELL after 138 steps at 180 degrees (measured on the earlier torque
     * policy; the run-7 heading integral bounds SLOW wander around a held
     * heading, it does not change this large-fixed-offset failure mode, so
     * treat the number as indicative rather than re-verified).
     *
     * Pinning yaw to zero removes the failure entirely, and costs almost
     * nothing because every other input is already body-referenced.
     *
     * Equivalent to scipy's from_euler('xyz', [roll, pitch, 0]); verified exact.
     */
    const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    const float quat[4] = { cp * cr, cp * sr, sp * cr, -sp * sr };

    /* pitch is used directly for the history sample here rather than
     * re-extracted from the quaternion just built above - the two are
     * mathematically identical (the roll terms cancel in the extraction
     * formula policy_build_obs() uses), so this just skips a redundant
     * asinf() call. */
    finish_obs(obs, quat, pitch, wheel_angle_l, wheel_angle_r, v_forward,
              gyro_xyz, wheel_vel_l, wheel_vel_r, cmd_forward, cmd_turn);
}
