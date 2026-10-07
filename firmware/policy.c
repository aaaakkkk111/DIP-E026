/* Balance-and-drive policy inference for STM32.
 *
 * The network itself is three matrix-vector products with tanh in between, so
 * it is written out directly rather than pulled in through a framework. At
 * 5442-ish parameters (exact count depends on the checkpoint; see the header
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
 * Weights live in flash as `static const`; only a few hundred bytes of
 * activations and history state sit in RAM.
 */
#include "policy.h"
#include "policy_weights.h"

#include <math.h>

/* Activation.
 *
 * The STM32F103 is a Cortex-M3: NO hardware FPU, so every float operation is a
 * library call. Profiling the operation mix shows the tanhf() calls cost more
 * than all the multiply-accumulates put together, because tanhf() pulls in
 * expf().
 *
 * POLICY_FAST_TANH swaps in a rational approximation that touches only
 * multiply, add and divide - no exp, no branches beyond the clamp. Validated
 * against the trained (torque-output) network over 2000 random observations:
 * worst-case action error 0.0113 against a 0.6 limit (1.9%), mean 0.0025, and
 * every commanded behaviour survived a full episode in simulation. Has not
 * been re-validated against the current PWM-output checkpoint specifically -
 * do that before trusting it here (see test_policy.c).
 *
 * Define POLICY_FAST_TANH=0 to use the exact libm version on a target with an
 * FPU, where the saving is not worth the approximation.
 */
#ifndef POLICY_FAST_TANH
#define POLICY_FAST_TANH 1
#endif

static inline float policy_tanh(float x)
{
#if POLICY_FAST_TANH
    if (x >  3.0f) return  1.0f;
    if (x < -3.0f) return -1.0f;
    const float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
#else
    return tanhf(x);
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

    dense_tanh(policy_w0, policy_b0, obs, h1, POLICY_N_H1, POLICY_N_OBS);
    dense_tanh(policy_w1, policy_b1, h1, h2, POLICY_N_H2, POLICY_N_H1);
    dense_linear(policy_w2, policy_b2, h2, action, POLICY_N_ACT, POLICY_N_H2);

    /* Stable-Baselines3 clips to the action space inside predict(), so the
     * same clamp has to happen here or the motors will be commanded actions
     * the policy was never evaluated at. POLICY_ACTION_LIMIT is 1.0 for this
     * (PWM) checkpoint - see policy_action_to_pwm() for what happens next. */
    for (int i = 0; i < POLICY_N_ACT; ++i) {
        if (action[i] >  POLICY_ACTION_LIMIT) action[i] =  POLICY_ACTION_LIMIT;
        if (action[i] < -POLICY_ACTION_LIMIT) action[i] = -POLICY_ACTION_LIMIT;
    }
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
    g_pos_err = POLICY_POS_DECAY * g_pos_err
              + (v_forward - cmd_forward) * POLICY_DT;

    g_yaw_err = POLICY_YAW_DECAY * g_yaw_err
              + (gyro_xyz[2] - cmd_turn) * POLICY_DT;
    if (g_yaw_err >  POLICY_YAW_CLIP_RAD) g_yaw_err =  POLICY_YAW_CLIP_RAD;
    if (g_yaw_err < -POLICY_YAW_CLIP_RAD) g_yaw_err = -POLICY_YAW_CLIP_RAD;

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
