/* Balance-and-drive policy inference for STM32. See policy.c.
 *
 * Written for the run-8 checkpoint trained by train_real_robot.py against
 * real_robot.xml: 34 inputs (17 instantaneous + 5 history taps x 3 signals +
 * 2 leaky integrals), PWM output in [-1, 1], 200 Hz control. Run 8 trains on
 * exactly what this file computes from the IMU and encoders - every input
 * below has a twin in train_real_robot.py's PWMCommandWrapper._sense() /
 * _get_conditioned_obs(). Change one, change the other, and retrain.
 *
 * Run 7 checkpoints load (same 34-input shape) but were trained on ground
 * truth and a motor model the car contradicts; they are not usable here. */
#ifndef POLICY_H
#define POLICY_H

#include "policy_weights.h"

#if POLICY_N_OBS != 34
#error "policy.c's observation builder is written for the 34-input layout " \
       "(base 17 + history taps + both leaky integrals). This header was " \
       "generated from a checkpoint with a different input count."
#endif

/* Chassis height above the floor: real_robot.xml puts the chassis origin at
 * the axle, one wheel radius up. The car cannot measure it; training feeds the
 * same constant. (Was 0.05, the superseded their_robot.xml's wheel radius.) */
#define POLICY_NOMINAL_HEIGHT 0.0334f

/* Wheel radius (m) and half track width (m), measured (PARAMS.md) and baked
 * into real_robot.xml. */
#define POLICY_WHEEL_RADIUS 0.0335f
#define POLICY_HALF_TRACK   0.0835f

/* Encoder: 4x quadrature x 11 ppr x 1:30 gearbox = 1320 counts per wheel
 * revolution (app_motor.h). CHECK YOUR VARIANT - the JGB37-520 ships in gear
 * ratios from 1:30 to 1:131 and ppr is occasionally 13. Training assumes 1320.
 *
 * Wheel speed is differenced over 4 ticks, exactly as training emulates it:
 * one count per 5 ms tick is already 0.95 rad/s, so a 1-tick difference is
 * mostly quantisation noise. */
#define POLICY_ENC_COUNTS_PER_REV 1320.0f
#define POLICY_ENC_VEL_WINDOW     4

/* Control period: 200 Hz (train_real_robot.py, FRAME_SKIP=5 on real_robot.xml's
 * 1 ms timestep), matching the stock firmware's MPU6050-interrupt loop. Run
 * the loop from a hardware interrupt. The leaky-integral decay constants and
 * the history-tap spacing are both baked in at this rate - on the car, a tick
 * overrun that halved the loop rate made every time-based input wrong by 2x. */
#define POLICY_CONTROL_HZ 200.0f
/* 1/POLICY_CONTROL_HZ, spelled as a division of two literals so the compiler
 * folds it to a constant - no runtime divide on a target with no FPU. */
#define POLICY_DT (1.0f / POLICY_CONTROL_HZ)

/* Fall cutoff, matching FALL_ANGLE_LIMIT in train_real_robot.py: the firmware's
 * real 40 degree cut-out. */
#define POLICY_FALL_ANGLE_LIMIT 0.70f

/* Input sanitising, applied identically in training. On the car a single
 * corrupted encoder sample (a stack overflow) produced a ~1e6 m/s forward
 * speed; the then-unclamped position integral latched it and pinned the
 * network in saturation - constant PWM -1923/-1466 - for ~15 s. Physical
 * values never reach these clamps; NaN is mapped to 0. */
#define POLICY_V_FORWARD_CLIP  2.0f    /* m/s */
#define POLICY_WHEEL_VEL_CLIP  40.0f   /* rad/s, above the 35 rad/s no-load speed */

/* --- observation layout: history buffer -----------------------------------
 *
 * A single frame cannot distinguish "oscillating about equilibrium" from
 * "drifting away". Three signals (pitch, pitch rate, forward speed) are
 * remembered at five dilated taps spanning 235 ms, matching HISTORY_TAPS in
 * train_real_robot.py exactly - re-export the checkpoint if that ever changes,
 * do not hand-edit these to match. */
#define POLICY_N_BASE_OBS        17
#define POLICY_N_HISTORY_SIGNALS 3   /* pitch, pitch rate, v_forward, in that order */
#define POLICY_N_HISTORY_TAPS    5
#define POLICY_HISTORY_LEN       48  /* max(taps) + 1 */
/* Ticks (at 200 Hz, 5 ms each) back from "now": 10/25/55/115/235 ms.
 * Defined once in policy.c. */
extern const int policy_history_taps[POLICY_N_HISTORY_TAPS];

/* --- observation layout: leaky integrals -----------------------------------
 *
 * Both are "decay * accumulator + error * dt" run once per control tick,
 * mirroring a PI controller's I term. dt = 1/200 s is baked into the decay
 * constants below, so they are only valid at POLICY_CONTROL_HZ.
 *
 * Position (forward-velocity error): tau=2s, deliberately short so the robot
 * is never asked to chase distance lost under a stale command. Clipped to
 * +/-0.5 m since run 8 (see the sanitising note above).
 *
 * Heading (yaw-rate error, which integrates to a heading error): tau=30s,
 * deliberately long - driving it to zero is exactly "point where commanded".
 * Clipped to +/-0.2 rad. Both leaky rather than absolute: the MPU6050 has no
 * magnetometer, so this only ever integrates the last few tau of gyro-z.
 * Training now gives the gyro a random bias, so the policy has learned that a
 * heading integral sitting at a small offset is not necessarily a real turn. */
#define POLICY_POS_DECAY      0.99750312f  /* exp(-dt / 2.0s) */
#define POLICY_POS_OBS_SCALE  10.0f
#define POLICY_POS_ERR_CLIP   0.5f         /* m */
#define POLICY_YAW_DECAY      0.99983335f  /* exp(-dt / 30.0s) */
#define POLICY_YAW_OBS_SCALE  5.0f         /* 1 / POLICY_YAW_CLIP_RAD */
#define POLICY_YAW_CLIP_RAD   0.2f

/* --- motor drive: PWM, not torque -------------------------------------------
 *
 * The policy commands PWM directly, pre-dead-band-compensation, exactly as the
 * firmware's own controller output is defined. Feed policy_action_to_pwm()'s
 * output into the stock firmware's PWM_Ignore() (adds POLICY_PWM_DEADBAND),
 * then the 2800 clamp; do not add the compensation twice.
 *
 * POLICY_PWM_DEADBAND is the firmware's COMPENSATION, not the motor's actual
 * dead zone, and it must stay 1300: run 8 was trained with exactly this
 * compensation in front of a motor whose real dead zone was randomised over
 * 1350-1600 counts (motor_model.py), so the policy already handles the gap.
 * Do NOT raise it to a bench-measured dead zone - that changes the action
 * mapping the network learned. If the measured dead zone falls outside
 * 1350-1600, update MOTOR_DEAD_ZONE_RANGE in motor_model.py and retrain. */
#define POLICY_PWM_LIMIT     2800.0f
#define POLICY_PWM_DEADBAND  1300.0f
#define POLICY_PWM_USABLE    (POLICY_PWM_LIMIT - POLICY_PWM_DEADBAND)

#ifdef __cplusplus
extern "C" {
#endif

/* Reset everything stateful - the history buffer, both leaky integrals and the
 * encoder odometry - exactly as train_real_robot.py does on an episode reset.
 * The history is primed with HISTORY_LEN copies of one real sample rather than
 * zeros, which would look like a violent transient.
 *
 * Call this when the controller is armed: robot upright and still, motors off.
 * Call it again every time the robot is re-armed after a fall. Pass the forward
 * speed the odometry would report with the wheels still:
 * POLICY_WHEEL_RADIUS * pitch_rate * cosf(pitch). */
void policy_reset_state(float pitch, float pitch_rate, float v_forward);

/* Encoder odometry, once per control tick, BEFORE policy_build_obs*():
 *
 *   dcount_l/r  encoder counts since the previous tick, sign-corrected so
 *               + is forward on both wheels
 *   pitch       fused pitch, rad (+ = leaning forward)
 *   pitch_rate  body pitch rate from the gyro, rad/s (the same value you pass
 *               as gyro_xyz[1])
 *
 * Outputs the wheel speeds (rad/s, relative to the chassis) and the forward
 * speed (m/s) the policy expects. Encoders measure the wheel relative to the
 * chassis, so the axle's rolling speed adds the pitch rate, and the policy's
 * forward speed is in the pitched body frame, hence the cos(pitch):
 *     v = r * (mean wheel rate + pitch rate) * cos(pitch)
 * Wheel speeds are clamped to +/-POLICY_WHEEL_VEL_CLIP before v is formed. */
void policy_odom_update(long dcount_l, long dcount_r, float pitch, float pitch_rate,
                        float *wheel_vel_l, float *wheel_vel_r, float *v_forward);

/* obs -> action, already clamped to +/-POLICY_ACTION_LIMIT (see
 * policy_weights.h): a PWM duty fraction in [-1, 1], pre-dead-band-
 * compensation. action[0] is the left wheel, action[1] the right. */
void policy_infer(const float obs[POLICY_N_OBS], float action[POLICY_N_ACT]);

/* Assemble the full 34-input observation vector: the 17 instantaneous inputs,
 * the history taps, and both leaky integrals (updated as a side effect of this
 * call - see policy.c). Exactly once per control tick, after
 * policy_odom_update().
 *
 *   quat_wxyz    orientation from the IMU, w first, world-referenced
 *   v_forward    from policy_odom_update()
 *   gyro_xyz     body angular rate, rad/s: roll, pitch, yaw (+ = turning left)
 *   wheel_vel_*  from policy_odom_update()
 *   cmd_*        your setpoints: m/s and rad/s
 *
 * Not passed in, because the car cannot measure them and training feeds the
 * same constants: chassis height (POLICY_NOMINAL_HEIGHT), lateral and vertical
 * velocity (0), and - since run 8 - the wheel angles (0). Absolute wheel angle
 * is physically meaningless, yet run 7 learned to depend on it; long drives
 * fell once it left the range a 10 s training episode reaches. */
void policy_build_obs(float obs[POLICY_N_OBS],
                      const float quat_wxyz[4],
                      float v_forward,
                      const float gyro_xyz[3],
                      float wheel_vel_l, float wheel_vel_r,
                      float cmd_forward, float cmd_turn);

/* Preferred entry point on an MPU6050 (6-axis, no magnetometer): takes roll
 * and pitch directly and pins yaw to zero, which is also what training does.
 * Use this rather than policy_build_obs() unless you have a magnetometer. */
void policy_build_obs_rp(float obs[POLICY_N_OBS],
                         float roll, float pitch,
                         float v_forward,
                         const float gyro_xyz[3],
                         float wheel_vel_l, float wheel_vel_r,
                         float cmd_forward, float cmd_turn);

/* Network output (already clamped by policy_infer()) -> PWM pulse count,
 * PRE-dead-band-compensation, matching motor_model.py:action_to_pwm() before
 * its PWM_Ignore step. This is the entire conversion - no motor model here. */
static inline int policy_action_to_pwm(float action)
{
    if (action >  1.0f) action =  1.0f;
    if (action < -1.0f) action = -1.0f;
    return (int)(action * POLICY_PWM_USABLE);
}

#ifdef __cplusplus
}
#endif

#endif /* POLICY_H */
