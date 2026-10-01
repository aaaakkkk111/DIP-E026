/* Balance-and-drive policy inference for STM32. See policy.c.
 *
 * Written for the run-7 checkpoint trained by train_real_robot.py against
 * real_robot.xml: 34 inputs (17 instantaneous + 5 history taps x 3 signals +
 * 2 leaky integrals), PWM output in [-1, 1], 200 Hz control. If you export an
 * older checkpoint (32 or 33 inputs, or the original 17-input torque policy),
 * this file's obs-builder will NOT match it - see export_stm32.py's own
 * dimension check, which warns you at export time. */
#ifndef POLICY_H
#define POLICY_H

#include "policy_weights.h"

#if POLICY_N_OBS != 34
#error "policy.c's observation builder is written for the 34-input run-7 " \
       "layout (base 17 + history taps + both leaky integrals). This header " \
       "was generated from a checkpoint with a different input count - " \
       "either re-export the run-7 checkpoint, or port policy_build_obs() " \
       "to the layout the loaded checkpoint actually expects."
#endif

/* Chassis height the simulator reported while upright. Substituted for the
 * unmeasurable height input; see policy_build_obs(). */
#define POLICY_NOMINAL_HEIGHT 0.05f

/* Wheel radius (m) and half track width (m), measured (PARAMS.md) and baked
 * into real_robot.xml. Needed to turn encoder readings into the body-frame
 * velocity the policy expects. (The pre-run-1 their_robot.xml plant used
 * 0.05/0.08 here - wrong for this hardware by a wide margin; do not reuse
 * those numbers.) */
#define POLICY_WHEEL_RADIUS 0.0335f
#define POLICY_HALF_TRACK   0.0835f

/* Control period the run-7 policy was trained at: 200 Hz (train_real_robot.py,
 * FRAME_SKIP=5 on real_robot.xml's 1 ms timestep), matching the stock
 * firmware's MPU6050-interrupt loop. This is higher than the 80 Hz the
 * original their_robot.xml policy used - that plant's PID sweep (91/120
 * stable at 80 Hz, 24/120 at 40 Hz) does not carry over to this one, and has
 * not been re-run here. Run the loop from a hardware timer interrupt, not a
 * delay loop, and treat 200 Hz as the rate this policy was actually verified
 * at, not a floor with headroom below it. */
#define POLICY_CONTROL_HZ 200.0f
/* 1/POLICY_CONTROL_HZ, spelled as a division of two literals so the compiler
 * folds it to a constant at compile time - no runtime divide on a target
 * with no FPU. Used by the leaky integrals below. */
#define POLICY_DT (1.0f / POLICY_CONTROL_HZ)

/* Fall cutoff, matching FALL_ANGLE_LIMIT in train_real_robot.py: the firmware's
 * real 40 degree cut-out, not the 23 degrees the original policy trained to. */
#define POLICY_FALL_ANGLE_LIMIT 0.70f

/* --- observation layout: history buffer -----------------------------------
 *
 * A single frame cannot distinguish "oscillating about equilibrium" from
 * "drifting away", and this policy has to dither the PWM sign to synthesise
 * small average torques - a decision that depends on recent history, not the
 * present instant. Three signals (pitch, pitch rate, forward speed) are
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
 * is never asked to chase distance lost under a stale command. Unclipped.
 *
 * Heading (yaw-rate error, which integrates to a heading error): tau=30s,
 * deliberately long - this axis WANTS to remember over tens of seconds, since
 * driving the integral to zero is exactly "point where commanded". Clipped to
 * +/-0.2 rad because idle wander (~0.02 rad) and a real tracking error
 * (~1.5 rad) span two orders of magnitude and one linear weight cannot serve
 * both unclipped. Both leaky rather than absolute: the MPU6050 has no
 * magnetometer, so this only ever integrates the last few tau of gyro-z,
 * which is what the sensor actually gives you, and never depends on an
 * absolute heading the hardware cannot measure. */
#define POLICY_POS_DECAY      0.99750312f  /* exp(-dt / 2.0s) */
#define POLICY_POS_OBS_SCALE  10.0f
#define POLICY_YAW_DECAY      0.99983335f  /* exp(-dt / 30.0s) */
#define POLICY_YAW_OBS_SCALE  5.0f         /* 1 / POLICY_YAW_CLIP_RAD */
#define POLICY_YAW_CLIP_RAD   0.2f

/* --- motor drive: PWM, not torque -------------------------------------------
 *
 * The policy commands PWM directly, pre-deadband-compensation, exactly as the
 * firmware's own controller output is defined - there is no Kt/R torque model
 * involved on this path at all (that was the older, superseded torque policy;
 * see the removed policy_torque_to_duty() note in policy.c's history). Feed
 * policy_action_to_pwm()'s output into the SAME PWM_Ignore()-style deadband
 * add the stock firmware already applies downstream; do not add it twice.
 *
 * PWM_LIMIT/PWM_DEADBAND are per-build - re-measure and override rather than
 * trusting these (they match the sim's motor_model.py, not necessarily your
 * unit; the parameter sheet itself quotes 1480 fwd / 1455 rev for a different
 * build than the 1300 used here). */
#define POLICY_PWM_LIMIT     2800.0f
#define POLICY_PWM_DEADBAND  1300.0f
#define POLICY_PWM_USABLE    (POLICY_PWM_LIMIT - POLICY_PWM_DEADBAND)

#ifdef __cplusplus
extern "C" {
#endif

/* Zero the leaky integrals and prime the history buffer with HISTORY_LEN
 * copies of one real sample, exactly as train_real_robot.py does on an
 * episode reset (no real history exists yet, so repeat the current sample
 * rather than feeding zeros, which would look like a violent transient the
 * policy has to learn to ignore).
 *
 * Call this once at controller start-up, and again every time the robot is
 * re-armed after a fall - the leaky integrals otherwise carry state across
 * an event they were never trained to see across. */
void policy_reset_state(float pitch, float pitch_rate, float v_forward);

/* obs -> action, already clamped to +/-POLICY_ACTION_LIMIT (see
 * policy_weights.h). For the current checkpoint this is a PWM duty fraction
 * in [-1, 1], pre-deadband-compensation - feed it to policy_action_to_pwm(),
 * NOT to a torque-to-duty conversion. action[0] is the left wheel, action[1]
 * the right. */
void policy_infer(const float obs[POLICY_N_OBS], float action[POLICY_N_ACT]);

/* Assemble the full 34-input observation vector from sensors: the 17
 * instantaneous inputs, the history taps, and both leaky integrals (updated
 * as a side effect of this call - see policy.c). Must be called exactly once
 * per control tick, in the same order every fall-to-reset cycle as training:
 * policy_reset_state() once, then this once per tick from then on.
 *
 * Fills the three inputs the hardware cannot measure with the constants
 * validated in sim:
 *
 *   quat_wxyz     orientation from the IMU, w first, world-referenced
 *   wheel_angle_* accumulated encoder position, radians
 *   v_forward     body-frame forward speed, m/s (see policy_odometry)
 *   gyro_xyz      body angular rate, rad/s
 *   wheel_vel_*   encoder speed, rad/s
 *   cmd_*         your setpoints: m/s and rad/s
 */
void policy_build_obs(float obs[POLICY_N_OBS],
                      const float quat_wxyz[4],
                      float wheel_angle_l, float wheel_angle_r,
                      float v_forward,
                      const float gyro_xyz[3],
                      float wheel_vel_l, float wheel_vel_r,
                      float cmd_forward, float cmd_turn);

/* Preferred entry point on an MPU6050 (6-axis, no magnetometer).
 *
 * Takes roll and pitch directly - the two angles a gyro+accel fusion can hold
 * absolutely - and pins yaw to zero internally. Use this rather than
 * policy_build_obs() unless you have a magnetometer: MPU6050 yaw drifts
 * without bound, and the policy was measured to FALL at a 180 degree yaw
 * error (it tolerates 90 degrees fine, so the failure arrives late and looks
 * random). See policy.c for the measurements.
 *
 * Note this policy also carries its OWN bounded defence against yaw drift -
 * the heading-keeping integral above - but that corrects small, slow wander
 * around a held heading; it is not a substitute for keeping yaw pinned to
 * zero in the observation itself, which is what actually removes the 180
 * degree failure mode.
 */
void policy_build_obs_rp(float obs[POLICY_N_OBS],
                         float roll, float pitch,
                         float wheel_angle_l, float wheel_angle_r,
                         float v_forward,
                         const float gyro_xyz[3],
                         float wheel_vel_l, float wheel_vel_r,
                         float cmd_forward, float cmd_turn);

/* Forward speed from wheel encoders: the mean wheel rim speed. */
static inline float policy_odometry(float wheel_vel_l, float wheel_vel_r)
{
    return 0.5f * (wheel_vel_l + wheel_vel_r) * POLICY_WHEEL_RADIUS;
}

/* Network output (already clamped to +/-POLICY_ACTION_LIMIT by policy_infer())
 * -> PWM pulse count, PRE-deadband-compensation, matching the sim's
 * motor_model.py:action_to_pwm() exactly.
 *
 * This is the ENTIRE conversion - there is no motor model here, deliberately.
 * The policy was trained commanding PWM, not torque, specifically because the
 * team has no motor Kt/R to build a torque loop from; both unknowns are
 * already folded into the deadband and scale the firmware itself applies.
 * Feed this value into the stock firmware's own deadband-add step
 * (PWM_Ignore() in the reference firmware) exactly as any other controller
 * output would be - do not add the deadband twice, and do not run this
 * through a Kt/R torque conversion; that path belongs to the superseded
 * torque-output policy and would double-convert a value that is already PWM. */
static inline int policy_action_to_pwm(float action)
{
    if (action >  1.0f) action =  1.0f;
    if (action < -1.0f) action = -1.0f;
    return (int)(action * POLICY_PWM_USABLE);
}

/* Encoder counts -> wheel angle in radians.
 *
 * The JGB37-520's hall encoder sits on the MOTOR shaft, before the gearbox,
 * so one wheel revolution produces (ppr * 4 * G) counts in 4x quadrature
 * decoding - which is what a STM32 timer in Encoder Mode gives you by default
 * (TIM_ENCODERMODE_TI12). Typical ppr for this motor is 11, so a 1:30 box
 * yields 1320 counts per wheel revolution: plenty of resolution.
 *
 * CHECK YOUR VARIANT. The JGB37-520 ships in many gear ratios (1:30 through
 * 1:131) and ppr is occasionally 13 rather than 11.
 */
static inline float policy_counts_to_rad(long counts, float ppr,
                                          float gear_ratio, float quadrature)
{
    return (float)counts * 6.28318531f / (ppr * quadrature * gear_ratio);
}

#ifdef __cplusplus
}
#endif

#endif /* POLICY_H */
