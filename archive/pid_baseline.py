"""Classical cascade PID baseline on the teammate's robot model.

Ports the control structure from DIP-E026 (branch Non-harness-JiangLi),
firmware/controllers.py -- balance PD + velocity PI + turn, summed into the
motor command, exactly as the STM32 firmware does:

    balance  = Kp_bal * (pitch - mid_angle) + Kd_bal * pitch_rate
    velocity = Kp_vel * v_err + Ki_vel * integral(v_err)
    turn     = Kp_turn * yaw_err
    motor_l  = balance + velocity + turn
    motor_r  = balance + velocity - turn

Their firmware gains are in PWM-counts-per-encoder-count and do not transfer
to a torque-controlled model, so gains here are tuned in physical units (Nm,
rad, rad/s) against their MuJoCo model.

Purpose: determine whether a classical controller can hold a commanded
forward velocity while balancing on this plant. If it can, the plant supports
sustained driving and the remaining problem is purely RL training.
"""
import numpy as np
import mujoco

XML = "their_robot.xml"
TAU_MAX = 0.6          # Nm per wheel, from RobotParams.tau_max
CONTROL_HZ = 200.0     # firmware runs its loops at 200 Hz


class CascadePID:
    """Outer velocity PI -> clamped pitch setpoint -> inner balance PD.

    The firmware sums its velocity term straight into the motor command,
    which works inside its own PWM/encoder unit scaling. Ported directly into
    physical torque units that arrangement is positive feedback and diverges
    in <0.1 s (wheels roll back -> velocity error grows -> rolls back harder).
    The equivalent stable formulation, and the standard one for this plant, is
    an explicit cascade: the velocity loop asks for a *lean*, bounded by
    PITCH_TARGET_LIMIT, and the faster balance loop tracks that lean. Bounding
    the setpoint is what prevents the runaway.
    """

    def __init__(self, kp_bal, kd_bal, kp_vel, ki_vel, kp_turn=0.5,
                 integral_limit=1.0, vel_lpf=0.9, pitch_target_limit=0.20):
        self.kp_bal, self.kd_bal = kp_bal, kd_bal
        self.kp_vel, self.ki_vel = kp_vel, ki_vel
        self.kp_turn = kp_turn
        self.integral_limit = integral_limit
        self.vel_lpf = vel_lpf
        self.pitch_target_limit = pitch_target_limit
        self.reset()

    def reset(self):
        self.v_integral = 0.0
        self.v_filt = 0.0
        self.pitch_target = 0.0

    def __call__(self, pitch, pitch_rate, v_forward, yaw_rate, dt,
                 target_v=0.0, target_yaw=0.0):
        # --- outer loop: velocity error -> desired lean -------------------
        self.v_filt = self.vel_lpf * self.v_filt + (1 - self.vel_lpf) * v_forward
        v_err = target_v - self.v_filt
        self.v_integral = float(np.clip(self.v_integral + v_err * dt,
                                        -self.integral_limit, self.integral_limit))
        # Verified empirically on this model: POSITIVE pitch leans forward and
        # a held lean of +0.05 rad cruises at +1.17 m/s, so a positive velocity
        # error asks for a positive lean.
        self.pitch_target = float(np.clip(
            self.kp_vel * v_err + self.ki_vel * self.v_integral,
            -self.pitch_target_limit, self.pitch_target_limit))

        # --- inner loop: track that lean ----------------------------------
        balance = self.kp_bal * (pitch - self.pitch_target) + self.kd_bal * pitch_rate
        turn = self.kp_turn * (yaw_rate - target_yaw)

        motor_l = balance + turn
        motor_r = balance - turn
        return (float(np.clip(motor_l, -TAU_MAX, TAU_MAX)),
                float(np.clip(motor_r, -TAU_MAX, TAU_MAX)))


def decode(d):
    """pitch, pitch_rate, body-frame forward velocity, yaw rate."""
    qw, qx, qy, qz = d.qpos[3:7]
    # pitch from quaternion (rotation about body y)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    c, s = np.cos(-yaw), np.sin(-yaw)
    vx, vy = d.qvel[0], d.qvel[1]
    v_forward = c * vx - s * vy
    return pitch, d.qvel[4], v_forward, d.qvel[5]


def rollout(gains, target_v=0.0, seconds=20.0, sign=1.0, report=False, kick=0.0):
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    d.qvel[4] = kick          # perturbation, so balancing is a real test
    mujoco.mj_forward(m, d)
    pid = CascadePID(*gains)

    steps = int(seconds / m.opt.timestep)
    ctrl_every = max(1, int(round((1.0 / CONTROL_HZ) / m.opt.timestep)))
    dt = ctrl_every * m.opt.timestep
    tl = tr = 0.0
    vs, pitches = [], []
    fell_at = None

    for i in range(steps):
        pitch, pitch_rate, v_fwd, yaw_rate = decode(d)
        if i % ctrl_every == 0:
            tl, tr = pid(pitch, pitch_rate, v_fwd, yaw_rate, dt, target_v=target_v)
        d.ctrl[0], d.ctrl[1] = sign * tl, sign * tr
        mujoco.mj_step(m, d)
        vs.append(v_fwd)
        pitches.append(pitch)
        if abs(pitch) > 0.6:
            fell_at = i * m.opt.timestep
            break

    vs = np.array(vs)
    settled = vs[len(vs) // 2:] if len(vs) > 10 else vs
    out = dict(fell_at=fell_at,
               survived=fell_at is None,
               v_mean_settled=float(settled.mean()) if len(settled) else 0.0,
               v_err=float(abs(settled.mean() - target_v)) if len(settled) else 9.9,
               max_pitch=float(np.abs(pitches).max()) if pitches else 9.9)
    if report:
        print(f"    target={target_v:+.2f}  survived={out['survived']}"
              f"  v_settled={out['v_mean_settled']:+.3f}"
              f"  err={out['v_err']:.3f}  max|pitch|={out['max_pitch']:.3f}"
              + ("" if out['survived'] else f"  FELL at {fell_at:.2f}s"))
    return out
