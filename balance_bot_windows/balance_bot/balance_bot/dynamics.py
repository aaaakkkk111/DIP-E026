"""Analytic dynamics of a two-wheeled inverted pendulum robot.

This is the *fast* plant used for PPO training (roughly 10^5 steps/s on one
CPU core).  The MuJoCo and Gazebo backends expose the same interface so a
policy trained here can be dropped straight into either of them.

Model
-----
Planar longitudinal dynamics coupled with a decoupled yaw channel.

State (9):
    X, Y      world position of the wheel-axle midpoint        [m]
    psi       heading (yaw)                                    [rad]
    psi_dot   yaw rate                                         [rad/s]
    s         distance travelled along the heading             [m]
    v         forward speed                                    [m/s]
    theta     pitch from vertical, + = leaning forward         [rad]
    th_dot    pitch rate                                       [rad/s]
    phi       average wheel angle                              [rad]

Equations of motion (longitudinal), with
    A = 2 m_w + m_b + 2 I_w / r^2 ,  B = m_b l ,  C = I_b + m_b l^2

    [ A          B cos(th) ] [ v_dot  ]   [ tau_sum / r + B sin(th) th_dot^2 + F_ext - drag ]
    [ B cos(th)  C         ] [ th_ddot] = [ m_b g l sin(th) - tau_sum + M_ext - b_p th_dot  ]

The sign convention makes a *positive* wheel torque drive the robot forward
and pitch the body backwards, which reproduces the non-minimum-phase
behaviour that makes these robots interesting to control.

Yaw:
    I_z psi_ddot = (tau_R - tau_L) * (track/2) / r - b_yaw psi_dot + M_ext_yaw
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .params import RobotParams, GRAVITY

# state vector indices
IX, IY, IPSI, IPSID, IS, IV, ITH, ITHD, IPHI = range(9)
STATE_DIM = 9


@dataclass
class ExternalWrench:
    """Disturbance applied to the body, expressed in the *body* frame."""
    fx: float = 0.0        # N, forward
    fy: float = 0.0        # N, lateral (turns into a yaw moment)
    pitch_moment: float = 0.0
    yaw_moment: float = 0.0

    def is_zero(self) -> bool:
        return (self.fx == 0.0 and self.fy == 0.0
                and self.pitch_moment == 0.0 and self.yaw_moment == 0.0)


def make_state(x=0.0, y=0.0, psi=0.0, theta=0.0, v=0.0,
               th_dot=0.0, psi_dot=0.0) -> np.ndarray:
    s = np.zeros(STATE_DIM)
    s[IX], s[IY], s[IPSI] = x, y, psi
    s[IPSID], s[IV], s[ITH], s[ITHD] = psi_dot, v, theta, th_dot
    return s


class BalanceBotDynamics:
    """Continuous dynamics + RK4 integrator for the analytic plant."""

    def __init__(self, params: RobotParams | None = None):
        self.p = params or RobotParams()

    # ------------------------------------------------------------------
    def derivative(self, state: np.ndarray, tau_l: float, tau_r: float,
                   wrench: ExternalWrench | None = None,
                   slip: float = 0.0) -> np.ndarray:
        p = self.p
        w = wrench or ExternalWrench()

        psi_dot = state[IPSID]
        v = state[IV]
        th = state[ITH]
        thd = state[ITHD]

        # --- traction loss ------------------------------------------------
        grip = float(np.clip(1.0 - slip, 0.0, 1.0))
        tau_l_eff = tau_l * grip
        tau_r_eff = tau_r * grip
        tau_sum = tau_l_eff + tau_r_eff

        # --- longitudinal 2x2 system -------------------------------------
        cos_th = np.cos(th)
        sin_th = np.sin(th)

        A = p.A
        B = p.B
        C = p.C

        drag = (p.b_wheel * v / p.r_wheel ** 2
                + p.rolling_mu * (p.m_body + 2 * p.m_wheel) * GRAVITY
                * np.tanh(v / 0.02))

        rhs1 = tau_sum / p.r_wheel + B * sin_th * thd ** 2 + w.fx - drag
        rhs2 = (p.m_body * GRAVITY * p.l_com * sin_th - tau_sum
                + w.pitch_moment - p.b_pitch * thd)

        # solve [[A, B c], [B c, C]] @ [v_dot, th_ddot] = [rhs1, rhs2]
        det = A * C - (B * cos_th) ** 2
        det = det if abs(det) > 1e-12 else 1e-12
        v_dot = (C * rhs1 - B * cos_th * rhs2) / det
        th_ddot = (A * rhs2 - B * cos_th * rhs1) / det

        # --- yaw ----------------------------------------------------------
        yaw_torque = ((tau_r_eff - tau_l_eff) * (0.5 * p.track) / p.r_wheel
                      + w.yaw_moment + w.fy * p.l_com
                      - p.b_yaw * psi_dot)
        psi_ddot = yaw_torque / p.I_yaw

        d = np.zeros(STATE_DIM)
        d[IX] = v * np.cos(state[IPSI])
        d[IY] = v * np.sin(state[IPSI])
        d[IPSI] = psi_dot
        d[IPSID] = psi_ddot
        d[IS] = v
        d[IV] = v_dot
        d[ITH] = thd
        d[ITHD] = th_ddot
        d[IPHI] = v / p.r_wheel
        return d

    # ------------------------------------------------------------------
    def step(self, state: np.ndarray, tau_l: float, tau_r: float, dt: float,
             wrench: ExternalWrench | None = None,
             slip: float = 0.0) -> np.ndarray:
        """One RK4 step."""
        p = self.p
        # motor back-emf: torque falls off as the wheel approaches free speed
        wheel_omega = state[IV] / p.r_wheel
        derate = float(np.clip(
            1.0 - abs(wheel_omega) / max(p.wheel_speed_max, 1e-6), 0.05, 1.0))
        tl = float(np.clip(tau_l, -p.tau_max, p.tau_max)) * derate
        tr = float(np.clip(tau_r, -p.tau_max, p.tau_max)) * derate

        f = lambda s: self.derivative(s, tl, tr, wrench, slip)
        k1 = f(state)
        k2 = f(state + 0.5 * dt * k1)
        k3 = f(state + 0.5 * dt * k2)
        k4 = f(state + dt * k3)
        nxt = state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        nxt[IPSI] = wrap_pi(nxt[IPSI])
        return nxt

    # ------------------------------------------------------------------
    def linearize(self):
        """Longitudinal state-space linearisation about the upright point.

        Returns (Ac, Bc) for x = [v, theta, theta_dot], u = tau_sum.
        Handy for sanity-checking the nominal PID gains against LQR.
        """
        p = self.p
        A, B, C = p.A, p.B, p.C
        det = A * C - B ** 2
        # v_dot   = ( C*(tau/r) - B*(m g l th - tau) ) / det
        # th_ddot = ( A*(m g l th - tau) - B*(tau/r) ) / det
        mgl = p.m_body * GRAVITY * p.l_com
        Ac = np.zeros((3, 3))
        Ac[0, 1] = -B * mgl / det
        Ac[1, 2] = 1.0
        Ac[2, 1] = A * mgl / det
        Bc = np.zeros((3, 1))
        Bc[0, 0] = (C / p.r_wheel + B) / det
        Bc[2, 0] = (-A - B / p.r_wheel) / det
        return Ac, Bc


def wrap_pi(a):
    """Wrap an angle into (-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi
