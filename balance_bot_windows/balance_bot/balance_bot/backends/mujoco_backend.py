"""MuJoCo backend.

``MujocoCore`` subclasses :class:`BalanceCore` and overrides *only* the two
plant hooks, so the controller, the observation vector, the reward and the
episode logic are byte-for-byte the same as during training.  A policy
trained on the analytic model therefore runs here unchanged -- if it behaves
differently, that difference is genuine sim-to-sim transfer error and not an
artefact of two divergent code paths.

The pitch angle is recovered from the body quaternion rather than from a
joint, because in MuJoCo the chassis has a free joint (it can, and should,
fall over).
"""
from __future__ import annotations

import numpy as np

from ..dynamics import (make_state, wrap_pi,
                        IX, IY, IPSI, IPSID, IS, IV, ITH, ITHD, IPHI)
from ..env import BalanceCore
from ..mjcf import build_mjcf


def quat_to_rpy(q):
    """MuJoCo quaternion (w, x, y, z) -> roll, pitch, yaw."""
    w, x, y, z = q
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr, cosr)
    sinp = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny, cosy)
    return roll, pitch, yaw


class MujocoCore(BalanceCore):
    """Same environment, MuJoCo physics."""

    def __init__(self, *args, **kwargs):
        self._mj = None
        super().__init__(*args, **kwargs)

    # ------------------------------------------------------------------
    def _build_model(self):
        """(Re)compile the MJCF -- but only when something actually changed.

        A passive ``mujoco.viewer`` holds pointers into ``model``/``data``;
        recompiling on every reset would invalidate them and kill the 3D
        window.  Compiling is also ~10 ms, which is a lot inside a training
        loop.  So the model is cached against a signature of everything that
        can affect it.
        """
        import mujoco
        sig = (tuple(sorted(self.robot.__dict__.items())),
               self.arena.obstacles.tobytes(),
               self.arena.p.half_x, self.arena.p.half_y, self.sim.dt_sub)
        if self._mj is not None and sig == getattr(self, "_sig", None):
            return
        xml = build_mjcf(self.robot, self.arena, timestep=self.sim.dt_sub)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self._mj = mujoco
        self._bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                      "chassis")
        self._xml = xml
        self._sig = sig

    # ------------------------------------------------------------------
    def _plant_reset(self, start_xy, theta0, psi0, thd0):
        self._build_model()          # obstacles/params may have changed
        mj = self._mj
        mj.mj_resetData(self.model, self.data)

        # qpos layout for a free joint: [x y z qw qx qy qz] then the 2 wheels
        half = 0.5 * theta0
        cy, sy = np.cos(0.5 * psi0), np.sin(0.5 * psi0)
        cp, sp = np.cos(half), np.sin(half)
        # yaw * pitch (ZYX order, roll = 0)
        q = np.array([cy * cp, -sy * sp, cy * sp, sy * cp])
        self.data.qpos[0:3] = [start_xy[0], start_xy[1], self.robot.r_wheel]
        self.data.qpos[3:7] = q / np.linalg.norm(q)
        self.data.qvel[:] = 0.0
        self.data.qvel[4] = thd0          # angular velocity about body y
        mj.mj_forward(self.model, self.data)
        self._sync_state()

    # ------------------------------------------------------------------
    def _plant_step(self, tau_l, tau_r, wrench, slip):
        mj = self._mj
        # 这个钳位是**驱动器的电流限**，不是「tau_max 顺手拿来用」。
        # 电机直线本身允许 |tau| 到 2*stall：满占空比顶着一个以额定转速反转的
        # 轮子时，电枢两端是 12 V 加上反电动势，电流约为堵转电流的两倍。真实的
        # H 桥（和电池）给不出这个电流，也扛不住这个发热，所以出口按堵转力矩截。
        #
        # 2026-09-10 实测过放开：上限抬到 2*tau_max 后，空载默认从
        # 摆 0.06°/位 0.6mm/离地 0% 变成 摆 5.76°/位 192mm/离地 19.7%，
        # 停车振荡从收敛（0.04）变成发散（1.31）。抬到 1.5 倍也一样坏。
        # 静止时这个钳位根本不生效（顶限 0.0% 的拍），只在行驶中才起作用。
        #
        # This clamp is the DRIVER'S CURRENT LIMIT, not an incidental reuse of
        # tau_max.  The motor line itself permits |tau| up to 2*stall (full
        # duty against a wheel back-spinning at rated speed puts roughly twice
        # the stall current through the armature), which no real H-bridge will
        # deliver.  Relaxing it was measured and is much worse; at standstill
        # it never binds.
        grip = float(np.clip(1.0 - slip, 0.0, 1.0))
        self.data.ctrl[0] = np.clip(tau_l * grip, -self.robot.tau_max,
                                    self.robot.tau_max)
        self.data.ctrl[1] = np.clip(tau_r * grip, -self.robot.tau_max,
                                    self.robot.tau_max)

        # external wrench, body frame -> world frame
        psi = self.state[IPSI]
        fx_w = wrench.fx * np.cos(psi) - wrench.fy * np.sin(psi)
        fy_w = wrench.fx * np.sin(psi) + wrench.fy * np.cos(psi)
        self.data.xfrc_applied[self._bid, :] = 0.0
        self.data.xfrc_applied[self._bid, 0] = fx_w
        self.data.xfrc_applied[self._bid, 1] = fy_w
        self.data.xfrc_applied[self._bid, 5] = wrench.yaw_moment

        for _ in range(self.sim.phys_per_ctrl):
            mj.mj_step(self.model, self.data)
            self._sync_state()
            self._on_substep(self.sim.dt_sub)

    # ------------------------------------------------------------------
    def _sync_state(self):
        d = self.data
        x, y, _ = d.qpos[0:3]
        roll, pitch, yaw = quat_to_rpy(d.qpos[3:7])
        vx, vy = d.qvel[0], d.qvel[1]
        # forward speed = world velocity projected on the heading
        v = vx * np.cos(yaw) + vy * np.sin(yaw)
        # angular velocity in the world frame -> body pitch / yaw rates
        wx, wy, wz = d.qvel[3], d.qvel[4], d.qvel[5]
        pitch_rate = -wx * np.sin(yaw) + wy * np.cos(yaw)

        s = self.state if self.state is not None else make_state()
        s[IX], s[IY] = x, y
        s[IPSI] = wrap_pi(yaw)
        s[IPSID] = wz
        s[IV] = v
        s[ITH] = pitch
        s[ITHD] = pitch_rate
        s[IPHI] = 0.5 * (d.qpos[7] + d.qpos[8]) if len(d.qpos) > 8 else 0.0
        self.state = s
        self.roll = roll

    # ------------------------------------------------------------------
    @property
    def xml(self):
        return self._xml
