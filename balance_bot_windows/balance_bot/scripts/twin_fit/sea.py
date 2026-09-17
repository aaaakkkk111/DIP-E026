# -*- coding: utf-8 -*-
"""串联弹性传动：电机转子 --(扭转弹簧 k、阻尼 c、间隙 b)--> 轮子。

证据（全部来自真车录数）：
  - kd=48 静止：车身在晃（陀螺 8.6 °/s），编码器几乎不动（0.39 计数/拍）
    -> 车身相对轮子在摆，电机轴没转
  - 闭环扫频 12.5 Hz 有轻阻尼共振，相位 11->16 Hz 转 150°
  - kd=120 极限环锁在 13.96 Hz
刚性孪生里电机力矩直接作用在轮子上，没有这个自由度。

实现（每个物理子步 1.25 ms）：
  delta = phi - theta_w                    转子相对轮子的扭转角
  delta_e = sign(delta)*max(|delta|-b, 0)  扣掉间隙
  tau_s = k*delta_e + c*(w_m - w_w)        （在间隙里时阻尼也为 0）
  轮子关节力矩 = tau_s
  I_r * dw_m/dt = tau_m - tau_s            阻尼项隐式积分，保证稳定
反电动势用转子转速 w_m；编码器读转子角 phi（编码器装在电机轴上）。
转子惯量从 MuJoCo 轮子里拿掉，挪到这里，避免算两次。
"""
import numpy as np


def install(core, k, c, bl_rad, I_r, drive_limit=0.40, inner=5):
    mj = core._mj
    S = dict(phi=None, wm=None)

    def ensure():
        if S["phi"] is None:
            S["phi"] = np.array([core.data.qpos[7], core.data.qpos[8]], dtype=float)
            S["wm"] = np.array([core.data.qvel[6], core.data.qvel[7]], dtype=float)

    orig_reset = core.reset

    def reset(*a, **kw):
        S["phi"] = None
        out = orig_reset(*a, **kw)
        S["phi"] = None
        return out

    def wheel_angles():
        if not hasattr(core, "data"):
            return 0.0, 0.0
        ensure()
        return float(S["phi"][0]), float(S["phi"][1])

    def wheel_speeds():
        if not hasattr(core, "data"):
            return 0.0, 0.0
        ensure()
        return float(S["wm"][0]), float(S["wm"][1])

    def plant_step(tau_l, tau_r, wrench, slip):
        ensure()
        tmax = drive_limit          # 驱动器电流限，作用在电机力矩上
        tm = np.clip(np.array([tau_l, tau_r], dtype=float), -tmax, tmax)
        from balance_bot.dynamics import IPSI
        psi = core.state[IPSI]
        fx_w = wrench.fx * np.cos(psi) - wrench.fy * np.sin(psi)
        fy_w = wrench.fx * np.sin(psi) + wrench.fy * np.cos(psi)
        core.data.xfrc_applied[core._bid, :] = 0.0
        core.data.xfrc_applied[core._bid, 0] = fx_w
        core.data.xfrc_applied[core._bid, 1] = fy_w
        core.data.xfrc_applied[core._bid, 5] = wrench.yaw_moment
        H = core.sim.dt_sub
        h = H / inner
        core.model.opt.timestep = h     # 轮子惯量只有 1.4e-5，配弹簧必须细分步长
        for _sub in range(core.sim.phys_per_ctrl * inner):
            th = np.array([core.data.qpos[7], core.data.qpos[8]])
            ww = np.array([core.data.qvel[6], core.data.qvel[7]])
            dl = S["phi"] - th
            if bl_rad > 0:
                de = np.sign(dl) * np.maximum(np.abs(dl) - bl_rad, 0.0)
                eng = (np.abs(dl) > bl_rad).astype(float)
            else:
                de, eng = dl, np.ones(2)
            ts = k * de + c * eng * (S["wm"] - ww)
            core.data.ctrl[0] = ts[0]
            core.data.ctrl[1] = ts[1]
            mj.mj_step(core.model, core.data)
            if (_sub + 1) % inner == 0:
                core._sync_state()
                core._on_substep(H)
            # 转子：阻尼隐式
            num = S["wm"] + h * (tm - k * de + c * eng * ww) / I_r
            S["wm"] = num / (1.0 + h * c * eng / I_r)
            S["phi"] = S["phi"] + h * S["wm"]

    core.reset = reset
    core._wheel_angles = wheel_angles
    core._wheel_speeds = wheel_speeds
    core._plant_step = plant_step
    return S
