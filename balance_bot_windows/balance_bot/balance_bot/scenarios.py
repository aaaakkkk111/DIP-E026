"""训练情境：负重、坡道、台阶跌落、冲击。
Training scenarios: payload, slope, step drop, impact.

用户要的是「不同情境下自己切 PID」，所以训练环境必须真的把这些情境造出来，
而不是只改一个难度标量。这里每个情境都落到**物理上**：

* 负重  —— 改底盘的质量、质心和惯量（和 GUI 载重滑块同一条路径）
* 坡道  —— **转重力向量**，不建坡道几何。对滚动的车来说，绕 y 轴转重力和把
            地面倾斜是等价的，省掉一套接触几何和它的标定。代价：视觉上车还
            是在平地上，而且这个等价只对「地面是平面」成立。
* 跌落  —— 把车整体抬高再放手，落地靠接触模型自己算
* 冲击  —— 复用 DisturbanceModel.fire_impulse，和 GUI 的踢击是同一条路径

Each scenario is physical, not a difficulty scalar: payload edits the chassis
mass/COM/inertia, slope rotates gravity (equivalent to tilting a flat floor and
avoids an uncalibrated ramp geometry), the drop teleports the car up and lets
the contact model do the landing, and the impulse reuses the same path as the
UI kick button.

**一句话免责声明**：孪生只在模式 1 静止那个角落标定过（轮速 5.5%、力矩权限
2.7%），这四个情境全在角落外面。见 TWIN_BASELINE.md 第 10.6 节——在这里训出来
的策略，趋势可信，绝对值不可信。
The twin is calibrated only in the standstill corner; these four scenarios all
live outside it (see TWIN_BASELINE.md 10.6).  Trust trends, not absolutes.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

G = 9.81


# ----------------------------------------------------------------------
def set_payload_live(core, kg: float):
    """立刻改载重，不用重建模型（重建会丢当前状态）。
    Apply a payload in place; rebuilding the model would drop the state."""
    core.set_payload(float(kg))
    base = core.robot_nominal
    core.robot = (base.with_payload(float(kg), core.payload_height)
                  if kg > 0 else base)
    p = core.robot
    if hasattr(core, "dyn"):
        from .dynamics import BalanceBotDynamics
        core.dyn = BalanceBotDynamics(p)
    if hasattr(core, "pid"):
        core.pid.tau_max = p.tau_max
    mj = getattr(core, "_mj", None)
    model = getattr(core, "model", None)
    if mj is None or model is None:
        return
    bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "chassis")
    if bid < 0:
        return
    model.body_mass[bid] = p.m_body
    model.body_ipos[bid] = (0.0, 0.0, p.l_com)
    hw = 0.5 * p.track
    i_diam = 0.25 * p.m_wheel * p.r_wheel ** 2
    izz = max(p.I_yaw - 2.0 * (p.m_wheel * hw ** 2 + i_diam), 1e-4)
    model.body_inertia[bid] = (p.I_body, p.I_body, izz)


def set_slope_live(core, deg: float):
    """把重力绕 y 轴转 ``deg`` 度：正值 = 车头朝上的上坡。
    Rotate gravity about y; positive means nose-up (climbing)."""
    model = getattr(core, "model", None)
    if model is None:
        return False
    a = np.radians(float(deg))
    model.opt.gravity[:] = (G * np.sin(a), 0.0, -G * np.cos(a))
    return True


def lift_car(core, height_m: float):
    """把车整体抬高 ``height_m`` 并清零线速度，松手自由落体。
    Teleport the car up and zero its linear velocity: a free fall."""
    data = getattr(core, "data", None)
    if data is None or len(data.qpos) < 3:
        return False
    data.qpos[2] += float(height_m)
    data.qvel[0:3] = 0.0
    mj = core._mj
    mj.mj_forward(core.model, data)
    core._sync_state()
    return True


# ----------------------------------------------------------------------
@dataclass
class ScenarioPlan:
    """一局的情境安排。所有时间都是秒。
    One episode's scenario; all times in seconds."""
    name: str = "none"
    payload_kg: float = 0.0
    payload_at: float = -1.0          # <0 = 开局就挂上 / attached at reset
    slope_deg: float = 0.0
    slope_at: float = -1.0
    slope_ramp: float = 1.0           # 坡度渐变时长 / ramp duration
    drop_m: float = 0.0
    drop_at: float = -1.0
    kick_n: float = 0.0
    kick_at: float = -1.0
    kick_dir: float = 0.0
    kick_ms: float = 50.0
    fired: set = field(default_factory=set)


SCENARIOS = ("none", "payload", "slope", "drop", "kick", "mixed")


def sample_plan(rng: np.random.Generator, difficulty: float = 1.0,
                allowed=SCENARIOS, episode_seconds: float = 20.0
                ) -> ScenarioPlan:
    """按难度抽一局。difficulty 0 -> 最温和，1 -> 表里的上限。
    Sample one episode; difficulty scales every magnitude."""
    d = float(np.clip(difficulty, 0.0, 1.0))
    name = str(rng.choice(list(allowed)))
    p = ScenarioPlan(name=name)
    t_lo, t_hi = 2.0, max(3.0, episode_seconds - 4.0)

    def when():
        return float(rng.uniform(t_lo, t_hi))

    if name in ("payload", "mixed"):
        # 0.5~4 kg（官方额定 4 kg）。一半开局就挂上，一半中途压上去——后者
        # 才逼策略去「发现」负载变了。
        p.payload_kg = float(rng.uniform(0.5, 0.5 + 3.5 * d))
        p.payload_at = -1.0 if rng.random() < 0.5 else when()
    if name in ("slope", "mixed"):
        # ±(2~12)°，上坡下坡各一半
        mag = float(rng.uniform(2.0, 2.0 + 10.0 * d))
        p.slope_deg = mag if rng.random() < 0.5 else -mag
        p.slope_at = when()
        p.slope_ramp = float(rng.uniform(0.5, 2.0))
    if name in ("drop", "mixed"):
        # 1~5 cm。5 cm 已经是"从台阶上摔下来"的量级，落地冲击很大。
        p.drop_m = float(rng.uniform(0.01, 0.01 + 0.04 * d))
        p.drop_at = when()
    if name in ("kick", "mixed"):
        # 2~9 N x 50 ms。孪生实测原厂模式 1 扛得住 8.5 N、9 N 倒，所以上限
        # 压在 9：再往上是"物理上也救不回来"，不是策略的问题。
        p.kick_n = float(rng.uniform(2.0, 2.0 + 7.0 * d))
        p.kick_at = when()
        p.kick_dir = float(rng.choice([0.0, np.pi]))
        p.kick_ms = float(rng.uniform(30.0, 80.0))
    return p


class ScenarioRunner:
    """把 ScenarioPlan 逐拍落到孪生上。
    Applies a plan to the core, tick by tick."""

    def __init__(self, core):
        self.core = core
        self.plan = ScenarioPlan()
        self._slope_now = 0.0

    def reset(self, plan: ScenarioPlan):
        self.plan = plan
        self.plan.fired = set()
        self._slope_now = 0.0
        set_slope_live(self.core, 0.0)
        set_payload_live(self.core, plan.payload_kg if plan.payload_at < 0 else 0.0)

    def tick(self, t: float):
        """在每个固件拍之后调用。返回这一拍触发了什么（给日志用）。
        Call after each firmware tick; returns whatever fired, for logging."""
        p = self.plan
        fired = []
        if p.payload_at >= 0 and t >= p.payload_at and "payload" not in p.fired:
            set_payload_live(self.core, p.payload_kg)
            p.fired.add("payload"); fired.append("payload")
        if p.drop_at >= 0 and t >= p.drop_at and "drop" not in p.fired:
            lift_car(self.core, p.drop_m)
            p.fired.add("drop"); fired.append("drop")
        if p.kick_at >= 0 and t >= p.kick_at and "kick" not in p.fired:
            self.core.dist.fire_impulse(force=p.kick_n, direction=p.kick_dir,
                                        duration=p.kick_ms / 1000.0)
            p.fired.add("kick"); fired.append("kick")
        if p.slope_at >= 0 and t >= p.slope_at and p.slope_deg != 0.0:
            # 渐变，不要一拍跳到 12°——那等于又一次冲击，两个情境就混在一起了
            frac = min(1.0, (t - p.slope_at) / max(p.slope_ramp, 1e-3))
            want = p.slope_deg * frac
            if abs(want - self._slope_now) > 1e-6:
                set_slope_live(self.core, want)
                self._slope_now = want
            if frac >= 1.0 and "slope" not in p.fired:
                p.fired.add("slope"); fired.append("slope")
        return fired

    @property
    def slope_deg(self) -> float:
        return self._slope_now
