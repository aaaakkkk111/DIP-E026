"""自适应增益环境：策略按情境改 PID，PPO 训它。
Adaptive-gain environment: the policy retunes the PID from what it can sense.

和 `STM32Env` 的关系
--------------------
`STM32Env` 已经把「策略每拍改增益」这条链路做完了：有界乘性增益空间、变化率
限制、七项线性奖励。这里只补两件它没有的事：

1. **情境**（`scenarios.py`）：负重、坡道、台阶跌落、冲击，逐局随机。
2. **情境特征**（`firmware/features.py`）：8 个 IIR 特征。没有它们，策略看到
   的只有瞬时状态，**分辨不出**「压了 3 kg」和「在上坡」——两者的瞬时俯仰角
   长得一样。

`STM32Env` already does per-tick gain modulation; this adds the situations and
the eight features needed to tell them apart.

设计约束：**能上车**
--------------------
观测里的每一个数，板子上都拿得到（卡尔曼角度、陀螺、加速度计两轴、编码器、
速度环积分、自己写的 CCR）。没有真值状态、没有 MuJoCo 专有量。网络默认
32x32，q15 量化后约 3.8 KB，F103RC 上余量 200 KB。

Every observation is available on the MCU; no ground truth, no simulator-only
quantity.  The default 32x32 net is ~3.8 KB as q15.
"""
from __future__ import annotations

import numpy as np
from gymnasium import spaces

from .dynamics import ITH, ITHD, IV
from .firmware.features import N_FEATURES, SituationFeatures
from .scenarios import SCENARIOS, ScenarioRunner, sample_plan
from .stm32_env import STM32Env

# 摔倒以外，这个环境额外关心的两件事 / two extra reward terms
W_SLOPE_HOLD = 0.5      # 坡上别往下溜 / do not slide back down a slope
W_LAND = 0.02           # 落地那一下的冲击 / the landing spike


class STM32AdaptEnv(STM32Env):
    """PID 自适应：观测 = 状态 + 8 情境特征 + 上一步动作。
    Observation = state + eight situation features + last action."""

    def __init__(self, *args, scenarios=SCENARIOS,
                 scenario_difficulty: float | None = None, **kw):
        super().__init__(*args, **kw)
        if self.backend != "mujoco":
            # 坡道靠改重力、跌落靠改 qpos，两个都只有 MuJoCo 后端有。
            raise ValueError(
                "STM32AdaptEnv 需要 mujoco 后端（坡道改重力、跌落改 qpos）/ "
                "needs the mujoco backend")
        self.scenarios = tuple(scenarios)
        self.scenario_difficulty = scenario_difficulty
        self.feat = SituationFeatures()
        self.runner = ScenarioRunner(self.core)
        self._feat = SituationFeatures.zeros()
        self._plan = None
        base = self.observation_space.shape[0]
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(base + N_FEATURES,),
            dtype=np.float32)

    # ------------------------------------------------------------------
    def _build_core(self, seed=None):
        super()._build_core(seed=seed)
        # 换 core 之后 runner 要跟着换（_build_core 在 __init__ 里先跑）
        if hasattr(self, "runner"):
            self.runner = ScenarioRunner(self.core)

    def _get_obs(self) -> np.ndarray:
        return np.concatenate([super()._get_obs(), self._feat]).astype(np.float32)

    # ------------------------------------------------------------------
    def _update_features(self):
        """从**固件看得见的量**更新特征。
        Update the features from what the firmware can see."""
        core = self.core
        # 复位后这两个还是 None（固件一拍都没跑过），当 0 处理。
        # Both are None right after a reset; treat them as zero.
        gyro_dps = float(getattr(core, "_gyro_raw", 0.0) or 0.0) / 16.4
        angle_deg = float(getattr(core, "_angle_filt", 0.0) or 0.0)
        try:
            a_fwd, a_up, _ = core._imu_raw(core.sim.dt_ctrl)
        except Exception:
            a_fwd, a_up = 0.0, 9.81
        v_enc = float(core.state[IV])
        ccr = float(np.mean(core.fw_ccr)) if getattr(core, "fw_ccr", None) is not None else 0.0
        vint = float(getattr(getattr(core.fw, "st", None), "encoder_integral", 0.0))
        self._feat = self.feat.update(gyro_dps, a_fwd, a_up, angle_deg,
                                      v_enc, ccr, vint)

    # ------------------------------------------------------------------
    def reset(self, seed=None, options=None):
        obs, info = super().reset(seed=seed, options=options)
        d = (self.difficulty if self.scenario_difficulty is None
             else self.scenario_difficulty)
        self._plan = sample_plan(self._rng, difficulty=d,
                                 allowed=self.scenarios,
                                 episode_seconds=self.episode_seconds)
        self.runner.reset(self._plan)
        self.feat.reset()
        self._update_features()
        info = dict(info or {})
        info["scenario"] = self._plan.name
        return self._get_obs(), info

    def step(self, action):
        obs, reward, term, trunc, info = super().step(action)
        fired = self.runner.tick(float(self.core.t))
        self._update_features()

        # 坡上别往下溜：有坡时按"速度和下坡方向同号"罚。平地上这一项恒为 0。
        # On a slope, penalise sliding downhill; zero on flat ground.
        slope = self.runner.slope_deg
        if slope != 0.0:
            v = float(self.core.state[IV])
            downhill = -np.sign(slope)          # 坡度为正 = 上坡，下溜是 -v
            reward -= W_SLOPE_HOLD * max(0.0, downhill * v)

        # 落地冲击：|陀螺| 尖峰按量罚，逼策略在悬空时就把增益准备好
        # The landing spike, so the policy prepares its gains mid-air.
        if "drop" in (self._plan.fired if self._plan else set()):
            reward -= W_LAND * abs(float(self.core.state[ITHD]))

        info = dict(info)
        info["scenario"] = self._plan.name if self._plan else "none"
        info["slope_deg"] = slope
        info["features"] = self._feat
        if fired:
            info["fired"] = fired
        return self._get_obs(), float(reward), term, trunc, info


class STM32AdaptEnvFactory:
    """可 pickle 的工厂，供 SubprocVecEnv 用。
    Picklable factory for SubprocVecEnv."""

    def __init__(self, rank: int, seed: int, episode_seconds: float = 20.0,
                 scenarios=SCENARIOS, imu_filter: str = "kalman",
                 command_prob: float = 0.3, per_gain_span: bool = True,
                 penalise_chatter: bool = True):
        self.rank = rank
        self.seed = seed
        self.episode_seconds = episode_seconds
        self.scenarios = tuple(scenarios)
        self.imu_filter = imu_filter
        self.command_prob = command_prob
        self.per_gain_span = per_gain_span
        self.penalise_chatter = penalise_chatter

    def __call__(self):
        from .firmware.twin_baseline import MODE_STM32_PID
        return STM32AdaptEnv(
            firmware=MODE_STM32_PID, backend="mujoco", randomize=True,
            episode_seconds=self.episode_seconds, imu_filter=self.imu_filter,
            command_prob=self.command_prob, per_gain_span=self.per_gain_span,
            penalise_chatter=self.penalise_chatter,
            scenarios=self.scenarios, seed=self.seed + self.rank)
