"""Lightweight runtime policy loader and forward inference for STM32 Balance Car.

Requires only NumPy for inference -- no PyTorch or heavy dependencies needed.
"""
from __future__ import annotations

import os
import numpy as np
from .stm32_env import STM32GainSpace, OBS_POS_SCALE
from .firmware.twin_baseline import MODE_STM32_HYBRID
from .dynamics import IV, IPSID, ITH, ITHD
from .firmware.twin_baseline import MODE_STM32_LQR


class STM32InferencePolicy:
    """NumPy-only MLP inference for the exported policy."""
    def __init__(self, npz_path: str):
        data = np.load(npz_path, allow_pickle=True)
        self.w1 = data["w1"]
        self.b1 = data["b1"]
        self.w2 = data["w2"]
        self.b2 = data["b2"]
        self.w_out = data["w_out"]
        self.b_out = data["b_out"]
        self.firmware = str(data["firmware"])
        # Which attitude filter the policy was trained against.  A policy
        # trained on one and run on the other is seeing a different
        # observation distribution -- see TWIN_BASELINE.md 2, where the choice moves
        # the PID baseline by 45 %.  Files written before this field existed
        self.imu_filter = str(data["imu_filter"]) if "imu_filter" in data             else "kalman"
        # Provenance, written by scripts/train_stm32_rl.py.  Older files have
        # none of it; report that rather than inventing a number.
        def _get(key, default=None):
            if key not in data:
                return default
            v = data[key]
            try:
                return v.item()
            except (AttributeError, ValueError):
                return v

        self.version = _get("version", 0)
        self.trained_utc = _get("trained_utc", "")
        self.steps = _get("steps", 0)
        self.backend = _get("backend", "")
        self.n_envs = _get("n_envs", 0)
        # 站定外环版（观测 18 维 / 动作 8 维）。老文件没有这个字段，按动作
        # 维度反推即可——权重矩阵本身就是唯一可靠的事实源。
        # The station-hold build has an 8-wide action.  Older files carry no
        # flag, so infer it from the weights, which cannot be wrong.
        self.hold_station = bool(self.w_out.shape[1] > 6)
        # 从 #9 起 npz 里带 gain_names。8 维动作既可能是 6+hold（#7/#8），
        # 也可能是别的组合，光看维度已经不够了。有名字就照名字建空间。
        # From #9 the npz carries gain_names: an 8-wide action is no longer
        # unambiguous, so trust the names when they are there.
        names = data["gain_names"].tolist() if "gain_names" in data else None
        if names is not None:
            names = [str(n) for n in names]
        if names is not None:
            self.hold_station = "hold_kpos" in names
        self.gain_space = STM32GainSpace(
            firmware=self.firmware, hold=self.hold_station,
            per_gain=bool(_get("per_gain_span", False)))
        # 跨度必须来自文件，不能来自源码常量。PID_SPANS 是可编辑的模块常量，
        # 改一次就会把所有老 npz 重新解释成另一个控制器：#9 被这样静默改过，
        # 同一份权重顶风位置从 0.4252 m 变成 0.2745 m，两次评测不可比。
        #
        # #9 早于这个字段，它训练时的跨度只能硬编码在这里。之后的模型都自带。
        #
        # Spans must come from the file, not from the module constants:
        # editing PID_SPANS silently reinterprets every existing policy.  #9
        # predates the field, so its training-time spans are pinned here.
        if "gain_spans" in data:
            self.gain_space.spans = np.asarray(data["gain_spans"],
                                               dtype=np.float64)
        elif bool(_get("per_gain_span", False)):
            self.gain_space.spans = np.array(
                [3.0, 6.0, 3.0, 6.0, 6.0, 6.0, 4.0, 4.0, 1.8][
                    :self.gain_space.dim], dtype=np.float64)
        # low/high 同理，而且对贴边的维度影响比 spans 更大。#9 早于这个字段，
        # 它训练时 hold_kpsi 的上限是 15（现在源码里是 30）。
        if "gain_low" in data:
            self.gain_space.low = np.asarray(data["gain_low"], dtype=np.float64)
            self.gain_space.high = np.asarray(data["gain_high"],
                                              dtype=np.float64)
        elif self.firmware != MODE_STM32_LQR:
            # 历史边界表。这些模型早于 gain_low/high 字段，而源码里的常量之后
            # 被我改过两次，必须按训练时的值还原，否则它们会被解释成从没训练
            # 过的控制器。
            #   turn_kp 上限   60 -> 100   （#9 之前改的，#7/#8 用 60）
            #   hold_kpsi 上限 15 -> 30    （#10 之前改的，#7/#8/#9 都用 15）
            # Legacy bounds: these predate the fields and the constants have
            # since moved twice; without this they load as controllers that
            # were never trained.
            legacy_high = [400.0, 2.50, 250.0, 2.00,
                           100.0 if bool(_get("per_gain_span", False)) else 60.0,
                           1.5, 8.0, 15.0, 2400.0]
            n = self.gain_space.dim
            self.gain_space.high = np.array(legacy_high[:n], dtype=np.float64)
        for nm, arr in (("gain_spans", self.gain_space.spans),
                        ("gain_low", self.gain_space.low),
                        ("gain_high", self.gain_space.high)):
            if len(arr) != self.gain_space.dim:
                raise ValueError(f"{path}: {nm} 长度和动作维度对不上")
        if names is not None and list(self.gain_space.names) != names:
            raise ValueError(
                f"{path}: gain_names {names} 和重建出来的空间 "
                f"{list(self.gain_space.names)} 对不上")

        # Observation normalization if present
        self.has_norm = "obs_mean" in data
        if self.has_norm:
            self.obs_mean = data["obs_mean"]
            self.obs_var = data["obs_var"]
            self.obs_clip = float(data["obs_clip"])

    @property
    def obs_dim(self) -> int:
        return int(self.w1.shape[0])

    def observe(self, core, last_action, v_ref=0.0, yaw_ref=0.0):
        """按这个策略训练时的定义搭观测向量。
        Build the observation exactly as this policy was trained on it.

        以前 UI、bench 和训练环境各写了一份，三份互相飘——``v_ref`` 在训练时
        恒为零那个 bug 能活下来就是因为这个。现在只有这一份。

        The UI, the bench and the training env each used to build this
        separately and drifted apart; that is how the "v_ref was always zero
        in training" bug survived.  There is one copy now.
        """
        st = core.state
        core_obs = [
            np.sin(st[ITH]), np.cos(st[ITH]), st[ITHD] / 10.0,
            st[IV] / 2.0, st[IPSID] / 10.0, v_ref / 2.0, yaw_ref / 10.0,
        ]
        if self.hold_station:
            e_f, e_l, e_p = core.station_error()
            core_obs += [e_f / OBS_POS_SCALE, e_l / OBS_POS_SCALE, e_p]
        obs = np.concatenate([np.array(core_obs, dtype=np.float32),
                              np.asarray(last_action, dtype=np.float32)])
        return np.clip(obs, -20.0, 20.0)

    def drive(self, core, last_action, v_ref=0.0, yaw_ref=0.0):
        """观测 -> 网络 -> 0.7/0.3 平滑 -> 写回增益。返回新的 last_action。
        One agent step of gain scheduling; returns the new action."""
        act = self.predict(self.observe(core, last_action, v_ref, yaw_ref),
                           deterministic=True)
        # 训练环境用的就是这个 0.7/0.3 变化率限制，所以推理时也必须用，
        # 否则策略看到的动作序列和它优化过的那个不是一回事。
        # The training env applies this slew, so inference must too.
        last_action = 0.7 * np.asarray(last_action) + 0.3 * act
        g = self.action_to_gains(last_action)
        if self.firmware in (MODE_STM32_LQR, MODE_STM32_HYBRID):
            core.fw.K = np.array([g[f"K{i}"] for i in range(1, 7)], dtype=float)
            if self.firmware == MODE_STM32_HYBRID:
                core.fw.w_gyro = float(g["w_gyro"])
                core.fw.w_pid_vel = float(g["w_pid_vel"])
        else:
            core.fw.g.update({k: v for k, v in g.items()
                              if not k.startswith("hold_")})
        if self.hold_station:
            core.hold_kpos = g["hold_kpos"]
            core.hold_kpsi = g["hold_kpsi"]
        return last_action

    def _normalize(self, obs: np.ndarray) -> np.ndarray:
        if not self.has_norm:
            return obs
        normed = (obs - self.obs_mean) / np.sqrt(self.obs_var + 1e-8)
        return np.clip(normed, -self.obs_clip, self.obs_clip)

    def predict(self, obs: np.ndarray, deterministic: bool = True) -> np.ndarray:
        """Run 2-layer MLP with Tanh activation."""
        x = self._normalize(obs)
        h1 = np.tanh(x @ self.w1 + self.b1)
        h2 = np.tanh(h1 @ self.w2 + self.b2)
        action = h2 @ self.w_out + self.b_out
        return np.clip(action, -1.0, 1.0)

    def label(self) -> str:
        """One line naming this policy for a menu."""
        n = f"#{self.version}" if self.version else "?"
        base = "LQR" if self.firmware == MODE_STM32_LQR else "PID"
        bits = [f"PPO {n}", f"{base} 底座"]
        if self.hold_station:
            bits.append("站定版")
        if self.steps:
            bits.append(f"{self.steps / 1e6:.1f}M 步" if self.steps >= 1e6
                        else f"{self.steps / 1e3:.0f}k 步")
        if self.trained_utc:
            bits.append(self.trained_utc.split()[0])
        return " · ".join(bits)

    def action_to_gains(self, action: np.ndarray) -> dict[str, float]:
        return self.gain_space.action_to_gains(action)
