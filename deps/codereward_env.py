"""Env variant whose shaped reward comes from an LLM-written, sandboxed reward
FUNCTION instead of the eight fixed weighted terms.

It reuses every physical detail of OllamaTzejunEnv (1 ms physics, 200 Hz control,
34-obs layout, motor transfer, randomisation, fall at 0.70 rad). The ONLY change is
how the per-step reward is formed: a callable `reward_fn(info, action, prev_action)`
returns a bounded shaped reward, and the immutable fall penalty (-25) is still added
here, outside the LLM code, exactly as in the base env.
"""
from __future__ import annotations

import numpy as np

from ollama_tzejun_env import OllamaTzejunEnv, FALL_PENALTY


class CodeRewardTzejunEnv(OllamaTzejunEnv):
    def __init__(self, reward_fn, **kwargs) -> None:
        # Base still needs valid default weights (used only by its unused path).
        super().__init__(**kwargs)
        if not callable(reward_fn):
            raise TypeError("reward_fn must be callable")
        self._reward_fn = reward_fn

    def step(self, action):
        prev = None if self.prev_action is None else self.prev_action.copy()
        obs, _base_reward, terminated, truncated, info = super().step(action)
        command = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        shaped = float(self._reward_fn(info, command, prev))
        reward = float(shaped - (FALL_PENALTY if terminated else 0.0))
        info["reward_shaped_llm"] = shaped
        info["reward"] = reward
        info["fall_penalty"] = -FALL_PENALTY if terminated else 0.0
        return obs, reward, terminated, truncated, info
