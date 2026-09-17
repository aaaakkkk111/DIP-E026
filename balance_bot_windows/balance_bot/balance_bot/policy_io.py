"""Load / save the gain-scheduling policy as a plain ``.npz``.

The point of this module is that *inference never needs a deep-learning
framework*.  Whether the weights came from the NumPy PPO or from
stable-baselines3, the control panel, the MuJoCo viewer and the ROS 2 node
all load them the same way and evaluate them with NumPy alone.
"""
from __future__ import annotations

import numpy as np

from .nn import MLP, RunningNorm

FORMAT_VERSION = 1


class GainPolicy:
    """Deterministic policy: observation -> action in [-1, 1]^9."""

    def __init__(self, obs_dim, act_dim, hidden=64, rng=None,
                 bias_init=None, log_std_init=-1.0):
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.net = MLP(obs_dim, hidden, act_dim, out_gain=0.01, rng=rng)
        if bias_init is not None:
            # Start the policy *at the hand-tuned nominal PID* instead of at
            # the middle of the gain box.  The agent then spends its budget
            # learning when to deviate rather than rediscovering a working
            # controller from scratch, which is the difference between
            # converging in 200k steps and not converging at all.
            self.net.b3 = np.asarray(bias_init, dtype=np.float64).copy()
        self.log_std = np.full(act_dim, float(log_std_init))
        self.norm = RunningNorm(obs_dim)

    # ------------------------------------------------------------------
    def act(self, obs, deterministic=True, rng=None):
        """Gaussian policy with clipping (same convention as SB3 for Box)."""
        x = self.norm(np.asarray(obs, dtype=np.float64).reshape(1, -1))
        mean = self.net.forward(x)[0]
        if deterministic:
            return np.clip(mean, -1.0, 1.0)
        rng = rng or np.random.default_rng()
        z = mean + np.exp(self.log_std) * rng.normal(size=self.act_dim)
        return np.clip(z, -1.0, 1.0)

    # ------------------------------------------------------------------
    def save(self, path):
        d = {f"p{i}": p for i, p in enumerate(self.net.params)}
        d["log_std"] = self.log_std
        d["obs_dim"] = np.array([self.obs_dim])
        d["act_dim"] = np.array([self.act_dim])
        d["version"] = np.array([FORMAT_VERSION])
        for k, v in self.norm.state().items():
            d[f"norm_{k}"] = v
        np.savez(path, **d)
        return path

    @classmethod
    def load(cls, path):
        z = np.load(path, allow_pickle=False)
        obs_dim = int(z["obs_dim"][0])
        act_dim = int(z["act_dim"][0])
        hidden = int(z["p0"].shape[1])
        p = cls(obs_dim, act_dim, hidden)
        p.net.load_params([z[f"p{i}"] for i in range(6)])
        p.log_std = z["log_std"]
        p.norm.load({k[5:]: z[k] for k in z.files if k.startswith("norm_")})
        return p


def constant_policy(action):
    """Wrap a fixed action (e.g. the nominal gains) in the policy interface."""
    a = np.asarray(action, dtype=float)

    class _Const:
        act_dim = a.size

        def act(self, obs, deterministic=True, rng=None):
            return a
    return _Const()
