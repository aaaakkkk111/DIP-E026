"""A minimal NumPy MLP with hand-written gradients, plus Adam.

Two reasons this exists instead of just using PyTorch:

1.  The runtime side (control panel, ROS 2 node, MuJoCo viewer) can then load
    and run a trained policy with *nothing but NumPy* -- no torch install on
    the robot side, no GPU, deterministic, ~20 us per inference.
2.  It gives a dependency-free PPO trainer so the whole pipeline can be
    verified anywhere, including machines where PyTorch cannot be installed.

``train_sb3.py`` exports stable-baselines3 policies into exactly the same
``.npz`` format, so both training paths produce interchangeable artefacts.
"""
from __future__ import annotations

import numpy as np


def orthogonal(shape, gain=1.0, rng=None):
    rng = rng or np.random.default_rng()
    a = rng.normal(size=shape)
    u, _, vh = np.linalg.svd(a, full_matrices=False)
    q = u if u.shape == shape else vh
    return (gain * q).astype(np.float64)


class MLP:
    """``in -> tanh(h) -> tanh(h) -> out`` with explicit forward/backward."""

    def __init__(self, n_in, n_hidden, n_out, out_gain=0.01, rng=None):
        rng = rng or np.random.default_rng()
        self.W1 = orthogonal((n_in, n_hidden), np.sqrt(2), rng)
        self.b1 = np.zeros(n_hidden)
        self.W2 = orthogonal((n_hidden, n_hidden), np.sqrt(2), rng)
        self.b2 = np.zeros(n_hidden)
        self.W3 = orthogonal((n_hidden, n_out), out_gain, rng)
        self.b3 = np.zeros(n_out)

    # ------------------------------------------------------------------
    @property
    def params(self):
        return [self.W1, self.b1, self.W2, self.b2, self.W3, self.b3]

    def load_params(self, plist):
        (self.W1, self.b1, self.W2, self.b2, self.W3, self.b3) = \
            [np.asarray(p, dtype=np.float64) for p in plist]

    # ------------------------------------------------------------------
    def forward(self, x, cache=None):
        h1 = np.tanh(x @ self.W1 + self.b1)
        h2 = np.tanh(h1 @ self.W2 + self.b2)
        out = h2 @ self.W3 + self.b3
        if cache is not None:
            cache["x"], cache["h1"], cache["h2"] = x, h1, h2
        return out

    def backward(self, grad_out, cache):
        x, h1, h2 = cache["x"], cache["h1"], cache["h2"]
        gW3 = h2.T @ grad_out
        gb3 = grad_out.sum(axis=0)
        g2 = (grad_out @ self.W3.T) * (1.0 - h2 ** 2)
        gW2 = h1.T @ g2
        gb2 = g2.sum(axis=0)
        g1 = (g2 @ self.W2.T) * (1.0 - h1 ** 2)
        gW1 = x.T @ g1
        gb1 = g1.sum(axis=0)
        return [gW1, gb1, gW2, gb2, gW3, gb3]


class Adam:
    def __init__(self, params, lr=3e-4, betas=(0.9, 0.999), eps=1e-8):
        self.lr = lr
        self.b1, self.b2 = betas
        self.eps = eps
        self.m = [np.zeros_like(p) for p in params]
        self.v = [np.zeros_like(p) for p in params]
        self.t = 0

    def step(self, params, grads, max_norm=0.5):
        self.t += 1
        if max_norm is not None:
            total = np.sqrt(sum(float(np.sum(g ** 2)) for g in grads))
            if total > max_norm:
                scale = max_norm / (total + 1e-12)
                grads = [g * scale for g in grads]
        for i, (p, g) in enumerate(zip(params, grads)):
            self.m[i] = self.b1 * self.m[i] + (1 - self.b1) * g
            self.v[i] = self.b2 * self.v[i] + (1 - self.b2) * g ** 2
            mh = self.m[i] / (1 - self.b1 ** self.t)
            vh = self.v[i] / (1 - self.b2 ** self.t)
            p -= self.lr * mh / (np.sqrt(vh) + self.eps)


class RunningNorm:
    """Welford running mean/variance for observation normalisation."""

    def __init__(self, dim, clip=10.0):
        self.mean = np.zeros(dim)
        self.var = np.ones(dim)
        self.count = 1e-4
        self.clip = clip

    def update(self, x):
        x = np.atleast_2d(x)
        bm = x.mean(axis=0)
        bv = x.var(axis=0)
        bc = x.shape[0]
        delta = bm - self.mean
        tot = self.count + bc
        self.mean += delta * bc / tot
        m_a = self.var * self.count
        m_b = bv * bc
        self.var = (m_a + m_b + delta ** 2 * self.count * bc / tot) / tot
        self.count = tot

    def __call__(self, x):
        return np.clip((x - self.mean) / np.sqrt(self.var + 1e-8),
                       -self.clip, self.clip)

    def state(self):
        return dict(mean=self.mean, var=self.var,
                    count=np.array([self.count]), clip=np.array([self.clip]))

    def load(self, d):
        self.mean = np.asarray(d["mean"])
        self.var = np.asarray(d["var"])
        self.count = float(np.asarray(d["count"]).ravel()[0])
        self.clip = float(np.asarray(d["clip"]).ravel()[0])
