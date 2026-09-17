"""定点推理：和板子上那份 C **逐位相同**的 Python 参照实现。
Fixed-point inference: a Python reference that is bit-exact with the C.

为什么要有这个文件
------------------
F103 是 Cortex-M3，**没有硬件浮点**。一个 21-32-32-6 的网络用软件浮点跑单次要
4 ms 以上，挤占 200 Hz 主环；定点只要 0.08 ms。但量化会改变策略行为，所以
「float 训完直接上车」是又一个 sim-to-real 缺口。规矩是：

    量化之后的网络，必须回到孪生里重新评测，用的就是这个文件。

The M3 has no FPU; quantisation is mandatory, and it changes behaviour, so the
*quantised* policy is what must be re-scored in the twin -- with this file.

数值格式
--------
* 输入、隐层输入、输出：**Q11**（1.0 = 2048，范围 ±16，分辨率 4.9e-4）
* 权重：每层单独取 2 的幂次 ``2^s``，使 ``max|W| * 2^s`` 刚好顶到 int16。
  于是一层就是「int32 累加 + 一次右移」，**没有乘法系数、没有除法**。
* 偏置：Q11，累加前左移 s 位对齐。
* tanh：65 点查找表 + 线性插值，表在 x∈[-4, 4]，表外直接饱和到 ±1。
  C 和 Python 用同一张表、同一套插值，所以两边逐位相同。

Q11 activations, per-layer power-of-two weight scales (accumulate in int32,
one right shift, no multiplier), and a 65-entry interpolated tanh LUT shared
by both implementations.
"""
from __future__ import annotations

import numpy as np

QB = 11                      # 定点小数位 / fractional bits
QONE = 1 << QB               # 1.0
TANH_N = 64                  # 表区间数（表长 N+1）/ LUT intervals
TANH_MAX = 4.0               # 表的边界，|x| 超过就饱和 / saturation bound
INT16_MAX = 32767


def tanh_lut() -> np.ndarray:
    """Q15 的 tanh 查找表，65 点。C 里是同一张表。
    The 65-point Q15 tanh table; the C uses the same one."""
    xs = np.linspace(-TANH_MAX, TANH_MAX, TANH_N + 1)
    return np.round(np.tanh(xs) * INT16_MAX).astype(np.int32)


TANH_TAB = tanh_lut()
# 表的步长（Q11）。TANH_N 个区间铺满 [-4, 4]，所以一格是 8/64 = 0.125。
TANH_STEP_Q = int(round((2.0 * TANH_MAX / TANH_N) * QONE))


def tanh_q(x_q: np.ndarray) -> np.ndarray:
    """Q11 进，Q11 出。整数运算，和 C 逐位一致。
    Q11 in, Q11 out; integer only, bit-exact with the C."""
    x = np.asarray(x_q, dtype=np.int64)
    lim = int(TANH_MAX * QONE)
    xc = np.clip(x, -lim, lim)
    idx_f = (xc + lim)                      # 0 .. 2*lim
    idx = np.minimum(idx_f // TANH_STEP_Q, TANH_N - 1)
    frac = idx_f - idx * TANH_STEP_Q        # 0 .. TANH_STEP_Q-1
    y0 = TANH_TAB[idx]
    y1 = TANH_TAB[idx + 1]
    y15 = y0 + ((y1 - y0) * frac) // TANH_STEP_Q      # Q15
    return (y15 >> (15 - QB)).astype(np.int64)        # -> Q11


def _pick_shift(w: np.ndarray) -> int:
    """给一层权重挑 2 的幂次，顶满 int16 又不溢出。
    Pick the power-of-two scale that fills int16 without overflowing."""
    m = float(np.max(np.abs(w))) if w.size else 1.0
    if m <= 0.0:
        return 15
    s = int(np.floor(np.log2(INT16_MAX / m)))
    return int(np.clip(s, 0, 20))


class Q15Net:
    """两层 tanh + 线性输出的定点网络。
    Two tanh layers plus a linear head, in fixed point."""

    def __init__(self, layers):
        # layers: [(W_q int16[in,out], b_q int32[out], shift, act), ...]
        self.layers = layers

    # ------------------------------------------------------------------
    @classmethod
    def from_float(cls, w1, b1, w2, b2, w_out, b_out):
        layers = []
        for w, b, act in ((w1, b1, "tanh"), (w2, b2, "tanh"),
                          (w_out, b_out, "none")):
            w = np.asarray(w, dtype=np.float64)
            b = np.asarray(b, dtype=np.float64)
            s = _pick_shift(w)
            wq = np.clip(np.round(w * (1 << s)), -INT16_MAX, INT16_MAX
                         ).astype(np.int32)
            bq = np.round(b * QONE).astype(np.int64)
            layers.append((wq, bq, s, act))
        return cls(layers)

    @classmethod
    def from_npz(cls, path_or_dict):
        d = (path_or_dict if isinstance(path_or_dict, dict)
             else dict(np.load(path_or_dict, allow_pickle=False)))
        return cls.from_float(d["w1"], d["b1"], d["w2"], d["b2"],
                              d["w_out"], d["b_out"])

    # ------------------------------------------------------------------
    def forward_q(self, x_q: np.ndarray) -> np.ndarray:
        """Q11 进，Q11 出（动作，未裁剪）。
        Q11 in, Q11 out (the raw action, unclipped)."""
        h = np.asarray(x_q, dtype=np.int64)
        for wq, bq, s, act in self.layers:
            acc = h @ wq.astype(np.int64)          # Q(QB+s)
            acc = acc + (bq << s)
            y = acc >> s                           # -> Q11
            h = tanh_q(y) if act == "tanh" else y
        return h

    def forward(self, obs: np.ndarray) -> np.ndarray:
        """float 进，float 出。内部全程定点。
        Float in, float out; fixed point inside."""
        x = np.asarray(obs, dtype=np.float64)
        lim = 8.0                                  # 观测硬裁剪 / obs clamp
        xq = np.round(np.clip(x, -lim, lim) * QONE).astype(np.int64)
        return self.forward_q(xq).astype(np.float64) / QONE

    # ------------------------------------------------------------------
    def size_bytes(self) -> dict[str, int]:
        w = sum(l[0].size * 2 for l in self.layers)      # int16
        b = sum(l[1].size * 4 for l in self.layers)      # int32
        lut = TANH_TAB.size * 2
        return dict(weights=w, biases=b, tanh_lut=lut, total=w + b + lut)

    def macs(self) -> int:
        return sum(int(l[0].size) for l in self.layers)


def quantisation_error(net_f, net_q, obs_batch) -> dict[str, float]:
    """float 和定点在同一批观测上的差距。上车前必须看这个数。
    The float-vs-fixed gap on one batch of observations."""
    a_f = np.stack([net_f(o) for o in obs_batch])
    a_q = np.stack([net_q.forward(o) for o in obs_batch])
    d = np.abs(a_f - a_q)
    return dict(max=float(d.max()), mean=float(d.mean()),
                rms=float(np.sqrt((d ** 2).mean())))
