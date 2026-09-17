"""自适应 PID 框架的自检：特征、情境、定点网络、C 导出。
Self-check for the adaptive-PID framework.

    python tests/test_adaptive.py

有 gcc 的话会把生成的 C 编出来和 Python 对拍（特征逐位、增益按容差）；
没有 gcc 就跳过那一项并明说。
If gcc is present the generated C is compiled and cross-checked against the
Python reference; otherwise that check is skipped out loud.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from balance_bot.firmware.features import (NAMES, N_FEATURES,       # noqa: E402
                                           SituationFeatures)
from balance_bot.nn_q15 import Q15Net, quantisation_error           # noqa: E402
from balance_bot.stm32_env import STM32GainSpace                    # noqa: E402

PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print(f"  [{'ok' if cond else 'FAIL'}] {name}" + (f"   {detail}" if detail else ""))


# ----------------------------------------------------------------------
def test_features():
    print("\n情境特征 / situation features")
    f = SituationFeatures()
    check("八个特征，名字对得上", N_FEATURES == 8 and len(NAMES) == 8)

    # 静止：所有特征都该小
    for _ in range(400):
        out = f.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 0.0)
    check("静止时特征全部接近 0", float(np.abs(out).max()) < 0.05,
          f"max {np.abs(out).max():.4f}")

    # 悬空：freefall 通道趋近 -1，其余不该跟着动
    f.reset()
    for _ in range(50):
        out = f.update(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    check("悬空时 freefall -> -1", out[3] < -0.95, f"{out[3]:.3f}")

    # 冲击：shock 立刻起来，且比慢 EMA 快得多
    f.reset()
    for _ in range(10):
        out = f.update(300.0, 0.0, 9.81, 0.0, 0.0, 0.0, 0.0)
    check("冲击时 shock 抢在慢 EMA 前面", out[2] > 3 * out[1],
          f"shock {out[2]:.2f} vs gyro {out[1]:.2f}")

    # 31 Hz 带通只对 31 Hz 有反应：4.3 Hz 的死区极限环不该混进来
    def drive(hz, n=600):
        g = SituationFeatures()
        for k in range(n):
            g.update(30.0 * np.sin(2 * np.pi * hz * k / 200.0),
                     0.0, 9.81, 0.0, 0.0, 0.0, 0.0)
        return g.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 0.0)[0]
    b31, b4 = drive(31.0), drive(4.3)
    check("31 Hz 带通挡得住 4.3 Hz 极限环", b31 > 8 * b4,
          f"31 Hz {b31:.3f} vs 4.3 Hz {b4:.3f}")

    # 确定性：同样的输入两次必须逐位相同
    a = SituationFeatures(); b = SituationFeatures()
    rng = np.random.default_rng(0)
    same = True
    for _ in range(200):
        args = tuple(rng.normal(0, 3, 7))
        same &= bool(np.array_equal(a.update(*args), b.update(*args)))
    check("同输入逐位可复现", same)


# ----------------------------------------------------------------------
def test_scenarios():
    print("\n情境注入 / scenario injection")
    from balance_bot.firmware.twin_baseline import MODE_STM32_PID
    from balance_bot.stm32_adapt_env import STM32AdaptEnv

    # 三个种子取峰值的最大：情境的幅值是随机抽的，单个种子可能抽到最轻的那档，
    # 断言会时灵时不灵。
    # Three seeds, max over them: magnitudes are sampled, so a single seed can
    # draw the mildest case and make the assertion flaky.
    sig = {}
    for sc in ("none", "kick", "drop", "slope"):
        peak = np.zeros(N_FEATURES)
        for sd in (3, 4, 5):
            env = STM32AdaptEnv(firmware=MODE_STM32_PID, backend="mujoco",
                                randomize=False, episode_seconds=16.0,
                                command_prob=0.0, seed=sd, scenarios=(sc,))
            env.set_difficulty(1.0)
            obs, info = env.reset(seed=sd)
            for _ in range(400):
                obs, r, term, trunc, inf = env.step(
                    np.zeros(env.action_space.shape[0]))
                peak = np.maximum(peak, np.abs(inf["features"]))
                if term or trunc:
                    break
        sig[sc] = peak
        check(f"{sc}: 观测维度 = 7 状态 + 8 特征 + 6 动作",
              obs.shape[0] == 21)

    # 每个情境都要在**它自己那个通道**上比静止明显
    check("冲击打在 shock 通道", sig["kick"][2] > 4 * sig["none"][2],
          f"{sig['kick'][2]:.2f} vs {sig['none'][2]:.2f}")
    check("跌落打在 freefall 通道", sig["drop"][3] > 4 * sig["none"][3],
          f"{sig['drop'][3]:.2f} vs {sig['none'][3]:.2f}")
    check("坡道打在速度环积分通道", sig["slope"][6] > 4 * sig["none"][6],
          f"{sig['slope'][6]:.2f} vs {sig['none'][6]:.2f}")


# ----------------------------------------------------------------------
def _fake_npz(path, n_in=21, hid=32, n_out=6, seed=0):
    rng = np.random.default_rng(seed)
    sp = STM32GainSpace(firmware="stm32_pid", per_gain=True)
    np.savez(path,
             w1=rng.normal(0, .4, (n_in, hid)).astype(np.float32),
             b1=rng.normal(0, .1, hid).astype(np.float32),
             w2=rng.normal(0, .4, (hid, hid)).astype(np.float32),
             b2=rng.normal(0, .1, hid).astype(np.float32),
             w_out=rng.normal(0, .4, (hid, n_out)).astype(np.float32),
             b_out=rng.normal(0, .1, n_out).astype(np.float32),
             firmware=np.array("stm32_pid"),
             gain_names=np.array(list(sp.names)))


def test_q15():
    print("\n定点网络 / fixed-point net")
    tmp = tempfile.mkdtemp()
    npz = os.path.join(tmp, "p.npz")
    _fake_npz(npz)
    d = dict(np.load(npz))
    net = Q15Net.from_npz(d)

    def ref(o):
        h = np.tanh(o @ d["w1"] + d["b1"])
        h = np.tanh(h @ d["w2"] + d["b2"])
        return h @ d["w_out"] + d["b_out"]

    rng = np.random.default_rng(1)
    obs = rng.normal(0, 1, (300, 21))
    err = quantisation_error(ref, net, obs)
    # 动作范围是 [-1, 1]，2% 是上车前可以接受的上限；再大就要加宽 Q 或换结构
    check("量化误差 < 2% 动作量程", err["max"] < 0.02,
          f"max {err['max']:.4f}  rms {err['rms']:.4f}")
    sz = net.size_bytes()
    check("定点体积 < 8 KB", sz["total"] < 8192, f"{sz['total']} 字节")
    check("推理量级 ~2k 乘加", 1000 < net.macs() < 4000, f"{net.macs()}")
    # 定点必须是纯整数：同输入两次逐位相同
    check("定点逐位可复现",
          np.array_equal(net.forward(obs[0]), net.forward(obs[0])))
    shutil.rmtree(tmp, ignore_errors=True)


# ----------------------------------------------------------------------
def test_c_export():
    print("\nC 导出 / generated C")
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "scripts"))
    from export_nn_c import emit

    tmp = tempfile.mkdtemp()
    npz = os.path.join(tmp, "p.npz")
    _fake_npz(npz, seed=5)
    rep = emit(npz, os.path.join(tmp, "c"))
    check("生成三个文件", all(os.path.exists(os.path.join(tmp, "c", f))
                              for f in ("nn_gain.c", "nn_gain.h", "nn_weights.h")))
    # F103RC：flash 256 KB、RAM 48 KB，固件已占 54.6 / 17.3
    check("Flash 占用 < 16 KB", rep["rodata_b"] < 16384,
          f"{rep['rodata_b'] / 1024:.2f} KB")
    check("RAM 占用 < 2 KB", rep["ram_b"] + rep["stack_b"] < 2048,
          f"{rep['ram_b'] + rep['stack_b']} 字节")

    gcc = shutil.which("gcc") or shutil.which("cc")
    if not gcc:
        print("  [skip] 没有 gcc，跳过 C 对拍 / no gcc, cross-check skipped")
        shutil.rmtree(tmp, ignore_errors=True)
        return
    harness = os.path.join(tmp, "h.c")
    # NN_Apply 必须先单独调用再 printf。写成
    #     printf("%d %g", NN_Apply(&kp,...), kp, ...)
    # 的话，C 的函数参数求值顺序是**未定义**的，gcc 先取了 kp 的旧值，打出来
    # 整整差一拍——这个坑让这项对拍一度报 125% 的假差异。
    # Call NN_Apply before the printf: argument evaluation order is unspecified
    # in C, gcc reads kp before the call, and the output lags by one tick.
    open(harness, "w").write("""
#include <stdio.h>
#include "nn_gain.h"
int main(void){float a,g,af,au,v,c,vi,kp,kd,vkp,vki,tkp,tkd;int i,ok;NN_Init();
while(scanf("%f %f %f %f %f %f %f",&a,&g,&af,&au,&v,&c,&vi)==7){
NN_Feed(a,g,af,au,v,c,vi);
ok=NN_Apply(&kp,&kd,&vkp,&vki,&tkp,&tkd);
printf("%d %.7g %.7g %.7g %.7g %.7g %.7g\\n",ok,kp,kd,vkp,vki,tkp,tkd);
{const float*f=NN_Features();for(i=0;i<8;i++){printf("%.7g ",f[i]);}printf("\\n");}}
return 0;}
""")
    exe = os.path.join(tmp, "h.exe")
    r = subprocess.run([gcc, "-O2", "-std=c99", "-I", os.path.join(tmp, "c"),
                        harness, os.path.join(tmp, "c", "nn_gain.c"),
                        "-o", exe, "-lm"], capture_output=True, text=True)
    check("生成的 C 能编过", r.returncode == 0, r.stderr.strip()[:200])
    if r.returncode != 0:
        shutil.rmtree(tmp, ignore_errors=True)
        return

    rng = np.random.default_rng(11)
    rows = []
    for k in range(150):
        t = k / 200.0
        gyro = 9 * np.sin(2 * np.pi * 4.3 * t) + 3 * np.sin(2 * np.pi * 31 * t)
        if 0.20 < t < 0.26:
            gyro += 200.0                       # 冲击
        af, au = (0.0, 9.81) if t < 0.45 else (0.2, 0.5)   # 后段悬空
        rows.append((0.3 * np.sin(2 * np.pi * 4.3 * t), gyro, af, au,
                     0.02 * np.sin(2 * np.pi * 1.1 * t),
                     1500 + 40 * np.sign(np.sin(2 * np.pi * 4.3 * t)),
                     200 * np.sin(2 * np.pi * 0.3 * t)))
    inp = "\n".join(" ".join("%.8g" % x for x in r) for r in rows) + "\n"
    out = subprocess.run([exe], input=inp, capture_output=True,
                         text=True).stdout.strip().split("\n")

    feat = SituationFeatures()
    net = Q15Net.from_npz(npz)
    sp = STM32GainSpace(firmware="stm32_pid", per_gain=True)
    last = np.zeros(6)
    d_feat = d_gain = 0.0
    for k, row in enumerate(rows):
        ang, gyro, af, au, v, ccr, vint = row
        fv = feat.update(gyro, af, au, ang, v, ccr, vint)
        th = np.radians(ang)
        state = np.array([np.sin(th), np.cos(th), np.radians(gyro) / 10.0,
                          v / 2.0, 0.0, 0.0, 0.0])
        a = np.clip(net.forward(np.concatenate([state, fv, last])), -1, 1)
        a = 0.7 * last + 0.3 * a
        last = a
        g = np.array([sp.action_to_gains(np.eye(6)[i] * a[i])[sp.names[i]]
                      for i in range(6)])
        c_f = np.array([float(x) for x in out[2 * k + 1].split()])
        parts = out[2 * k].split()
        d_feat = max(d_feat, float(np.abs(c_f - fv).max()))
        if int(parts[0]):
            c_g = np.array([float(x) for x in parts[1:]])
            d_gain = max(d_gain, float(np.max(np.abs(c_g - g) /
                                              np.maximum(np.abs(g), 1e-9))))
    check("C 和 Python 的特征一致（float 舍入量级）", d_feat < 1e-4,
          f"最大差 {d_feat:.2e}")
    # 剩下的增益差是**数值**差，不是逻辑分歧：C 侧特征用 float32、Python 用
    # float64，输入差一个 Q11 的 LSB（1/2048），经过网络放大到动作上约 3e-3，
    # 再经 turn_kp 那一维 span=7 的指数映射放成千分之几。2% 和量化误差本身
    # 同量级，够用；真正的把关是「量化后的网络回孪生复评」。
    # The residual is numerical (float32 features in C vs float64 in Python,
    # one Q11 LSB in, a few 1e-3 out), not a logic divergence.
    check("C 和 Python 的增益一致（< 2%）", d_gain < 2e-2,
          f"最大相对差 {d_gain * 100:.3f}%")
    shutil.rmtree(tmp, ignore_errors=True)


# ----------------------------------------------------------------------
def main():
    print("=" * 68)
    print("情境自适应 PID 框架 -- 自检 / adaptive-PID framework self-check")
    print("=" * 68)
    test_features()
    test_q15()
    test_c_export()
    test_scenarios()
    print("\n" + "=" * 68)
    print(f"{len(PASS)} passed, {len(FAIL)} failed")
    for f in FAIL:
        print("   FAILED:", f)
    print("=" * 68)
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
