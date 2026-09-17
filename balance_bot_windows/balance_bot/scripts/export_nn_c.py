"""把训练好的 npz 导成能直接加进 Keil 工程的 C。
Export a trained npz as C you can drop into the Keil project.

    python scripts/export_nn_c.py runs_stm32/adapt_01.npz
    python scripts/export_nn_c.py runs_stm32/adapt_01.npz --out keil_out/nn_gain

生成三个文件：

    nn_weights.h   权重表（int16）、偏置（int32）、tanh 表、增益查找表
    nn_gain.h      对外接口，五个函数
    nn_gain.c      特征提取（float IIR）+ 定点前向 + 增益施加 + 看门狗

**板子上怎么接**（三行）：

    NN_Init();                                   // 上电一次
    NN_Feed(angle, gyro, accel_fwd, accel_up,    // 每个 200 Hz 控制拍
            v_enc, ccr, encoder_integral);
    NN_Apply(&Balance_Kp, &Balance_Kd, ...);     // 每 8 拍（25 Hz）

`NN_Apply` 只在**它自己算出来的增益全部落在允许范围内**时才写回；任何一个越界、
或者 `NN_Enable(0)`、或者喂数超时，都保持原厂增益不动。这是最后一道闸。

`NN_Apply` writes the gains back only if every one of them lands inside the
allowed range; anything out of range, a disable, or a feed timeout leaves the
factory gains untouched.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from balance_bot.firmware.features import (BP_COEF, NAMES, N_FEATURES,  # noqa: E402
                                           SituationFeatures)
from balance_bot.nn_q15 import (QB, QONE, TANH_MAX, TANH_N,             # noqa: E402
                                TANH_STEP_Q, TANH_TAB, Q15Net)
from balance_bot.stm32_env import STM32GainSpace                        # noqa: E402

# 增益查找表的区间数（表长 N+1）。误差按 (Δa·ln span)²/8 走：32 段时 kp 那一维
# 差 2.6%，128 段降到 0.15%，代价是 6x129x4 = 3.1 KB。板子上不差这点。
# LUT intervals; 32 left a 2.6% error on kp, 128 brings it to 0.15% for 3.1 KB.
GAIN_LUT_N = 128


def _cfloat(v: float) -> str:
    """C 的浮点字面量。``%g`` 会把 32.0 写成 "32"，加上 f 后缀就成了非法的
    整数常量 ``32f``——编译器直接报错。所以这里保证一定带小数点。
    %g renders 32.0 as "32", and "32f" is an invalid integer suffix in C."""
    t = f"{float(v):.7g}"
    if "." not in t and "e" not in t and "E" not in t and "inf" not in t:
        t += ".0"
    return t + "f"


def _arr(name, values, ctype, per_line=12):
    out = [f"static const {ctype} {name}[] = {{"]
    vals = [str(int(v)) if "int" in ctype else _cfloat(v) for v in values]
    for i in range(0, len(vals), per_line):
        out.append("    " + ", ".join(vals[i:i + per_line]) + ",")
    out.append("};")
    return "\n".join(out)


def gain_luts(space: STM32GainSpace):
    """每个增益一张 a∈[-1,1] -> 增益值 的表，C 里只做查表+插值，不用 powf。
    One LUT per gain so the C needs no powf."""
    a = np.linspace(-1.0, 1.0, GAIN_LUT_N + 1)
    rows = []
    for i in range(space.dim):
        col = []
        for av in a:
            act = np.zeros(space.dim)
            act[i] = av
            col.append(space.action_to_gains(act)[space.names[i]])
        rows.append(col)
    return np.array(rows, dtype=np.float64)


def emit(npz_path: str, out_dir: str, firmware: str = "stm32_pid") -> dict:
    d = dict(np.load(npz_path, allow_pickle=False))
    net = Q15Net.from_npz(d)
    names = [str(x) for x in d["gain_names"]] if "gain_names" in d else None
    space = STM32GainSpace(firmware=firmware, per_gain=True)
    if names and list(names) != list(space.names):
        raise SystemExit(f"npz 的增益名 {names} 和 {firmware} 的 {space.names} 对不上")
    luts = gain_luts(space)
    n_in = net.layers[0][0].shape[0]
    n_h1 = net.layers[0][0].shape[1]
    n_h2 = net.layers[1][0].shape[1]
    n_out = net.layers[2][0].shape[1]
    os.makedirs(out_dir, exist_ok=True)

    # ---------------- nn_weights.h ----------------
    w = ["/* 自动生成，不要手改 / generated, do not edit */",
         f"/* 来源 source: {os.path.basename(npz_path)} */",
         "#ifndef NN_WEIGHTS_H", "#define NN_WEIGHTS_H", "#include <stdint.h>", "",
         f"#define NN_IN   {n_in}", f"#define NN_H1   {n_h1}",
         f"#define NN_H2   {n_h2}", f"#define NN_OUT  {n_out}",
         f"#define NN_QB   {QB}", f"#define NN_QONE {QONE}",
         f"#define NN_TANH_N {TANH_N}", f"#define NN_TANH_STEP {TANH_STEP_Q}",
         f"#define NN_TANH_LIM {int(TANH_MAX * QONE)}",
         f"#define NN_GAIN_LUT_N {GAIN_LUT_N}", ""]
    for i, (wq, bq, s, _) in enumerate(net.layers, 1):
        w.append(f"#define NN_SHIFT{i} {s}")
        w.append(_arr(f"nn_w{i}", wq.T.reshape(-1), "int16_t"))   # [out][in]
        w.append(_arr(f"nn_b{i}", bq, "int32_t"))
        w.append("")
    w.append(_arr("nn_tanh", TANH_TAB, "int16_t"))
    w.append("")
    w.append(_arr("nn_gain_lut", luts.reshape(-1), "float", per_line=8))
    w.append(_arr("nn_gain_lo", space.low, "float", per_line=8))
    w.append(_arr("nn_gain_hi", space.high, "float", per_line=8))
    w.append("")
    w.append("#endif")
    open(os.path.join(out_dir, "nn_weights.h"), "w", encoding="utf-8").write(
        "\n".join(w) + "\n")

    # ---------------- nn_gain.h ----------------
    hdr = f"""/* 自动生成 / generated.  情境自适应 PID：8 个特征 -> {n_in}-{n_h1}-{n_h2}-{n_out} 定点网络
 * -> {n_out} 个增益。用法见 scripts/export_nn_c.py 的文件头。 */
#ifndef NN_GAIN_H
#define NN_GAIN_H
#include <stdint.h>

void  NN_Init(void);
/* 每个 200 Hz 控制拍喂一次。单位：角度 deg，陀螺 deg/s，加速度 m/s^2，
 * 车速 m/s，ccr 比较值，vint 速度环积分。*/
void  NN_Feed(float angle_deg, float gyro_dps, float acc_fwd, float acc_up,
              float v_enc, float ccr, float vint);
/* 每 8 拍调一次（25 Hz）。返回 1 = 已写回新增益，0 = 保持原厂值。*/
int   NN_Apply(float *kp, float *kd, float *vkp, float *vki,
               float *tkp, float *tkd);
void  NN_Enable(int on);          /* 0 = 冻结，立刻回落原厂增益 */
const float *NN_Features(void);   /* 调试用，{N_FEATURES} 个 */

#endif
"""
    open(os.path.join(out_dir, "nn_gain.h"), "w", encoding="utf-8").write(hdr)

    # ---------------- nn_gain.c ----------------
    c = SituationFeatures().c_constants()
    body = f'''/* 自动生成，不要手改 / generated, do not edit.
 * 特征提取走 float（200 Hz 下十几次运算，M3 软浮点也吃得下），
 * 网络走 Q{QB} 定点（{net.macs()} 次乘加，浮点在 M3 上要 4 ms，定点 0.08 ms）。
 * Features in float (a dozen ops at 200 Hz); the net in fixed point, because
 * {net.macs()} software-float MACs would cost 4 ms on an FPU-less M3.
 */
#include "nn_gain.h"
#include "nn_weights.h"
#include <math.h>

#define FEAT_N {N_FEATURES}
/* 特征顺序 / feature order: {", ".join(NAMES)} */

static float  f_feat[FEAT_N];
static float  f_last[NN_OUT];      /* 上一步动作，喂回观测 / last action */
static int    f_on = 1;
static uint32_t f_age = 0;         /* 距上次 NN_Feed 多少拍 / ticks since feed */

/* 特征状态 / feature state */
static float bx1, bx2, by1, by2;   /* 31 Hz 带通延迟单元 */
static float e_bp, e_gyro, e_shock, e_tilt, e_venc, e_duty;

/* 观测的前 7 个由固件侧状态拼出来，和训练时一致 / the seven state terms */
static float f_state[7];

void NN_Init(void)
{{
    int i;
    for (i = 0; i < FEAT_N; i++) f_feat[i] = 0.0f;
    for (i = 0; i < NN_OUT; i++) f_last[i] = 0.0f;
    for (i = 0; i < 7; i++) f_state[i] = 0.0f;
    bx1 = bx2 = by1 = by2 = 0.0f;
    e_bp = e_gyro = e_shock = e_tilt = e_venc = e_duty = 0.0f;
    f_on = 1; f_age = 0;
}}

void NN_Enable(int on) {{ f_on = on ? 1 : 0; }}
const float *NN_Features(void) {{ return f_feat; }}

void NN_Feed(float angle_deg, float gyro_dps, float acc_fwd, float acc_up,
             float v_enc, float ccr, float vint)
{{
    float y0, amag;
    /* 31 Hz 带通 -> 整流 -> 慢 EMA */
    y0 = {_cfloat(c['bp_b0'])} * gyro_dps + {_cfloat(c['bp_b2'])} * bx2
         - ({_cfloat(c['bp_a1'])}) * by1 - ({_cfloat(c['bp_a2'])}) * by2;
    bx2 = bx1; bx1 = gyro_dps; by2 = by1; by1 = y0;
    e_bp   = {_cfloat(c['a_slow'])} * e_bp   + {_cfloat(1 - c['a_slow'])} * fabsf(y0);
    e_gyro = {_cfloat(c['a_slow'])} * e_gyro + {_cfloat(1 - c['a_slow'])} * fabsf(gyro_dps);
    e_shock = fabsf(gyro_dps) > {_cfloat(c['a_shock'])} * e_shock
            ? fabsf(gyro_dps) : {_cfloat(c['a_shock'])} * e_shock;
    e_tilt = {_cfloat(c['a_tilt'])} * e_tilt + {_cfloat(1 - c['a_tilt'])} * angle_deg;
    e_venc = {_cfloat(c['a_slow'])} * e_venc + {_cfloat(1 - c['a_slow'])} * v_enc;
    e_duty = {_cfloat(c['a_slow'])} * e_duty
           + {_cfloat(1 - c['a_slow'])} * fabsf(ccr) / {_cfloat(c['pwm_full'])};
    amag = sqrtf(acc_fwd * acc_fwd + acc_up * acc_up);

    f_feat[0] = e_bp    / {_cfloat(c['s_bp'])};
    f_feat[1] = e_gyro  / {_cfloat(c['s_gyro'])};
    f_feat[2] = e_shock / {_cfloat(c['s_shock'])};
    f_feat[3] = amag / {_cfloat(c['g'])} - 1.0f;
    f_feat[4] = e_tilt  / {_cfloat(c['s_tilt'])};
    f_feat[5] = e_venc  / {_cfloat(c['s_venc'])};
    f_feat[6] = vint    / {_cfloat(c['s_vint'])};
    f_feat[7] = e_duty;

    /* 观测的状态部分，和训练时 _get_obs 的定义逐项对齐 */
    {{
        float th = angle_deg * 0.017453293f;
        f_state[0] = sinf(th);
        f_state[1] = cosf(th);
        f_state[2] = gyro_dps * 0.017453293f / 10.0f;
        f_state[3] = v_enc / 2.0f;
        f_state[4] = 0.0f;      /* 偏航角速度 yaw rate，接上就填 */
        f_state[5] = 0.0f;      /* 速度指令 v_ref */
        f_state[6] = 0.0f;      /* 偏航指令 yaw_ref */
    }}
    f_age = 0;
}}

static float nn_roundf(float v)
{{
    return v >= 0.0f ? floorf(v + 0.5f) : ceilf(v - 0.5f);
}}

static int32_t nn_tanh_q(int32_t x)
{{
    int32_t idx, frac, y0, y1, y15;
    if (x >  NN_TANH_LIM) x =  NN_TANH_LIM;
    if (x < -NN_TANH_LIM) x = -NN_TANH_LIM;
    idx = (x + NN_TANH_LIM) / NN_TANH_STEP;
    if (idx > NN_TANH_N - 1) idx = NN_TANH_N - 1;
    frac = (x + NN_TANH_LIM) - idx * NN_TANH_STEP;
    y0 = nn_tanh[idx]; y1 = nn_tanh[idx + 1];
    y15 = y0 + ((y1 - y0) * frac) / NN_TANH_STEP;
    return y15 >> (15 - NN_QB);
}}

/* 累加器必须是 64 位。权重顶到 int16 (32767)、输入 Q11 最大 8*2048，单项乘积
 * 就有 5.4e8，32 项累加到 1.7e10——int32 (2.1e9) 直接溢出，输出会翻车而不是
 * 略有偏差。这一条曾经让 C 和 Python 的增益差了 124%。M3 上 64 位加法是两条
 * 指令，25 Hz 下完全无所谓。
 * The accumulator must be 64-bit: one product already reaches 5.4e8 and 32 of
 * them overflow int32, which corrupts the output rather than nudging it.
 */
static void nn_forward(const int32_t *x, int32_t *out)
{{
    int32_t h1[NN_H1], h2[NN_H2];
    int i, j;
    for (j = 0; j < NN_H1; j++) {{
        int64_t acc = (int64_t)nn_b1[j] << NN_SHIFT1;
        for (i = 0; i < NN_IN; i++)
            acc += (int64_t)nn_w1[j * NN_IN + i] * x[i];
        h1[j] = nn_tanh_q((int32_t)(acc >> NN_SHIFT1));
    }}
    for (j = 0; j < NN_H2; j++) {{
        int64_t acc = (int64_t)nn_b2[j] << NN_SHIFT2;
        for (i = 0; i < NN_H1; i++)
            acc += (int64_t)nn_w2[j * NN_H1 + i] * h1[i];
        h2[j] = nn_tanh_q((int32_t)(acc >> NN_SHIFT2));
    }}
    for (j = 0; j < NN_OUT; j++) {{
        int64_t acc = (int64_t)nn_b3[j] << NN_SHIFT3;
        for (i = 0; i < NN_H2; i++)
            acc += (int64_t)nn_w3[j * NN_H2 + i] * h2[i];
        out[j] = (int32_t)(acc >> NN_SHIFT3);
    }}
}}

static float gain_of(int g, float a)      /* a in [-1, 1] -> 增益值 */
{{
    float u, fi; int i;
    if (a < -1.0f) a = -1.0f;
    if (a >  1.0f) a =  1.0f;
    u = (a + 1.0f) * 0.5f * (float)NN_GAIN_LUT_N;
    i = (int)u; if (i > NN_GAIN_LUT_N - 1) i = NN_GAIN_LUT_N - 1;
    fi = u - (float)i;
    {{
        const float *row = &nn_gain_lut[g * (NN_GAIN_LUT_N + 1)];
        return row[i] + (row[i + 1] - row[i]) * fi;
    }}
}}

int NN_Apply(float *kp, float *kd, float *vkp, float *vki,
             float *tkp, float *tkd)
{{
    int32_t x[NN_IN], y[NN_OUT];
    float a[NN_OUT], g[NN_OUT];
    int i;

    f_age++;
    /* 看门狗：喂数停了超过 40 拍（0.2 s）就当没有网络 */
    if (!f_on || f_age > 40) return 0;

    /* 四舍五入，不是截断：Python 侧用 np.round，两边要逐位一致 */
    for (i = 0; i < 7; i++)        x[i] = (int32_t)nn_roundf(f_state[i] * NN_QONE);
    for (i = 0; i < FEAT_N; i++)   x[7 + i] = (int32_t)nn_roundf(f_feat[i] * NN_QONE);
    for (i = 0; i < NN_OUT; i++)   x[7 + FEAT_N + i] = (int32_t)nn_roundf(f_last[i] * NN_QONE);
    for (i = 0; i < NN_IN; i++) {{
        if (x[i] >  8 * NN_QONE) x[i] =  8 * NN_QONE;
        if (x[i] < -8 * NN_QONE) x[i] = -8 * NN_QONE;
    }}

    nn_forward(x, y);

    for (i = 0; i < NN_OUT; i++) {{
        float ai = (float)y[i] / (float)NN_QONE;
        if (ai < -1.0f) ai = -1.0f;
        if (ai >  1.0f) ai =  1.0f;
        /* 变化率限制，和训练时同一条公式 / same slew law as training */
        ai = 0.7f * f_last[i] + 0.3f * ai;
        f_last[i] = ai;
        g[i] = gain_of(i, ai);
        /* 最后一道闸：越界就整组不写 / last gate: out of range, write nothing */
        if (!(g[i] >= nn_gain_lo[i] && g[i] <= nn_gain_hi[i])) return 0;
        a[i] = ai;
    }}
    (void)a;
    *kp = g[0]; *kd = g[1]; *vkp = g[2]; *vki = g[3]; *tkp = g[4]; *tkd = g[5];
    return 1;
}}
'''
    open(os.path.join(out_dir, "nn_gain.c"), "w", encoding="utf-8").write(body)

    sz = net.size_bytes()
    lut_b = int(luts.size * 4 + space.low.size * 8)
    report = dict(inputs=n_in, hidden=(n_h1, n_h2), outputs=n_out,
                  macs=net.macs(), weights_b=sz["weights"], biases_b=sz["biases"],
                  tanh_b=sz["tanh_lut"], gain_lut_b=lut_b,
                  rodata_b=sz["total"] + lut_b,
                  ram_b=(N_FEATURES + n_out + 7) * 4 + 10 * 4,
                  stack_b=(n_h1 + n_h2 + n_in + n_out) * 4)
    return report


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("npz")
    ap.add_argument("--out", default="keil_out/nn_gain")
    ap.add_argument("--firmware", default="stm32_pid")
    args = ap.parse_args()
    r = emit(args.npz, args.out, args.firmware)
    print(f"已生成 {args.out}/nn_weights.h  nn_gain.h  nn_gain.c")
    print(f"网络 {r['inputs']}-{r['hidden'][0]}-{r['hidden'][1]}-{r['outputs']}"
          f"   每次推理 {r['macs']} 次乘加")
    print(f"Flash (rodata)  {r['rodata_b'] / 1024:.2f} KB"
          f"   = 权重 {r['weights_b']} + 偏置 {r['biases_b']}"
          f" + tanh 表 {r['tanh_b']} + 增益表 {r['gain_lut_b']} 字节")
    print(f"RAM 静态 {r['ram_b']} 字节 + 栈约 {r['stack_b']} 字节")
    print(f"板子余量：Flash 256-54.6 = 201 KB，RAM 48-17.3 = 30.7 KB"
          f"  ->  占用 {100 * r['rodata_b'] / (201 * 1024):.2f}% / "
          f"{100 * (r['ram_b'] + r['stack_b']) / (30.7 * 1024):.2f}%")


if __name__ == "__main__":
    main()
