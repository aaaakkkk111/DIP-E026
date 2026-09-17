# -*- coding: utf-8 -*-
"""理想环境下的串级 PID：只有刚体力学 + 固件 PID 公式，其余全部拿掉。

保留
  - 平面轮式倒立摆（小角度线性化）：车身质量 / 质心高度 / 车身惯量，
    两个轮子的质量 / 半径 / 惯量，折算到轮端的电机转子惯量（随「轮子相对车身」转）
  - 0.Large program 的串级 PID，原样公式，200 Hz 采样、零阶保持：
        balance  = Kp * angle_deg + Kd * gyro_lsb                (pid_control.c Balance_PD)
        E        = -(enc_l + enc_r)                               (Velocity_PI)
        bias     = 0.84 * bias + 0.16 * E
        integral = integral + bias
        velocity = -bias * Vkp - integral * Vki
        u        = balance + velocity                             (/100 已折进增益)
  - 一个执行器常数：每侧轮端力矩 = K_u * u。死区补偿恰好抵消死区时，
    u 以上那段占空比线性地产生力矩：K_u = 堵转力矩 / (2880 - 1500)
去掉
  死区失配、PWM 限幅 ±2600、PWM 延迟、齿轮间隙、传动弹性、传感器噪声与量化、
  卡尔曼滤波（角度直接取真值）、反电动势、地面接触（纯滚动）

因为全部线性，闭环是 z[k+1] = F z[k]，稳定性由 F 的特征值精确给出（|λ|<1），
不靠仿真判断。脚本末尾用 RK4 细步长仿真交叉核对这个 F。

用法：  python scripts/ideal_pid.py
"""
import os
import sys

import numpy as np
from scipy.linalg import expm

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ---------------------------------------------------------------- 物理参数
from balance_bot.firmware.robot import STM32_CAR as CAR  # noqa: E402

G = 9.81
M_B = CAR.m_body          # 车身质量 kg
L = CAR.l_com             # 轮轴到车身质心 m
I_B = CAR.I_body          # 车身绕自身质心的俯仰惯量 kg·m²
M_W = CAR.m_wheel         # 单个轮子质量 kg
R = CAR.r_wheel           # 轮半径 m
I_W = 0.5 * M_W * R ** 2  # 单个轮盘惯量
I_R = CAR.I_rotor         # 单侧折算转子惯量（挂在轮子相对车身的转角上）

T = 0.005                 # 控制周期 5 ms
CPR = 1320.0              # 编码器计数 / 轮子一圈
LSB_PER_RAD_S = 939.8     # MPU6050 ±2000 dps
K_U = 0.5679 / (2880.0 - 1500.0)   # N·m / 计数（每侧）

NORMAL = dict(kp=96.0, kd=0.48, vkp=62.0, vki=0.31)          # 模式 1，/100 已折
DB1300_NORMAL = dict(kp=384.0, kd=0.48, vkp=110.0, vki=0.92)  # sweep_300 那组

# ---------------------------------------------------------------- 连续时间被控对象
# 广义坐标 q = [x, θ]：x 轮心位移，θ 车身俯仰（前倾为正）。
# 轮子相对车身的转角 ψ = x/R - θ，电机力矩作用在 ψ 上，每侧一个。
#   T = ½(2m_w + 2I_w/R²)ẋ² + ½·2I_r(ẋ/R - θ̇)² + ½m_b[(ẋ + L θ̇)²] + ½I_b θ̇²   (小角度)
#   V = m_b g L cosθ
#   广义力：Q_x = 2τ/R，Q_θ = -2τ
M = np.array([[2 * M_W + 2 * I_W / R ** 2 + 2 * I_R / R ** 2 + M_B, M_B * L - 2 * I_R / R],
              [M_B * L - 2 * I_R / R, M_B * L ** 2 + I_B + 2 * I_R]])
Minv = np.linalg.inv(M)
# 状态 p = [x, θ, ẋ, θ̇]
A = np.zeros((4, 4))
A[0, 2] = A[1, 3] = 1.0
A[2:, 1] = Minv @ np.array([0.0, M_B * G * L])
B = np.zeros((4, 1))
B[2:, 0] = Minv @ np.array([2.0 / R, -2.0])      # 输入 = 每侧力矩 τ


def zoh(A, B, T):
    n, m = B.shape
    E = expm(np.block([[A, B], [np.zeros((m, n + m))]]) * T)
    return E[:n, :n], E[:n, n:]


AD, BD = zoh(A, B, T)
ENC = CPR / (2 * np.pi)                           # 计数 / 弧度


def closed_loop(g, delay_ticks=0):
    """闭环离散矩阵 F。状态 z = [x, θ, ẋ, θ̇, ψ_prev, bias, integral, u_pipe...]。
    delay_ticks > 0 时加上 PWM 生效延迟（仅用于对照，理想情形为 0）。"""
    n = 7 + delay_ticks
    F = np.zeros((n, n))
    for j in range(n):
        z = np.zeros(n)
        z[j] = 1.0
        x, th, xd, thd, psi_prev, bias, integ = z[:7]
        psi = x / R - th
        E = -2.0 * ENC * (psi - psi_prev)
        bias_n = 0.84 * bias + 0.16 * E
        integ_n = integ + bias_n
        u = (g["kp"] * np.degrees(th) + g["kd"] * thd * LSB_PER_RAD_S
             - g["vkp"] * bias_n - g["vki"] * integ_n)
        if delay_ticks:
            pipe = z[7:]
            applied = pipe[-1]
            new_pipe = np.concatenate([[u], pipe[:-1]])
        else:
            applied = u
        p_n = AD @ z[:4] + BD[:, 0] * K_U * applied
        col = np.concatenate([p_n, [psi, bias_n, integ_n]])
        if delay_ticks:
            col = np.concatenate([col, new_pipe])
        F[:, j] = col
    return F


def reduce(F):
    """分离掉平移模态。

    把车整体挪 c 米、同时把 ψ_prev 挪 c/R，控制器完全看不出来（它只看 Δψ 和
    积分），所以 v = [1,0,0,0,1/R,0,0,...] 是 F 的特征向量，特征值严格等于 1。
    它不是失稳，但浮点里 |λ| 在 1±1e-15 抖，会让「稳 / 不稳」来回翻。
    用 v 做第一个基向量做相似变换，其余特征值就在右下角那块里。
    Split off the exact translation mode (eigenvalue 1) before judging stability."""
    n = F.shape[0]
    Tm = np.eye(n)
    Tm[:, 0] = 0.0
    Tm[0, 0] = 1.0
    Tm[4, 0] = 1.0 / R
    Fr = np.linalg.solve(Tm, F @ Tm)
    assert np.allclose(Fr[:, 0], np.eye(n)[:, 0], atol=1e-9), "平移向量不是特征向量"
    return Fr[1:, 1:]


def modes(F):
    lam = np.linalg.eigvals(reduce(F))
    out = []
    for l in lam:
        if abs(l) < 1e-12:
            continue
        s = np.log(l) / T
        wn = abs(s)
        out.append(dict(mag=abs(l), f=abs(s.imag) / (2 * np.pi), wn_hz=wn / (2 * np.pi),
                        zeta=(-s.real / wn if wn > 1e-9 else 1.0), s=s))
    # 去掉共轭重复
    uniq = []
    for m in sorted(out, key=lambda m: -m["mag"]):
        if not any(abs(m["s"] - np.conj(u["s"])) < 1e-6 for u in uniq):
            uniq.append(m)
    return uniq


def spectral_radius(F):
    """去掉平移模态之后的谱半径；< 1 即稳定。"""
    return float(np.max(np.abs(np.linalg.eigvals(reduce(F)))))


def simulate(F, th0_deg=2.0, seconds=5.0, g=None):
    """线性离散迭代：初始前倾 th0_deg。返回倾角、位移、u 的时间序列。"""
    n = F.shape[0]
    z = np.zeros(n)
    z[1] = np.radians(th0_deg)
    z[4] = -z[1]                     # ψ_prev 与初始 ψ 一致（车静止）
    th, x, u = [], [], []
    for _ in range(int(seconds / T)):
        # 本拍 u（用于报告）
        psi = z[0] / R - z[1]
        E = -2.0 * ENC * (psi - z[4])
        bias_n = 0.84 * z[5] + 0.16 * E
        integ_n = z[6] + bias_n
        u.append(g["kp"] * np.degrees(z[1]) + g["kd"] * z[3] * LSB_PER_RAD_S
                 - g["vkp"] * bias_n - g["vki"] * integ_n)
        th.append(np.degrees(z[1]))
        x.append(z[0])
        z = F @ z
    return np.array(th), np.array(x), np.array(u)


def rk4_check(g, seconds=3.0, sub=50):
    """交叉核对：连续被控对象 RK4 细步长积分 + 同样的离散控制器，和 F 迭代比。"""
    F = closed_loop(g)
    th_f, x_f, _ = simulate(F, 2.0, seconds, g)
    p = np.array([0.0, np.radians(2.0), 0.0, 0.0])
    psi_prev = p[0] / R - p[1]
    bias = integ = 0.0
    h = T / sub
    th_r = []
    for _ in range(int(seconds / T)):
        th_r.append(np.degrees(p[1]))
        psi = p[0] / R - p[1]
        E = -2.0 * ENC * (psi - psi_prev)
        bias = 0.84 * bias + 0.16 * E
        integ = integ + bias
        u = (g["kp"] * np.degrees(p[1]) + g["kd"] * p[3] * LSB_PER_RAD_S
             - g["vkp"] * bias - g["vki"] * integ)
        psi_prev = psi
        tau = K_U * u
        f = lambda s: A @ s + B[:, 0] * tau
        for _ in range(sub):
            k1 = f(p); k2 = f(p + h / 2 * k1); k3 = f(p + h / 2 * k2); k4 = f(p + h * k3)
            p = p + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return float(np.max(np.abs(np.array(th_r) - th_f)))


def sim_saturated(g, th0_deg, seconds=12.0, limit=1100.0, sub=20):
    """理想被控对象 + 固件 PID + 只有一个非线性：|u| <= limit。RK4 细步长。"""
    p = np.array([0.0, np.radians(th0_deg), 0.0, 0.0])
    psi_prev = p[0] / R - p[1]
    bias = integ = 0.0
    h = T / sub
    th, sat = [], []
    for _ in range(int(seconds / T)):
        th.append(np.degrees(p[1]))
        if abs(th[-1]) > 60:
            return dict(fate="倒了", std=float("nan"), sat=float("nan"), freq=float("nan"))
        psi = p[0] / R - p[1]
        E = -2.0 * ENC * (psi - psi_prev)
        bias = 0.84 * bias + 0.16 * E
        integ = integ + bias
        u = (g["kp"] * np.degrees(p[1]) + g["kd"] * p[3] * LSB_PER_RAD_S
             - g["vkp"] * bias - g["vki"] * integ)
        sat.append(abs(u) > limit)
        u = float(np.clip(u, -limit, limit))
        psi_prev = psi
        tau = K_U * u
        f = lambda s: A @ s + B[:, 0] * tau
        for _ in range(sub):
            k1 = f(p); k2 = f(p + h / 2 * k1); k3 = f(p + h / 2 * k2); k4 = f(p + h * k3)
            p = p + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    th = np.array(th)
    tail = th[-int(3.0 / T):]
    y = tail - tail.mean()
    sp = np.abs(np.fft.rfft(y * np.hanning(len(y))))
    fr = np.fft.rfftfreq(len(y), T)
    freq = float(fr[1:][np.argmax(sp[1:])]) if y.std() > 1e-3 else 0.0
    fate = "持续振荡" if y.std() > 0.2 else "收敛"
    return dict(fate=fate, std=float(y.std()),
                sat=float(np.mean(sat[-int(3.0 / T):]) * 100), freq=freq)


def main():
    print("=" * 72)
    print("理想串级 PID：刚体力学 + 固件 PID 公式，无死区/限幅/延迟/噪声/弹性")
    print("=" * 72)
    print("物理参数  车身 %.3f kg，质心高 %.1f mm，车身惯量 %.3e，轮 %.3f kg × 2，"
          "半径 %.1f mm，转子 %.2e" % (M_B, L * 1000, I_B, M_W, R * 1000, I_R))
    print("执行器    每侧 %.3e N·m / 计数（堵转 0.568 N·m 分摊在补偿以上的 1380 计数）" % K_U)
    ol = np.linalg.eigvals(A)
    print("开环      极点 %s  -> 倒立摆发散时间常数 %.0f ms" % (
        np.round(ol.real, 2), 1000.0 / max(ol.real)))
    err = rk4_check(NORMAL)
    print("自检      F 迭代 vs RK4 细步长仿真，3 s 内倾角最大差 %.2e 度（应 ≈0）" % err)

    # ---------------------------------------------------- 1. 模式 1 的闭环极点
    print("\n--- 1. 模式 1（Kp 9600 / Kd 48 / Vkp 6200 / Vki 31）闭环模态 ---")
    F = closed_loop(NORMAL)
    rho = spectral_radius(F)
    print("谱半径 %.5f -> %s" % (rho, "稳定" if rho < 1 else "不稳定"))
    print("%10s %10s %10s" % ("频率 Hz", "阻尼比", "|λ|"))
    for m in modes(F):
        print("%10.2f %10.3f %10.5f" % (m["f"], m["zeta"], m["mag"]))

    # ---------------------------------------------------- 2. 稳定边界
    print("\n--- 2. 稳定边界（其余增益保持模式 1）---")
    def boundary(key, grid, base):
        stable = []
        for v in grid:
            g = dict(base); g[key] = v
            stable.append(spectral_radius(closed_loop(g)) < 1.0)
        stable = np.array(stable)
        edges = [grid[i] for i in range(1, len(grid)) if stable[i] != stable[i - 1]]
        return stable, edges
    kp_grid = np.linspace(0, 2000, 2001)
    s_kp, e_kp = boundary("kp", kp_grid, NORMAL)
    kd_grid = np.linspace(0, 5, 5001)
    s_kd, e_kd = boundary("kd", kd_grid, NORMAL)
    def ranges(grid, stable):
        out, start = [], None
        for v, s in zip(grid, stable):
            if s and start is None:
                start = v
            if not s and start is not None:
                out.append((start, prev)); start = None
            prev = v
        if start is not None:
            out.append((start, prev))
        return out
    print("Kp 稳定区间（固件单位 x100，扫 0-200000）: %s" % [
        "%d-%d" % (round(a * 100), round(b * 100)) for a, b in ranges(kp_grid, s_kp)])
    print("Kd 稳定区间（固件单位 x100，扫 0-500）   : %s" % [
        "%d-%d" % (round(a * 100), round(b * 100)) for a, b in ranges(kd_grid, s_kd)])

    print("\nKp × Kd 稳定图（# 稳定，. 不稳定；Vkp/Vki 保持模式 1）")
    kps = [24, 48, 96, 144, 192, 288, 384, 576, 768, 1152]
    kds = [0.12, 0.24, 0.48, 0.72, 0.96, 1.20, 1.60, 2.40, 3.20]
    print("%9s " % "Kd\\Kp" + "".join("%6d" % (k * 100) for k in kps))
    for kd in kds:
        row = ""
        for kp in kps:
            g = dict(NORMAL, kp=kp, kd=kd)
            mark = "#" if spectral_radius(closed_loop(g)) < 1 else "."
            row += "%6s" % mark
        print("%9d " % round(kd * 100) + row)

    # ---------------------------------------------------- 3. 真车三个实测点
    print("\n--- 3. 真车实测点在理想模型里是什么样 ---")
    cases = [("模式1 Kd48（真车：稳，倾角 std 0.25°，4.3 Hz 小极限环）", NORMAL),
             ("模式1 Kd120（真车：14 Hz 极限环，倾角 std 4°，饱和 66%）", dict(NORMAL, kd=1.20)),
             ("DB1300 Kp384（真车：3-4 Hz 大极限环，倾角 std 19°，饱和 80%）", DB1300_NORMAL)]
    for name, g in cases:
        F = closed_loop(g)
        rho = spectral_radius(F)
        ms = modes(F)
        worst = min(ms, key=lambda m: m["zeta"] if m["f"] > 0.05 else 9)
        print("%s\n    理想模型：谱半径 %.4f（%s），最弱振荡模态 %.2f Hz、阻尼比 %.3f" % (
            name, rho, "稳定" if rho < 1 else "不稳定", worst["f"], worst["zeta"]))

    # ---------------------------------------------------- 4. 时域响应
    print("\n--- 4. 初始前倾 2° 的响应（线性，不限幅）---")
    print("%-16s %9s %11s %12s %13s %14s" % ("", "倾角峰值", "回到±0.1°", "最终位移", "峰值 |u|", "超过 1100 余量"))
    for name, g in (("模式1 Kd48", NORMAL), ("模式1 Kd120", dict(NORMAL, kd=1.20)), ("DB1300 Kp384", DB1300_NORMAL)):
        F = closed_loop(g)
        th, x, u = simulate(F, 2.0, 8.0, g)
        idx = np.nonzero(np.abs(th) > 0.1)[0]
        settle = (idx[-1] + 1) * T if len(idx) else 0.0
        diverged = not np.all(np.isfinite(th)) or np.abs(th[-1]) > 90
        print("%-16s %8.2f° %10s %10.1f mm %12.0f %13s" % (
            name, np.max(np.abs(th)), "发散" if diverged else "%.2f s" % settle,
            x[-1] * 1000, np.max(np.abs(u)), "是" if np.max(np.abs(u)) > 1100 else "否"))

    # ---------------------------------------------------- 5. 只加一样东西：延迟
    print("\n--- 5. 对照：理想模型只加 PWM 延迟（仍然线性）---")
    print("%8s" % "延迟" + "".join("%16s" % n for n in ("Kd48", "Kd120", "Kp384")))
    for d in (0, 1, 2, 4):
        row = "%6d 拍" % d
        for g in (NORMAL, dict(NORMAL, kd=1.20), DB1300_NORMAL):
            rho = spectral_radius(closed_loop(g, delay_ticks=d))
            row += "%16s" % ("%.4f %s" % (rho, "稳" if rho < 1 else "不稳"))
        print(row)

    # ---------------------------------------------------- 6. 只加一样东西：限幅
    print("\n--- 6. 对照：理想模型只加 PWM 限幅（非线性，初始倾角由小到大）---")
    print("固件：u -> +1500 补偿 -> 限幅 ±2600，所以补偿以上能用的只有 ±1100 计数。")
    print("仍然无延迟、无死区失配、无噪声；被控对象用线性化方程（大角度下只是定性）。")
    print("%-14s %7s %10s %12s %10s %8s" % ("", "初始倾角", "结局", "后 3 s 倾角std", "饱和占比", "主频"))
    for name, g in (("模式1 Kd48", NORMAL), ("DB1300 Kp384", DB1300_NORMAL)):
        for th0 in (1.0, 3.0, 6.0, 10.0):
            r = sim_saturated(g, th0)
            print("%-14s %6.0f° %10s %11.2f° %9.0f%% %6.1f Hz" % (
                name, th0, r["fate"], r["std"], r["sat"], r["freq"]))


if __name__ == "__main__":
    main()
