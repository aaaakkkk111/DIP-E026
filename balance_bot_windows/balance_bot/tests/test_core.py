"""Headless self-check.  Runs with nothing but NumPy.

    python tests/test_core.py

Checks the things that are easy to get silently wrong in a robotics sim:
sign conventions, energy sanity, integrator time bookkeeping, ray casting,
anti-windup, observation shape/finiteness, policy round-tripping, and that
the controller actually stabilises and tracks.
"""
from __future__ import annotations

import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from balance_bot.arena import Arena                                  # noqa: E402
from balance_bot.controller import CascadePID, lqr_reference_gains   # noqa: E402
from balance_bot.dynamics import (BalanceBotDynamics, make_state,    # noqa: E402
                                  ITH, IV, IPSID, IX, IY)
from balance_bot.env import BalanceCore, MODE_PID, MODE_PPO_GAINS    # noqa: E402
from balance_bot.params import (ArenaParams, DisturbanceConfig,      # noqa: E402
                                GainSpace, RobotParams, SimParams)
from balance_bot.policy_io import GainPolicy                         # noqa: E402

PASS, FAIL, SKIP = [], [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print(f"  [{'ok' if cond else 'FAIL'}] {name}" + (f"   {detail}" if detail else ""))


def skip(name, why=""):
    """An optional dependency is absent -- not a pass, and not a failure."""
    SKIP.append(name)
    print(f"  [skip] {name}" + (f"   {why}" if why else ""))


# ----------------------------------------------------------------------
def test_params():
    print("\nparameters and timing")
    sp = SimParams()
    check("dt_sub tiles dt_ctrl exactly",
          abs(sp.phys_per_ctrl * sp.dt_sub - sp.dt_ctrl) < 1e-12,
          f"{sp.phys_per_ctrl} x {sp.dt_sub}")
    p = RobotParams()
    check("pendulum is unstable (omega_n > 0)", p.omega_n > 1.0,
          f"omega_n = {p.omega_n:.2f} rad/s")
    check("effective mass A > body mass", p.A > p.m_body)

    gs = GainSpace()
    a = gs.gains_to_action(gs.nominal)
    g = gs.action_to_gains(a)
    check("gain <-> action round-trips", np.allclose(g, gs.nominal, rtol=1e-6),
          f"max err {np.max(np.abs(g - gs.nominal)):.2e}")
    check("nominal sits inside the bounds",
          bool(np.all(gs.nominal >= gs.low) and np.all(gs.nominal <= gs.high)))


def test_dynamics_signs():
    print("\ndynamics sign conventions")
    dyn = BalanceBotDynamics()

    s = make_state(theta=0.05)
    for _ in range(40):
        s = dyn.step(s, 0.0, 0.0, 0.0025)
    check("an unactuated lean falls further (inverted pendulum)",
          s[ITH] > 0.05, f"theta {s[ITH]:.4f}")

    s = make_state()
    for _ in range(200):
        s = dyn.step(s, 0.2, 0.2, 0.0025)
    check("positive wheel torque drives forward", s[IV] > 0.0,
          f"v {s[IV]:+.3f}")
    check("...and pitches the body backwards (non-minimum phase)",
          s[ITH] < 0.0, f"theta {s[ITH]:+.4f}")

    s = make_state()
    for _ in range(200):
        s = dyn.step(s, -0.15, 0.15, 0.0025)
    check("tau_r > tau_l turns left (yaw rate > 0)", s[IPSID] > 0.0,
          f"yaw rate {s[IPSID]:+.3f}")

    Ac, Bc = dyn.linearize()
    ev = np.linalg.eigvals(Ac)
    check("open loop has a right-half-plane pole",
          bool(np.any(ev.real > 0.5)), f"eigs {np.round(ev.real, 2)}")

    K = lqr_reference_gains(dyn)
    if K is None:
        skip("LQR solves on the linearised model",
             "scipy 没装（可选依赖），装上才会跑这一项")
    else:
        check("LQR solves on the linearised model", True,
              f"K = {np.round(K, 3)}")


def test_controller():
    print("\ncascade PID")
    p = RobotParams()
    sp = SimParams()
    gs = GainSpace()

    def run(theta0, v_ref, yaw_ref, T=14.0, gains=None):
        dyn = BalanceBotDynamics(p)
        pid = CascadePID(gs, tau_max=p.tau_max)
        pid.reset(gs.nominal if gains is None else np.asarray(gains))
        s = make_state(theta=theta0)
        tail_v, tail_y, tail_t, n = 0.0, 0.0, 0.0, 0
        for k in range(int(T / sp.dt_ctrl)):
            t = k * sp.dt_ctrl
            vr = v_ref if t > 1.0 else 0.0
            yr = yaw_ref if t > 1.0 else 0.0
            o = pid.step(s[ITH], s[6 + 1], s[IV], s[IPSID], vr, yr, sp.dt_ctrl)
            for _ in range(sp.phys_per_ctrl):
                s = dyn.step(s, o.tau_l, o.tau_r, sp.dt_sub)
            if abs(s[ITH]) > sp.pitch_fail:
                return None
            if t > T - 3.0:
                tail_v += abs(s[IV] - vr)
                tail_y += abs(s[IPSID] - yr)
                tail_t += abs(s[ITH])
                n += 1
        return tail_v / n, tail_y / n, tail_t / n

    r = run(0.10, 0.0, 0.0)
    check("recovers from a 5.7 deg lean and stands still",
          r is not None and r[0] < 0.05 and r[2] < 0.02,
          "" if r is None else f"|v| {r[0]:.3f}  |pitch| {r[2]:.4f}")

    r = run(0.02, 0.6, 0.0)
    check("tracks 0.6 m/s", r is not None and r[0] < 0.10,
          "" if r is None else f"speed error {r[0]:.3f} m/s")

    r = run(0.02, 0.4, 1.5)
    check("tracks 1.5 rad/s while driving",
          r is not None and r[1] < 0.15,
          "" if r is None else f"yaw error {r[1]:.3f} rad/s")

    r = run(0.30, 0.0, 0.0)
    check("survives a 17 deg initial lean", r is not None)

    bad = gs.nominal.copy()
    bad[0] = gs.low[0]
    bad[2] = gs.low[2]
    r = run(0.20, 0.5, 0.0, gains=bad)
    check("a badly detuned controller does noticeably worse",
          r is None or r[2] > 0.02,
          "fell" if r is None else f"|pitch| {r[2]:.4f}")

    # anti-windup: the integral term must stay bounded in output units
    pid = CascadePID(gs, tau_max=p.tau_max)
    pid.reset(gs.nominal)
    for _ in range(4000):
        pid.step(0.5, 0.0, 0.0, 0.0, 2.0, 0.0, sp.dt_ctrl)
    check("pitch integral respects its output-unit clamp",
          abs(pid.gains[1] * pid.i_pitch) <= pid.lim.i_pitch_out_max + 1e-6,
          f"{pid.gains[1] * pid.i_pitch:+.3f} N·m")
    check("velocity integral respects its output-unit clamp",
          abs(pid.gains[4] * pid.i_vel) <= pid.lim.i_vel_out_max + 1e-6)


def test_arena():
    print("\narena and ray casting")
    a = Arena(ArenaParams(half_x=4, half_y=4, n_obstacles=0, n_rays=8,
                          ray_max=2.5))
    a.set_obstacles(np.zeros((0, 3)))
    d = a.raycast(0.0, 0.0, 0.0)
    check("empty arena: every ray reaches its max", np.allclose(d, 2.5))

    a.set_obstacles([[1.0, 0.0, 0.2]])
    d = a.raycast(0.0, 0.0, 0.0)
    check("ray 0 hits an obstacle 1 m ahead", abs(d[0] - 0.8) < 1e-6,
          f"{d[0]:.4f} m (expected 0.80)")
    check("the backward ray is unobstructed", d[4] > 2.4)

    a.set_obstacles(np.zeros((0, 3)))
    d = a.raycast(3.0, 0.0, 0.0)
    check("walls are detected", abs(d[0] - 1.0) < 1e-6, f"{d[0]:.4f} m")
    check("clearance matches the wall distance",
          abs(a.clearance(3.0, 0.0) - 1.0) < 1e-9)

    a.set_obstacles([[0.5, 0.0, 0.3]])
    check("collision detection fires", a.in_collision(0.3, 0.0, 0.11))
    check("...and not when clear", not a.in_collision(-2.0, 0.0, 0.11))
    check("proximity penalty is 0 far away and >0 near",
          a.obstacle_penalty(-3.0, 0.0, 0.11) == 0.0
          and a.obstacle_penalty(0.95, 0.0, 0.11) > 0.0)


def test_env():
    print("\nenvironment")
    c = BalanceCore(seed=0, mode=MODE_PID)
    o = c.reset(seed=1)
    check("observation has the advertised shape", o.shape == (c.obs_dim,),
          f"{o.shape} vs {c.obs_dim}")
    check("observation is finite and bounded",
          bool(np.all(np.isfinite(o)) and np.all(np.abs(o) <= 10.0)))

    t_before = c.t
    c.step(np.zeros(9))
    check("one agent step advances exactly dt_agent",
          abs((c.t - t_before) - c.sim.dt_agent) < 1e-9,
          f"{c.t - t_before:.6f} s")

    c = BalanceCore(seed=3, mode=MODE_PID, randomize=False, obstacles=False,
                    disturbance=DisturbanceConfig())
    c.reset(seed=3)
    c.set_command(0.0, 0.0)
    for _ in range(c.sim.max_agent_steps):
        _, _, term, trunc, info = c.step(np.zeros(9))
        if term or trunc:
            break
    check("clean, undisturbed 20 s episode does not fall", not info["fell"],
          f"pitch {np.degrees(info['pitch']):+.2f} deg")

    def shove(force):
        cc = BalanceCore(seed=3, mode=MODE_PID, randomize=False,
                         obstacles=False, disturbance=DisturbanceConfig())
        cc.reset(seed=3)
        cc.set_command(0.0, 0.0)
        for _ in range(30):
            cc.step(np.zeros(9))
        cc.fire_impulse(force=force, direction=np.pi, duration=0.06)
        peak, fell = 0.0, False
        for _ in range(80):
            _, _, term, _, inf = cc.step(np.zeros(9))
            peak = max(peak, abs(inf["pitch"]))
            fell = inf["fell"]
            if term:
                break
        return peak, fell

    peak, fell = shove(9.0)
    check("a 9 N shove disturbs it but the PID recovers",
          peak > 0.02 and not fell,
          f"peak pitch {np.degrees(peak):.2f} deg")
    peak_big, fell_big = shove(25.0)
    check("a 25 N shove is genuinely too much for the fixed PID",
          fell_big or peak_big > peak,
          f"peak pitch {np.degrees(peak_big):.2f} deg, fell={fell_big}")

    # determinism
    a = BalanceCore(seed=7, mode=MODE_PID)
    b = BalanceCore(seed=7, mode=MODE_PID)
    oa, ob = a.reset(seed=42), b.reset(seed=42)
    same = np.allclose(oa, ob)
    for _ in range(30):
        oa = a.step(np.zeros(9))[0]
        ob = b.step(np.zeros(9))[0]
        same &= np.allclose(oa, ob)
    check("same seed -> identical trajectory", bool(same))


def test_policy_io():
    print("\npolicy save / load")
    c = BalanceCore(seed=0, mode=MODE_PPO_GAINS)
    gs = GainSpace()
    p = GainPolicy(c.obs_dim, 9, hidden=32, bias_init=gs.nominal_action)
    obs = c.reset(seed=5)
    a0 = p.act(obs)
    check("an untrained policy with a nominal bias emits ~nominal gains",
          np.allclose(gs.action_to_gains(a0), gs.nominal, rtol=0.35),
          f"max ratio {np.max(gs.action_to_gains(a0) / gs.nominal):.2f}")

    with tempfile.TemporaryDirectory() as d:
        fn = os.path.join(d, "p.npz")
        p.norm.update(np.random.default_rng(0).normal(size=(64, c.obs_dim)))
        p.save(fn)
        q = GainPolicy.load(fn)
        check("weights round-trip through .npz",
              np.allclose(p.act(obs), q.act(obs)))
        check("observation normaliser round-trips",
              np.allclose(p.norm.mean, q.norm.mean)
              and np.allclose(p.norm.var, q.norm.var))

    check("actions are clipped into [-1, 1]",
          bool(np.all(np.abs(p.act(obs * 50, deterministic=False,
                                   rng=np.random.default_rng(0))) <= 1.0)))


def test_mjcf():
    print("\nMuJoCo model generation")
    from balance_bot.mjcf import build_mjcf
    import xml.dom.minidom as minidom
    a = Arena(ArenaParams(n_obstacles=4), np.random.default_rng(0))
    a.randomize()
    xml = build_mjcf(RobotParams(), a)
    try:
        minidom.parseString(xml)
        ok = True
    except Exception as e:
        ok = False
        print("   ", e)
    check("generated MJCF is well-formed XML", ok)
    check("MJCF declares both motors",
          'name="m_l"' in xml and 'name="m_r"' in xml)
    check("MJCF contains the obstacles", xml.count("obs") >= 4)
    try:
        import mujoco
        m = mujoco.MjModel.from_xml_string(xml)
        check("MuJoCo compiles the model",
              m.nu == 2, f"nu={m.nu}, nq={m.nq}")
    except ImportError:
        print("  [skip] mujoco not installed -- compile check skipped")


def test_mujoco_twin():
    """The MuJoCo-only half of the twin: contact friction and wheel slip."""
    print("\nMuJoCo twin (contact model)")
    try:
        import mujoco
    except ImportError:
        print("  [skip] mujoco not installed")
        return
    from balance_bot.firmware.twin_baseline import make_mujoco_twin, MODE_STM32_LQR
    from balance_bot.params import DisturbanceConfig

    core = make_mujoco_twin(firmware=MODE_STM32_LQR, randomize=False,
                            obstacles=False, disturbance=DisturbanceConfig(),
                            arena=ArenaParams(half_x=3.0, half_y=3.0,
                                              n_obstacles=0),
                            episode_seconds=20.0)

    def survive(mu, steps=500):
        core.reset(seed=1)
        core.set_command(0.0, 0.0)
        for name in ("floor", "gw_l", "gw_r"):
            gid = mujoco.mj_name2id(core.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            core.model.geom_friction[gid, 0] = mu
        n, peak_w = 0, 0.0
        x0, y0 = float(core.data.qpos[0]), float(core.data.qpos[1])
        for _ in range(steps):
            _, _, _, _, info = core.step()
            n += 1
            peak_w = max(peak_w, abs(float(core.data.qvel[6])))
            if info["fell"]:
                break
        d = float(np.hypot(core.data.qpos[0] - x0, core.data.qpos[1] - y0))
        return n, peak_w, d

    n_grip, w_grip, _ = survive(1.0)
    check("the firmware stands on a grippy floor", n_grip == 500,
          f"{n_grip}/500 steps, peak wheel {w_grip:.1f} rad/s")

    # The motor's back-EMF has to be computed from the wheel's real speed, not
    # from v/r.  If it is not, a slipping wheel is told it is barely turning,
    # keeps getting near-stall torque and runs away -- which used to make any
    # mu below ~0.8 an instant, and entirely artificial, crash.  That is what
    # this check is for, and the peak-wheel-speed one below is its real teeth.
    n_slip, w_slip, _ = survive(0.6)
    check("it also stands on a slippery floor (mu = 0.6)", n_slip == 500,
          f"{n_slip}/500 steps, peak wheel {w_slip:.1f} rad/s")
    check("a slipping wheel does not run away past a few times free speed",
          w_slip < 5.0 * 19.6, f"peak {w_slip:.1f} rad/s vs 19.6 free")

    # 摩擦扫描（mjcf.py 换成 Euler + solref 0.020 之后实测，seed 1，零指令）：
    #   mu   1.00  0.60  0.40  0.30  0.20  0.10  0.05  0.02
    #   步数   500   500   500   248   208   500   500   500
    # 中间 0.2~0.3 有一条摔倒带，两侧都能活满，我还解释不了它，先如实记下来，
    # 不要为了让测试通过而假装它不存在。它**不是**托底判据造成的（把
    # bottoms_out 关掉，248/208 只变成 251/211），车是真的俯仰超了 40 度。
    #
    # 这里改测 0.6 而不是 0.3：这条断言的本意是「反电动势要用轮子真实转速算，
    # 否则打滑的轮子会失控狂转」，0.6 一样能测到（峰值轮速 42 rad/s，两倍于
    # 空载的 19.6），而 0.3 会撞上那条还没查清的摔倒带，把两件事混在一起。
    #
    # A 0.2-0.3 band falls while both sides survive; that is unexplained and
    # recorded rather than papered over.  It is not the bottoming-out rule
    # (disabling it moves 248/208 only to 251/211).  This check moved to 0.6,
    # which still exercises what it is actually for -- back-EMF from the real
    # wheel speed -- without straddling the unexplained band.

    # 「冰面会不会把车放倒」这条判据在这个项目里翻过三次：RK4 时代摔（其实是
    # 数值原因，车在跳）、换 Euler 后不摔（静态上稳住 1 度只要 0.17 N，而
    # mu=0.05 给得起 0.49 N）、补上 PWM 延迟又摔、加上转子惯量又不摔。
    # 它测的不是一个稳定的物理性质，而是刀刃上的偶然行为，每次改被控对象就
    # 翻一次面，翻第四次没有意义。
    #
    # 换成一个真正稳健的不变量：**冰面上车守不住位置**。不管它最后倒不倒，
    # 摩擦力不够就一定会滑开，这是 mu 决定的，不依赖控制环的相位裕度。
    #
    # "Does ice put it down" flipped four times as the plant improved -- it
    # sits on a knife edge, not on a stable property.  Replaced by one that
    # does not: on ice the car cannot hold station, whatever its final pose.
    # 这条判据在这个项目里已经翻了四次，每次改被控对象就翻一面：
    #   RK4 时代       摔 —— 数值原因（车在跳，离地 13.6%），不是被冰滑倒
    #   换 Euler       不摔 —— 静态上对：稳住 1 度只要 0.17 N，mu=0.05 给得起 0.49 N
    #   补 PWM 延迟     摔 —— 相位裕度不够，需要的修正比静态大得多
    #   补转子惯量      不摔 —— 惯量把响应压慢，不再需要那么猛的修正
    #
    # 我试过换成两个「更稳健」的不变量，都没立住：
    #   「冰面会滑开」 —— 零指令下没有东西推它，冰 40 mm vs 抓地 43 mm，一样
    #   「冰面加速不起来」 —— 这个孪生的指令接口在这里读不出速度，两边都是 0
    # 所以这里就如实锁住**当前行为**，并把上面这段历史留着：它不是一个稳定
    # 的物理性质，而是刀刃上的偶然结果，下次被控对象一改就该重新看，不要以为
    # 它失败就一定是回归。
    #
    # This check has flipped four times as the plant improved, and two attempts
    # at a "more robust" invariant both failed to hold.  It locks the current
    # behaviour and keeps the history: treat a future flip as a question, not
    # automatically as a regression.
    #   2026-09-16 真车回放拟合（弹性传动 + 新电机参数 + 延迟 2 拍） 摔
    #       LQR 12 步倒（关掉弹性传动也 12 步，原因是电机参数和延迟）；
    #       PID Normal 17 步倒（关掉弹性传动能站住）。扭转角全程 0.012 rad，
    #       不是数值发散。冰面上的真车没测过，这是模型在无实测工况下的外推，
    #       所以第五次翻面起**不再断言**，只报数。
    # From the 2026-09-16 replay fit on, reported only: ice was never measured
    # on the real car, and this has flipped five times.
    n_ice, _, _ = survive(0.05)
    print(f"  [--] mu=0.05 ice: {n_ice}/500 steps "
          f"(not asserted -- never measured on the real car)")

    # The wheel-speed hook must read the joints, not infer them.
    # 2026-09-16 起电机看到的是**转子**转速（弹性传动）；转子状态在复位后从
    # 轮子关节初始化，所以这里清掉转子状态再读，仍然验证「读关节、不推算」。
    # With drivetrain compliance the motor sees the rotor; the rotor state is
    # seeded from the wheel joints, which is what this still checks.
    core.reset(seed=1)
    core.data.qvel[6] = 7.5
    core.data.qvel[7] = -3.25
    core._rotor = None
    wl, wr = core._wheel_speeds()
    check("wheel speed comes from the wheel joints",
          abs(wl - 7.5) < 1e-9 and abs(wr + 3.25) < 1e-9,
          f"({wl:.3f}, {wr:.3f}) rad/s")

    # And the attitude filter must be reading the model's own IMU.
    a_adr, g_adr = core._imu_adr()
    check("the IMU hook is wired to the model's own sensors",
          a_adr >= 0 and g_adr >= 0,
          f"s_acc at {a_adr}, s_gyro at {g_adr}")


def main():
    print("=" * 66)
    print("balance-bot self-check")
    print("=" * 66)
    test_params()
    test_dynamics_signs()
    test_controller()
    test_arena()
    test_env()
    test_policy_io()
    test_mjcf()
    test_mujoco_twin()
    print("\n" + "=" * 66)
    print(f"{len(PASS)} passed, {len(FAIL)} failed"
          + (f", {len(SKIP)} skipped" if SKIP else ""))
    for name in SKIP:
        print("   SKIPPED:", name)
    if FAIL:
        for f in FAIL:
            print("   FAILED:", f)
    print("=" * 66)
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
