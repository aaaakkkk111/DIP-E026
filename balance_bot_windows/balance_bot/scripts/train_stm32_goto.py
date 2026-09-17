"""训练一套**静态**、能烧进板子的 PID 增益，玩「开到指定点并停稳」。

Train one static, flashable PID gain set for the drive-to-a-point game.

    python scripts/train_stm32_goto.py --steps 4500000 --n-envs 20 --n-steps 205

产物三个：
    <out>.npz   增益 + 元数据（给 bench / GUI 用）
    <out>.json  同样的增益，人看的
    <out>.c     **可以直接抄进板子的 C**：6 个 PID 常量 + 导航外环

为什么策略是「状态无关」的
--------------------------
观测恒为 0，动作在开局冻结整局（见 ``stm32_goto_env`` 模块说明），所以学出来的
就是一组常数，而不是一个需要在车上跑神经网络的增益调度器。PPO 在这里退化成
带 clip 的 REINFORCE——对 8 个参数完全够用，代价是每步梯度信息少，靠回合数补。

The observation is constant and the action is frozen per episode, so what PPO
learns is literally one gain vector: no network needs to run on the board.
"""
from __future__ import annotations

import argparse
import datetime
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from balance_bot.stm32_goto_env import (          # noqa: E402
    STM32GotoEnv, NAV_V_MAX, NAV_YAW_MAX, ARRIVE_R, PAYLOAD_LADDER)
from balance_bot.dynamics import ITH               # noqa: E402

# 固件把增益写成整数、用的时候再除以 100（pid_control.c: Balance_Kp/100）。
# 孪生里已经把 /100 折进去了，导出到 C 必须乘回来，否则烧进去差 100 倍。
# The firmware stores gains x100; the twin folds the /100 in, so the C export
# has to multiply back or the board gets gains 100x too small.
C_SCALE = {"balance_kp": 100.0, "balance_kd": 100.0,
           "velocity_kp": 100.0, "velocity_ki": 100.0,
           "turn_kp": 100.0, "turn_kd": 100.0}
C_NAME = {"balance_kp": "Balance_Kp", "balance_kd": "Balance_Kd",
          "velocity_kp": "Velocity_Kp", "velocity_ki": "Velocity_Ki",
          "turn_kp": "Turn_Kp", "turn_kd": "Turn_Kd"}


def make_const_actor_policy():
    """actor 看不见观测，critic 看得见。

    观测是回合上下文（载重、目标远近）。critic 需要它——不然不同载重的回报
    差着量级，价值函数只能预测一个全局均值，优势退化成纯噪声（实测
    explained_variance = 6e-8、clip_fraction = 0，策略几乎不动）。
    actor 绝不能看它：一旦策略能按载重改增益，产物就变成「按载重切换」，
    正是要避免的。这里把 actor 那一路的输入强制置零，从结构上保证
    **学出来的就是一组常数**，任何载重下都是同一组。

    The critic sees the episode context; the actor's input is zeroed, so the
    learned policy is provably one constant vector for every payload.
    """
    import torch as th
    from stable_baselines3.common.policies import ActorCriticPolicy

    class ConstActorPolicy(ActorCriticPolicy):
        def extract_features(self, obs, features_extractor=None):
            if features_extractor is None:
                out = super().extract_features(obs)
                pi_f, vf_f = out            # share_features_extractor=False
                return th.zeros_like(pi_f), vf_f
            f = super().extract_features(obs, features_extractor)
            if features_extractor is self.pi_features_extractor:
                return th.zeros_like(f)
            return f

        def get_distribution(self, obs):
            # ActorCriticPolicy.get_distribution 里写的是
            # ``super().extract_features(...)``（即 BaseModel 的），会绕过上面
            # 那个重载——predict() 走的正是这条路。所以这里直接把原始观测清零。
            # 这个坑是被脚本末尾那个 spread 自检抓出来的（当时 spread=4.5e-2）。
            # get_distribution calls BaseModel.extract_features directly,
            # bypassing the override above; predict() takes that path.
            return super().get_distribution(th.zeros_like(obs))

    return ConstActorPolicy


def worst_case(per):
    """最差载重的表现决定分数 —— 用户要的是「任何负载都能用」。

    平均分会允许策略牺牲 4 kg 去换 0 kg 的漂亮数字。这里按最差档打分：
    先看有没有哪档摔了，再比最差档的胜率和最近距离。
    Score by the WORST payload, not the mean: an average would let the policy
    trade 4 kg away for a prettier 0 kg number.
    """
    worst_win, worst_d, fell_any = 1.0, 0.0, False
    for kg, p in per.items():
        if not p["n"]:
            continue
        wr = p["win"] / p["n"]
        worst_win = min(worst_win, wr)
        worst_d = max(worst_d, float(np.mean(p["d"])))
        if p["arr"] == 0:
            fell_any = True
    return worst_win * 100.0 - worst_d * 100.0 - (50.0 if fell_any else 0.0)


def evaluate(gains_action, seeds=range(20), episode_seconds=25.0,
             difficulty=0.0):
    """在固定的一组种子上评估一个动作向量，逐档载重给结果。

    ``difficulty`` 必须和训练时一致，否则报出来的是「干净世界」的成绩。
    """
    env = STM32GotoEnv(seed=0, episode_seconds=episode_seconds,
                       randomize=difficulty > 0.0)
    env.set_difficulty(difficulty)
    per = {kg: dict(n=0, win=0, arr=0, t=[], d=[], sway=[])
           for kg in PAYLOAD_LADDER}
    tot = 0.0
    for sd in seeds:
        env.reset(seed=int(sd))
        kg = env.payload
        r_ep, best, sway = 0.0, 9.9, 0.0
        while True:
            _, r, term, trunc, info = env.step(gains_action)
            r_ep += r
            best = min(best, info["dist"])
            sway = max(sway, abs(float(env.core.state[ITH])))
            if term or trunc:
                break
        p = per[kg]
        p["n"] += 1
        p["win"] += int(info["won"])
        p["arr"] += int(info["arrived"] is not None)
        p["d"].append(best)
        p["sway"].append(sway)
        if info["arrived"] is not None:
            p["t"].append(info["arrived"])
        tot += r_ep
    return tot / len(list(seeds)), per


def print_eval(tag, score, per):
    print(f"\n{tag}  平均奖励 {score:.1f}")
    print(f'{"载重":>6s}{"局数":>5s}{"到达":>7s}{"胜利":>7s}'
          f'{"到达用时":>10s}{"最近距离":>10s}{"峰值摆幅":>10s}')
    for kg, p in per.items():
        if not p["n"]:
            continue
        t = f'{np.mean(p["t"]):8.2f}s' if p["t"] else "      --"
        print(f'{kg:5.1f}kg{p["n"]:5d}{p["arr"]:5d}/{p["n"]:<2d}'
              f'{p["win"]:5d}/{p["n"]:<2d}{t}'
              f'{np.mean(p["d"]) * 1000:8.0f}mm'
              f'{np.degrees(np.mean(p["sway"])):9.2f}°')


def export_c(path, gains, meta):
    """写出可以直接抄进板子的 C。"""
    g = gains
    lines = [
        "/* ------------------------------------------------------------------",
        " * PPO 调出来的静态 PID 增益 + 导航外环",
        f" * 生成时间 {meta['when']}   训练步数 {meta['steps']:,}",
        " *",
        " * 用法：",
        " *   1) 用下面的常量替换 APP/PID/pid_control.c 顶部那六个 float。",
        " *      注意固件用的是 x100 的整数形式，这里已经乘回去了。",
        " *   2) 把 nav_step() 抄进 APP/app_control.c，在 200 Hz 中断里、",
        " *      调用 Balance_PD/Velocity_PI/Turn_PD **之前**调用它，用它的",
        " *      输出覆盖 Move_X / Move_Z。",
        " *   3) 见文件末尾「板子上还缺什么」。",
        " * ------------------------------------------------------------------ */",
        "",
        "#include <math.h>",
        "",
        "/* app_control.c 里已经有 PI 了；单独编译这个文件时才需要这个兜底 */",
        "#ifndef PI",
        "#define PI 3.14159265358979f",
        "#endif",
        "",
        "/* ---- 1. PID 增益（替换 pid_control.c 顶部）---- */",
    ]
    for k, cname in C_NAME.items():
        lines.append(f"float {cname:<12s} = {g[k] * C_SCALE[k]:.1f}f;"
                     f"   /* 孪生里的 {k} = {g[k]:.4g} */")
    lines += [
        "",
        "/* 负载模式的三个系数必须全部设为 1.0：这套增益就是为 0~4 kg 全程",
        " * 训练的，再乘 Balance_K 会把 D 项放大，那正是空车振荡的原因。 */",
        "float Balance_K  = 1.0f;",
        "float Velocity_K = 1.0f;",
        "float Turn_K     = 1.0f;",
        "",
        "/* ---- 2. 导航外环（抄进 app_control.c）---- */",
        "/* 固件原本没有位置/航向反馈，转向环是纯前馈，所以再怎么调 PID 也",
        " * 开不到指定坐标。这段就是缺的那条通路。 */",
        "",
        f"#define NAV_KV      {g['nav_kv']:.4f}f   /* 距离 -> 速度，1/s */",
        f"#define NAV_KW      {g['nav_kw']:.4f}f   /* 方位角 -> 偏航，1/s */",
        f"#define NAV_V_MAX   {NAV_V_MAX:.3f}f",
        f"#define NAV_W_MAX   {NAV_YAW_MAX:.3f}f",
        f"#define NAV_ARRIVE  {ARRIVE_R:.3f}f   /* m，进这个圈算到了 */",
        "",
        "/* 车当前位姿，由 nav_odom() 递推；目标点由上位机/按键设定 */",
        "float nav_x = 0, nav_y = 0, nav_psi = 0;",
        "float nav_tx = 0, nav_ty = 0;",
        "",
        "void nav_step(float *out_v, float *out_w)",
        "{",
        "    float dx = nav_tx - nav_x;",
        "    float dy = nav_ty - nav_y;",
        "    float dist = sqrtf(dx * dx + dy * dy);",
        "    float bearing = atan2f(dy, dx) - nav_psi;",
        "    while (bearing >  PI) bearing -= 2.0f * PI;",
        "    while (bearing < -PI) bearing += 2.0f * PI;",
        "",
        "    /* 符号：Motor_Left = ...+Turn_Pwm 让左轮快、车顺时针转，而 psi",
        "     * 是逆时针为正，所以指令要取负。写反了车会越转越远。 */",
        "    float w = -NAV_KW * bearing;",
        "    if (w >  NAV_W_MAX) w =  NAV_W_MAX;",
        "    if (w < -NAV_W_MAX) w = -NAV_W_MAX;",
        "",
        "    if (dist <= NAV_ARRIVE) { *out_v = 0.0f; *out_w = 0.0f; return; }",
        "",
        "    /* cos 门控：没对准目标就先转、少走，对准了才全速 */",
        "    float gate = cosf(bearing);",
        "    if (gate < 0.0f) gate = 0.0f;",
        "    float v = NAV_KV * dist;",
        "    if (v >  NAV_V_MAX) v =  NAV_V_MAX;",
        "    if (v < -NAV_V_MAX) v = -NAV_V_MAX;",
        "",
        "    *out_v = v * gate;",
        "    *out_w = w;",
        "}",
        "",
        "/* ---- 3. 里程计递推，每个 200 Hz 拍调一次 ---- */",
        "/* 4 x 11 x 30 = 1320 计数/圈（app_motor.h），轮径 67 mm，轮距 167 mm */",
        "#define CPR         1320.0f",
        "#define WHEEL_D     0.067f",
        "#define TRACK       0.167f",
        "",
        "void nav_odom(int enc_l, int enc_r)",
        "{",
        "    float sl = (float)enc_l / CPR * PI * WHEEL_D;",
        "    float sr = (float)enc_r / CPR * PI * WHEEL_D;",
        "    float ds = 0.5f * (sl + sr);",
        "    float dpsi = (sr - sl) / TRACK;",
        "    nav_psi += dpsi;",
        "    nav_x += ds * cosf(nav_psi);",
        "    nav_y += ds * sinf(nav_psi);",
        "}",
        "",
        "/* ------------------------------------------------------------------",
        " * 板子上还缺什么（老实说明，别以为抄完就能跑）",
        " *",
        " * a) nav_odom 用的是编码器递推的航向。板子上也有 angle_z，但",
        " *    app_control.c 里它被算错了：Wheel_spacing 是 161.0 毫米，代码",
        " *    先除 Wheel_spacing 再除 1000，等于除了 161000 而不是 0.161，",
        " *    偏航反馈项被缩掉 10^6 倍。要用 angle_z 就得先修这个标度。",
        " *    上面的 nav_odom 绕开了它，只用编码器——代价是长时间会累积漂移。",
        " *",
        " * b) TRACK 用的是 STEP 装配量出来的 167 mm，不是固件里写的 161 mm。",
        " *    固件那个值偏小 3.6%，会让递推出来的航向偏快。",
        " *",
        " * c) 里程计只用编码器，打滑就丢定位。这套增益在孪生里是在**不打滑**",
        " *    的地面上训的。",
        " *",
        " * d) 本文件用 gcc -std=c99 -Wall -Wextra 编译无警告并跑通过",
        " *    （nav_odom + nav_step 的数值和 Python 侧一致）。但**没有在",
        " *    Keil / STM32F103 上编过**：Cortex-M3 没有硬件浮点，这里的",
        " *    sqrtf/atan2f/cosf 走软件库，在 200 Hz 中断里要量一下耗时。",
        " *    真嫌慢就把 nav_step 降到 50 Hz 调用（导航外环不需要 200 Hz）。",
        " *",
        " * e) 这些增益是在数字孪生里训的，孪生本身对真车的标定见 TWIN_BASELINE.md。",
        " *    上车第一次务必**空载、低速、有人扶**着试。",
        " * ------------------------------------------------------------------ */",
        "",
    ]
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    # 一步 = 一整局 25 s 试验（bandit）。约 1.34 s/局，20 并行 -> 约 15 局/秒
    ap.add_argument("--steps", type=int, default=28_000,
                    help="试验局数，不是仿真步数")
    ap.add_argument("--n-envs", type=int, default=20)
    ap.add_argument("--n-steps", type=int, default=8)
    ap.add_argument("--batch-size", type=int, default=40)
    ap.add_argument("--n-epochs", type=int, default=10)
    ap.add_argument("--episode-seconds", type=float, default=25.0)
    ap.add_argument("--backend", choices=("mujoco", "analytic"),
                    default="mujoco")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="policy_stm32_goto.npz")
    ap.add_argument("--logdir", default="runs_stm32_goto")
    # 训练时的域随机化强度。0 = 完全干净的世界（第一版就是这么训的，学出来的
    # 增益只保证理想条件下能赢）。bench_twin_baseline 里难度 0.25 原厂两套都
    # 满分存活、0.50 开始掉，所以 0.30 是「有压力但不是送死」的档。
    # Domain randomisation strength; 0 is the spotless world the first run used.
    ap.add_argument("--difficulty", type=float, default=0.30)
    ap.add_argument("--lr", type=float, default=3e-3)
    ap.add_argument("--init-std", type=float, default=0.40,
                    help="初始探索幅度，动作单位；0.4 约等于增益 1.5 倍")
    ap.add_argument("--device", default="cpu")
    args = ap.parse_args()

    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

    def mk(i):
        def f():
            # 每个环境的载重相位错开，保证一个 rollout 里五档都出现
            e = STM32GotoEnv(seed=args.seed + i, backend=args.backend,
                             episode_seconds=args.episode_seconds,
                             payload_phase=i,
                             randomize=args.difficulty > 0.0)
            e.set_difficulty(args.difficulty)
            return e
        return f

    VecCls = SubprocVecEnv if args.n_envs > 1 else DummyVecEnv
    venv = VecCls([mk(i) for i in range(args.n_envs)])

    model = PPO(make_const_actor_policy(), venv, verbose=1, device=args.device,
                seed=args.seed, n_steps=args.n_steps,
                batch_size=args.batch_size, n_epochs=args.n_epochs,
                # bandit 超参：一步一局，没有时序信用分配可言。
                #   gamma / gae_lambda 无意义（每局都是终止步），保留默认值不影响。
                #   学习率比长回合那套高一个量级：每次更新只有 n_envs*n_steps 个
                #     样本，3e-4 下 175 次更新几乎不动均值（实测 clip_fraction=0）。
                #   ent_coef 归零：熵奖励会把 std 撑在 1.0，bandit 里那等于永远
                #     在整个增益范围内乱撒，收敛不了。
                # Bandit hyper-parameters: one step per trial, so there is no
                # temporal credit assignment; the learning rate has to be an
                # order of magnitude higher and the entropy bonus has to go.
                gamma=0.99, gae_lambda=0.95, clip_range=0.2,
                ent_coef=0.0, learning_rate=args.lr,
                target_kl=0.05,   # lr 调高之后 approx_kl 摸到 0.06，加个闸
                tensorboard_log=args.logdir,
                policy_kwargs=dict(net_arch=dict(pi=[32], vf=[64, 64]),
                                   share_features_extractor=False,
                                   activation_fn=torch.nn.Tanh))
    # 从固件出发：零动作 = 原厂增益，所以动作头置零意味着训练从一个已经
    # 能站住的控制器开始。 Start at the firmware.
    with torch.no_grad():
        model.policy.action_net.weight.mul_(0.01)
        model.policy.action_net.bias.zero_()
        # 初始 std=1 意味着每次采样都在整个 3 倍增益范围里乱撒，太粗。
        # std=1 explores the whole 3x gain range every sample -- far too coarse.
        model.policy.log_std.data.fill_(float(np.log(args.init_std)))

    model.learn(total_timesteps=args.steps, progress_bar=False)
    venv.close()

    # actor 与观测无关，喂什么都一样；用零上下文取确定性动作
    obs = np.zeros((1, 4), dtype=np.float32)
    act, _ = model.predict(obs, deterministic=True)
    act = np.clip(np.asarray(act, dtype=float).ravel(), -1.0, 1.0)

    probe = STM32GotoEnv(seed=0, episode_seconds=args.episode_seconds)
    names = list(probe.gain_space.names)

    # 自检：actor 真的与观测无关吗？保证产物是**一组**数，不是按载重切换
    spread = 0.0
    for c in (np.zeros((1, 4), np.float32), np.ones((1, 4), np.float32),
              -np.ones((1, 4), np.float32),
              np.array([[1.0, 0.3, -0.7, 0.7]], np.float32)):
        a2, _ = model.predict(c, deterministic=True)
        spread = max(spread, float(np.max(np.abs(np.asarray(a2).ravel() - act))))
    print(f"\n策略对观测的依赖（应为 0）：{spread:.2e}")
    assert spread < 1e-6, "actor 看见观测了，产物不是单一增益集"

    # 最差载重决定：从策略均值和几个扰动里挑「任何负载都能用」的那个
    print("\n候选筛选（按最差载重打分，不看平均）")
    cands = [("策略均值", act)]
    rng = np.random.default_rng(0)
    for i in range(6):
        cands.append((f"扰动{i}", np.clip(act + rng.normal(0, 0.08, act.size),
                                          -1.0, 1.0)))
    best = None
    for tag, a in cands:
        sc, per = evaluate(a, seeds=range(10),
                           episode_seconds=args.episode_seconds,
                           difficulty=args.difficulty)
        wc = worst_case(per)
        wins = "/".join(str(per[k]["win"]) for k in PAYLOAD_LADDER)
        print(f"  {tag:<8s} 最差档评分 {wc:7.1f}  平均奖励 {sc:8.1f}  各档胜利 {wins}")
        if best is None or wc > best[0]:
            best = (wc, a.copy(), tag)
    if best[2] != "策略均值":
        print(f"  -> 选中「{best[2]}」，它在最差载重上更好")
    act = best[1]
    gains = probe.gains_from_action(act)

    print("\n" + "=" * 70)
    print("学到的静态增益（0~4 kg 全程同一组）")
    for n in names:
        print(f"  {n:<14s}{gains[n]:10.4f}"
              f"   (原厂 {probe.gain_space.nominal[names.index(n)]:.4g})")

    s0, p0 = evaluate(np.zeros(len(names)),
                      episode_seconds=args.episode_seconds,
                      difficulty=args.difficulty)
    print_eval("原厂增益基线", s0, p0)
    s1, p1 = evaluate(act, episode_seconds=args.episode_seconds,
                      difficulty=args.difficulty)
    print_eval("PPO 增益", s1, p1)
    print(f"\n最差载重评分：原厂 {worst_case(p0):.1f}  ->  PPO {worst_case(p1):.1f}")

    stem = os.path.splitext(args.out)[0]
    meta = dict(when=datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
                difficulty=float(args.difficulty),
                arrive_r=float(ARRIVE_R),
                steps=int(args.steps), backend=args.backend,
                episode_seconds=float(args.episode_seconds),
                score_stock=float(s0), score_ppo=float(s1))
    np.savez(stem + ".npz", action=act,
             gain_names=np.array(names), gains=np.array([gains[n] for n in names]),
             nominal=probe.gain_space.nominal, **{k: np.array(v)
                                                  for k, v in meta.items()})
    with open(stem + ".json", "w", encoding="utf-8") as f:
        json.dump(dict(name="PPO goto (static PID)", firmware="stm32_pid",
                       gains={k: float(v) for k, v in gains.items()},
                       _provenance=meta), f, indent=2, ensure_ascii=False)
    export_c(stem + ".c", gains, meta)
    print(f"\n写出 -> {stem}.npz / {stem}.json / {stem}.c")


if __name__ == "__main__":
    main()
