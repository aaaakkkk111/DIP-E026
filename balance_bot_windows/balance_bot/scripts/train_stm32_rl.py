"""PPO gain scheduling for the STM32 twin.

    python scripts/train_stm32_rl.py --steps 3000000 --n-envs 8
    python scripts/train_stm32_rl.py --firmware stm32_pid --out policy_pid.npz

**This is not ``balance_bot.train_sb3``.**  That one trains the *generic*
robot: a 33-dimensional observation, nine cascade-PID gains, and an export
format (``policy_io.GainPolicy``) the STM32 runtime cannot read.  This script
trains the car in TWIN_BASELINE.md: 13-dimensional observation, six firmware gains
(K1..K6 for the LQR build, or the six PID gains), and the flat ``.npz`` that
``balance_bot.stm32_policy.STM32InferencePolicy`` loads with nothing but
NumPy.  Confusing the two produces a file that silently fails to load in the
UI, so they are kept apart on purpose.

What the policy actually controls
---------------------------------
Not torque.  Every 40 ms it emits six numbers in [-1, 1], read as *log
multipliers* on the shipped firmware gains (``span ** action``, span = 3), so
**action = 0 is the firmware verbatim**.  The 200 Hz firmware loop underneath
is unchanged and still does the stabilising.  Training therefore starts from a
controller that already balances, and the agent spends its budget learning
*when to deviate* rather than rediscovering how to stand up.

Two things to keep honest
-------------------------
* ``--imu`` picks the attitude filter, and it is recorded in the ``.npz``.
  Scoring a policy against a baseline measured with a different ``--imu`` is
  comparing two different twins -- see TWIN_BASELINE.md 2, where that choice moves the
  PID baseline by 45 %.
* Training seeds and the benchmark's evaluation seeds must not overlap.
  ``bench_twin_baseline.py`` evaluates from 20000; this trains from ``--seed * 1000``,
  which is 0 by default.  Keep it that way, or move both deliberately.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from balance_bot.firmware.twin_baseline import (MODE_STM32_HYBRID,
                                                MODE_STM32_LQR,          # noqa: E402
                                       MODE_STM32_PID)
from balance_bot.stm32_env import STM32EnvFactory, STM32GainSpace  # noqa: E402


# --------------------------------------------------------------------------
def next_version(firmware, where="."):
    """Lowest unused vN for this firmware, so runs never overwrite each other."""
    import glob
    import re
    used = set()
    for f in glob.glob(os.path.join(where, f"policy_{firmware}_v*.npz")):
        m = re.search(r"_v(\d+)\.npz$", f)
        if m:
            used.add(int(m.group(1)))
    n = 1
    while n in used:
        n += 1
    return n


# --------------------------------------------------------------------------
def export_npz(model, vecnorm, out_path, firmware, imu_filter, meta=None,
               gain_names=(), per_gain_span=False,
               gain_spans=(), gain_low=(), gain_high=()):
    """SB3 policy -> the flat ``.npz`` STM32InferencePolicy reads.

    The runtime does ``tanh(x@w1+b1) -> tanh(@w2+b2) -> @w_out+b_out``, so the
    torch weights are transposed on the way out and the observation normaliser
    is folded in as plain mean/var arrays.  Keeping the runtime this dumb is
    the point: the board-side story is "numpy and nothing else".
    """
    import torch

    pol = model.policy
    lin = [m for m in pol.mlp_extractor.policy_net
           if isinstance(m, torch.nn.Linear)]
    if len(lin) != 2:
        raise RuntimeError(f"expected a 2-layer policy MLP, found {len(lin)}")

    W = lambda m: m.weight.detach().cpu().numpy().T.astype(np.float32)
    B = lambda m: m.bias.detach().cpu().numpy().astype(np.float32)

    blob = dict(
        w1=W(lin[0]), b1=B(lin[0]),
        w2=W(lin[1]), b2=B(lin[1]),
        w_out=W(pol.action_net), b_out=B(pol.action_net),
        firmware=np.array(firmware),
        # 动作维度已经不能唯一反推增益空间了（8 维 = 6+hold，9 维 = 再加
        # 死区），所以把名字直接写进来当事实源。
        # The action width no longer identifies the space, so record names.
        gain_names=np.array([str(n) for n in gain_names]),
        per_gain_span=np.array(bool(per_gain_span)),
        # 必须记**数值**，不能只记「用了逐增益跨度」这个布尔值。跨度常量在
        # 源码里，改一次就会把所有老模型重新解释成另一个控制器——#9 就这么
        # 被静默改掉过：同一份权重，顶风位置从 0.4252 m 变成 0.2745 m。
        # Record the span *values*, not just a flag: the constants live in
        # source, and editing them silently reinterprets every existing policy
        # as a different controller.
        gain_spans=np.array(gain_spans, dtype=np.float64),
        # low/high 和 spans 一样是源码常量，一样会静默重解释老模型，而且更
        # 隐蔽：贴边的维度全靠 high 截断，抬高一次上限就等于换了个控制器。
        # #9 实测：hold_kpsi 上限 15 -> 30，顶风位置 0.4252 -> 0.0643 m。
        # gain_low/high are source constants too, and matter more than spans
        # for any dimension that sits pinned: raising #9's hold_kpsi ceiling
        # from 15 to 30 moved its in-wind position from 0.4252 to 0.0643 m.
        gain_low=np.array(gain_low, dtype=np.float64),
        gain_high=np.array(gain_high, dtype=np.float64),
        imu_filter=np.array(imu_filter),
    )
    # Provenance: enough to tell two policies apart a month later, and to
    # know whether a score is comparable with a given baseline.
    for k, v in (meta or {}).items():
        blob[k] = np.array(v)
    if vecnorm is not None:
        blob.update(obs_mean=vecnorm.obs_rms.mean.astype(np.float32),
                    obs_var=vecnorm.obs_rms.var.astype(np.float32),
                    obs_clip=np.array(float(vecnorm.clip_obs), dtype=np.float32))
    np.savez(out_path, **blob)
    return out_path


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--firmware",
                    choices=(MODE_STM32_LQR, MODE_STM32_PID,
                             MODE_STM32_HYBRID),
                    default=MODE_STM32_LQR)
    # 2026-09-10 实测（4000 步 x 3 次，取最好）：mujoco 1400 us/步、
    # analytic 1974 us/步——**MuJoCo 反而快 1.4 倍**。旧的「analytic 快 20 倍」
    # 是错的：两边的固件仿真都是 Python，差别只在被控对象，而 analytic 的
    # 800 Hz 子步在 NumPy 里跑，MuJoCo 的在 C 里跑。
    # 加上 analytic 没有接触模型，和 MuJoCo 在已标定的行为上差到 37 倍
    # （负载+4kg：摆 0.06° vs 2.23°），拿它训练再部署到 MuJoCo 是无效的。
    # Measured: MuJoCo is 1.4x FASTER, not 20x slower.  The old claim was
    # wrong -- the firmware emulation is Python on both sides, and MuJoCo's
    # plant steps in C while the analytic one steps in NumPy.
    ap.add_argument("--backend", choices=("analytic", "mujoco"),
                    default="mujoco",
                    help="mujoco（默认）有接触模型，实测还比 analytic 快 1.4 倍")
    ap.add_argument("--imu", choices=("kalman", "complementary", "dmp"), default="kalman",
                    help="姿态滤波模型；评测时 bench_twin_baseline.py --imu 必须一致")
    # 【2026-09-10 实测吞吐，20 核，MuJoCo 后端，CPU-only torch】
    #   n_envs=8   1209 步/秒   -> 300 万步 41 分钟
    #   n_envs=16  2086 步/秒   -> 300 万步 24 分钟
    #   n_envs=20  2050 步/秒   -> 300 万步 24 分钟   <- 机器满了
    #   n_envs=24  1979 步/秒   -> 超订，反而更慢
    # 40 分钟大约够跑：n_envs=8 -> 290 万步，n_envs=20 -> 490 万步。
    #
    # 开销**不在物理**：把物理从 800 Hz 降到 400 Hz（子步 4->2）只快 2%，
    # 降到 200 Hz 也才快 40%，而且行为全变（LQR 静止摆幅 0.55° -> 20.87°）。
    # 真正的开销是每个智能体拍要跑 8 个固件拍的 Python 仿真。所以别动 dt_phys，
    # 要更快就加环境数或减步数。
    #
    # Measured throughput on 20 cores.  The cost is the Python firmware
    # emulation (8 ticks per agent step), NOT the physics: halving the
    # physics rate buys 2 % and wrecks the calibrated behaviour.
    ap.add_argument("--steps", type=int, default=3_000_000)
    ap.add_argument("--n-envs", type=int, default=8)
    # n_steps x n_envs = 一次 rollout 收集多少 transition。改 n_envs 而不改
    # n_steps 会连带改掉 PPO 的更新次数（8 环境 x 512 = 4096；20 环境 x 512 =
    # 10240，同样步数下梯度更新只有 40%），那不是纯粹的提速。想只提速不动算法，
    # 把 n_envs 提到 20 的同时把 n_steps 降到 205（20 x 205 = 4100 ~= 4096）。
    #
    # Changing n_envs alone also changes the rollout size and therefore the
    # number of PPO updates per step; scale n_steps down to compensate.
    ap.add_argument("--n-steps", type=int, default=512,
                    help="每个环境每次 rollout 的步数；n_steps x n_envs "
                         "是 rollout 大小（默认 8x512=4096）")
    ap.add_argument("--batch-size", type=int, default=1024)
    ap.add_argument("--n-epochs", type=int, default=10)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--episode-seconds", type=float, default=20.0)
    ap.add_argument("--curriculum", type=float, default=0.5,
                    help="ramp difficulty 0->1 over this fraction of training")
    ap.add_argument("--hold", action="store_true",
                    help="打开站定外环：观测多出「相对锚点的前向/横向/航向"
                         "误差」，动作多出两个站定增益，奖励多出位置和航向"
                         "两项。出厂固件没有航向状态，所以「松手后朝向不变」"
                         "只能这么做，见 twin_baseline.HOLD_* 的注释")
    ap.add_argument("--per-gain-span", action="store_true",
                    help="逐增益的搜索跨度而不是统一 3 倍。#8 有四个增益贴死"
                         "在边界上（balance_kd/velocity_ki 贴下限，turn_kp"
                         "贴上限），说明是空间截断了它")
    ap.add_argument("--no-chatter", action="store_true",
                    help="关掉 PWM 换向惩罚。这一项不在用户提的三个症状里，"
                         "而在 45%% 死区的执行器上，低速控制权限就是靠换向"
                         "换来的——原厂 0.0076 m 的抗风位置是用 16.6 次/秒"
                         "买的。#7~#9 抖动 12.2->5.8->0.0 的同时，风中位置"
                         "0.1256->0.1068->0.4252、跟踪 0.92->0.79->0.38")
    ap.add_argument("--out", default="")
    ap.add_argument("--version", type=int, default=0,
                    help="强制模型编号；默认按固件自动取最小未用的 vN。"
                         "编号在 UI 下拉框里显示为「PPO #N」，是跨固件连续读的，"
                         "所以给 PID 底座的第一个模型指定 --version 2 是合理的")
    ap.add_argument("--logdir", default="runs_stm32",
                    help="tensorboard 日志目录；没装 tensorboard 会自动关掉")
    ap.add_argument("--device", default="cpu",
                    help="cpu is usually faster than cuda for a 64x64 MLP")
    args = ap.parse_args()

    if args.out:
        out = args.out
        version = args.version
    else:
        version = args.version or next_version(args.firmware)
        out = f"policy_{args.firmware}_v{version}.npz"
        if os.path.exists(out):
            raise SystemExit(f"{out} 已存在，换一个 --version 或删掉它")

    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.vec_env import (DummyVecEnv, SubprocVecEnv,
                                                  VecMonitor, VecNormalize)

    # Windows has no fork: SubprocVecEnv spawns interpreters and pickles the
    # factory to reach them, which is why STM32EnvFactory is a class and not
    # a closure.
    fns = [STM32EnvFactory(i, args.seed * 1000, args.firmware, args.backend,
                           0.0, args.imu, args.episode_seconds,
                           hold_station=args.hold,
                           per_gain_span=args.per_gain_span,
                           penalise_chatter=not args.no_chatter)
           for i in range(args.n_envs)]
    VecCls = SubprocVecEnv if args.n_envs > 1 else DummyVecEnv
    venv = VecMonitor(VecCls(fns))
    venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)

    # tensorboard is an optional extra; SB3 raises rather than degrading if
    # it is missing, so degrade here instead of making it a hard dependency.
    logdir = args.logdir
    if logdir:
        try:
            import tensorboard          # noqa: F401
        except ImportError:
            print("[提示] 没装 tensorboard，本次不写日志"
                  "（pip install tensorboard 就有了）")
            logdir = None

    gs = STM32GainSpace(firmware=args.firmware, hold=args.hold,
                        per_gain=args.per_gain_span)
    gain_names = tuple(gs.names)
    print(f"固件底座 {args.firmware}   姿态滤波 {args.imu}   "
          f"后端 {args.backend}")
    print(f"动作 = {gs.dim} 个增益的对数倍数 ({', '.join(gs.names)})，"
          f"0 就是原厂值")

    class Curriculum(BaseCallback):
        """Ramp the disturbance ladder instead of starting at full difficulty.

        Starting hard is the classic way to get a flat reward curve: the
        firmware baseline falls in almost every episode, so nearly every
        rollout ends the same way and the advantage estimate is noise.
        """

        def _on_step(self) -> bool:
            frac = (self.num_timesteps / max(args.steps, 1)) / max(
                args.curriculum, 1e-6)
            self.training_env.env_method("set_difficulty", min(1.0, frac))
            return True

    model = PPO(
        "MlpPolicy", venv, verbose=1, device=args.device, seed=args.seed,
        n_steps=args.n_steps, batch_size=args.batch_size,
        n_epochs=args.n_epochs, gamma=0.99,
        gae_lambda=0.95, clip_range=0.2, ent_coef=0.002,
        learning_rate=3e-4, tensorboard_log=logdir,
        policy_kwargs=dict(net_arch=dict(pi=[64, 64], vf=[64, 64]),
                           activation_fn=torch.nn.Tanh),
    )

    # Start at the firmware: zero action == nominal gains, so a zero-mean
    # action head means training begins from a controller that already works.
    with torch.no_grad():
        model.policy.action_net.weight.mul_(0.01)
        model.policy.action_net.bias.zero_()

    model.learn(total_timesteps=args.steps, callback=Curriculum(),
                progress_bar=False)

    stem = os.path.splitext(out)[0]
    model.save(f"{stem}_sb3")
    venv.save(f"{stem}_vecnorm.pkl")
    import datetime
    export_npz(model, venv, out, args.firmware, args.imu,
               gain_names=gain_names, per_gain_span=args.per_gain_span,
               gain_spans=tuple(gs.spans),
               gain_low=tuple(gs.low), gain_high=tuple(gs.high),
               meta=dict(
        version=version,
        trained_utc=datetime.datetime.now(datetime.timezone.utc)
                    .strftime("%Y-%m-%d %H:%M"),
        steps=int(args.steps),
        backend=args.backend,
        n_envs=int(args.n_envs),
        curriculum=float(args.curriculum),
        seed=int(args.seed),
    ))

    print(f"\n写出 -> {out}  (+ {stem}_sb3.zip, {stem}_vecnorm.pkl)")
    print(f"现在打分（--imu 必须和训练时一致）：\n"
          f"  python scripts/bench_twin_baseline.py --imu {args.imu} --episodes 40\n"
          f"把 {out} 放在工程根目录，scripts/sim_gui.py 会自动把它列进"
          f"「控制模型」下拉框。")


if __name__ == "__main__":
    main()
