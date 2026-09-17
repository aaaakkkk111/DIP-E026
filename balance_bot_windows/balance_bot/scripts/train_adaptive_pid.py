"""PPO 训练：按情境自适应改 PID 增益。
PPO training for situation-adaptive PID gains.

    python scripts/train_adaptive_pid.py --steps 300000 --n-envs 8
    python scripts/export_nn_c.py runs_stm32/adapt_01.npz     # 导成 C

和 `train_stm32_rl.py` 的分工
-----------------------------
那个脚本训的是**一台车、一种工况**下的增益调制。这个脚本换成
`STM32AdaptEnv`：逐局随机抽情境（负重 / 坡道 / 台阶跌落 / 冲击 / 混合），
观测里多了 8 个情境特征，网络默认缩到 32x32 好塞进 F103。

三条为了「能上车」而定的规矩
----------------------------
1. **不做观测归一化**（`--obs-norm` 才开）。VecNormalize 的滑动均值方差会变成
   板子上另一组必须同步的常数，而特征本来就已经缩放到 ±1 附近了。
2. **网络小**：默认 32x32。21-32-32-6 定点后 4.2 KB。
3. **训完必须量化复评**：脚本最后自动跑一遍定点网络的评测，float 和 q15 两个
   分数都打出来。差太多就别上车。

Three deployment-driven rules: no observation normalisation (it would become
another set of constants to keep in sync), a small net, and an automatic
fixed-point re-score at the end -- the quantised policy is what ships.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from balance_bot.nn_q15 import Q15Net                      # noqa: E402
from balance_bot.scenarios import SCENARIOS                # noqa: E402
from balance_bot.stm32_adapt_env import (STM32AdaptEnv,    # noqa: E402
                                         STM32AdaptEnvFactory)


def export_npz(model, out_path, gain_names, scenarios):
    """SB3 -> 扁平 npz，和 stm32_policy / export_nn_c 读的是同一种格式。
    SB3 -> the flat npz both stm32_policy and export_nn_c read."""
    import torch
    pol = model.policy
    lin = [m for m in pol.mlp_extractor.policy_net if isinstance(m, torch.nn.Linear)]
    W = lambda m: m.weight.detach().cpu().numpy().T.astype(np.float32)   # noqa: E731
    B = lambda m: m.bias.detach().cpu().numpy().astype(np.float32)       # noqa: E731
    np.savez(out_path,
             w1=W(lin[0]), b1=B(lin[0]), w2=W(lin[1]), b2=B(lin[1]),
             w_out=W(pol.action_net), b_out=B(pol.action_net),
             firmware=np.array("stm32_pid"),
             imu_filter=np.array("kalman"),
             gain_names=np.array(list(gain_names)),
             scenarios=np.array(list(scenarios)),
             obs_layout=np.array("state7+feat8+lastact6"))


def rollout(policy_fn, scenarios, episodes=12, difficulty=1.0, seed0=9000,
            episode_seconds=20.0):
    """按情境分别评测：存活步数 + 平均回报。
    Score per scenario: survival steps and mean return."""
    out = {}
    for sc in scenarios:
        steps, rets, falls = [], [], 0
        for k in range(episodes):
            env = STM32AdaptEnv(firmware="stm32_pid", backend="mujoco",
                                randomize=True, episode_seconds=episode_seconds,
                                command_prob=0.3, per_gain_span=True,
                                scenarios=(sc,), seed=seed0 + k)
            env.set_difficulty(difficulty)
            obs, _ = env.reset(seed=seed0 + k)
            total, n = 0.0, 0
            while True:
                a = policy_fn(obs)
                obs, r, term, trunc, info = env.step(a)
                total += r; n += 1
                if term or trunc:
                    falls += bool(info.get("fell"))
                    break
            steps.append(n); rets.append(total)
        out[sc] = dict(steps=float(np.mean(steps)), ret=float(np.mean(rets)),
                       falls=falls, n=episodes)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--steps", type=int, default=300_000)
    ap.add_argument("--n-envs", type=int, default=8)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--episode-seconds", type=float, default=20.0)
    ap.add_argument("--hidden", type=int, default=32,
                    help="每层宽度，默认 32（F103 上 4.2 KB）")
    ap.add_argument("--scenarios", default=",".join(SCENARIOS))
    ap.add_argument("--curriculum", type=float, default=0.5,
                    help="难度在训练前这个比例内从 0 升到 1")
    ap.add_argument("--obs-norm", action="store_true",
                    help="开 VecNormalize 的观测归一化（上车要多同步一组常数）")
    ap.add_argument("--out", default=None)
    ap.add_argument("--eval-episodes", type=int, default=8)
    args = ap.parse_args()

    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.vec_env import (DummyVecEnv, SubprocVecEnv,
                                                  VecNormalize)

    scenarios = tuple(s for s in args.scenarios.split(",") if s)
    fns = [STM32AdaptEnvFactory(i, args.seed * 1000, args.episode_seconds,
                                scenarios=scenarios)
           for i in range(args.n_envs)]
    VecCls = SubprocVecEnv if args.n_envs > 1 else DummyVecEnv
    venv = VecCls(fns)
    venv = VecNormalize(venv, norm_obs=bool(args.obs_norm), norm_reward=True)

    class Curriculum(BaseCallback):
        """难度 0 -> 1。先学会站住，再挨揍。"""
        def _on_step(self) -> bool:
            frac = min(1.0, self.num_timesteps / max(
                1.0, args.curriculum * args.steps))
            if self.n_calls % 2000 == 0:
                self.training_env.env_method("set_difficulty", float(frac))
            return True

    model = PPO("MlpPolicy", venv, verbose=1, seed=args.seed,
                n_steps=512, batch_size=1024, learning_rate=3e-4,
                gamma=0.995, gae_lambda=0.95, ent_coef=0.003,
                policy_kwargs=dict(
                    net_arch=dict(pi=[args.hidden, args.hidden],
                                  vf=[64, 64]),
                    activation_fn=__import__("torch").nn.Tanh))
    model.learn(total_timesteps=args.steps, callback=Curriculum())

    os.makedirs("runs_stm32", exist_ok=True)
    out = args.out or "runs_stm32/adapt_01.npz"
    from balance_bot.stm32_env import STM32GainSpace
    space = STM32GainSpace(firmware="stm32_pid", per_gain=True)
    export_npz(model, out, space.names, scenarios)
    print(f"\n权重已存 {out}")

    # ------------------------------------------------------------------
    # 量化复评：上车跑的是定点网络，所以拿定点网络打分
    # The quantised net is what ships, so the quantised net is what is scored.
    # ------------------------------------------------------------------
    qnet = Q15Net.from_npz(out)
    sz = qnet.size_bytes()
    print(f"定点体积 {sz['total'] / 1024:.2f} KB，每次推理 {qnet.macs()} 次乘加")

    def pol_float(obs):
        a, _ = model.predict(obs, deterministic=True)
        return a

    def pol_q15(obs):
        return np.clip(qnet.forward(obs), -1.0, 1.0)

    print("\n=== 分情境评测（difficulty 1.0，没参与训练的种子）===")
    print(f"{'情境':<10}{'float 步数':>12}{'float 摔':>10}"
          f"{'q15 步数':>12}{'q15 摔':>10}")
    for sc in scenarios:
        rf = rollout(pol_float, (sc,), episodes=args.eval_episodes,
                     episode_seconds=args.episode_seconds)[sc]
        rq = rollout(pol_q15, (sc,), episodes=args.eval_episodes,
                     episode_seconds=args.episode_seconds)[sc]
        print(f"{sc:<10}{rf['steps']:>12.0f}{rf['falls']:>10d}"
              f"{rq['steps']:>12.0f}{rq['falls']:>10d}")
    print("\n下一步：python scripts/export_nn_c.py " + out)


if __name__ == "__main__":
    main()
