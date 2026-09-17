"""PPO training with stable-baselines3 (the recommended path on your machine).

    pip install "stable-baselines3[extra]" gymnasium
    python -m balance_bot.train_sb3 --steps 3000000 --n-envs 8

Two details that matter more than the hyper-parameters:

*   The policy's output bias is initialised so that a zero action means the
    hand-tuned nominal gains.  Training then starts from a controller that
    already balances and spends its budget learning *when to deviate*.
*   The final network is exported to the same ``.npz`` the NumPy runtime
    loads, so the UI and the ROS 2 node never need torch.
"""
from __future__ import annotations

import argparse
import os
import numpy as np


class EnvFactory:
    """Picklable env constructor.

    A closure would read more naturally here, but Windows has no ``fork``:
    ``SubprocVecEnv`` spawns fresh interpreters and pickles the factory to
    reach them, and nested functions do not pickle.  A small class does, so
    this is what makes ``--n-envs > 1`` work on Windows at all.
    """

    def __init__(self, rank, seed, obstacles, difficulty):
        self.rank = rank
        self.seed = seed
        self.obstacles = obstacles
        self.difficulty = difficulty

    def __call__(self):
        from .env import make_gym_env
        env = make_gym_env(seed=self.seed + self.rank, randomize=True,
                           obstacles=self.obstacles)
        env.set_difficulty(self.difficulty)
        return env


def make_env_fn(rank, seed, obstacles, difficulty):
    return EnvFactory(rank, seed, obstacles, difficulty)


class CurriculumCallback:
    """Ramps domain-randomisation difficulty 0 -> 1 over the first `frac`."""

    def __new__(cls, total, frac=0.5):
        from stable_baselines3.common.callbacks import BaseCallback

        class _CB(BaseCallback):
            def __init__(self):
                super().__init__()
                self.total, self.frac = total, frac

            def _on_step(self):
                d = min(1.0, (self.num_timesteps / max(self.total, 1)) /
                        max(self.frac, 1e-6))
                self.training_env.env_method("set_difficulty", d)
                return True
        return _CB()


def export_to_npz(model, vecnorm, out_path, obs_dim, act_dim):
    """stable-baselines3 policy -> the NumPy ``.npz`` used at runtime."""
    import torch
    from .policy_io import GainPolicy

    pol = model.policy
    pi_net = pol.mlp_extractor.policy_net
    lin = [m for m in pi_net if isinstance(m, torch.nn.Linear)]
    if len(lin) != 2:
        raise RuntimeError(
            f"expected a 2-layer policy MLP, found {len(lin)}; "
            f"train with policy_kwargs=dict(net_arch=dict(pi=[64,64], vf=[64,64]))")

    gp = GainPolicy(obs_dim, act_dim, hidden=lin[0].out_features)
    W = lambda m: m.weight.detach().cpu().numpy().T.astype(np.float64)
    B = lambda m: m.bias.detach().cpu().numpy().astype(np.float64)
    gp.net.load_params([W(lin[0]), B(lin[0]), W(lin[1]), B(lin[1]),
                        W(pol.action_net), B(pol.action_net)])
    gp.log_std = pol.log_std.detach().cpu().numpy().astype(np.float64)

    if vecnorm is not None:
        gp.norm.mean = vecnorm.obs_rms.mean.astype(np.float64)
        gp.norm.var = vecnorm.obs_rms.var.astype(np.float64)
        gp.norm.count = float(vecnorm.obs_rms.count)
        gp.norm.clip = float(vecnorm.clip_obs)
    gp.save(out_path)
    return out_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=3_000_000)
    ap.add_argument("--n-envs", type=int, default=8)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="policy_sb3.npz")
    ap.add_argument("--logdir", default="runs")
    ap.add_argument("--no-obstacles", action="store_true")
    ap.add_argument("--curriculum", type=float, default=0.5)
    ap.add_argument("--device", default="cpu",
                    help="cpu is usually faster than cuda for 64x64 MLPs")
    args = ap.parse_args()

    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import (SubprocVecEnv, DummyVecEnv,
                                                  VecNormalize)
    from .env import ACT_DIM
    from .params import GainSpace

    fns = [make_env_fn(i, args.seed * 1000, not args.no_obstacles, 0.0)
           for i in range(args.n_envs)]
    VecCls = SubprocVecEnv if args.n_envs > 1 else DummyVecEnv
    venv = VecCls(fns)
    venv = VecNormalize(venv, norm_obs=True, norm_reward=True,
                        clip_obs=10.0, gamma=0.99)

    model = PPO(
        "MlpPolicy", venv,
        learning_rate=3e-4, n_steps=1024, batch_size=1024, n_epochs=10,
        gamma=0.99, gae_lambda=0.95, clip_range=0.2, ent_coef=0.004,
        vf_coef=0.5, max_grad_norm=0.5, target_kl=0.03,
        policy_kwargs=dict(net_arch=dict(pi=[64, 64], vf=[64, 64]),
                           activation_fn=torch.nn.Tanh,
                           log_std_init=-1.0),
        tensorboard_log=args.logdir, seed=args.seed, device=args.device,
        verbose=1)

    # start at the hand-tuned PID rather than at the middle of the gain box
    with torch.no_grad():
        nominal = torch.as_tensor(GainSpace().nominal_action,
                                  dtype=model.policy.action_net.bias.dtype)
        model.policy.action_net.bias.copy_(nominal)
        model.policy.action_net.weight.mul_(0.01)

    model.learn(total_timesteps=args.steps,
                callback=CurriculumCallback(args.steps, args.curriculum),
                progress_bar=False)

    obs_dim = venv.observation_space.shape[0]
    model.save(os.path.splitext(args.out)[0] + "_sb3")
    venv.save(os.path.splitext(args.out)[0] + "_vecnorm.pkl")
    export_to_npz(model, venv, args.out, obs_dim, ACT_DIM)
    print(f"exported runtime policy -> {args.out}")

    from .train_numpy import evaluate
    from .policy_io import GainPolicy
    m, s, f, l = evaluate(GainPolicy.load(args.out),
                          obstacles=not args.no_obstacles)
    print(f"eval: return {m:.1f} +- {s:.1f}  fall-rate {f:.2f}  mean len {l:.0f}")


if __name__ == "__main__":
    main()
