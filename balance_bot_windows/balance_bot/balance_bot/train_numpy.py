"""Dependency-free PPO (NumPy only).

This is a faithful implementation of PPO-clip with GAE(lambda), observation
normalisation, advantage normalisation, an entropy bonus, gradient clipping
and a linearly decaying learning rate -- i.e. the same recipe
stable-baselines3 uses, minus the framework.

It exists so the whole pipeline can be trained and verified on a machine
where PyTorch is unavailable, and so the produced ``.npz`` can be loaded by
the runtime with nothing but NumPy.  If you *do* have PyTorch, prefer
``train_sb3.py`` -- it is faster and better tested.

    python -m balance_bot.train_numpy --steps 300000 --out policy.npz
"""
from __future__ import annotations

import argparse
import time
import numpy as np

from .env import BalanceCore, ACT_DIM, MODE_PPO_GAINS
from .nn import MLP, Adam
from .params import GainSpace
from .policy_io import GainPolicy

LOG2PI = float(np.log(2.0 * np.pi))


# ----------------------------------------------------------------------
def gaussian_logp(a, mean, log_std):
    std = np.exp(log_std)
    z = (a - mean) / std
    return -0.5 * np.sum(z ** 2 + 2.0 * log_std + LOG2PI, axis=-1)


class PPO:
    def __init__(self, envs, obs_dim, act_dim, hidden=64, lr=3e-4,
                 clip=0.2, ent_coef=0.002, vf_coef=0.5, gamma=0.99,
                 lam=0.95, seed=0, bias_init=None):
        self.envs = envs
        self.rng = np.random.default_rng(seed)
        self.pi = GainPolicy(obs_dim, act_dim, hidden, self.rng,
                             bias_init=bias_init, log_std_init=-1.6)
        self.vf = MLP(obs_dim, hidden, 1, out_gain=1.0, rng=self.rng)
        self.opt_pi = Adam(self.pi.net.params + [self.pi.log_std], lr=lr)
        self.opt_vf = Adam(self.vf.params, lr=lr)
        self.clip, self.ent_coef, self.vf_coef = clip, ent_coef, vf_coef
        self.gamma, self.lam = gamma, lam
        self.base_lr = lr

    # ------------------------------------------------------------------
    def collect(self, n_steps, obs_buf_state):
        n_env = len(self.envs)
        per = int(np.ceil(n_steps / n_env))
        O = np.zeros((per, n_env, self.pi.obs_dim))
        A = np.zeros((per, n_env, self.pi.act_dim))
        LP = np.zeros((per, n_env))
        R = np.zeros((per, n_env))
        D = np.zeros((per, n_env))
        V = np.zeros((per, n_env))
        ep_returns, ep_lens, ep_falls = [], [], []

        obs, ep_r, ep_l = obs_buf_state
        for t in range(per):
            # The running normaliser is updated *here*, during collection, and
            # the NORMALISED observation is what gets stored.  Updating it
            # later (between collection and the update) silently invalidates
            # every stored log-prob: the "old policy" would then be the
            # network evaluated on differently-scaled inputs, so the PPO ratio
            # measures normaliser drift instead of policy change.
            self.pi.norm.update(obs)
            on = self.pi.norm(obs)
            mean = self.pi.net.forward(on)
            val = self.vf.forward(on)[:, 0]
            std = np.exp(self.pi.log_std)
            z = mean + std * self.rng.normal(size=mean.shape)
            act = np.clip(z, -1.0, 1.0)
            lp = gaussian_logp(z, mean, self.pi.log_std)

            O[t], A[t], LP[t], V[t] = on, z, lp, val
            for i, e in enumerate(self.envs):
                o2, r, term, trunc, info = e.step(act[i])
                R[t, i] = r
                ep_r[i] += r
                ep_l[i] += 1
                done = term or trunc
                D[t, i] = 1.0 if term else 0.0   # bootstrap through timeouts
                if done:
                    ep_returns.append(ep_r[i])
                    ep_lens.append(ep_l[i])
                    ep_falls.append(1.0 if info["fell"] else 0.0)
                    ep_r[i], ep_l[i] = 0.0, 0
                    o2 = e.reset()
                obs[i] = o2

        last_v = self.vf.forward(self.pi.norm(obs))[:, 0]
        return (O, A, LP, R, D, V, last_v,
                (obs, ep_r, ep_l), ep_returns, ep_lens, ep_falls)

    # ------------------------------------------------------------------
    def gae(self, R, D, V, last_v):
        T, N = R.shape
        adv = np.zeros_like(R)
        last = np.zeros(N)
        for t in reversed(range(T)):
            nxt = last_v if t == T - 1 else V[t + 1]
            nonterm = 1.0 - D[t]
            delta = R[t] + self.gamma * nxt * nonterm - V[t]
            last = delta + self.gamma * self.lam * nonterm * last
            adv[t] = last
        return adv, adv + V

    # ------------------------------------------------------------------
    def update(self, O, A, LP, adv, ret, epochs=10, batch=256, lr_frac=1.0,
               target_kl=0.03):
        n = O.shape[0] * O.shape[1]
        obs_n = O.reshape(n, -1)          # already normalised in collect()
        act = A.reshape(n, -1)
        lp_old = LP.reshape(n)
        adv = adv.reshape(n)
        ret = ret.reshape(n)

        adv = (adv - adv.mean()) / (adv.std() + 1e-8)

        self.opt_pi.lr = self.base_lr * lr_frac
        self.opt_vf.lr = self.base_lr * lr_frac

        idx = np.arange(n)
        stats = {"pg": 0.0, "vf": 0.0, "kl": 0.0, "clipfrac": 0.0, "n": 0}
        for _ in range(epochs):
            self.rng.shuffle(idx)
            for s in range(0, n, batch):
                mb = idx[s:s + batch]
                if len(mb) < 8:
                    continue
                x, a, lpo, ad, rt = obs_n[mb], act[mb], lp_old[mb], adv[mb], ret[mb]

                # ---- policy ------------------------------------------
                cache = {}
                mean = self.pi.net.forward(x, cache)
                lp = gaussian_logp(a, mean, self.pi.log_std)
                ratio = np.exp(np.clip(lp - lpo, -20, 20))
                un = ratio * ad
                cl = np.clip(ratio, 1 - self.clip, 1 + self.clip) * ad
                use_un = un <= cl                      # min() picks unclipped
                m = len(mb)

                # d(-min)/d(logp) = -ratio*adv where the unclipped branch wins
                dlogp = np.where(use_un, -ratio * ad, 0.0) / m
                std = np.exp(self.pi.log_std)
                zscore = (a - mean) / std
                grad_mean = dlogp[:, None] * (zscore / std)
                grad_logstd = np.sum(dlogp[:, None] * (zscore ** 2 - 1.0), axis=0)
                grad_logstd -= self.ent_coef                      # entropy bonus

                gpi = self.pi.net.backward(grad_mean, cache) + [grad_logstd]
                self.opt_pi.step(self.pi.net.params + [self.pi.log_std], gpi)
                self.pi.log_std = np.clip(self.pi.log_std, -3.5, -0.4)

                # ---- value -------------------------------------------
                cv = {}
                v = self.vf.forward(x, cv)[:, 0]
                gv = (2.0 * (v - rt) / m)[:, None] * self.vf_coef
                self.opt_vf.step(self.vf.params, self.vf.backward(gv, cv))

                stats["pg"] += float(np.mean(np.maximum(-un, -cl)))
                stats["vf"] += float(np.mean((v - rt) ** 2))
                stats["kl"] += float(np.mean(lpo - lp))
                stats["clipfrac"] += float(np.mean(~use_un))
                stats["n"] += 1

            # early stop on KL, like SB3's target_kl: without it a single
            # rollout can move the policy far enough to wreck a controller
            # that was working
            with np.errstate(over="ignore"):
                full_lp = gaussian_logp(act, self.pi.net.forward(obs_n),
                                        self.pi.log_std)
            if float(np.mean(lp_old - full_lp)) > target_kl:
                break
        k = max(stats.pop("n"), 1)
        return {a: b / k for a, b in stats.items()}


# ----------------------------------------------------------------------
def evaluate(policy, n_episodes=12, seed=9000, **env_kw):
    core = BalanceCore(mode=MODE_PPO_GAINS, sample_difficulty=False, **env_kw)
    rets, falls, lens = [], 0, []
    for i in range(n_episodes):
        obs = core.reset(seed=seed + i)
        R, n = 0.0, 0
        while True:
            a = policy.act(obs, deterministic=True)
            obs, r, term, trunc, info = core.step(a)
            R += r
            n += 1
            if term or trunc:
                falls += int(info["fell"])
                break
        rets.append(R)
        lens.append(n)
    return float(np.mean(rets)), float(np.std(rets)), falls / n_episodes, float(np.mean(lens))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=300_000)
    ap.add_argument("--n-envs", type=int, default=8)
    ap.add_argument("--rollout", type=int, default=2048)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--hidden", type=int, default=64)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="policy_numpy.npz")
    ap.add_argument("--no-obstacles", action="store_true")
    ap.add_argument("--curriculum", type=float, default=0.6,
                    help="fraction of training over which difficulty ramps 0->1")
    args = ap.parse_args()

    envs = [BalanceCore(seed=args.seed * 1000 + i, mode=MODE_PPO_GAINS,
                        randomize=True, obstacles=not args.no_obstacles)
            for i in range(args.n_envs)]
    obs_dim = envs[0].obs_dim
    gs = GainSpace()
    ppo = PPO(envs, obs_dim, ACT_DIM, hidden=args.hidden, lr=args.lr,
              seed=args.seed, bias_init=gs.nominal_action)

    obs = np.stack([e.reset(seed=args.seed * 977 + i) for i, e in enumerate(envs)])
    state = (obs, np.zeros(args.n_envs), np.zeros(args.n_envs, dtype=int))

    done_steps, it, t0 = 0, 0, time.time()
    while done_steps < args.steps:
        frac = done_steps / max(args.steps, 1)
        diff = min(1.0, frac / max(args.curriculum, 1e-6))
        for e in envs:
            e.difficulty = diff

        (O, A, LP, R, D, V, lastv, state,
         ep_ret, ep_len, ep_fall) = ppo.collect(args.rollout, state)
        adv, ret = ppo.gae(R, D, V, lastv)
        st = ppo.update(O, A, LP, adv, ret, lr_frac=max(0.1, 1.0 - frac))
        done_steps += O.shape[0] * O.shape[1]
        it += 1

        if it % 5 == 0 or done_steps >= args.steps:
            er = np.mean(ep_ret) if ep_ret else float("nan")
            el = np.mean(ep_len) if ep_len else float("nan")
            ef = np.mean(ep_fall) if ep_fall else float("nan")
            print(f"it {it:4d} | steps {done_steps:8d} | diff {diff:.2f} | "
                  f"ret {er:8.1f} | len {el:6.1f} | fall {ef:.2f} | "
                  f"kl {st['kl']:+.4f} | clip {st['clipfrac']:.2f} | "
                  f"std {np.exp(ppo.pi.log_std).mean():.3f} | "
                  f"{done_steps / (time.time() - t0):.0f} sps", flush=True)

    ppo.pi.save(args.out)
    print("saved ->", args.out)
    m, s, f, l = evaluate(ppo.pi, obstacles=not args.no_obstacles)
    print(f"final eval: return {m:.1f} +- {s:.1f}  fall-rate {f:.2f}  len {l:.0f}")


if __name__ == "__main__":
    main()
