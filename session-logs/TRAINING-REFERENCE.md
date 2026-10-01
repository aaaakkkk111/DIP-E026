# Training reference: PPO balance-and-drive policy

Complete reference for `train_yahboom_3d.py` as it currently stands — plant,
observation/action spaces, reward function, curriculum, PPO hyperparameters,
and full convergence data for both completed training runs. Numbers below are
pulled directly from the code (`train_yahboom_3d.py`) and from the raw
training logs (`session-logs/train_their_model*.log`), not from memory.

For the *reasoning trail* behind these choices — what was tried, what
regressed, and why — see `2026-09-20-pid-baseline-and-model-switch.md` and
`FINDINGS-turn-axis-and-next-steps.md`. This document is the settled state.

---

## 1. Plant

`their_robot.xml` — a two-wheeled inverted-pendulum robot generated from the
teammate's (DIP-E026) `RobotParams`, calibrated against real firmware.
Replaced an earlier model, `my_robot.xml`, which no classical PID controller
could stabilize under any gains and is no longer used anywhere.

| property | value |
|---|---|
| `nq` / `nv` / `nu` | 9 / 8 / 2 |
| total mass | 1.700 kg |
| simulator timestep | 0.0025 s |
| integrator | Euler |
| actuator | direct torque, `ctrlrange = ±0.6 Nm` per wheel |

---

## 2. Environment

**Control rate:** `FRAME_SKIP = 5` → 1 control step per 5 sim steps →
**80 Hz**. Not a free parameter — a sweep of 120 random PID gain sets showed
only 24/120 could stabilize the plant at 40 Hz (the original rate, inherited
from `my_robot.xml`) versus 91/120 at 80 Hz. 80 Hz sits in the flat part of
the curve and is comfortably achievable on an STM32.

**Episode length:** `MAX_EPISODE_STEPS = 1000` steps = 12.5 s of sim time,
enforced by a `TimeLimit` wrapper.

**Observation (17-dim, float32):**

| index | content |
|---|---|
| 0 | chassis height (z) |
| 1–4 | orientation quaternion |
| 5–6 | left / right wheel angle |
| 7–9 | **body-frame** linear velocity (forward, lateral, vertical) |
| 10–12 | angular velocity (roll-rate, pitch-rate, yaw-rate) |
| 13–14 | left / right wheel angular velocity |
| 15–16 | commanded forward velocity, commanded turn rate |

World x/y position is dropped entirely (translation-invariant task). Linear
velocity is rotated into the body frame before the policy sees it — MuJoCo
reports a freejoint's linear velocity in the world frame, but "forward" is a
body-frame concept, and the same physical motion produced opposite-looking
observations at different headings before this fix. Yaw rate needs no such
correction and is passed through raw.

**Action (2-dim, float32):** per-wheel torque, bounds taken directly from
`model.actuator_ctrlrange` (±0.6 Nm) so the action space can never silently
disagree with the plant.

**Reset:**
- Roll/pitch perturbed by `±0.05 rad`, velocity (all 6 freejoint DOF)
  perturbed by `±0.10` (m/s or rad/s). Required: this plant sits at its
  equilibrium indefinitely, and a zero-torque episode with no reset noise
  survives all 1000 steps (measured return 1634) — without noise, "output
  nothing" is optimal and balance is never learned.
- Payload mass sampled `uniform(0, curriculum_payload_kg)` each episode
  (curriculum-gated, see §4).
- Commanded forward/turn velocity sampled uniformly within the current
  curriculum caps (training) or taken from a fixed 20-command eval schedule
  (`EVAL_COMMANDS`, evaluation) and held for a randomly-sampled duration:
  30% chance of a long hold (400–1000 steps), otherwise 100–300 steps — so
  the policy regularly practices sustaining one direction (e.g. a continuous
  turn) rather than only reacting to frequent command changes.

**Termination:** after a 10-step grace period post-reset, `|pitch| > 0.4` or
`|roll| > 0.4` rad ends the episode with a `-10.0` reward that step.

**Evaluation:** `EvalCallback`, `eval_freq=10000`, `n_eval_episodes=20` — one
episode per entry in `EVAL_COMMANDS`, each held for the full episode at full
magnitude (not curriculum-gated), so eval score always reflects the complete
task regardless of training progress.

---

## 3. Reward function

Per control step:

```
effort = Σ (action / τ_max)²

reward = 2.0                                    # alive bonus
        − 3.0 · |pitch|
        − 0.1 · effort
        − 2.0 · |v_forward,actual − v_forward,target|
        − TURN_WEIGHT · |turn_filt − v_turn,target|            (TURN_WEIGHT = 2.0)
        − TURN_INSTANT_WEIGHT · |v_turn,actual − v_turn,target| (= 0.2)
        + γ·Φ(s′) − Φ(s)                          # potential-based shaping, γ = 0.99

on fall: reward = −10.0, episode terminates
```

**`turn_filt`** is an exponential low-pass of the raw yaw rate
(`turn_filt ← 0.9·turn_filt + 0.1·yaw_rate`, ~0.1 s time constant),
seeded from the true yaw rate at reset rather than 0.

**Effort** is normalized by the actual torque limit (`τ_max`), not a fixed
constant — so the penalty means the same thing (fraction of available torque)
regardless of which plant is loaded, capped at 0.2 when both motors saturate.

**Potential** `Φ`:

```
w_fwd  = min(1, |v_forward,target| / max_v_forward)
w_turn = min(1, |v_turn,target|    / max_v_turn)

forward_weight = 0.5 + 0.5·w_fwd        # 0.5 → 1.0
turn_weight    = 0.1 + 0.9·w_turn       # 0.1 → 1.0
posture_weight = 1 − max(w_fwd, w_turn)

Φ(s) = − posture_weight · (pitch² + roll²)
       − forward_weight · (v_forward,actual − v_forward,target)²
       − turn_weight    · (turn_filt      − v_turn,target)²
       − 0.5 · v_lateral,actual²
```

Being potential-based (Ng et al. 1999), `γΦ(s′) − Φ(s)` is policy-invariant
in principle — it telescopes to `−Φ(s₀)` over an episode and cannot change
which policy is optimal, only how easy it is to find. Each axis blends its
own "hold still" term against its own "track the target" term, weighted by
how much *that axis* specifically is being commanded, so a large turn command
can't starve the forward axis of gradient signal (an earlier shared-weight
design did exactly that).

**Known caveat:** using the low-passed `turn_filt` inside `Φ` means `Φ` is a
function of hidden filter state, not strictly the policy's observed state —
so the textbook policy-invariance guarantee doesn't hold rigorously here. Kept
this way deliberately: having the shaping term grade instantaneous yaw while
the main reward term grades filtered yaw would pull the two signals against
each other, judged the worse problem.

### Why the turn term is filtered, not instantaneous

This plant tracks a commanded yaw rate well **on average** (0.485 rad/s
achieved against a 0.50 command, 97%) but poorly **instantaneously** (best
achievable instantaneous error 0.277 rad/s, 55% of command) — turning
necessarily involves a small yaw oscillation. Measured per-step turn cost
under each scoring rule, PID controller at various gains:

| controller | survives | instantaneous cost | filtered cost |
|---|---|---|---|
| never turns | 1000 | 0.237 | 0.236 |
| PID, kp_turn=0.5 | 1000 | 0.288 | 0.134 |
| PID, kp_turn=2.0 | 965 | 0.469 | 0.095 |

Under instantaneous scoring, not turning is cheapest and turning harder is
monotonically worse — confirmed to produce a policy that never turns (Run 1,
§6). Under filtered scoring the ranking inverts. `TURN_WEIGHT = 2.0` was then
raised from 1.0 to be worth roughly as much reward per episode as the forward
axis (~235 vs ~323 available per 1000-step episode) — raising it before
filtering would have been pointless, since turning and not-turning cost the
same and any weight scaled both sides equally.

---

## 4. Curriculum (`CurriculumCallback`)

Progress = `num_timesteps / total_timesteps` (total = 5,000,000):

| progress | timesteps | forward cap | turn cap | payload cap |
|---|---|---|---|---|
| `< 0.15` | 0 – 750k | 0 | 0 | 0 |
| `0.15 – 0.5` | 750k – 2.5M | ramps 0→0.3 | ramps 0→0.5 | 0 |
| `≥ 0.5` | 2.5M – 5M | 0.3 (fixed) | 0.5 (fixed) | ramps 0→0.2 kg |

Forward and turn ramp **together** in the middle phase. This was changed
partway through the project (§6) — forward and turn used to be staged
sequentially (forward alone, then turn added later), inherited from
`my_robot.xml` where turning was the *easy* behavior that crowded out
forward. On this plant that reasoning doesn't transfer: staging turn last
meant 1.75M steps of the turn target sitting at exactly 0 while any yaw was
penalized, which trained an explicit yaw *suppressor* rather than simply
leaving turning unlearned.

---

## 5. PPO hyperparameters

| parameter | value | why |
|---|---|---|
| policy | `MlpPolicy`, `net_arch=[64, 64]` | ~22 KB of float32 weights — fits comfortably in the STM32's 256 KB flash (a 256×256 net would not) |
| `log_std_init` | `log(0.25 · τ_max)` ≈ −1.897 → initial std 0.15 (25% of the ±0.6 range) | SB3's default (`0.0` → std 1.0) is 167% of this plant's action range — roughly a third of sampled actions clipped to saturation, making the effective policy bang-bang. Derived from `τ_max` so it can't silently mismatch the plant again. |
| `ent_coef` | 0.01 | SB3 default (0.0) let an earlier run's action std collapse to ~1% of range with no exploration left, unable to discover the forward-drive maneuver |
| `learning_rate` | linear schedule, `3e-4 → 0` | — |
| `n_steps` | 2048 | rollout length per env before each PPO update |
| `batch_size` | 128 | — |
| `γ` (discount) | 0.99 (SB3 default) | also used as `SHAPING_GAMMA` for potential-based shaping, deliberately kept consistent |
| `device` | `cpu` | — |
| parallel envs | `os.cpu_count() − 1` via `SubprocVecEnv` | — |
| total timesteps | 5,000,000 | — |

---

## 6. Convergence — three attempts

### Attempt 1 (killed at 380k) — exploration noise larger than the action range

Used SB3's default `log_std_init=0`. Measured action std still at 111% of
the action range after 380k steps (0.67 vs a ±0.6 limit), episode length only
72 steps. Root cause: SB3's default assumes a much larger action range than
this plant has. Killed and restarted with the `log_std_init` fix in §5.
Log: `train_their_model_ATTEMPT1_default_logstd.log`.

### Run 1 — corrected exploration, staged curriculum, instantaneous turn reward

Completed the full 5M steps. Log: `train_their_model_RUN1_instant_turn.log`.

| timesteps | episode length | return |
|---|---|---|
| 190,000 | 80.1 ± 36.3 | 78.1 ± 50.8 |
| 380,000 | 423.2 ± 182.3 | 564.3 ± 346.5 |
| 570,000 | 940.1 ± 154.6 | 1340.2 ± 375.7 |
| 760,000 | 961.3 ± 95.6 | 1376.8 ± 340.5 |
| 950,000 | 935.4 ± 141.4 | 1350.1 ± 378.9 |
| 1,140,000 | 894.9 ± 136.7 | 1294.0 ± 400.7 |
| 1,330,000 | 935.5 ± 110.6 | 1385.4 ± 352.7 |
| 1,520,000 | 927.8 ± 128.2 | 1347.4 ± 387.8 |
| 1,710,000 | 947.6 ± 95.0 | 1397.6 ± 379.2 |
| 1,900,000 | 985.6 ± 31.5 | 1447.1 ± 373.8 |
| 2,090,000 | 943.3 ± 93.3 | 1392.9 ± 347.6 |
| 2,280,000 | 884.7 ± 280.8 | 1287.9 ± 566.9 |
| 2,470,000 | 998.9 ± 4.8 | 1491.7 ± 329.8 |
| 2,660,000 | 987.6 ± 54.3 | 1437.7 ± 350.7 |
| 2,850,000 | **1000.0 ± 0.0** | 1563.5 ± 268.4 |
| 3,040,000 | 1000.0 ± 0.0 | 1571.5 ± 282.3 |
| 3,230,000 | 996.7 ± 14.6 | 1596.4 ± 240.2 |
| 3,420,000 | 1000.0 ± 0.0 | 1627.4 ± 220.9 |
| 3,610,000 | 1000.0 ± 0.0 | 1517.3 ± 332.5 |
| 3,800,000 | 1000.0 ± 0.0 | 1542.0 ± 317.6 |
| 3,990,000 | 1000.0 ± 0.0 | 1666.5 ± 217.9 |
| 4,180,000 | 1000.0 ± 0.0 | 1689.5 ± 193.7 |
| **4,370,000** | 1000.0 ± 0.0 | **1699.0 ± 187.7** (best) |
| 4,560,000 | 974.2 ± 83.4 | 1666.7 ± 247.6 |
| 4,750,000 | 974.8 ± 89.0 | 1674.8 ± 246.8 |
| 4,940,000 | 975.0 ± 77.0 | 1672.8 ± 238.4 |

**Outcome:** balance and forward driving solved (forward tracks a +0.30 m/s
command to +0.306). **Turning never appeared** — final policy produced
+0.032 rad/s against a ±0.50 command. Diagnosed as a *reward* problem, not a
training failure: a hand-tuned PID couldn't profit from turning under this
same instantaneous-error reward either (turning cost 0.469/step vs 0.237/step
for never turning). See §3's table and `FINDINGS-turn-axis-and-next-steps.md`
for the full diagnosis, including the probe showing the policy actively
*cancels* an externally-imposed turning bias below ~1.4σ of its own action
noise.

### Run 2 (current) — filtered turn reward (§3), merged curriculum (§4)

Completed the full 5M steps. Log: `train_their_model.log`. This is the
checkpoint at `models/best_their/best_model.zip`.

| timesteps | episode length | return |
|---|---|---|
| 190,000 | 104.3 ± 35.3 | 75.2 ± 72.9 |
| 380,000 | 438.2 ± 173.5 | 365.1 ± 337.4 |
| 570,000 | 933.4 ± 123.6 | 884.0 ± 622.6 |
| 760,000 | 1000.0 ± 0.0 | 1097.8 ± 481.0 |
| 950,000 | 1000.0 ± 0.0 | 1171.4 ± 422.6 |
| 1,140,000 | 965.4 ± 68.5 | 1221.5 ± 422.6 |
| 1,330,000 | 990.4 ± 42.1 | 1241.0 ± 386.8 |
| 1,520,000 | 984.0 ± 69.7 | 1275.4 ± 367.5 |
| 1,710,000 | 921.3 ± 163.3 | 1248.6 ± 425.5 |
| 1,900,000 | 935.8 ± 149.4 | 1278.5 ± 418.1 |
| 2,090,000 | 921.7 ± 167.0 | 1311.9 ± 431.0 |
| 2,280,000 | 870.6 ± 177.6 | 1237.1 ± 432.8 |
| 2,470,000 | 935.3 ± 156.3 | 1356.3 ± 427.6 |
| 2,660,000 | 944.3 ± 149.5 | 1429.3 ± 388.3 |
| 2,850,000 | 955.2 ± 134.7 | 1491.8 ± 367.1 |
| 3,040,000 | 950.5 ± 132.1 | 1476.8 ± 362.9 |
| 3,230,000 | 936.0 ± 134.9 | 1497.8 ± 374.0 |
| 3,420,000 | 942.1 ± 138.1 | 1526.9 ± 348.5 |
| 3,610,000 | 923.9 ± 158.8 | 1502.3 ± 411.2 |
| 3,800,000 | 938.8 ± 132.0 | 1441.8 ± 502.7 |
| 3,990,000 | 929.3 ± 146.8 | 1548.3 ± 361.9 |
| 4,180,000 | 925.8 ± 144.1 | 1540.7 ± 392.2 |
| 4,370,000 | 917.9 ± 148.4 | 1582.8 ± 385.9 |
| 4,560,000 | 914.7 ± 139.1 | 1586.8 ± 387.9 |
| 4,750,000 | 922.4 ± 130.6 | 1602.8 ± 374.2 |
| **4,940,000** | 914.3 ± 136.4 | **1610.5 ± 353.8** (best) |

**Outcome — turning works.** Turn-command weight in the policy network's
first layer rose from an untrained baseline of ~0.14 to 0.4419 (vs. 0.1565 in
Run 1's final model). Directly measured yaw-rate tracking:

| command (v, turn) | Run 1 turn output | Run 2 turn output | gain |
|---|---|---|---|
| (0.00, +0.50) | +0.043 | **+0.378** | 76% |
| (0.00, −0.50) | −0.172 | **−0.353** | 71% |
| (+0.30, +0.50) | −0.003 | **+0.474** | 95% |

**Cost:** mean episode length dropped from 997.0 (Run 1) to 919.1 (Run 2,
−8%) — Run 1 partly "won" on survival by declining the hard part of the
task. A new defect appeared: forward/turn **cross-coupling** — commanding
pure yaw now produces unwanted forward drift (+0.214 m/s where it should be
0), and combined commands overshoot forward by roughly 2×. Pure forward
tracking is unaffected (+0.30 → +0.307 in both runs). This cross-coupling is
the top open item; see `FINDINGS-turn-axis-and-next-steps.md` §3.4 for the
proposed next fix.

### Benchmark comparison — Run 2 vs. tuned PID

`policy_eval_benchmark.py` vs. `pid_eval_benchmark.py`, identical 20-command
task:

| | mean episode length | mean return |
|---|---|---|
| Run 2 policy | 919.1–919.2 (re-measured twice, consistent) | 1610–1624 |
| Tuned PID (`kp_turn=0.5`) | 1000.0 ± 0.0 | 360.2 ± 163.3 |

The PID's episode length is higher because its default turn gain barely
turns at all — see §3's cost table for what happens to PID survival at a turn
gain that actually tracks well (kp_turn=2.0 → 965/1000, still short of the
policy's real driving+turning behaviour at comparable difficulty).
