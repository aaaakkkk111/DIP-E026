# Findings and proposed fixes — balance, driving, and the turn axis

Written for: the DIP team (anyone picking up the RL controller work).

Companion to the chronological logs
`2026-09-20-episode-length-and-driving.md` and
`2026-09-20-pid-baseline-and-model-switch.md`. This document is the summary:
what is solved, what is not, why, and what to do about it.

---

## 1. Headline

> **Status: both proposed fixes were applied and retrained (run 2). Turning
> now works — see §3.4.** Sections 1–3.3 describe the run-1 diagnosis that led
> there and are kept as the reasoning trail.

| capability | status after run 2 |
|---|---|
| Balance | **Solved.** 919/1000 mean (997 in run 1, which refused to turn) |
| Forward / reverse driving | **Solved.** +0.30 → +0.307 |
| Turning | **Solved.** 69–95% of commanded yaw, both directions |
| Turning while driving | **Works**, but couples into forward velocity (§3.4) |
| Payload (0–0.2 kg) | Absorbed without loss of stability |

The original question was *"if it can balance while moving, then it's just the
RL training left."* On the teammate's model the answer is yes for balance and
forward motion, and both are now solved. Turning turned out **not** to be a
training problem at all.

### Final run (5M steps) vs the PID baseline, identical task

| | mean episode length | mean return |
|---|---|---|
| **Trained policy** | **1000.0 ± 0.0** | **1694.0 ± 189.1** |
| Tuned cascade PID | 1000.0 ± 0.0 | 360.2 ± 163.3 |

Forward velocity tracking in the final policy:

| commanded | achieved | | commanded | achieved |
|---|---|---|---|---|
| +0.30 | +0.306 | | −0.30 | −0.325 |
| +0.22 | +0.200 | | −0.22 | −0.203 |
| +0.15 | +0.122 | | −0.15 | −0.136 |
| +0.07 | +0.082 | | | |

Yaw rate against a ±0.50 command: **+0.032 / −0.067** — effectively zero. That
single row is the whole remaining problem, and §3 explains it.

---

## 2. What was wrong before, and what fixed it

### 2.1 The plant itself (biggest single factor)

`my_robot.xml` cannot be stabilized by a classical controller under **any**
gains — 0 of ~500 combinations across a 64× `kp` range, both sign conventions,
three torque limits. The RL policy could balance it, which masked the problem.

| | teammate's model | `my_robot.xml` |
|---|---|---|
| Inertia about wheel axle | 0.0296 | 0.00284 (10.4× lower) |
| COM height | 120 mm | 37.6 mm |
| Zero-torque spawn | sits still | topples by 0.6 s, contacts flicker 2→4→10 |
| `ctrl=+0.02` → velocity | +0.62 | +0.008 |

The last row is the clearest: on the old model torque spins the chassis instead
of driving the wheels. Training moved to `their_robot.xml`, whose parameters
are calibrated against the real firmware — which also makes the eventual STM32
port meaningful.

### 2.2 Exploration noise larger than the action range

SB3 defaults `log_std_init=0` → action std 1.0. That was a sensible 25% of
`my_robot.xml`'s ±4.0 action space, but **167%** of the real motor's ±0.6 N·m
limit, so ~1/3 of sampled actions clipped to saturation and the effective
policy was bang-bang. Measured: after 380k steps std was still 111% of range
and episodes lasted 72 steps.

Fixed with `log_std_init = log(0.25 · τ_max)`, derived from the model's own
`ctrlrange` so it cannot drift. Effect at equal progress:

| timesteps | before | after |
|---|---|---|
| 190k | 49.5 steps | 80.1 steps |
| 380k | 72.0 steps | 423.2 steps |

### 2.3 The model was too stable to learn from

Their model sits at equilibrium indefinitely. A **zero-torque episode survived
all 1000 steps and banked a return of 1634** — the optimal policy was
literally "output nothing". `my_robot.xml` had accidentally supplied a
disturbance by being numerically unstable at spawn.

Added reset noise (±0.05 rad tilt, ±0.10 velocity). Zero-torque episodes now
end in 11–23 steps (mean 15.1).

### 2.4 Control rate sat on a cliff

Counting, per control rate, how many of 120 random gain sets can both balance
and hold a 0.3 m/s cruise:

| 400 Hz | 200 Hz | 100 Hz | 80 Hz | 50 Hz | 40 Hz | 25 Hz |
|---|---|---|---|---|---|---|
| 93 | 95 | 97 | 91 | 64 | **24** | **0** |

The original 40 Hz rate leaves only 20% of controllers viable. Moved to 80 Hz
(`FRAME_SKIP=5`), comfortably within an STM32's budget for a 64×64 MLP.

### 2.5 Eval measured the wrong thing

Eval forced both commands to zero, so "best model" was selected on how long the
robot could stand still — the one thing the failing policy already did
perfectly. It could not distinguish a good driver from a statue.

Eval now runs 20 fixed commands, each held for a full episode, so sustained
driving and continuous turning are what get scored. Fixed rather than sampled
so numbers stay comparable across evaluations.

---

## 3. The turn axis: diagnosis

Turning never appeared. The first-layer weight on the turn command confirms it
was never learned, only drifting, while the forward command weight grew
steadily:

| checkpoint | CMD_forward | CMD_turn |
|---|---|---|
| 380k (both provably untrained) | 0.1488 | 0.1404 |
| 1.33M | 0.1789 | 0.1404 |
| 1.9M | 0.2176 | 0.1404 |
| 2.47M | 0.2393 | 0.1357 |
| 2.85M | — | 0.1408 |

### 3.1 It is not an exploration failure

Probing the trained policy with a constant differential-torque bias:

| bias (N·m) | in σ | steps survived | yaw rate | fell? |
|---|---|---|---|---|
| 0.02 | 0.5 | 1000 | −0.002 | no |
| 0.05 | 1.4 | 1000 | −0.014 | no |
| 0.10 | 2.7 | **35** | −2.161 | **yes** |
| 0.25 | 6.8 | 56 | −21.586 | **yes** |

There is no gradual middle: below ~1.4σ the policy actively cancels the bias,
above it the robot falls within 35 steps. (The −21 rad/s figures are post-fall
artifacts — the chassis spinning on the ground.)

### 3.2 It is a reward problem — turning earns nothing

The reward charges **instantaneous** yaw error. Measured per-step turn cost,
averaged over the 20 eval commands:

| controller | per-step turn cost |
|---|---|
| never turns at all | **0.256** |
| PID at its best-scoring gain | 0.266 |
| PID at high turn gain | 0.418 |

**Turning at the best available gain buys −0.010 reward/step.** A policy that
ignores turn commands is near-optimal under this reward. That is exactly what
PPO found.

Isolating a single 0.5 rad/s command, the best turn controller found cuts
per-step cost from 0.500 to 0.277 — so turning does pay on strongly-commanded
episodes (~0.22/step), but the error never approaches zero: 0.277 is still 55%
of the command.

### 3.3 The root cause

**The plant can track yaw on average but not moment to moment.**

- achievable *mean* yaw: 0.485 against a 0.500 command (97%)
- achievable *instantaneous* error: 0.277 at best (55% of command)

Aggressive turn gains drive a yaw limit cycle — the average is accurate while
the instantaneous error gets worse. The reward charges the instantaneous
quantity, so accurate-on-average turning is punished.

> **Measurement warning for whoever continues this.** A gain sweep scored by
> *episode-mean* yaw rate shows error falling monotonically to 0.026, which
> looks like "tracking is fine". It is not — that statistic cancels the
> oscillation the reward is charging for. Any judgement of this axis must use
> per-step error. This produced one wrong conclusion during the session.

---

## 3.4 OUTCOME — both fixes applied and run (5M steps, "run 2")

Applied and retrained. **Turning works.**

| command (v, turn) | run 1 turn | run 2 turn | gain |
|---|---|---|---|
| (0.00, +0.50) | +0.043 | **+0.378** | 76% |
| (0.00, −0.50) | −0.172 | **−0.353** | 71% |
| (0.00, +0.25) | +0.009 | **+0.172** | 69% |
| (0.00, −0.25) | −0.016 | **−0.177** | 71% |
| (+0.30, +0.50) | −0.003 | **+0.474** | 95% |
| (+0.07, +0.38) | +0.008 | **+0.305** | 80% |

`CMD_turn` weight 0.1565 → **0.4419** (2.8×). Sustained turning while driving —
the original requirement — now works.

### Cost

| | run 1 | run 2 |
|---|---|---|
| mean episode length | 997.0 | 919.1 (−8%) |
| pure forward +0.30 | +0.306 | +0.307 (unchanged) |
| forward during pure turn cmd (should be 0) | −0.012 | **+0.214** |
| forward at (+0.30, +0.50) | +0.275 | **+0.614** (2× overshoot) |

Run 1 survived better largely because it declined the hard part of the task.
The new defect is forward/turn **cross-coupling**: commanding yaw alone now
produces unwanted forward motion, and combined commands overshoot forward ~2×.
That contamination could not exist before, since the turn axis was inert.

### A third change was needed beyond the two proposed

Verifying the incentive flip in the live environment showed the sign changed
(−232 → +94 reward per episode) but the signal was weak — forward tracking is
worth ~323/episode and *that* is the strength that demonstrably got learned.
The filtered turn weight was therefore raised 1.0 → 2.0 to match the forward
axis (`TURN_WEIGHT`), giving ~235/episode. Raising the weight would have been
useless before filtering, because turning and not turning cost the same and
any weight scaled both sides equally.

**A measurement trap worth recording:** comparing total returns after the fix
showed run 1's policy (1645) beating the turning PIDs (~450) even under the
new reward, which looks like the fix failed. It does not isolate anything —
the RL policy is simply far better at balance and forward driving, which
dominates both totals. Only the marginal per-step turn cost answers the
question.

### Remaining work

Forward/turn cross-coupling is now the top defect. The `_potential` function
already carries an always-on `−0.5 · v_lateral²` term for unwanted lateral
drift; an analogous decoupling term, or simply raising the forward-tracking
weight so forward error is not ignored while turning, is the obvious next
step. Untested.

## 4. Proposed fixes, in priority order

### Fix 1 — reward filtered yaw, not instantaneous yaw *(addresses the root cause)*

In `VelocityCommandWrapper.step` and `_potential`, replace the raw
`actual_v_turn` with a low-passed version:

```python
self.turn_filt = 0.9 * self.turn_filt + 0.1 * actual_v_turn   # ~0.1 s at 80 Hz
total_reward -= abs(self.turn_filt - self.target_v_turn) * 1.0
```

Rationale: the plant *can* deliver the commanded yaw on average (0.485 vs
0.500). Charging the instantaneous error prices in a limit cycle the robot
cannot avoid, making the correct behaviour unprofitable. Filtering asks for
what is physically achievable.

Note: raising the turn weight **alone will not work** — it scales both the
turning and not-turning costs, and those are currently equal.

Risk: a filtered term rewards slower yaw response. Pair it with a modest
instantaneous term (e.g. weight 0.2) to retain some responsiveness.

### Fix 2 — introduce turn commands early *(addresses a secondary contributor)*

For the 1.75M steps before the turn phase opens, the turn target is always 0
and the reward penalizes any yaw, so the policy learns an explicit yaw
**suppressor** — which §3.1 shows it doing. The turn phase then has to unlearn
that with exploration already decayed to 6% of range.

In `CurriculumCallback`, ramp turn alongside forward rather than after it.
Optionally re-inflate `log_std` at each phase boundary.

This staging was inherited from `my_robot.xml`, where turning was the *easy*
behaviour that crowded out forward driving. **On this plant the ordering is
reversed** — one more old-plant assumption that did not transfer.

### ~~Fix 3 — forward-tracking calibration~~ *(no longer needed)*

Mid-training checkpoints overshot forward commands by ~1.8× (0.30 → 0.602 at
1.9M), and a reward-weight change was drafted for it. **The final policy fixed
this on its own** — it now tracks +0.30 → +0.306. Recorded here only so nobody
re-derives the problem from the intermediate logs. No action needed.

---

## 5. Honest limits of these conclusions

- The turn analysis tested one controller family (P / PI on yaw rate, with and
  without low-pass). "No controller can profit from turning under this reward"
  is supported for that family, not proven in general.
- The PID baseline is a *tuned-by-search* controller, not an optimal one. It
  establishes the task is achievable; it does not bound what is achievable.
- Fix 1 is reasoned from measurements but **has not been run**. It changes the
  task definition, which is a team decision rather than a tuning choice.
- Mid-training robustness was seed-dependent (a 2.85M checkpoint averaged
  953.8 ± 201.6 on different seeds, with one catastrophic episode). The final
  policy is not: 1000.0 ± 0.0 across all 20 commands. Note this was only
  checked on two seed sets, not exhaustively.
- Everything here is simulation. Nothing has been run on hardware, and the
  reset-noise magnitudes (±0.05 rad, ±0.10 m/s) were chosen to make training
  non-trivial, not measured from the real robot.

---

## 6. Artifacts

| file | purpose |
|---|---|
| `their_robot.xml` | the plant now trained on (`payload_body` added) |
| `train_yahboom_3d.py` | training, retargeted to their model |
| `pid_baseline.py` | classical cascade PID, ported from the team firmware |
| `pid_eval_benchmark.py` | scores the PID on the exact EvalCallback task |
| `policy_eval_benchmark.py` | same task, for a trained policy (with plant-mismatch guard) |
| `enjoy_drive.py` | interactive driving; refuses mismatched checkpoints |
| `models/best_their/` | this run's best checkpoint |
| `models/best_their_ATTEMPT1_default_logstd/` | superseded run, kept for comparison |
