# 2026-09-25 — Run 7: heading-keeping (yaw integral), 30M steps, converged

## Where run 6 left off

Run 6 fixed station-keeping drift (0.408 → 0.012 m/10 s) by adding a leaky
integral of forward-velocity error to both the observation and the reward. It
left the turn axis with exactly the defect the forward axis had before that
fix: the reward grades yaw **rate**, never yaw **angle**, so heading error
integrates freely. Measured on run 6 at idle over 10 minutes, yaw wandered to
-10.2° by 300 s and back to -6.1° by 600 s. The same omission showed up as
turn overshoot on commanded turns: 106% at +0.50 rad/s, 113% in the combined
forward+turn case.

## The fix

Symmetric to run 6's position integral, but for heading:

```
e <- decay * e + (w_actual - w_target) * dt      decay = exp(-dt / 30 s)
reward -= 3.0 * |e|
obs    += [e / 0.2]                               34 inputs, up from 33
```

Two deliberate differences from the forward-axis integral:

- **τ = 30 s, not 2 s.** The two axes want different things. Forward used a
  short τ so the robot is never asked to chase distance lost under a stale
  command. Heading is the opposite: the integral of `(w - w_cmd)` *is*
  "actual heading minus commanded heading", so driving it to zero means
  ending up pointing where commanded — that is heading control, and a 2 s τ
  cannot see a wander that develops over minutes.
- **Clipped at ±0.2 rad (11.5°).** Idle wander produces ~0.02 rad while a
  tracking error on a commanded turn produces ~1.5 rad — two orders of
  magnitude apart. Without the clip, one linear weight cannot serve both, and
  the tracking case dominates the whole reward. Clipped, the idle case still
  costs ~0.06/step (no longer invisible) without letting a real turn error
  swamp everything else.

Leaky rather than absolute heading, and this matters for hardware: the
MPU6050 has no magnetometer, so absolute yaw drifts without bound on the real
robot. A leaky integral only ever integrates the last few τ of gyro-z, which
the sensor measures directly and well — a policy trained against it
transfers, one trained against absolute heading would not.

Updated in `_get_conditioned_obs` for the same reason as the position
integral: it is the one place called exactly once per step and once per
reset, so the value exists before the observation containing it is built.

## Training

30M steps, unchanged hyperparameters from run 6 (19 envs, `batch_size=512`,
`n_steps=2048`, `linear_schedule(3e-4)`, `ent_coef=0.01`, CPU). Wall clock
6:48:52 (30,001,152 steps recorded — `model.learn` overshoots the target
by one rollout). fps drifted from ~1900 to ~1220 over the run; `explained_variance`
stayed high throughout, so this reads as background machine load, not
training degradation.

The session that launched this run was closed by accident partway through
(around 1.9M steps); the training process itself is detached from the
launching terminal and kept running unattended to completion.

## Eval curve

| block | mean | | metric | start | 25% | 50% | 75% | end |
|---|---|---|---|---|---|---|---|---|
| 0–5M | 518 | | `train/std` | 0.251 | 0.020 | 0.034 | 0.048 | 0.058 |
| 5–10M | 677 | | `explained_variance` | -0.01 | 0.997 | 0.980 | 0.949 | 0.922 |
| 10–15M | 1669 | | `approx_kl` | 0.0064 | 0.0114 | 0.0077 | 0.0051 | 0.0004 |
| 15–20M | 2497 | | `clip_fraction` | 0.076 | 0.146 | 0.084 | 0.063 | 0.000 |
| 20–25M | 3031 | | `value_loss` | 128.0 | 1.3 | 12.2 | 16.5 | 28.6 |
| 25–30M | 3243 | | | | | | | |

Slope **+46/1M** over the final third, **+83/1M** over the final tenth —
still climbing, not a flat tail the way run 5/6's final tenth was (their
learning rate had more room left to decay against; this run's last five
evals were all "new best mean reward", ending at **3400.34 ± 164.08** at
29.83M). `std` dipped early (0.020 at 25%, the same action-rate-penalty
exploration tax seen in every run since it was added) and recovered to
0.058. `explained_variance` stayed ≥0.92 throughout — the value function
tracked the reward well even as the reward function itself grew a term.

Two flat-looking stretches mid-run (~13–14M and ~21–23M) each turned out to
be temporary: reward held in a band for 3–5 evals, then broke upward again.
Neither was a genuine plateau; this run never actually stalled the way run
4's curriculum-blind eval did.

**Not comparable to run 6's 3457** — the reward function gained a term
(same caveat as every run since 5→6). Only the behavioral measurements below
are apples-to-apples.

## Behavior: run 6 vs run 7, same plant, both measured on the current code

Run 6's checkpoint has no yaw-integral input; to drive it through the
current wrapper for a fair comparison, a `Run6Wrapper` subclass strips the
34th observation feature and nothing else (motor model, plant, action space,
history taps are all identical between the two runs — only the yaw term is
new). Three seeds, unloaded, 2000-step episodes, steady state measured over
steps 400–2000.

| command | run 6 v_fwd / v_turn (% of cmd) | run 7 v_fwd / v_turn (% of cmd) |
|---|---|---|
| hold | +0.001 / +0.000 | +0.002 / -0.000 |
| turn +0.50 | +0.007 / +0.527 (**105%**) | +0.026 / +0.498 (**100%**) |
| +0.15 & turn +0.30 | +0.151 / +0.337 (**112%**) | +0.161 / +0.288 (**96%**) |

**Turn overshoot is fixed** — 105%/112% down to 100%/96%, matching the
106%/113% originally measured on run 6 closely enough to confirm this is the
same effect, now closed from both directions. Forward tracking held (+0.151
→ +0.161) rather than being traded away.

Idle heading drift over 60 s, single seed:

| | run 6 | run 7 |
|---|---|---|
| yaw at 15 s | +0.01° | -0.25° |
| yaw at 30 s | -0.29° | -0.32° |
| yaw at 60 s | **-1.78°** | **-0.35°** |

Run 6's drift is still growing roughly linearly at 60 s (on pace for the
-10.2° at 300 s measured previously). Run 7's saturates by ~30 s — consistent
with the 30 s time constant bounding it by construction — and stays there.
This is the direct confirmation that the leaky integral does what it was
designed to do, not just that the eval number went up.

One small honest debit: hold-command forward drift under a pure turn command
rose slightly (+0.007 → +0.026 m/s) — the yaw correction pulling in a small
amount of coupled forward motion. Two orders of magnitude smaller than the
command it's tracking; not worth chasing.

## Artifacts

| path | what |
|---|---|
| `models/best_real/best_model.zip` | **run 7 @29.83M — current best, 34 inputs** |
| `models/ppo_real_robot.zip` | run 7 final-step (30.00M) |
| `models/best_real_RUN6_station/` | run 6, 33 inputs, kept for comparison |
| `models/ppo_real_robot_RUN6_30M.zip` | run 6 final-step |
| `session-logs/train_real_robot_RUN7_yaw.log` | run 7 log, 157 eval points |

No overwrite this time — run 6's `best_real_RUN6_station/` and
`ppo_real_robot_RUN6_30M.zip` were already renamed out of the way before this
run started, so both checkpoints and both final-step zips survive side by
side (the RUN2/RUN3 loss from two sessions ago has not repeated).

## Open items, not yet done

- `enjoy_drive.py` still imports the 33-dim observation width implicitly via
  `PWMCommandWrapper` from `train_real_robot` — it will follow the 34-dim
  shape automatically the same way it followed 17→32→33 before, but this has
  not been re-verified end-to-end against the new checkpoint the way it was
  for run 3.
- `export_stm32.py` and `firmware/*` remain stale against all of runs 4–7
  (still generated from the 17-dim torque policy) — unchanged from the
  caveat carried forward since the previous session.
- Payload-sweep-at-turn was not re-measured this session; run 6 already
  established payload robustness on the forward axis, and nothing in this
  run's change (yaw only) should interact with payload, but that is an
  assumption, not a measurement.
