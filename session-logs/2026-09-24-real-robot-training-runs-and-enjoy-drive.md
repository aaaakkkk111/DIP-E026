# 2026-09-24 — Real-robot plant: training runs 1–3, and pointing `enjoy_drive` at the result

## Why a new plant at all

`their_robot.xml` — the model every earlier run trained on — is wrong for the
hardware the team actually built. Comparing it against the measured parameter
sheet (`PARAMS.md`):

| quantity | `their_robot.xml` | measured | error |
|---|---|---|---|
| COM height above axle | 0.120 m | 0.0340 m | 3.5× |
| inertia about the axle | ~0.017 kg·m² | 0.0014229 kg·m² | 12× |

Inertia sets the natural frequency of the inverted pendulum
(ω_n = √(mgl/I)), so a 12× error is not a tolerance question — it is a
different control problem. `real_robot.xml` was built from the sheet:
total mass 0.9420 kg, wheels 0.035 kg at r = 0.0335 m, half-track 0.0835 m,
1 ms timestep, `ctrlrange = ±0.4 N·m`, and a `payload_body` at z = 0.1050 m.

Two further changes came with it:

- **200 Hz control** (`FRAME_SKIP = 5` on a 1 ms timestep), matching the stock
  firmware's MPU6050-interrupt loop, instead of 80 Hz.
- **PWM actions instead of torque.** The team has no motor torque constant and
  no winding resistance, so a torque-commanding policy could not be converted
  to motor drive on hardware. `motor_model.py` lumps both unknowns into two
  quantities that *were* measured at the wheel:

  ```
  tau_wheel = TAU_STALL * duty - KV * omega
  TAU_STALL / KV = 0.5679 / 0.01620 = 35.06 rad/s
  no-load spec                      = 34.90 rad/s      -> agrees to 0.5%
  ```

## The actuator is bang-bang, and that drove most of the design

After the firmware's deadband compensation (`PWM_DEADBAND = 1300` of a 2800
count limit), the smallest non-zero command already delivers **0.257 N·m — 64%
of the 0.4 N·m driver limit**. There is no gentle nudge. The previous policy
commanded a *mean* of 0.053 N·m, five times smaller than anything this hardware
can produce, which on its own is enough to explain why it would not transfer.

A policy on this actuator has to balance by **dithering** — alternating sign
fast enough that the average torque is small — and that is a decision about
recent history, not the present instant. Hence the history taps added in run 3.

Two other constants moved for the same reason:

- `FALL_ANGLE_LIMIT` 0.40 → **0.70 rad**, matching the firmware's real 40° cut-out.
  Terminating at 23° with a −10 penalty taught the policy to treat as fatal a
  region the real robot recovers from routinely, and denied it any large-angle
  recovery experience at all.
- `log_std_init = ln(0.25)`. The action space is now ±1.0, so SB3's default
  std of 1.0 is 100% of the range and would clip constantly.

## The three runs

All three: 12 M steps, 19 parallel envs, `MlpPolicy` `net_arch=[64,64]`,
`linear_schedule(3e-4)`, `ent_coef=0.01`, `n_steps=2048`, `batch_size=128`,
CPU. Episodes are 2000 steps = 10 s at 200 Hz.

| run | what changed | payload ceiling | obs | wall clock | final eval reward |
|---|---|---|---|---|---|
| 1 | first run on `real_robot.xml` | 4.0 kg | 17 | — | not comparable |
| 2 | payload ceiling cut to 1.0 kg, deterministic eval payloads | 1.0 kg | 17 | 0:50:54 | 1391 ± 595 |
| 3 | + history taps, + command-timer fix | 1.0 kg | 32 | 2:50:09 | 1831 ± 362 |

**Runs 2 and 3 are directly comparable** — same plant, same eval command
schedule, same `EVAL_PAYLOADS`, same fall limit. Only the observation and the
training-time command distribution differ. **Run 1 is not**: its payload
ceiling was 4 kg, so its eval task was a different task.

### Run 1 — the payload ceiling was wrong

4.0 kg is four times the robot's own mass, and with `uniform(0, cap)` sampling
it dominated the second half of training. The result held 2000/2000 steps at
1–4 kg but drifted at 0.317 m/s when **unloaded** and commanded to hold
station. Payload makes this task *easier*, not harder — it raises the COM and
slows the dynamics, which suits a coarse bang-bang actuator — so training
mostly-loaded optimised for the easy case. Ceiling dropped to 1.0 kg, which
still spans 0–106% of the robot's mass.

A second defect surfaced here: the parent wrapper sampled payload randomly at
eval reset, so "new best mean reward" partly reflected *which payloads were
drawn*. Fixed with a fixed `EVAL_PAYLOADS` schedule — the same defect already
fixed for commands and for slope, missed here when the range grew.

### Run 3 — history taps and one inheritance bug

`HISTORY_TAPS = (2, 5, 11, 23, 47)` at 200 Hz spans 235 ms, matching the span
the team's own RL design uses. Three signals are remembered (pitch, pitch rate,
forward speed), giving 17 + 5×3 = **32 inputs**.

Run 3 was restarted minutes in, after `_sample_command_timer` was found to be
inherited from `train_yahboom_3d` — where it closes over *that module's*
`MAX_EPISODE_STEPS = 1000`, not this module's 2000. Its "long hold" branch
therefore topped out at half an episode, and the policy never practised holding
one command for a full 10 s, which is the entire reason that code exists.
**Constants do not follow subclassing** — an inherited method closes over the
parent module's globals.

## Where run 3 actually stands

Good:

- 2000/2000 steps on all six unloaded command cases (run 2 failed three of them)
- station-keeping drift **+0.000 m/s**
- turn **+0.399** against a **+0.50 rad/s** command
- **+0.15 m/s** command tracks at **+0.155 m/s**

Still weak, and not yet addressed:

- **+0.30 m/s** command yields only **+0.100 m/s**
- the payload sweep at +0.15 is erratic: +0.030 / +0.092 / +0.013 / +0.029

Suspected cause: the reward weights were inherited unchanged from a plant with
12× the inertia, deliberately, so that this run isolated the change of plant and
actuator rather than confounding it with a reward rewrite. That isolation has
served its purpose; re-tuning the weights for the real plant is the obvious
next experiment.

## `enjoy_drive.py` updated for the 32-dim policy

The demo imports `RealRobotEnv`/`PWMCommandWrapper` from `train_real_robot`, so
the observation width followed automatically. Everything else did not, and the
mismatches were real:

1. **Fall limit.** The wrapper defaults to 0.40 rad, but this policy was trained
   to 0.70. The default would have reset the robot at attitudes it can actually
   recover from — a working policy looking broken. Now imports `FALL_ANGLE_LIMIT`.
2. **Command ranges** were hard-coded copies annotated `train_yahboom_3d.py`.
   Now imported as `TRAIN_MAX_V_FORWARD` / `TRAIN_MAX_V_TURN` so they cannot drift.
3. **Payload range** was a hard-coded `(0.0, 1.0)` and a `--payload` help string
   still quoting the very first run's `0.0-0.2`. Both now derive from
   `MAX_PAYLOAD_KG`.
4. **Clamp inconsistency**: the slider went to 3.0 kg but the CLI and terminal
   command clamped at 2.0, so the same value was reachable one way and not the
   other. Both now use `MAX_PAYLOAD_SLIDER = 3.0` — deliberately past the
   trained ceiling, since finding where the policy gives up is the point.
5. **Observation-shape guard** added. SB3 does raise on a mismatch, but from
   deep inside `predict()` and without saying which side is which.
6. **Actuator telemetry** added — commanded PWM counts and delivered torque per
   wheel. On a bang-bang drive this is the trace worth watching: a torque
   reading near zero means the wheels are idle, not gently driven.
7. The missing-checkpoint message still pointed at `their_robot.xml` and told
   you to run `train_yahboom_3d.py`. Corrected.

### Verified, not assumed

Earlier in this project a scripted patch to this same file silently failed
because the match string didn't match, and the verification constructed the env
directly instead of calling `main()` — so the failure reached the user. This
time every path was executed end-to-end:

| check | result |
|---|---|
| GUI mode, 25 s | exit 124 (still running), no traceback |
| `--no-gui --payload 0.5 --slope 3` | clean start, conditions reported correctly |
| 17-dim checkpoint | guard fires: "checkpoint expects 17 inputs, this env produces 32" |
| missing checkpoint | guard fires, names `train_real_robot.py` |

## Caveats carried forward

- **`apply_slope` tilts gravity, not the floor.** That is correct physics in the
  slope's own frame, but the observation and reward are world-referenced, which
  overstates the difficulty — measured at ~5× (−3.109 m vs −0.587 m drift at 5°).
  Treat the slope slider as a stress test, not as a calibrated incline.
- **The floor geom is world-attached, therefore static.** Its pose is baked at
  compile time and `geom_quat` changes never reach the simulator. This cost a
  5 M-step run once. Any future slope work must read the normal back from
  `data.geom_xmat`, not from the rotation object that was applied.
- **`export_stm32.py` and `firmware/*` are stale.** They were generated from the
  17-dim `best_their` torque policy, and are wrong on both counts now: 32 inputs,
  and a PWM action space that no longer needs `policy_torque_to_duty` at all.

## Model files

| path | what it is |
|---|---|
| `models/best_real/` | run 3, 32-dim, best-by-eval — **the current policy** |
| `models/best_real_RUN2_17dim/` | run 2, 17-dim, kept for comparison |
| `models/ppo_real_robot.zip` | run 3 final-step |

Run 2's final-step `.zip` was lost: only the folder was renamed before run 3
started, so run 3's `model.save()` overwrote `ppo_real_robot.zip`. Rename both
next time.

---

# Addendum — "the robot is too shaky and fidgeting"

Reported after driving run 3's policy. Measured rather than assumed, and it
turned out to be a plant bug, not a policy bug.

## What the fidgeting actually was

1500-step rollouts of `models/best_real/best_model.zip`:

| | hold station | +0.15 m/s |
|---|---|---|
| sign flips | 48.6% / 43.1% of steps (~92 Hz reversal) | 51% / 52% (~103 Hz) |
| mean torque delivered | −0.0008 N·m | +0.0004 N·m |
| RMS torque | **0.321 N·m** | 0.339 N·m |
| mean \|PWM\| | 1762 counts (deadband 1300, limit 2800) | 1703 |
| pitch peak-to-peak | **0.462 rad (26°)** | 0.285 rad (16°) |

0.32 N·m RMS — 80% of the driver limit — spent to deliver a net torque of zero.
Pure dither.

## Root cause: `armature="0"` on the wheel joints

`real_robot.xml` modelled the wheels with their own inertia only
(1.964e-5 kg·m²) and no gearbox-reflected rotor inertia. For a 1:30 box the
reflected term is `J_rotor · G²`, roughly 100× the wheel's own figure. Leaving
it out gave:

```
wheel accel at the minimum non-zero torque   13086 rad/s^2
speed change in ONE 5 ms control tick         65.4 rad/s
                     motor no-load top speed  34.9 rad/s   <-- exceeded 2x
mechanical time constant J/KV                  1.2 ms
```

A modelled gearmotor reaching full speed in ~4 ms. No 1:30 gearbox does that.
So 100 Hz torque reversals were **free in simulation** and the policy, quite
reasonably, learned to balance by shaking the chassis. On real hardware the
reflected inertia would filter that out entirely — the dither would achieve
nothing and the transfer would fail.

## Fixes

1. **`armature="2.25e-3"`** on both wheel joints (`real_robot.xml`), assuming a
   rotor inertia of 2.5e-6 kg·m² through the 1:30 box → time constant 140 ms.
2. **Randomised over 9.0e-4 … 4.5e-3 per training episode**
   (`ARMATURE_RANGE`), because the rotor inertia is *estimated, not measured*.
   The range brackets rotor inertias of 1.0e-6 … 5.0e-6 kg·m². Eval is pinned at
   the nominal so scores stay comparable.
   **To measure it properly**: command a fixed PWM, log encoder speed, fit the
   exponential rise; `tau_m` is the time to 63% of final speed, and
   `J = tau_m · KV`.
3. **`ACTION_RATE_WEIGHT = 0.2`** — a new `-w · Σ(a_t − a_{t−1})²` term. The
   inherited effort penalty costs the same whether the command is held or
   flipped every tick, so nothing discouraged chatter. Even with the inertia
   fixed the command still reverses on ~95% of ticks; the mechanics absorb it,
   but the H-bridge still has to execute every reversal, and a reversal swings
   the winding voltage by 2× supply.

   Weight reasoning: a full reversal at the observed amplitude (~0.28) costs
   ~0.13/step against run 3's ~0.92/step average, ~14% of the achievable
   reward. **0.5 was tried first** and charged 1.0 per full reversal — more than
   half the per-step budget. At that weight the policy would likely stop
   dithering altogether, and with a 0.257 N·m torque floor it *needs* to dither
   to produce small torques.

## Effect, same policy, plant changed

| command | old: steps / v_fwd / pitch std | corrected: steps / v_fwd / pitch std |
|---|---|---|
| hold | 2000 / −0.024 / 0.0531 | 2000 / +0.008 / **0.0013** |
| +0.15 | 2000 / +0.167 / 0.0529 | 2000 / +0.042 / **0.0010** |
| +0.30 | **1496 (fell)** / +0.254 / 0.0978 | 2000 / +0.087 / **0.0028** |
| −0.15 | 2000 / −0.124 / 0.0515 | 2000 / −0.026 / **0.0015** |
| turn +0.5 | **1382 (fell)** / +0.543 turn / 0.0650 | 2000 / +0.819 turn / **0.0015** |

Shaking down ~40×, no more falls. But forward authority collapses (+0.15
commanded → +0.042 actual) and turns overshoot (+0.5 → +0.819), because the
wheels are now ~100× harder to accelerate and the policy never trained against
that. **Retraining is required**; the current policy is only usable as a
demonstration that the shaking is gone.

This also retires run 3's "remaining weakness" (+0.30 → +0.100) as a finding —
it was measured on a plant with the wrong wheel inertia.

## Two harness bugs caught while measuring, worth not repeating

- Compared "held command" vs "flipping command" returns to test the rate
  penalty. The held run **fell at step 29** while the flipping run survived to
  40, so the flipping run scored higher purely on episode length. Replaced with
  a direct test: same state, same action, only `_prev_action` differing —
  confirmed the penalty exactly (`held − flip = 1.0000`, expected 1.0000).
- The first old-vs-new sweep returned **identical numbers for both plants**,
  because it set `dof_armature` *before* `reset()`, and the new reset code
  re-sets armature every episode. Armature is a MODEL field: it survives reset
  and must be written after it, exactly like payload.

---

# Run 4 — armature fix, stopped at the 1M-step check

Launched with the three fixes above, stopped deliberately at the 1M checkpoint
rather than running the full 12M.

## Eval curve

| steps | reward | note |
|---|---|---|
| 190,000 | 454.99 ± 218.51 | episode length 559.85 |
| 380,000 | 2079.10 ± 908.47 | new best; episode length 2000.00 |
| 570,000 | 2018.61 ± 903.62 | |
| 760,000 | 2067.57 ± 878.13 | |
| 950,000 | 1990.80 ± 880.63 | |
| 1,140,000 | 2079.80 ± 889.48 | new best |

Reward climbs sharply to ~2080 by 380k and then plateaus. The ±889 spread is
larger than most of the mean, which is the tell — see below.

## What the checkpoint actually does

Both policies evaluated on the **corrected** plant at `ARMATURE_NOMINAL`, three
seeds, scored after a 400-step settle. Same plant, same conditions, so this
comparison is valid.

| command | run 3: steps / v_fwd / v_turn / pitch p2p | run 4 @1.14M: steps / v_fwd / v_turn / pitch p2p |
|---|---|---|
| hold | 2000 / +0.008 / +0.002 / 0.008 | 2000 / −0.002 / −0.006 / 0.008 |
| +0.15 | 2000 / +0.042 / −0.020 / 0.005 | 2000 / +0.039 / +0.012 / 0.037 |
| +0.30 | 2000 / +0.087 / −0.166 / 0.021 | 2000 / +0.066 / +0.019 / 0.056 |
| −0.15 | 2000 / −0.026 / +0.008 / 0.008 | 2000 / −0.042 / −0.017 / 0.010 |
| turn +0.50 | 2000 / — / **+0.819** / 0.010 | 2000 / — / **+0.009** / 0.020 |
| +0.15/+0.30 | 2000 / +0.044 / **+0.469** / 0.038 | 2000 / +0.054 / **+0.015** / 0.061 |

Payload sweep at +0.15 m/s:
run 3 → +0.042 / +0.043 / +0.042 / +0.042 / +0.042 (flat);
run 4 → +0.010 / +0.021 / +0.024 / +0.022 / +0.028 (weak, rising).

**Run 4 does not turn at all** (+0.009 against a +0.50 command) and tracks
forward worse than run 3. On the face of it that looks like a regression.

## It is not a regression — the curriculum had not started

`CurriculumCallback(stand_phase_end=0.15)`: below 15% progress,
`forward_frac, turn_frac, payload_frac = 0.0, 0.0, 0.0`. At 1.14M of 12M steps
the run was at **9.5% progress — 63% of the way through the stand phase**. The
policy had never been given a non-zero velocity command or any payload.

Eval, however, uses the fixed full-range `EVAL_COMMANDS` / `EVAL_PAYLOADS`
schedule throughout. So the eval was asking for turns and payloads the policy
had never practised. Every observation follows from that:

- perfect balance, 2000/2000 steps everywhere
- no turning, weak forward, erratic payload response
- reward plateaued at ~2080 — that is the **stand-phase ceiling**, not convergence
- ±889 spread = scores well on hold-station eval episodes, badly on the rest

## What the 1M check does and does not establish

Establishes:

- **Learning is healthy on the corrected plant.** Reward 455 → 2080 inside 380k
  steps, full-length episodes from 380k onward, no divergence or collapse.
- **The armature fix holds.** Pitch peak-to-peak is 0.008–0.06 rad against the
  0.462 rad measured before it. This is true of *both* policies on the corrected
  plant, which confirms it is the plant fix doing the work, not the retraining.

Does not establish:

- **Whether the action-rate penalty works.** Command sign still flips on ~95% of
  ticks for both policies. During the stand phase the policy must hold station
  at zero net torque, which demands maximal dithering, so ~95% is expected here
  and says nothing yet. The penalty gets its first real test once commands ramp.
- **Any final-quality comparison against run 3.** Run 4 stopped at 9.5% of
  budget, and run 3's own plant differs, so its eval numbers are not on the same
  scale either way.

## Artifacts

| path | what |
|---|---|
| `models/best_real_RUN4_1M/best_model.zip` | run 4 @1.14M, stand phase only |
| `session-logs/train_real_robot_RUN4_armature_1M.log` | run 4 log |
| `models/best_real_RUN3_noarmature/` | run 3, kept |
| `models/ppo_real_robot_RUN3_noarmature.zip` | run 3 final-step |

No final `model.save()` — the run was killed, so only the EvalCallback best
checkpoint exists. To resume this experiment, rerun `train_real_robot.py` from
scratch; the useful part of the budget (the command and payload phases) is all
still ahead.

---

# Run 5 — 30M steps with diagnostics. Converged.

## The premise that was wrong

The working assumption going in was "non-convergence". The data said otherwise.
Runs 2 and 3 were checked for plateau, oscillation and entropy collapse, and
showed none of them:

| | run 2 | run 3 |
|---|---|---|
| mean reward, 0–2M | 0.7 | −5.2 |
| mean reward, 10–12M | 1344.2 | 1817.3 |
| slope over final third | **+202.2 / 1M** | **+93.2 / 1M** |
| best eval at | 11,780,000 of 11,970,000 | 11,590,000 of 11,970,000 |
| final `log_std` | −2.64 → std 0.072 | −2.36 → std 0.093 |

Every 2M block higher than the last, best eval inside the final 3% of the run,
and `log_std` narrowing gently from its 0.25 start rather than collapsing (a
collapse would be ~0.001). **These runs were not failing to converge — they were
being cut off mid-climb.** The fix was budget, not algorithm.

## Changes

1. `total_timesteps` 12M → **30M**.
2. `batch_size` 128 → **512**. 19 envs × 2048 `n_steps` = 38,912 samples per
   rollout; at batch 128 with `n_epochs=10` that was **3,040 gradient updates
   per rollout**. Result: **1:56:09 for 30M steps at ~4300 fps**, against run 3's
   2:50:09 for 12M at ~1180 fps — about **3.7× faster per step**.
3. `verbose=1` + CSV logger (`logs/run5/progress.csv`). Until now the logs held
   *only* eval rewards, so every "why is this slow" diagnosis was guesswork.

## Eval curve — it plateaued this time

| block | mean | | block | mean |
|---|---|---|---|---|
| 0–3M | 1872.3 | | 15–18M | 3209.2 |
| 3–6M | 2151.9 | | 18–21M | 3256.7 |
| 6–9M | 2759.3 | | 21–24M | 3314.8 |
| 9–12M | 3016.6 | | 24–27M | 3423.1 |
| 12–15M | 3098.1 | | 27–30M | **3467.4** |

Slope: **+26.7/1M** over the final third, **+16.4/1M** over the final fifth,
**−0.1/1M** over the final tenth. Best eval **3481.5 at 27.93M**.

*Caveat, stated because it matters:* `linear_schedule(3e-4)` decays the learning
rate to ~0 at the end, and `approx_kl` (0.0057 → 0.0007) and `clip_fraction`
(0.046 → 0.0003) confirm the last stretch was barely updating. So the flat final
tenth is partly the schedule, not purely the task. The block means are
decelerating on their own though (+108 then +44), so "converged or very close"
is fair; "converged" unqualified is not.

## Training diagnostics — first time we have had any

| metric | start | 25% | 50% | 75% | end |
|---|---|---|---|---|---|
| `train/std` | 0.2487 | **0.0202** | 0.0347 | 0.0474 | 0.0562 |
| `explained_variance` | 0.0045 | 0.9451 | 0.9608 | 0.9095 | 0.8907 |
| `approx_kl` | 0.0063 | 0.0095 | 0.0074 | 0.0057 | 0.0007 |
| `clip_fraction` | 0.0746 | 0.1137 | 0.0786 | 0.0460 | 0.0003 |
| `value_loss` | 86.52 | 0.77 | 4.27 | 10.48 | 13.39 |

- **`explained_variance` 0.89–0.96** — the value function is doing its job.
  This was the single most valuable thing to learn, and it had never been visible.
- **`std` dipped to 0.0202 at 25% and then RECOVERED to 0.0562.** The concern
  raised when `ACTION_RATE_WEIGHT` was added — that `Σ(a_t − a_{t−1})²` on the
  *sampled* action taxes exploration noise itself (≈4σ² per step) — shows up as
  exactly that early dip. `ent_coef=0.01` pulled it back. So the penalty is
  affordable at 0.2, but it is not free, and the mechanism is real.

## Behaviour: run 5 vs run 3, both on the corrected plant

Three seeds, scored after a 400-step settle. All cases 2000/2000 steps.

| command | run 3 v_fwd / v_turn | run 5 v_fwd / v_turn |
|---|---|---|
| hold | +0.008 / +0.002 | −0.013 / +0.004 |
| +0.15 | +0.042 / −0.020 | **+0.132** / +0.011 |
| +0.30 | +0.087 / −0.166 | **+0.244** / +0.019 |
| −0.15 | −0.026 / +0.008 | **−0.119** / −0.008 |
| turn +0.50 | — / +0.819 (164% overshoot) | — / **+0.496 (99%)** |
| +0.15 & +0.30 turn | +0.044 / +0.469 (156%) | **+0.148** / **+0.301 (100%)** |

Payload sweep at +0.15 m/s (0 → 1.0 kg):
run 3 → +0.042 flat but weak; run 5 → **+0.129 / +0.136 / +0.137 / +0.137 /
+0.136** — flat *and* strong. Payload robustness is now a solved problem.

**The turn axis is solved.** +0.496 against +0.50 and +0.301 against +0.30, with
no cross-coupling into forward speed. This was the longest-running defect in the
project, and it no longer needs the filtered-yaw workaround to look acceptable.

Forward tracking reaches **79–88%** of command, up from run 3's 28–29%. Still
short of unity — see open questions.

Chatter: sign-flip rate fell where it matters, 79.0% → 59.5% at +0.30 and
90.4% → 78.3% at +0.15, with RMS torque 0.263 → 0.249 at +0.30. At hold-station
it stays at ~96%, which is correct and should not be "fixed": holding position
demands a near-zero mean torque, and with a 0.257 N·m floor the only way to
produce that is to alternate every tick.

Pitch stays tiny throughout: SD 0.0010–0.0025 rad, peak-to-peak 0.005–0.012 rad,
against the 0.462 rad that started this whole investigation.

## Open questions

- **Forward tracking tops out at ~80–88%.** Unresolved whether this is the
  reward weighting (velocity error 2.0 against pitch 3.0 — never retuned for
  this plant) or a genuine limit of 0.4 N·m against the corrected wheel inertia.
  **Still untested, and it should be tested before more reward tuning**: measure
  the maximum sustained speed an oracle controller can hold.
- `ACTION_RATE_WEIGHT` at 0.2 costs exploration early. 0.05–0.1 may give the
  same chatter reduction without the dip.

## Artifacts

| path | what |
|---|---|
| `models/best_real/best_model.zip` | **run 5 @27.93M — current best policy** |
| `models/ppo_real_robot_RUN5_30M.zip` | run 5 final-step |
| `session-logs/train_real_robot_RUN5_30M.log` | run 5 log |
| `logs/run5/progress.csv` | 928 rollouts of training diagnostics |

---

# Run 6 — station keeping. Idle drift fixed, and forward tracking solved as a side effect.

## The defect

Run 5 drifted **0.408 m per 10 s** (~2.4 m/min) while commanded to hold station,
creeping at a steady ~0.039 m/s in a seed-dependent direction with the chassis
level. Measured across 5 seeds; the direction varied (−0.42 to +0.44 m) but the
speed did not.

Cause: **the reward graded speed, never place.** "Still" and "creeping at 4 cm/s"
scored within 0.08 of each other per step, and velocity error integrates into
unbounded position error. Run 3 looked better (0.073 m) only because it could
barely move at all — 28% of commanded speed. Better driving and worse standing
were two faces of the same gain increase, not separate bugs.

## Why a reward term alone would not have worked

`_get_conditioned_obs` deletes world x and y (`raw_obs[2:]`). **The policy cannot
see where it is.** Penalising an error it cannot perceive would only have injected
noise into the reward. The deletion is correct in itself — world coordinates do
not generalise, and commands and reward are body-frame — but it means position
has to be *re-introduced* in a frame-invariant form, in the observation and the
reward together.

## The change

A **leaky integral of forward velocity error**, in both places:

```
e <- decay * e + (v_actual - v_target) * dt       decay = exp(-dt / 2.0 s)
reward -= 2.5 * |e|
obs    += [e * 10.0]                              33 inputs, up from 32
```

Leaky rather than a raw displacement penalty, for two reasons:

- **Bounded by construction.** A steady error E settles at E·τ instead of growing
  all episode. Verified: a 0.039 m/s creep settles at 0.0781 m against theory
  0.0780, costing 0.195 reward/step — about 11% of run 5's ~1.74/step average,
  deliberately the same order as the effort and action-rate terms.
- **Self-forgetting.** A 0.2 m error decays to 0.0736 m in one time constant
  (e⁻¹·0.2 = 0.0736 ✓), so the robot is never asked to chase distance lost under a
  previous command. A raw integral would have demanded exactly that, and at the
  +0.30 command — where run 5 reached only 81% — it would have accumulated over a
  metre of unpayable debt and saturated the penalty.

Updated in `_get_conditioned_obs` because that is the single place called exactly
once per step and once per reset, and the value must exist before the observation
containing it is built.

## Result: idle drift

| | run 3 | run 5 | **run 6** |
|---|---|---|---|
| idle drift / 10 s | 0.073 m | 0.408 m | **0.012 m** |
| per minute | 0.4 m | 2.4 m | **0.07 m** |
| idle creep speed | ±0.006 m/s | ±0.039 m/s | **+0.001 m/s** |
| seed spread | ±0.04 m | −0.42 … +0.44 m | **+0.010 … +0.015 m** |

**34× better than run 5, 6× better than run 3** — and consistent across seeds
rather than picking a direction. Best station keeping in the project.

## Result: tracking improved rather than traded away

| command | run 5 | **run 6** |
|---|---|---|
| +0.15 m/s | +0.132 (88%) | **+0.156 (104%)** |
| +0.30 m/s | +0.244 (81%) | **+0.287 (96%)** |
| −0.15 m/s | −0.119 (79%) | **−0.151 (101%)** |
| turn +0.50 | +0.496 (99%) | +0.528 (106%) |
| +0.15 & turn +0.30 | +0.148 / +0.301 | +0.151 / **+0.338 (113%)** |

Payload sweep at +0.15 m/s: **+0.155 / +0.156 / +0.156 / +0.155 / +0.155** across
0 → 1.0 kg. Flat *and* accurate.

**This settles the open question from run 5.** Forward tracking was stuck at
79–88% and it was unresolved whether that was the reward weighting or a hard
limit of 0.4 N·m against the corrected wheel inertia. It was the reward:
**+0.30 m/s is physically reachable**, and it took a station-keeping term to get
there — because an integral term is exactly what removes steady-state tracking
error, the same reason a PI controller beats a P controller. No oracle-controller
test was needed after all.

Honest debit: **turn now overshoots slightly** (106% at +0.50, 113% in the
combined case) where run 5 was 99–100%. Small, and the likely cause is the same
integral authority helping forward while the turn axis has no equivalent term.
Adding a matching yaw integral is the obvious follow-up.

Also: idle `pitchSD` rose 0.0011 → 0.0059 rad and idle sign-flips 95.9% → 99.5%.
Both expected — *actively* holding position costs more corrective action than
drifting freely does. 0.0059 rad is 0.34°, still far inside anything visible.

## Convergence and diagnostics

| block | mean | | metric | start | 50% | end |
|---|---|---|---|---|---|---|
| 0–5M | 1378.4 | | `train/std` | 0.2505 | 0.0392 | 0.0640 |
| 10–15M | 2186.1 | | `explained_variance` | 0.0024 | 0.9701 | 0.9103 |
| 20–25M | 3284.2 | | `approx_kl` | 0.0057 | 0.0080 | 0.0003 |
| 25–30M | 3409.8 | | `clip_fraction` | 0.0715 | 0.0885 | 0.0000 |

Slope +23.4/1M over the final third, −6.1/1M over the final tenth. Best eval
3457.5 at 27.93M. 30M steps in 2:55:36.

`std` shows the same early dip (0.019 at 25%) and recovery (0.064) as run 5, so
the action-rate penalty's exploration tax is reproducible and still affordable.

**Run 6's eval reward is NOT comparable to run 5's 3481.** The reward function
gained a term, so the scale changed. Only the behavioural measurements above are
comparable, and they were taken identically for both.

## Artifacts

| path | what |
|---|---|
| `models/best_real/best_model.zip` | **run 6 @27.93M — current best, 33 inputs** |
| `models/ppo_real_robot_RUN6_30M.zip` | run 6 final-step |
| `models/best_real_RUN5_30M/` | run 5, 32 inputs |
| `session-logs/train_real_robot_RUN6_station.log` | run 6 log |
| `logs/run6/progress.csv` | diagnostics |

Note the observation layout has now been 17 → 32 → 33 across runs, so each
checkpoint only loads against its own revision. `enjoy_drive.py`'s guard names
both numbers on a mismatch.
