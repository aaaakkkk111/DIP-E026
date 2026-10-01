# Session log — 2026-09-20 (part 2): PID baseline, and switching the plant

Continues `2026-09-20-episode-length-and-driving.md`.

**Question asked:** *"use their model, and port their PID structure. if it can
balance while moving, then it just the training RL left."*

**Answer: yes — on their model.** A classical cascade PID holds any commanded
cruise there with zero steady-state error. But the same controller cannot
stabilize `my_robot.xml` under any gains, so the RL failures were never purely
a training problem: the old plant was a large part of it.

---

## 1. Porting the teammate's PID

Source: `DIP-E026`, branch `Non-harness-JiangLi`, `firmware/controllers.py`.
Their model was regenerated from their `RobotParams` via their `build_mjcf`
into `their_robot.xml` (nq=9, nv=8, nu=2, mass 1.700 kg, timestep 0.0025,
ctrlrange ±0.6 Nm).

Their firmware sums the velocity term straight into the motor command:

```
motor_l = balance + velocity + turn
```

That works inside their PWM/encoder unit scaling. Ported literally into
physical torque units it is **positive feedback** and diverged in under 0.1 s
(wheels roll back → velocity error grows → they roll back harder). Rewritten
as an explicit cascade in `pid_baseline.py`:

```
v_err        = target_v - lpf(v)
pitch_target = clip(Kp_vel*v_err + Ki_vel*∫v_err, ±0.20 rad)   <- the clamp is what stops the runaway
balance      = Kp_bal*(pitch - pitch_target) + Kd_bal*pitch_rate
motor_l/r    = balance ± turn
```

Two sign errors along the way, settled empirically rather than by argument
(a held lean of +0.05 rad cruises at +1.17 m/s, so positive pitch = forward).

### Result on their model (200 Hz, gains kp_bal=17.38 kd_bal=0.655 kp_vel=0.453 ki_vel=2.655)

| target v | survived | v settled | error |
|---|---|---|---|
| +0.10 | yes | +0.100 | 0.000 |
| +0.30 | yes | +0.300 | 0.000 |
| +0.40 | yes | +0.400 | 0.000 |
| −0.30 | yes | −0.300 | 0.000 |

60 s hold at 0.3 m/s: velocity pinned at 0.300, pitch ≈ 0.000, wheel angle
6.24 → 354.38 rad monotonically — ~56 revolutions, ~17.7 m. Genuinely rolling,
not rocking. Robust rather than knife-edge: **308 of 311** balancing gain sets
could also cruise.

Note it cruises at pitch ≈ 0 and leans only while accelerating, which is what
the physics predicts and what the RL policy never learned to do.

---

## 2. `my_robot.xml` is a badly conditioned plant

The same controller was swept against the old model: **0/300, 0/400, 0/48**
gain sets balanced it, across a 64× `kp` range, both sign conventions, and
three torque limits. The RL policy balances it for 20 s in the same harness,
so the harness is not at fault.

| | their model | `my_robot.xml` |
|---|---|---|
| Inertia about wheel axle | 0.0296 | **0.00284** (10.4× lower) |
| COM height | 120 mm | 37.6 mm |
| From spawn, no torque | sits still | **topples by 0.6 s**, `ncon` flickering 2→4→10 |
| `ctrl = +0.02` → velocity | +0.62 | **+0.008** |

That last row is the clearest statement of the problem: on the old model the
torque spins the chassis instead of driving the wheels.

---

## 3. Switching RL training to their model

`train_yahboom_3d.py` now loads `their_robot.xml`. Changes made, and why:

| change | reason |
|---|---|
| `XML_FILE_PATH` → `their_robot.xml` | section 2 |
| added `payload_body` to `their_robot.xml` | the generated MJCF has none; the payload curriculum needs it. `contype/conaffinity=0`, mass 0 at compile time. Verified a no-op: nq/nv/total-mass unchanged and the PID results above reproduce exactly |
| action space from `model.actuator_ctrlrange` (±0.6) | the real motor's stall torque, instead of `my_robot.xml`'s invented ±4.0. The policy can no longer rely on torque the hardware lacks |
| effort penalty normalized by τ_max | at a fixed 0.1 coefficient the term was tuned for ±4.0 (where the policy used ~1% of range and it was dead). Carried over unchanged it vanishes; rescaled to match the old magnitude it would cost 3.2 — more than the alive bonus — and punish the torque needed to stay upright. Now capped at 0.2 |
| **added reset noise** (±0.05 rad tilt, ±0.10 vel) | see below |
| `FRAME_SKIP` 10 → 5 (80 Hz) | see below |
| eval now issues held commands | see below |
| outputs → `models/best_their/` | old checkpoints share the 17-dim obs shape, so they would load silently against the wrong plant |

### 3a. Reset noise — the new model is *too* stable to learn from

Measured before adding it: a **zero-torque episode survived all 1000 steps and
banked a return of 1634.** `my_robot.xml` was numerically unstable at spawn and
began toppling on its own, which accidentally supplied the disturbance the
policy had to reject. Their model sits at its equilibrium indefinitely, so
without perturbation the optimal policy is literally "output nothing".

After adding noise, zero-torque episodes last **11–23 steps (mean 15.1)** over
20 seeds. The task is real again.

### 3b. Control rate is not a free parameter

The 200 Hz gains were run inside the RL harness at the old 40 Hz rate and
**fell in every condition** (20–381 steps). Sweeping 120 random gain sets per
rate, counting those that both balance and hold a 0.3 m/s cruise:

| rate | 400 Hz | 200 Hz | 100 Hz | 80 Hz | 50 Hz | 40 Hz | 25 Hz | 20 Hz |
|---|---|---|---|---|---|---|---|---|
| working gain sets /120 | 93 | 95 | 97 | 91 | 64 | **24** | **0** | **0** |

40 Hz sits on the cliff — only 20% of controllers can stabilize the plant
there. Asking PPO to find one in a basin that narrow is a needless handicap.
80 Hz is in the flat region and is easily achievable on the STM32 for a 64×64
MLP. Episodes are now ~12.5 s.

### 3c. Eval measured the wrong thing

Eval forced both commands to zero, so "best model" was selected purely on how
long the robot could stand still — the one thing the old policy already did
perfectly (1000±0) while being unable to drive at all. That metric cannot tell
a good driver from a statue, which is exactly the failure being chased.

Eval now runs a fixed list of 20 commands (`EVAL_COMMANDS`), one per episode,
**held for the whole episode** so sustained driving and continuous turning are
what get scored. Fixed rather than sampled so the number stays comparable
across evaluations; starts at (0,0) so standing is still measured.

---

## 4. Validation before spending compute

The known-good PID was run inside the RL environment itself, at 80 Hz, with
reset noise active, gains retuned for the rate (19.96 / 1.604 / 1.947 / 2.250):

| cmd v | cmd turn | steps | fell | v actual | turn actual |
|---|---|---|---|---|---|
| 0.00 | 0.00 | 1000 | no | 0.004 | −0.000 |
| 0.20 | 0.00 | 1000 | no | 0.197 | 0.000 |
| 0.30 | 0.00 | 1000 | no | 0.280 | 0.000 |
| −0.30 | 0.00 | 1000 | no | −0.296 | 0.000 |
| 0.00 | 0.50 | 1000 | no | 0.020 | 0.219 |
| 0.30 | 0.50 | 1000 | no | 0.288 | 0.403 |

(worst of 3 seeds each). **The environment is winnable** — a fixed-gain
controller does what the RL policy could not on the old plant.

## 5. The number RL has to beat

`pid_eval_benchmark.py` scores the PID on the identical EvalCallback task:

```
mean episode length : 1000.0 +/- 0.0   (cap 1000)
mean return         :  360.2 +/- 163.3
```

Per-command, forward tracking is near-exact (0.287 against 0.30) but **turn
tracking is the PID's weak axis** (0.348 against 0.50; the worst episode,
v=−0.30 with turn=−0.50, returns −21.2). Its turn term is a plain yaw-rate P
damper with no integral. That is where RL has the clearest room to win.

---

## 5a. Attempt 1 killed at 380k — exploration noise was larger than the action range

First run climbed, but far too slowly: 49.5 steps at 190k, 72.0 at 380k,
against a PID baseline of 1000. Measured the cause on the saved checkpoint
rather than guessing:

```
action limit       : 0.60
policy action std  : 0.668      <- 111% of the usable range
```

SB3 defaults `log_std_init=0`, an initial std of 1.0. On `my_robot.xml`'s
±4.0 action space that was 25% of range — sensible exploration. On this
plant's physical ±0.6 Nm limit the same default is **167% of range**, so
roughly a third of sampled actions clip to saturation and the effective policy
is bang-bang rather than Gaussian. Balancing needs fine torque modulation.

It does not self-correct either: `ent_coef=0.01` rewards keeping std large,
and there is far more entropy available outside the clip bounds than inside
them, so after 380k steps it had fallen only from 1.67 to 1.11 of range.

This is the same lesson as the reverted action-scaling experiment from part 1,
running in the opposite direction: **what matters is noise relative to the
action range.** Shrinking the action space to the real motor's torque limit was
correct; not compensating the initial noise was the mistake.

Fix: `log_std_init = log(0.25 * tau_max)` = −1.897, i.e. std 0.15 = 25% of
range, the same ratio the old plant had. Derived from the model's ctrlrange so
it cannot drift if the plant changes again.

Attempt 1 preserved as `models/best_their_ATTEMPT1_default_logstd/` and
`session-logs/train_their_model_ATTEMPT1_default_logstd.log`.

## 5b. Attempt 2 progress

Eval is **stationary**: `EVAL_COMMANDS` scale by `max_v_*`, not by the
curriculum caps, so eval runs the full ±0.3 m/s / ±0.5 rad/s task from step
one while only training difficulty ramps. Numbers across evals are therefore
directly comparable, and a best-model record set early by a policy that stands
still can only be beaten by one that balances *and* tracks.

| timesteps | ep length | return | curriculum phase |
|---|---|---|---|
| 190k | 80.1 | 78.1 | stand |
| 380k | 423.2 | 564.3 | stand |
| 570k | 940.1 | 1340.2 | stand |
| 760k | 961.3 | 1376.8 | forward ramp opens |
| 950k | 935.4 | 1350.1 | forward ~20% |
| 1140k | 894.9 | 1294.0 | forward ~39% |
| 1330k | 935.5 | **1385.4** | forward ~58% |

(attempt 1 for comparison: 49.5 at 190k, 72.0 at 380k)

Balance was solved inside the stand phase. The 950k–1140k dip is a real, if
marginal (~2 SEM), decline on a fixed task rather than a metric artifact; it
recovered at 1330k, which set a new best *while* carrying command-tracking
penalties — the first checkpoint that beats the standing-still record.

### Forward tracking is genuine as of 1.33M

At 380k the benchmark showed nonzero `v_act` that looked like tracking. It was
not: the first-layer weights on both command inputs were 0.1488 / 0.1404,
indistinguishable from untrained inputs (trained ones stand out clearly — obs3
at 0.46). With the curriculum holding commands at zero until 750k, those were
random weights acting as a constant bias.

At 1.33M:

```
CMD_forward 0.1488 -> 0.1789     (receiving gradient)
CMD_turn    0.1404 -> 0.1404     (unchanged; turn phase opens at 1.75M)
```

and the sign is correct in every pure-forward row (+0.30→+0.229,
−0.30→−0.162, +0.22→+0.327, −0.22→−0.219). Magnitudes are erratic and 0.15
barely moves, but this is real command response, which never appeared on the
old plant.

**Caveat:** driving still costs stability. Every row carrying a forward command
terminated early (691–897 steps) while every zero-forward row ran the full
1000. Mean length 928 vs the PID's 1000.

**On comparing returns:** the policy's 1361 against the PID's 360 is not a fair
comparison. The policy earns ~1.99/step in the zero-command episode (max is
2.0) while the PID earns 0.36/step, because the PID holds a visible limit cycle
that the pitch and effort terms penalize. Episode length is the fair summary,
and there the policy is still behind. Worth noting from the PID table though:
its return at cmd 0.30 (620.2) is essentially the same as standing still
(614.0), which is direct evidence the reward does **not** penalize sustained
cruise — a WIP cruises at pitch ≈ 0 and leans only while accelerating. The
PID's poor turn scores are its own tracking weakness, not a reward defect. No
reward changes are warranted on this evidence.

## 5c. The turn axis does not train — diagnosis

By 2.85M (turn commands at full ±0.5 rad/s since ~2.5M) the policy balances
and drives forward, but **does not turn at all**: `turn_act` is within ±0.015
for every command. The first-layer weight on the turn command confirms it is
not being learned, only drifting:

| checkpoint | CMD_forward | CMD_turn |
|---|---|---|
| 380k (both provably untrained) | 0.1488 | 0.1404 |
| 1.33M | 0.1789 | 0.1404 |
| 1.9M | 0.2176 | 0.1404 |
| 2.47M | 0.2393 | 0.1357 |
| 2.85M | — | 0.1408 |

### The incentive to turn is weak and fragile — this IS a reward issue

*(This section replaces an earlier, wrong conclusion. The first version
observed that the `(0.00, 0.50)` episode scores 1489.9 against the
uncommanded episode's 1986.6 and concluded ~500 return was "sitting
unclaimed" and that reward design was fine. That inferred the gap was
claimable without measuring what claiming it costs. It is not.)*

Measuring the turn cost the reward actually charges — per-step
`|yaw_rate − target|`, not the episode-mean yaw rate:

| controller | per-step turn cost, averaged over the 20 eval commands |
|---|---|
| never turns at all | **0.256** |
| PID at its best-scoring gain (kp_turn 0.25) | 0.266 |
| PID at kp_turn 2.0 | 0.418 |

Turning at the best available gain buys **−0.010 reward/step**. Aggregated
over the eval set, no PID gain beats simply not turning.

A caveat on scope: that is one controller family (P on yaw rate). Isolating a
single 0.5 rad/s target and trying P, PI and low-pass-filtered variants, the
best found cuts per-step cost from 0.500 (never turning) to **0.277** — so
turning does pay on strongly-commanded episodes, worth ~0.22/step. But error
never gets near zero: 0.277 is still 55% of the command magnitude.

An important measurement artifact that produced a second wrong conclusion
along the way: a gain sweep scored by *episode-mean* yaw rate showed error
falling monotonically to 0.026 at kp_turn=2.0, which looked like "better
tracking earns less reward". It is not — high gain drives a yaw limit cycle
whose mean is accurate while its instantaneous error is worse (0.418). The
reward charges instantaneous error, so it is internally consistent. Any
metric used to judge this axis must be per-step, not episode-mean.

**Conclusion:** the policy leaving turn at zero is near-optimal under this
reward. The incentive exists only on full-magnitude turn episodes, is small
(~0.22/step), and is wiped out by any stability cost. This is why the turn
weight never grows.

**Implied fix** (untested): reward *filtered* yaw tracking rather than
instantaneous, since the achievable mean yaw is good (0.485 against a 0.500
command at kp_turn=4.0) while the achievable instantaneous error is not, and/or
raise the turn-error weight above 1.0. Raising the weight alone will not fix
it if achievable error stays near the not-turning baseline, since the weight
scales both sides.

### It is not simple exploration distance either

First hypothesis was that the manoeuvre sat ~7 sigma away from a policy whose
action std had collapsed to 0.0367 (6% of range). Probing the trained policy
with a constant differential-torque bias (scratchpad/turn_probe.py) refuted
the simple version of that:

| bias (Nm) | in sigma | steps | turn_act | survived |
|---|---|---|---|---|
| 0.02 | 0.5 | 1000 | −0.002 | yes |
| 0.05 | 1.4 | 1000 | −0.014 | yes |
| 0.10 | 2.7 | **35** | −2.161 | **no** |
| 0.25 | 6.8 | 56 | −21.586 | **no** |

There is no gradual middle. Below ~1.4 sigma the policy **actively cancels**
the bias; above it the robot falls within 35 steps rather than turning. (The
−21 rad/s readings are post-fall artifacts — the chassis spinning on the
ground.) So turning is not a nearby action that sampling merely missed.

### Secondary contributor: the curriculum trains a yaw-zero prior first

*(Written before the reward measurement above, which supersedes it as the
primary explanation. Kept because the probe data stands and the two are
consistent: turning is both barely rewarded AND actively suppressed, and the
suppression is what the probe shows.)*

For the 1.75M steps before the turn phase opens, the turn target is always 0
and both the reward and the PBRS term penalize any yaw. The policy therefore
learns an explicit yaw **regulator** — which is exactly what the probe shows
it doing. The turn phase then has to unlearn that, using exploration noise
that has already decayed to 6% of range.

This staging was inherited from `my_robot.xml`, where turning was the *easy*
behaviour that crowded out forward driving, so it was deliberately staged
last. **On this plant the ordering is exactly reversed**, which is one more
case of an old-plant conclusion failing to transfer.

### Recommended fixes (none applied), in priority order

1. **Change what the turn term measures** — reward filtered/mean yaw tracking
   instead of instantaneous. This addresses the primary cause: achievable mean
   yaw is good (0.485 vs a 0.500 command) while achievable instantaneous error
   is not (0.277 at best, 55% of command). Without this, turning is not worth
   learning and no amount of exploration will change that.
2. **Introduce turn commands early**, alongside forward, so no yaw-suppression
   prior consolidates while entropy is still available. Addresses the
   secondary contributor. Optionally re-inflate `log_std` at phase boundaries.

Neither applied unilaterally: (1) changes the task definition and (2) is a
curriculum redesign, while the run in progress produces a genuinely good
balance + forward-driving policy that is worth keeping either way.

### Also noted

Eval reported 1000.00 ± 0.00 at 2.85M, but the benchmark on different seeds
averaged 953.8 ± 201.6 with one catastrophic episode (`−0.30, +0.50` → 75
steps, return −5.6). The perfect score is seed-dependent; robustness is not
uniform. Forward tracking is improving but still overshoots roughly 1.8x at
±0.30 (0.519 / −0.558), while small commands are now near-exact (0.07 →
0.078).

## 6. Status — run complete

5M timesteps finished cleanly on `their_robot.xml`. Log at
`session-logs/train_their_model.log`, best checkpoint at
`models/best_their/best_model.zip`, final at
`models/ppo_yahboom_drive_their.zip`.

Final benchmark (`policy_eval_benchmark.py`, identical task to the PID one):

| | mean episode length | mean return |
|---|---|---|
| Trained policy | **1000.0 ± 0.0** | **1694.0 ± 189.1** |
| Tuned cascade PID | 1000.0 ± 0.0 | 360.2 ± 163.3 |

Balance and forward driving are solved — forward tracks +0.30 → +0.306, and
the ~1.8× overshoot seen at 1.9M resolved itself by the end of training. Yaw
against a ±0.50 command remains +0.032 / −0.067, i.e. unlearned, for the
reward reason analysed in §5c.

**Summary of findings and proposed fixes:
`session-logs/FINDINGS-turn-axis-and-next-steps.md`.**

`enjoy_drive.py` now points at `models/best_their/` and refuses to run rather
than silently loading a `my_robot.xml` checkpoint (same obs shape, so it would
load without error and merely saturate the ±0.6 Nm motors — looking like a bad
policy rather than a mismatched one).

**Open:** whether `my_robot.xml` is worth salvaging. Nothing in this session
recommends it; their model is calibrated against real firmware, which also
makes the eventual STM32 port meaningful.

## Files touched

- `their_robot.xml` — generated from their `RobotParams`; `payload_body` added
- `pid_baseline.py` — new, cascade PID port
- `pid_eval_benchmark.py` — new, scores the PID on the EvalCallback task
- `train_yahboom_3d.py` — retargeted to their model (table in §3)
- `enjoy_drive.py` — checkpoint path + mismatch guard
