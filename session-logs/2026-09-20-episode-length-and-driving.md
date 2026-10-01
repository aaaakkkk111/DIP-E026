# Session Log — 2026-09-20 — Episode length, reward shaping, driving controls

Covers `train_yahboom_3d.py` and `enjoy_drive.py` changes and diagnostic
trials from this session, in chronological order.

## 1. Episode length improvements

**Problem:** episodes ended almost instantly — the robot is a 2-wheel
self-balancer with a free-floating base (`freejoint`), inherently unstable,
and a fresh/undertrained policy fell over in a few steps.

**Changes:**
- Added `TimeLimit(max_episode_steps=1000)` — previously there was no
  episode horizon at all; length was 100% determined by time-to-fall.
- Added a 10-step "settle" grace period after reset before the fall check
  applies, so reset transients can't trigger instant termination.
- Wrapped training envs in `Monitor` (previously only the eval env was),
  so `ep_len_mean` logging is accurate.
- `CurriculumCallback` (see §3) ramps difficulty instead of throwing full
  difficulty at the policy from step 0.

## 2. Potential-based reward shaping (PBRS)

Added `_potential()` blending two potentials by how much velocity is
commanded (`w = ||target|| / max`):
- `stand_potential`: rewards near-zero tilt/drift when idle.
- `move_potential`: rewards closing the velocity-tracking gap when driving.

`reward += gamma * Phi(s') - Phi(s)` — policy-invariant (Ng et al. 1999),
adds dense gradient without changing the optimal policy.

**Known issue found later (§6):** the shared scalar `w` conflates the
forward and turn axes — see §6.

## 3. Staged curriculum (replacing a simultaneous ramp)

`CurriculumCallback` now stages difficulty instead of ramping payload +
velocity + everything together from step 0:
1. 0-15% of training: pure standing (velocity commands and payload both 0).
2. 15-50%: ramp velocity commands in (payload still 0).
3. 50-100%: ramp payload in.

Rationale: ramping everything at once forced balance and velocity-tracking
to compete for gradient signal before balance was solid, producing a
fragile policy (eval oscillated: reached 1000±0 episode length once, then
regressed to ~813±373 two evals later).

Also bumped `n_eval_episodes` 5 → 20, since 5 was too small a sample —
one lucky/unlucky payload draw could swing the "best model" selection.

## 4. Command-hold duration widening

Training originally resampled commands every 100-300 steps (2.5-7.5s),
so the policy never practiced sustaining one direction longer than that.
Added `_sample_command_timer()`: 30% chance of a long hold (400 steps up
to a full episode), so sustained turning/driving became part of training.

## 5. `enjoy_drive.py` fixes

- **WASD → Arrow keys.** MuJoCo's viewer runs built-in keyboard shortcuts
  *alongside* any custom `key_callback`, not instead of it. Every letter
  key is bound to something (W=wireframe, A=auto-connect, S=shadow,
  D=static-body visibility) — confirmed via MuJoCo docs + testing. This
  caused visible glitches (e.g. background flipping black/white on `D` =
  toggling ground-plane visibility). Arrow keys avoid the collision since
  their built-in bindings (playback speed / frame-step while paused) are
  no-ops here — physics is stepped manually via `env.step()`, not MuJoCo's
  own physics thread.
- **Reset was silently wiping held commands.** `VelocityCommandWrapper.reset()`
  hardcodes targets to `(0,0)` when `is_eval=True`. `enjoy_drive.py` uses
  `is_eval=True` with no `TimeLimit`, so `done` only fires from a stumble
  past the fall threshold — which teleports the robot upright instantly
  (`set_state`), looking like "it just stopped" rather than an obvious
  fall. Every such stumble silently zeroed the held key command. Fixed by
  saving/restoring `target_v_forward`/`target_v_turn` across the internal
  reset ([enjoy_drive.py](../enjoy_drive.py) `if done:` block).
- Controls are tap-to-toggle (not hold-to-move), since MuJoCo's
  `key_callback` only fires on key-press, no held/release state:
  Up/Down toggle forward/backward, Left/Right toggle turn.

## 6. Reward reweighting — first diagnosed cause of "can't drive"

**Diagnostic finding (direct programmatic test, bypassing keyboard):**
holding a forward command steady for 200 steps produced **zero sustained
forward velocity** — a transient blip then decay back to ~0, same for
turning (transient peak then decay). Confirmed via `model.num_timesteps`
that the checkpoint was trained for 4.94M/5M steps — not an
undertrained-policy problem.

**Root cause (before this fix):** reward weights made "never tilt" the
dominant strategy regardless of commands:
- `-|pitch| * 5.0`: a mere 0.1 rad tilt (needed just to accelerate) cost
  **-0.5**/step.
- `-|Δv_forward| * 1.0`: the worst possible full miss on the *entire*
  forward range (0.3 m/s) only cost **-0.3**/step.

A small tilt already cost more than fully failing to track velocity, and
a fall also forfeits the `+2.0`/step alive bonus for the rest of the
episode — so "ignore commands, minimize tilt" was the rational policy
under those weights.

**Fix applied:** pitch weight `5.0 → 3.0`, forward tracking `1.0 → 2.0`,
turn tracking `0.5 → 1.0`. Retrained (4.56M/5M steps, `models/best/best_model.zip`
saved 2026-09-20 00:02).

## 7. Post-retrain diagnostic — turning fixed, driving still dead

Direct-control test (replicating `enjoy_drive.py`'s exact toggle logic)
against the retrained checkpoint:

| Command | Target | Actual (last 50 steps) | Peak | max\|pitch\| |
|---|---|---|---|---|
| Forward | +0.30 | -0.000 | -0.001 | 0.024 |
| Backward | -0.30 | +0.000 | +0.001 | 0.024 |
| Turn left | +0.50 | +0.001 | **+0.367** (73%) | 0.026 |
| Turn right | -0.50 | -0.027 | **-0.411** (82%) | 0.026 |

Turning now works well. Forward/backward is completely flat — not just
weak, genuinely zero attempt, and `max|pitch|` is identical whether
forward is commanded or not, meaning the policy isn't even trying to lean.

**Initial hypotheses (one later proven WRONG — see §8):**
1. **Mechanical asymmetry** — turning in place needs no net lean (wheels
   spin opposite directions, reaction torques cancel in pitch); forward
   motion needs a coordinated lean to accelerate (inverted-pendulum
   physics, non-minimum-phase: must first drive wheels *backward* to
   pitch the body forward). Genuinely the harder skill. **Confirmed.**
2. ~~**PBRS blend-weight bug** — the shared scalar `w` starves the forward
   axis when a turn command dominates.~~ **This was wrong.** Verified
   numerically in §8: the shared `w = max(...)` pulls *both* axes toward
   full tracking weight, so a large turn command gave the forward axis
   *more* tracking weight, not less. Retracted.

## 8. Root cause found: entropy collapse

Before retraining on hypothesis #2, ran a numerical equivalence check of
the proposed per-axis potential against the old one. It showed the
opposite of the claim: with `target=(0.06, 0.5)`, the old shared-`w`
design weighted forward tracking at 1.0, while the per-axis version
weights it 0.6. **The blend was not starving the forward axis.**
Hypothesis #2 retracted before it cost a training run.

Checked the model file (`my_robot.xml`) next: motors are `gear="1"`,
`ctrlrange="-4.0 4.0"` on 0.033 m radius wheels → ~121 N of force
available on a ~1 kg robot. Torque authority is not the constraint;
driving is physically easy for this model. So it is a *learning* problem.

**Then inspected the trained policy directly — conclusive:**

```
log_std: [-4.57, -4.66]  ->  action std: [0.0104, 0.0094]
ent_coef used in training: 0.0
```

Action std had collapsed to **0.01 on a ±4.0 action space** (0.25% of
range) — the policy is effectively deterministic with no exploration
capacity remaining.

Probing command sensitivity on a fixed state:

| Command swing | Resulting action change |
|---|---|
| forward −0.3 → +0.3 | ~0.009 (indistinguishable from noise) |
| turn −0.5 → +0.5 | ~0.06, **opposite sign per wheel** (differential-drive turn signature) |

The policy learned to read the turn input and to **completely ignore the
forward input**. With action std at 0.01 it cannot explore its way out —
any gradient toward attempting a lean is swamped. It converged early to
"stand still, respond to turn only," then froze.

**Root cause: entropy collapse with `ent_coef=0.0` (SB3's default),
combined with forward motion being the harder skill to discover.**

## 9. Fixes applied (pending retrain)

1. **`ent_coef=0.01`** on the PPO constructor — the critical fix. Keeps
   the action distribution from collapsing to deterministic before the
   hard translational skill is discovered.
2. **Isolated forward-only curriculum phase.** New 4-stage schedule:
   0-15% stand → 15-35% **forward/backward only (turn locked to 0)** →
   35-50% ramp turn in → 50-100% ramp payload. Removes the easier
   "distractor" skill (turning) during the window when forward motion
   must be discovered. Verified schedule output directly.
3. **Per-axis potential blend** — each axis now blends its own
   hold-still vs track-target weight by its own command magnitude, rather
   than one shared scalar governing both. Verified mathematically
   equivalent to the old design per-axis in isolation (w=0 → old
   "don't drift" term, w=1 → old "tracking" term). Kept as a principled
   cleanup, **not** claimed as the fix.

## 8. STM32 deployment sizing (context for `net_arch` choice)

Discussed deploying just the actor network (not the value net) to an
STM32 (256KB flash / 48KB RAM). At `net_arch=[256,256]` the actor alone is
~277KB float32 — doesn't fit. Dropped to `net_arch=[64,64]` (~22KB
float32, ~5,442 params) — plenty of capacity for a 17-obs/2-action task,
comfortably fits without needing int8 quantization. This is now the
standing architecture for training runs from this session onward.

## Open items going into next session

The one remaining problem: **motion is transient, not sustained** — the
robot reaches 96% of commanded speed then rocks back instead of cruising.
Wheel angles oscillate in [-1.5, +2.1] rather than accumulating.

Candidate directions, roughly in order of expected value:
- [ ] **Penalize oscillation directly.** Nothing in the current reward
      distinguishes "rocking around zero" from "standing still" very
      strongly (instantaneous |error| averages ~0.28 while rocking vs 0.30
      standing). A term on velocity *oscillation* (or on `d(v_fwd)/dt`)
      would separate them properly.
- [ ] **Check whether a stable cruise equilibrium is reachable at all** —
      hand-write a simple PD/LQR balance-and-drive controller for this
      exact model and see if *it* can hold a steady 0.3 m/s. If a scripted
      controller can't either, the problem is the plant/episode setup, not
      the policy. This is cheap and would settle the question definitively.
- [ ] Investigate the discount horizon: `gamma=0.99` at dt=0.025s gives an
      effective horizon of ~2.5s, which may be short relative to how long
      a cruise must be held to pay off.
- [ ] Once driving sustains, test live via `enjoy_drive.py` with a human at
      the keyboard (everything so far was verified via direct programmatic
      control, not the actual GUI/keyboard path)

**Do not** retry: action-space normalization (§11, regressed), or adding
more velocity shaping — PBRS telescopes to `-Phi(s_0)` and so is
mathematically incapable of changing which policy is optimal; only the
base-reward weights can do that.

## 10. Observation frame mismatch — the real driving bug

**Found by testing, after the §7 hypothesis was retracted.** MuJoCo reports
a freejoint's linear velocity in the **world** frame, but both the velocity
commands and the reward are **body** frame (`_decode_state` rotates via
`rot.inv().apply(...)`). So the observation fed the policy world-frame
velocity while grading it on body-frame velocity.

Demonstrated on the trained policy — identical physical motion (0.3 m/s
along the robot's own forward axis), varying only heading:

| Yaw | Motor command | Meaning |
|---|---|---|
| 0° | `[+0.21, +0.20]` | drive forward (correct) |
| 90° | `[-0.04, -0.03]` | ≈ nothing |
| **180°** | `[-0.25, -0.24]` | **drive backward (opposite)** |
| 270° | `[-0.10, -0.10]` | backward |

The policy was being trained against a self-contradictory target: the
"correct" action for a forward command was effectively randomized by
heading. Converging to "output ~0, don't move" was the only self-consistent
solution. Yaw rate (`qvel[5]`) is frame-invariant — verified identical at
0°/90°/180° — which is exactly why turning trained and driving did not.

**Fix:** `_get_conditioned_obs` now replaces the world-frame linear velocity
(obs slots 7-9) with body-frame `local_vel`. Verified yaw-invariant:
`[0.3, 0, 0]` at every heading. Observation stays 17-dim.

**Result after retrain:** forward peak 0.081 → **0.288** (96% of the 0.3
target), and `max|pitch|` now varies with command (0.130 forward vs 0.024
backward) where previously it was frozen at ~0.025 regardless — i.e. the
robot finally leans to accelerate. Eval also stabilized markedly:
1862 ± 28 reward at 1000 ± 0 episode length, versus earlier runs that
oscillated between 1000±0 and 813±373.

**Remaining issue:** motion is still not *sustained*. Wheel angles oscillate
in [-1.5, +2.1] rather than accumulating — the robot rocks forward and back
rather than driving. Peaks reach target, means stay ~0.

## 11. Action-space normalization — tried, REGRESSED, reverted

**Hypothesis:** the ±4.0 Nm action space was ~40x oversized (measured p99
torque actually used = 0.105 Nm, i.e. 2.6% of range) while SB3 explores with
initial std 1.0, forcing entropy collapse to std≈0.015 in every run
regardless of `ent_coef` (0.0→0.010, 0.01→0.0155).

**Change:** action space normalized to ±1.0 with `TORQUE_SCALE = 0.3` Nm.
Effort-penalty coefficient carried `TORQUE_SCALE**2` so the reward stayed
numerically identical in Nm terms (verified: 0.002205 both ways), making the
change purely about action scaling.

**Result — worse on every axis that matters:**

| Checkpoint | action_std | Eval reward | Episode length |
|---|---|---|---|
| PRE-FRAMEFIX | 0.0128 | — | — |
| **FRAMEFIX-ONLY** | 0.0155 | **1862 ± 28** | **1000 ± 0** |
| ACTION-SCALED | 0.0575 | 1188 ± 615 | 704 ± 352 |

The mechanism worked as predicted (std rose 3.7x, entropy collapse relieved)
but the policy became violently unstable: overshooting targets 3x (turn peaks
of −1.52 rad/s against a 0.5 target), pitch excursions to 0.404 rad, and an
actual fall during testing.

**Why the reasoning was wrong:** absolute action scale is not what matters —
*relative* noise is. Before: std 0.0155 vs mean output 0.105 ≈ 15%. After:
std 0.0575 vs mean ~0.35 ≈ 16%. Essentially unchanged. Normalizing rescaled
both together, while the sustained entropy bonus added enough persistent
noise during training to degrade the controller.

**Reverted.** Action space back to ±4.0 raw Nm, `TORQUE_SCALE` removed,
effort coefficient back to 0.1. `FRAMEFIX-ONLY` restored as the active
`models/best/best_model.zip`. Checkpoints kept for comparison:
`best_model_PRE_FRAMEFIX.zip`, `best_model_FRAMEFIX_ONLY.zip`,
`best_model_ACTIONSCALE_worse.zip`.

## 12. Teammate's PID baseline (DIP-E026, `Non-harness-JiangLi`)

Reviewed `github.com/aaaakkkk111/DIP-E026` — a bit-level digital twin of the
real STM32 firmware, deliberately reproducing even the firmware's bugs.
Three findings:

**a) Their velocity loop has integral action; the forward command feeds the
integrator directly**, rather than being a setpoint compared to measurement:
```python
st.encoder_integral += st.encoder_bias
st.encoder_integral += cmd.pid_move
velocity = -st.encoder_bias*velocity_kp - st.encoder_integral*velocity_ki
```
This is what kills steady-state error and holds a cruise.

**b) Their turn loop is open loop** — `turn = cmd.pid_turn*turn_kp +
gyro_yaw*kd`, no yaw-rate error feedback (their notes document the LQR's yaw
feedback being dead from a unit bug, so steering runs purely on the command
term). **Independent confirmation that turning is structurally the easy
channel on this robot** — the turn-works/drive-doesn't split is inherent,
not an artifact of our setup.

**c) Their physical model is far more realistic than `my_robot.xml`:**

| Parameter | Theirs | Ours |
|---|---|---|
| Body mass | 1.50 kg | ~0.91 kg |
| CoM height above axle | 0.12 m | **~0.035 m** |
| Wheel radius | 0.050 m | 0.033 m |
| Max torque/wheel | 0.60 N·m | **4.0 N·m** |
| Motor free speed | 60 rad/s (back-EMF) | **unmodelled** |
| Rolling resistance | `rolling_mu=0.02` | **unmodelled** |

Our pendulum is ~3.4x shorter (much twitchier) and our motors are effectively
infinite-power. Also of interest for the STM32 port: they already have
`nn_q15.py` (Q15 quantized NN) and `stm32_policy.py`.

## 13. Velocity-error integral in the observation — tried, REGRESSED, reverted

Following §12a, appended a clamped integral of forward-velocity error to the
observation (17 -> 18 dims), on the reasoning that a memoryless MLP cannot
do integral action the way the PI loop does. Verified mechanically correct
(accumulates 0.0075/step = 0.3 x 0.025, clamps at +/-2.0, resets per episode).

**Result — ~5x worse at equal training progress:**

| Steps | FRAMEFIX-ONLY | +INTEGRAL |
|---|---|---|
| 1.90M | 61 | 45 |
| 2.28M | **211** | 95 |
| 2.47M | **531** | 83 |
| 2.66M | **451** | 68 |
| 2.85M | **598** | 116 |

Baseline was climbing steeply; the integral run was flat-to-declining. Run
killed at 2.85M rather than burning the remaining 20 minutes.

**Feature scaling was NOT the cause** (the obvious suspect, and it was
checked): measured p95 of the integral was 0.077 — one of the *smallest*
inputs, versus wheel velocities at p95 ~25.

**Most likely reason:** episodes terminate long before the integral can
accumulate anything meaningful — max observed magnitude was 0.108 against a
+/-2.0 clamp. In the PID the integrator works because the hand-tuned balance
loop always keeps the robot up; in RL the robot is falling constantly while
balance is still being learned, so the integral is uninformative exactly when
it would need to be useful, and contributes noise instead. A feature that
only becomes meaningful *after* balance is solved is actively harmful during
the phase when balance is being learned.

Reverted (also required — the active 17-dim checkpoint would not otherwise
load). Checkpoint kept as `best_model_INTEGRAL_worse.zip`.

## Current best configuration

Frame fix + per-axis potential + 4-stage curriculum + `ent_coef=0.01` +
±4.0 action space. Eval 1862 ± 28 at 1000 ± 0 episode length. Drives to
96% of commanded speed transiently; does not yet sustain a cruise.

## Lesson learned this session

Two diagnoses were made from reasoning about the reward math alone. The
first (reward weight imbalance, §6) was correct. The second (PBRS blend
starving the forward axis, §7) was **wrong**, and would have cost an hour
of training to disprove. What actually settled it was *measuring the
policy directly* — dumping `log_std` and probing action output against
each command axis took one command and gave an unambiguous answer.
Inspect the trained artifact before theorizing about the objective.
