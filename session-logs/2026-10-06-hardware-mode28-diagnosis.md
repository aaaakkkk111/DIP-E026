# 2026-10-06 — Why run 7 fails on the car (mode 28, `rl_mode28_20261003/`)

Input: the team's mode-28 integration (run 7 inside the stock Yahboom Keil
project), its README, flight-recorder logs (`rl_serial_log.txt`, 11 runs) and
sources. Every on-car run fell within 0.2–2 s, or "ran" for up to 17 s while
being held. Goal: separate integration bugs from genuine sim-to-real gaps.

## 1. The frozen output is a saturated network — reproduced bit-exactly

In every run of the correctly-timed session (v5: 200 Hz, 1 KB stack) the PWM
locks at **L −1923 / R −1466** for the rest of the run — 100+ ticks, and ~15 s
in one run — while pitch swings −7° … +40° and pitch rate reaches 150 °/s. A
responsive network cannot do that.

Run at 17:00:07, ticks 0–12: normal output (L ≈ 1540, R ≈ 1485), then at
tick 11 `v = 24320` (logged as `short(v·1000)`, so a wrapped, far larger value),
and from tick 12 on: −1923/−1466, permanently.

Chain:

1. Stack overflow (README §3.3) overwrites odom.c's velocity ring buffer →
   wheel speed (and `v_forward`) astronomically large for the 4-tick window.
2. `v_forward` is integrated into the station-keeping leaky integral
   (`g_pos_err`, τ = 2 s), which has **no clamp** → thousands of metres.
3. `pos_err·10` hits the fixed-point input clamp (±32767) → first layer
   saturates → constant action.
4. τ = 2 s means it takes ~20 s to decay back in range → frozen output.

Proof: a bit-exact numpy replica of `policy_q.c` with `obs[32] = −32767` gives
**(−0.41557, −0.11099) → PWM (−1923, −1466)**, the exact car values. (Float
probes: a huge `v_forward` or wheel angle gives similar but not identical
values, ~(−1950, −1439) / (−1873, −1471).)

The v6 hex (4 KB stack) should remove the corruption source, but the
mechanism remains: any single bad sample poisons the policy for seconds.

## 2. Ruled out

| suspect | check | result |
|---|---|---|
| int16 fixed-point port | numpy replica of `policy_q.c` vs PyTorch, 4,000 sim-rollout observations | max action error 0.009, mean 0.0015 — faithful |
| axis/sign mapping | read `rl_mode.c` `to_body`, `att_from_acc`, `att_update` against sim conventions (scipy `as_euler('xyz')`, body x fwd/y left/z up) | correct; logged pitch rate matches d(pitch)/dt |
| `v_forward` formula | odom.c `r·(ω̄_enc + θ̇)·cosθ` vs sim chassis body-frame velocity | correct |
| obs builder | `policy.c` diffed against repo | identical except `NOMINAL_HEIGHT` 0.05 → 0.0334 (their fix of my bug, correct) |

Sim probe (`scratchpad/sim2real_probe.py`), hold, 3 seeds × 10 s, each effect
added to the exact training plant:

| effect | survived | sign-flip rate |
|---|---|---|
| none (as trained) | 3/3 | 1.00 |
| 1 tick action latency | 3/3 | 0.33 |
| 2 ticks latency | 3/3 | 0.20 |
| encoder quantisation (1320 cpr) + 4-tick window | 3/3 | 0.77 |
| pitch zero +1° / +2° | 3/3 | 0.96 / 0.91 |
| gyro-z bias 0.005 rad/s | 3/3 | 1.00 |
| latency 1 + encoder + pitch0 1° | 3/3 | 0.21 |

Run 7 tolerates all of these. They are not the cause.

## 3. The real sim-to-real gap: the motor model

`motor_model.py` maps PWM linearly through zero (`tau = TAU_STALL·duty − KV·ω`)
with Coulomb friction 0.0025 N·m, so the firmware's 1300-count dead-band
compensation is modelled as **useful torque** (0.257 N·m — the "no gentle
nudge, 64 % of the driver limit" premise behind runs 3–7). Physically,
`PWM_Ignore` exists because the gearmotor does not turn below ~1300–1480
counts: the dead band *is* friction (the parameter sheet's asymmetric
1480 fwd / 1455 rev is a friction signature).

Evidence from the car:

- **Free-spin run** (16:36:42, car held in the air): PWM L ≈ 1985, R ≈ 1705.
  Measured mean wheel speed 19.3 rad/s reported → **9.5 rad/s** true (README:
  the half-rate session inflated every rate by 2.03×).
  - `motor_model.py` predicts **22.3 rad/s** (2.3× too fast)
  - dead zone at 1480 counts (param sheet) predicts **9.1 rad/s** (within 5 %)
- **Starts of two correctly-timed runs**: PWM ≈ 1470/1440 for 65 ms and
  1540/1485 for 55 ms, encoders ~0. Sim predicts 0.29 N·m/wheel → ~0.23 m/s
  after 65 ms.

Sim with a dead-zone motor (`scratchpad/deadzone_probe.py`; same full-scale
torque and back-EMF; static friction holds the wheel below D):

| motor | firmware comp. | hold | drive +0.15 |
|---|---|---|---|
| training model | 1300 | 2000/2000/2000 | 2000/2000/2000 |
| dead zone 1480 | 1300 | **417 / 141 / 236** | 178 / 293 / 133 |
| dead zone 1480 | 1480 | 2000/2000/2000 | 378 / 333 / 392 |

With the parameter sheet's own dead zone, run 7 falls in 0.7–2.1 s holding
still — the same 0.2–2 s run lengths the team saw. Matching the firmware's
compensation to the real dead zone rescues hold-station only; driving still
falls in ~2 s. The policy's control law (100 Hz ±0.257 N·m dither, sign flip
on every tick at hold) is tuned to an actuator that does not exist.

## 4. Other design weaknesses found

- **Policy depends on absolute wheel angle** (obs 5, 6) — physically
  meaningless. The team's SIL found 6/10 long drives fall with a pure
  accumulator and added a 5 s leak not present in training.
- **No input sanitisation, `pos_err` unclamped** — see §1.
- **Training randomised only armature and payload** — no motor dead zone,
  stall-torque (battery) spread, latency, sensor noise, IMU offsets.

## Open items

1. Bench-measure the motors (wheels in the air, bypass `PWM_Ignore`): sweep
   raw PWM 0…2800 both directions, steady wheel speed per wheel → true dead
   zone and slope; step response → rotor inertia (armature).
2. Replace `motor_model.py` with a dead-zone/friction model fitted to (1);
   randomise dead zone, stall torque (battery 11.1–12.6 V), 0–2 ticks latency,
   encoder quantisation, gyro bias, pitch offset ±2°.
3. Drop wheel angles from the observation; clamp `pos_err` (±0.5 m) in both
   training and firmware.
4. Retrain (run 8), re-export, re-quantise.
5. Firmware hardening now, independent of retraining: clamp `v_forward`
   (±2 m/s) and `pos_err`, reject NaN/inf observations, watchdog/overrun
   disarm.
6. Interim on-car test, if wanted: v6 hex with `POLICY_PWM_DEADBAND` raised to
   the measured dead zone — sim says hold-station may work, driving will not.

---

# Part 2 — fix and retrain (run 8)

## What changed

**`motor_model.py` — true dead zone.** Torque is now zero below a per-wheel,
per-direction dead zone `D` and rises linearly to the measured stall torque at
full duty; no regenerative braking (fast-decay drive coasts); `pwm = 0` coasts.
Nominal `D = 1460` reproduces the car's free-spin run (9.35 vs 9.5 rad/s
measured; old model 22.3). Randomised per training episode: `D` 1350–1600
(each wheel × direction), stall torque ×0.85–1.05 (battery), back-EMF
×0.9–1.1, friction 0.002–0.02 N·m. The firmware's 1300-count compensation is
unchanged — the policy now learns the residual dead zone.

**`train_real_robot.py` — train on what the firmware observes.** The
observation is rebuilt every tick by `PWMCommandWrapper._sense()` /
`_get_conditioned_obs()`, a mirror of `firmware/policy.c finish_obs()`:

| input | run 7 (ground truth) | run 8 (emulated sensor) |
|---|---|---|
| attitude | full quaternion incl. yaw | roll/pitch only, yaw pinned 0, ±2° random zero offset, 0.002 rad noise |
| gyro | exact | ±0.003 rad/s random bias per axis, 0.01 rad/s noise |
| wheel speed | exact joint velocity | 1320-count encoders differenced over 4 ticks, clip ±40 rad/s |
| v_forward | exact chassis velocity | `r·(ω̄ + θ̇)·cosθ` from those encoders, clip ±2 m/s |
| wheel angles | exact, unbounded | dropped (always 0) |
| height, lateral/vertical v | exact | constants 0.0334, 0, 0 |
| pos integral | unclamped | clipped ±0.5 m |
| action | applied immediately | applied 1–2 ticks late (eval: 1) |

The reward stays on ground truth: the true position and heading integrals are
tracked separately from the sensed ones the policy sees, so a gyro bias
drifting the sensed heading integral is not rewarded as a real turn.

**Firmware (`firmware/policy.c`, `policy.h`)** — new `policy_odom_update()`
(4-tick encoder window + pitch-rate correction, mirrors training), wheel-angle
arguments removed from `policy_build_obs*()`, input clamps with NaN→0, position
integral clip, `POLICY_NOMINAL_HEIGHT` 0.05 → 0.0334. Deployment guide updated
(Step 2 check, Step 4/8 code, and Step 6 now says keep the 1300 compensation —
the old advice to re-tune it would break run 8).

**New tools**
- `firmware/check_obs_builder.py` + `test_obs_builder.c` (DLL): rolls the
  training wrapper, feeds the same raw signals through the C pipeline, compares
  all 34 inputs. **Worst disagreement 8.9e-7, nominal and noisy cars — PASS.**
- `firmware/quantize_weights.py`: the team's int16 format, reconstructed.
  Reproduces their run-7 `policy_weights_q.h` exactly (all 6,528 weights; needed
  float32 arithmetic — float64 rounds 2 weights the other way).

## Validation before the long run

- Motor model vs car free-spin: 9.35 vs 9.5 rad/s.
- Observation formulas vs scipy: 3e-16.
- Encoder `v_forward` vs true velocity: corr 0.999, rms 5.7 mm/s.
- Run 7 on the corrected plant: falls at 245–588 ticks (consistent with the car).
- 800k-step smoke run: training episode length 90 → 1899/2000. Learnable.

## Run 8

30M steps, same hyperparameters/curriculum as run 7, `logs/run8/`,
`session-logs/train_real_robot_RUN8_deadzone.log`. Run 7 backed up to
`models/best_real_RUN7_yaw/` and `models/ppo_real_robot_RUN7_30M.zip`.
Finished 2026-10-07 01:20 in 2:33:17 (machine sped up overnight; 6.5 h was
the estimate at the start).

### Training curve

| block | eval mean | eval ep length | | metric | start | 25 % | 50 % | 75 % | end |
|---|---|---|---|---|---|---|---|---|---|
| 0–5M | 991 | 989 | | `train/std` | 0.252 | 0.085 | 0.126 | 0.151 | 0.152 |
| 5–10M | 2468 | 1595 | | `explained_variance` | 0.00 | 0.957 | 0.934 | 0.905 | 0.881 |
| 10–15M | 3118 | 1882 | | `approx_kl` | 0.0026 | 0.0061 | 0.0052 | 0.0035 | 0.0001 |
| 15–20M | 3408 | 2000 | | | | | | | |
| 20–25M | 3503 | 2000 | | | | | | | |
| 25–30M | 3594 | 2000 | | | | | | | |

Best eval **3637.2 at 29.64M**; full-length episodes from 15M on. Slope
+18/1M over the final third — flattening, LR schedule near zero. `std` ends at
0.15 vs run 7's 0.057: the dead zone and the noisy observations keep more
action noise worthwhile. Not comparable to run 7's 3400 (different plant and
observation).

### Behaviour (corrected plant, 3 seeds × 10 s, steady state after 2 s)

Run 7 on the same plant: hold falls at 588 / 142 / 260 ticks, +0.15 m/s at
171 / 214 / 134.

Run 8, nominal car (dead zone 1460, 1-tick delay) — all 18 held 2000/2000:

| command | v_fwd | v_turn | PWM sign-flip rate |
|---|---|---|---|
| hold | −0.001 | +0.000 | 0.08 |
| +0.15 / +0.30 m/s | +0.136 (91 %) / +0.267 (89 %) | 0.000 | 0.01 / 0.02 |
| −0.15 m/s | −0.137 (91 %) | 0.000 | 0.00 |
| turn +0.50 rad/s | −0.002 | +0.500 (100 %) | 0.00 |
| +0.15 & +0.30 | +0.136 | +0.300 | 0.27 |

Run 8, stressed car, hold and +0.15 — **all 60 held 2000/2000**: dead zone 1350
and 1600, 2-tick delay, motor ×0.85, pitch zero ±2°, gyro bias 0.003 rad/s on
all axes, sensor noise, 1.0 kg payload, and everything worst at once (dead
zone 1600 + 2-tick delay + ×0.85 + +2° + noise: hold −0.020 m/s, +0.15 →
+0.126). Pitch zero ±2° costs a ∓2 cm/s creep at hold.

Run 7 flipped the PWM sign on ~100 % of ticks at hold; run 8 on 0–27 %. A
dead-zone motor gives nothing for dithering below it, and the H-bridge no
longer reverses at 100 Hz.

### Firmware side

- Re-exported `firmware/policy_weights.h` and the int16
  `firmware/policy_weights_q.h`.
- `test_policy`: the exact-tanh build matches PyTorch to 1.2e-7. The fast-tanh
  build failed its old 2e-2 tolerance at 2.2e-2: run 8's network is more
  sensitive to the approximation than run 7's. Measured: worst 0.031 on random
  ±1 inputs, 0.020 (mean 0.003) on 5,000 realistic run-8 observations.
  Tolerance re-based to 5e-2 with that record in `test_policy.c`; the
  closed-loop check below is what justifies the approximation.
- **Closed-loop SIL**: the actual C code (`policy_odom_update` +
  `policy_build_obs_rp` + `policy_infer` fast-tanh, and separately the team's
  `policy_q.c` with the new int16 header) compiled into a DLL and driving the
  simulated robot. Nominal and worst-case car, hold and +0.15: **all 24
  held**, tracking identical to PyTorch (+0.136 nominal, +0.126 worst).
- `policy.c` gained the team's `POLICY_NO_FLOAT_INFER` switch (and
  `<stddef.h>`) so it is a drop-in for their 64 KB-flash build.
- `rl_mode28_20261003/run8_dropin/`: `policy.c` (switch set), `policy.h`,
  both weight headers, `rl_mode.c` with the one changed call, and a README.
  Every `POLICY_*`/`policy_*` name `rl_mode.c` uses resolves; compiles clean
  with `policy_q.c`.
- Smart App Control intermittently blocks freshly built `.exe`/`.dll` files.
  The rebuilt `test_obs_builder.dll` was blocked after the last `policy.c`
  edit. By diff, that edit only added an include, the preprocessor switch and
  a comment; the observation code is byte-identical to the closed-loop-tested
  copy.

## Open items

1. **On-car test of run 8** — `run8_dropin/README.md` has the steps.
2. Bench sweep of the motors (raw PWM 0–2800 per wheel and direction, wheels
   in the air) to replace the one-log dead-zone fit. Widen
   `MOTOR_DEAD_ZONE_RANGE` and retrain if it falls outside 1350–1600.
3. Watchdog / overrun disarm in the mode-28 firmware.
4. Gearbox backlash is still unmodelled.
5. Slope and friction sweeps for run 8.
6. Nothing committed or pushed yet.
