# 2026-10-08 — Motor step test, motor-model fit, wobble with the fitted motor

Follows `2026-10-08-run9-run10-delay-and-gear-slack.md` §8 (run 11). Done on
the laptop; nothing in `motor_model.py` or `train_real_robot.py` was changed.
The fitted numbers are for the desktop to apply (§6).

## Summary

- **The motors were measured.** 52 raw-PWM steps (each wheel, both
  directions, PWM 1300–2800) were recorded at 200 Hz with the car on a box.
  All steps completed, and both wheels and both directions behave almost
  identically.
- **The motor differs from the model in three ways:**
  - the dead zone is about **1395**, not 1460;
  - the time constant is **92–97 ms**, not 140 ms (the rotor inertia is
    lower);
  - friction is about **7× the nominal Coulomb value**: the wheels coast to a
    stop in under half a second.

  Top speed is 35.7 rad/s at PWM 2800, against 32.8 in the model.
- **Free spin fixes only ratios** (torque : back-EMF : friction :
  inertia), not their absolute size. That ambiguity turned out not to
  matter for the wobble.
- **With the fitted motor, the simulator reproduces run 8 on the car**
  (95 deg/s, mean |PWM| ≈ 2240 against 2170). It also roughly doubles the
  simulated wobble of runs 9 and 11. **They are still under-predicted**:
  - run 9: at most 45 against 68 deg/s;
  - run 11: at most 33 against 57.5 deg/s, at 6–10 Hz against 4.2;
  - their mean |PWM| is 230–360 below the car's.
- **The prime suspect is the motor near its dead zone.** From PWM 1440 to
  1700, the car's wheel spins up 1.5–3× more slowly than any fitted model.
  Runs 9 and 11 spend 41–44 % of their ticks below 1700, against 26 % for
  run 8, the one the simulator now reproduces.

## 1. Firmware: `car_firmware_motortest`

`C:\Users\USER\Desktop\car_firmware_motortest` is a copy of
`car_firmware_run11` (run 11 weights, which stay intact). Only
`APP\RL\rl_mode.c` and `rl_mode.h` changed. The full change is
`firmware/motor_step_test_rl_mode.patch`, against `car_firmware_run11`; apply it
with `patch -p1 --binary` (or `-l`), because `rl_mode.h` has CRLF line
endings.

**New serial commands** (parsed in `TUNE_Poll`):
- `mt on` / `mt off`: start or end a session. While a session is on,
  auto-arm is blocked, so the policy cannot start while the car is held
  upright. Both commands reply with the car's own battery reading.
- `mt L <pwm>` / `mt R <pwm>`: one step, accepted only in a session, in
  RL_WAIT, with the motors enabled. Otherwise the car prints the reason.

**One step** runs in `RL_Tick` on the RL_WAIT path:
- it puts the raw signed PWM on one wheel and 0 on the other, for 200
  ticks, then 0 on both for 100 ticks;
- it records the `enc_l` / `enc_r` deltas into two int16 buffers of 300;
- the policy is bypassed and the stock ±2800 clamp still applies;
- if `Turn_Off()` cuts the motors, the step aborts and prints `M end cut`.

**Printing** happens only from `TUNE_Poll`: `M <side> <pwm> <k> <enc_l>
<enc_r>` for each tick, then `M end`. The `cal done` printf also moved out
of the ISR into `TUNE_Poll`. It now prints sane values (`pitch zero -0.69
roll zero -6.77 …`), so the garbage seen with run 11 came from printing
floats inside the ISR.

**What stays the same:** the RUN path of `RL_Tick` is unchanged; the new
code sits after it, on the WAIT path.

**Build:** Keil from the command line (`UV4 -b … -j0`): 0 errors and the
usual 4 warnings. The image ends at 0x0800FD10, 752 bytes under the 64 KB
this board loads reliably (run 11: 1416 bytes under).

**Flashing** went wrong first. The motor-test and run 11 hex files have the
same name, and FlyMCU reloaded the run 11 one, so the car answered `?` to
`mt on`. Choose the file explicitly. `motor_step_test.py` now stops with
that message when the car answers `?`.

## 2. The test (`sim_tools/motor_step_test.py`)

The script opens COM3 at 230400 with DTR and RTS held low, as
`rl_trim_helper.py` does, and repeats `mt on` until the car answers. It
then sends a 0-PWM probe step until the motors are enabled (KEY1), waits
3 s, and runs 52 steps:
- order: L forward, L reverse, R forward, R reverse;
- PWM 1300, 1350, 1400, 1450, 1500, 1550, 1600, 1700, 1800, 2000, 2200,
  2400, 2800;
- 1 s pause between steps.

Steps that are cut or refused are repeated. Here none were: no `M end cut`.

- Setup: mode 28, calibrated, KEY1, chassis upright on a box, both wheels
  in the air.
- Battery: 12.6 V on the multimeter before. The car's own reading was
  12.49 V before and 12.48 V after (no multimeter reading after).
- Output: `car_logs/2026-10-08/motor_steps.csv` (step, side, pwm, k, enc_l,
  enc_r: 52 × 300 rows) and `motor_steps_raw.txt` (everything the car sent).
- Tick convention: the PWM is written at the end of the ISR at ticks 0–199.
  `enc` at tick k counts the 5 ms that end at tick k, so the PWM acts on
  ticks 1–200.
- The undriven wheel never moved more than 1 count per tick, so there is
  no coupling worth modelling.

## 3. Measured (`sim_tools/motor_fit.py`, part 1)

Wheel speed in rad/s: 1320 counts per wheel revolution, mean over the last
0.4 s of the step.

| PWM | 1450 | 1500 | 1550 | 1600 | 1700 | 1800 | 2000 | 2200 | 2400 | 2800 |
|---|---|---|---|---|---|---|---|---|---|---|
| L fwd | 0 | 1.9 | 3.2 | 4.3 | 6.8 | 9.4 | 14.7 | 20.1 | 25.5 | 35.7 |
| L rev | 0 | 2.0 | 3.1 | 4.4 | 6.7 | 9.2 | 14.3 | 19.6 | 25.0 | 35.3 |
| R fwd | 0 | 2.5 | 3.8 | 4.9 | 7.3 | 9.8 | 15.0 | 20.6 | 25.9 | 35.9 |
| R rev | 0 | 2.5 | 3.8 | 5.0 | 7.5 | 10.2 | 15.6 | 21.1 | 26.7 | 36.1 |
| model (nominal) | 0 | 0.8 | 2.1 | 3.3 | 5.7 | 8.2 | 13.1 | 18.0 | 23.0 | 32.8 |

**Dead zone.** Every wheel and direction is still at 1450 and turns at
1500. Above that, speed is linear in PWM at 0.026 rad/s per count, and the
line extrapolates to zero at about 1425–1440.

**Times, ms after the PWM change**, as ranges over the four wheel and
direction combinations:

| PWM | 1500 | 1550 | 1600 | 1700 | 1800 | 2000 | 2400 | 2800 |
|---|---|---|---|---|---|---|---|---|
| first count | 15–45 | 10–30 | 10–20 | 10–15 | 10–15 | 5–15 | 5–10 | 5–10 |
| time to 63 % | 145–215 | 195–220 | 135–155 | 120–145 | 115–125 | 95–110 | 85–100 | 90–95 |
| coast to 37 % | 135–185 | 145–235 | 220–300 | 280–360 | 370–455 | 480 / – | – | – |
| coast to stop | 175–230 | 255–350 | 345–440 | – | – | – | – | – |

A dash means not reached within the 0.5 s recorded. The nominal model needs
**140–150 ms** to 63 % at every PWM. It also coasts for seconds, because
its only friction when the PWM is off is 0.0025 N·m of Coulomb friction.

## 4. Fit (`motor_fit.py`, part 2)

**Method.**
- Per wheel, the fit adjusts dead_zone (forward and reverse), tau_scale,
  kv_scale, coulomb and the rotor armature, through
  `motor_model.pwm_to_torque()`.
- The rotor and wheel are simulated as one inertia (armature plus the
  wheel's 1.96e-5). Torque is held over each 5 ms tick, from the speed at
  its start, as `PWMCommandWrapper.step` does.
- The residual is the 4-tick speed, weighted 1/(steady counts + 3), so the
  slow steps count as much as the fast ones.
- The simulated counts are not rounded, so the fit is smooth.
- A first version rounded them like the encoder, so the optimiser never
  moved. That bug was fixed before these results.

**The scale cannot be measured in free spin.** Scaling tau_scale, kv_scale,
coulomb and armature together gives the same wheel speeds. The fit cost
only moves from 15.6 to 16.0 between kv_scale 0.7 and 1.3. The only tie to
an absolute size is the 0.4 N·m driver limit, active in the first ticks of
the large steps. What is measured are the ratios:
- tau_scale/kv_scale: 1.07 (L), 1.11 (R);
- armature/kv_scale: 1.47e-3, 1.54e-3 kg·m²;
- coulomb/kv_scale: 0.0196, 0.0184 N·m;
- time constant (armature + wheel)/(KV·kv): 92 ms, 97 ms.

**Fitted values against the training ranges.**
- **A** holds kv_scale at 1, keeping KV as `motor_model.py` has it (stall
  torque / no-load speed). This is the set to use.
- **B** is the cost minimum, a weaker and lighter motor along the flat
  valley.

| | A: L | A: R | B: L | B: R | training value (range) |
|---|---|---|---|---|---|
| dead_zone fwd / rev | 1397 / 1398 | 1395 / 1394 | 1397 / 1397 | 1394 / 1393 | 1460 (1350–1600) |
| tau_scale | 1.070 | 1.111 | 0.761 | 0.744 | 1 (0.85–1.05) |
| kv_scale | 1 | 1 | 0.711 | 0.668 | 1 (0.9–1.1) |
| coulomb, N·m | 0.0192 | 0.0180 | 0.0140 | 0.0123 | 0.0025 (0.002–0.02) |
| armature, kg·m² | 1.45e-3 | 1.51e-3 | 1.04e-3 | 1.03e-3 | 2.25e-3 (9e-4 – 4.5e-3) |
| fit cost (nominal: 66 / 73) | 15.6 | 21.8 | 15.6 | 21.7 | |

**Extended model, not in `motor_model.py`.** Adding a viscous friction
term, −b·ω always active, halves the cost (7.9 and 12.5) with b = 0.0011
N·m·s/rad. The other parameters then shift: dead zone 1420, coulomb
0.005–0.007, tau_scale/kv_scale about 1.3. It did not help the wobble (§5).

**Check in `real_robot.xml`.** The chassis was held 0.3 m up, the free joint
reset every 1 ms, with 3° of slack and encoder counts rounded as on the
car. Steady rad/s and time to 63 %:

| | car | fitted A | nominal |
|---|---|---|---|
| L +1500 | 1.9, 150 ms | 1.4, 45 ms | 0.8, 140 ms |
| L +1600 | 4.3, 155 ms | 3.9, 95 ms | 3.3, 145 ms |
| L +1800 | 9.4, 120 ms | 9.0, 85 ms | 8.2, 130 ms |
| L +2400 | 25.5, 85 ms | 24.2, 95 ms | 23.0, 140 ms |
| L +2800 | 35.7, 90 ms | 34.3, 105 ms | 32.8, 150 ms |
| R +1500 | 2.5, 215 ms | 1.6, 100 ms | 0.8, 140 ms |

Above about 1700 the fitted model matches speed, rise time and coast-down.
**Below 1700 no variant fits.** The car's wheel spins up 1.5–3× more slowly
than the model, yet ends faster (1.9–2.5 against 1.4–1.6 rad/s at 1500).
At 1450 the model creeps at 0.3–0.6 rad/s, while the car's wheel does not
move at all, which points to static friction above the running friction.
A sharp dead zone plus Coulomb friction cannot produce both effects.
Full tables: `session-logs/2026-10-08-motor-fit-output.txt`.

## 5. Wobble with the fitted motor (`motor_fit.py --wobble`)

**Method.** `simlib.run()` standing still, 10 s, statistics after 2 s, 2
seeds. The motor and rotor armature were replaced after `simlib.make_env`'s
reset. Slack 2–4°, delay 5–20 ms. Ranges over those 12 settings
(pitch-rate sd in deg/s; mean |PWM|):

| | car | nominal motor | fitted A | fitted B | fitted + viscous |
|---|---|---|---|---|---|
| run 8 sd | **95.5** | 41–60 | 49–96 | 50–101 | 48–91 |
| run 8 Hz / full PWM / mean \|PWM\| | 6.9 / 41 % / 2170 | 6.0–11.1 / 1–29 % / 1937–2139 | 7.2–12.0 / 1–37 % / 1946–2244 | | |
| run 9 sd | **68.2** | 12–27 | 19–45 | 18–43 | 14–38 |
| run 9 Hz / mean \|PWM\| | 7.2 / 1886 | 5.3–8.9 / 1515–1647 | 7.1–12.8 / 1522–1649 | | |
| run 11 sd | **57.5** | 5–19 | 19–33 | 19–33 | 13–28 |
| run 11 Hz / mean \|PWM\| | 4.2 / 1800 | 4.8–11.3 / 1431–1505 | 5.9–9.8 / 1492–1566 | | |

Closest cells with fitted A:
- run 8, 2°/15 ms: 95.0 deg/s, 8.2 Hz, 24 %, 2244;
- run 8, 3°/10 ms: 96.3 deg/s, 8.4 Hz, 20 %, 2229;
- run 9, 4°/20 ms: 45.1 deg/s, 7.1 Hz, 1649;
- run 11, 4°/20 ms: 33.4 deg/s, 5.9 Hz, 1534.

The harness reproduces the desktop's nominal numbers: run 11 gives 22.2 at
5°/15 ms (desktop 23) and 12.1 at 3°/15 ms (desktop about 14).

**Does the simulator now reproduce the car?**
- **Run 8: yes.** Pitch-rate sd and mean |PWM| match at 2–3° and 10–15 ms.
  The frequency is 1.3–1.5 Hz high, and full PWM is 20–24 % in the matching
  cells against 41 %.
- **Runs 9 and 11: partly.** The fitted motor doubles their simulated
  wobble, but at best it stays about 35 % (run 9) and 40 % (run 11) below
  the car, and mean |PWM| stays 230–360 counts low. Run 11's frequency is
  6–10 Hz in simulation against 4.2 Hz on the car.
- **A and B behave alike**, so the unmeasured absolute scale does not
  change the conclusion.
- **Viscous friction makes it slightly calmer**, so it is not the missing
  piece.

**What is still missing.** The model is still wrong near the dead zone, and
that is where runs 9 and 11 spend their time. On the car (E lines,
`simlib.car_stats` windows), the share of ticks by |PWM|:

| | below 1440 (no torque) | 1440–1700 (car slower than the model) | 1700 or more |
|---|---|---|---|
| run 8 | 9 % | 17 % | 75 % |
| run 9 | 18 % | 23 % | 59 % |
| run 11 | 12 % | 32 % | 56 % |

The fitted model is right above 1700 and too fast between 1440 and 1700.
For the policies that live there, a sluggish motor acts like extra delay
that appears only on small corrections. Three of today's observations fit
that picture:
- the slower limit cycle (4.2 Hz);
- the higher mean |PWM| (the policy has to push harder);
- the only run that already matches is the one that spends least time
  there.

**Not tested by this rig:**
- reversals from a turning wheel (plugging);
- torque under load, which would fix the absolute scale;
- tyre and floor effects.

## 6. For the desktop

**`motor_model.py` (fitted A, battery about 12.5 V, full charge):**
- `MOTOR_DEAD_ZONE` = **1395**, the same for both wheels and directions
  within ±3. The range 1350–1600 contains it but is centred too high:
  suggest about (1350, 1450).
- `tau_scale` = **1.07 (L), 1.11 (R)**, with KV unchanged. This is above
  the training range (0.85–1.05). At 11.1 V it would be about 0.97.
- `kv_scale` = 1, held. Only tau/kv, J/kv and c/kv are measured, above.
- `COULOMB` = **0.018–0.019 N·m**. The nominal 0.0025 is about 7× too low,
  and the measured value sits at the top of today's 0.002–0.02 range.
- `ARMATURE_NOMINAL` (`train_real_robot.py`) = **1.45e-3 (L), 1.51e-3 (R)**,
  inside the range but below the nominal 2.25e-3. The time constant is
  92–97 ms, not 140.

Applying these will not, by itself, make runs 9 and 11 wobble in
simulation as much as on the car (§5).

**A next measurement, if wanted.** Two-level steps for the low-PWM region,
which need a small firmware addition:
- small steps 1500 ↔ 1700 from a turning wheel;
- reversals such as +2000 → −1600.

Then fit a model with a soft, slow response near the dead zone, for
example a first-order lag on torque whose time constant grows toward the
dead zone, plus static friction.

## Files

| file | what |
|---|---|
| `firmware/motor_step_test_rl_mode.patch` | the `car_firmware_motortest` change (`rl_mode.c`, `rl_mode.h`) |
| `sim_tools/motor_step_test.py` | runs the 52 steps over serial, writes the CSV and the raw transcript |
| `sim_tools/motor_fit.py` | measured summary, fit, in-air check in `real_robot.xml`, `--wobble` comparison |
| `car_logs/2026-10-08/motor_steps.csv`, `motor_steps_raw.txt` | the step test |
| `session-logs/2026-10-08-motor-fit-output.txt` | full output of `motor_fit.py --wobble` |

Outside the repo: `C:\Users\USER\Desktop\car_firmware_motortest` (built
hex in `OBJ\`). The car currently runs this firmware. Flash
`car_firmware_run11`'s hex to go back to run 11.
