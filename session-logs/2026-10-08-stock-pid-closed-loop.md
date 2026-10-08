# 2026-10-08 — Stock PID (mode 1) on the car and in the simulator

Follows `2026-10-08-motor-step-test.md`. The motor fit left runs 9 and 11
under-predicted in simulation. The question here: is the simulated car
still wrong, or is the gap on the policy side? To find out, the stock
Yahboom PID, whose code we have, was run on the car with a continuous log
and then in the simulator with exactly the same controller.

## Summary

- **On the car, the stock PID is calm** while standing:
  - pitch-rate sd **7.6 deg/s**, against 57–95 for the policies;
  - mean |PWM| 1461, with 26 % of ticks inside the motor's dead zone;
  - the sway is slow (≤ 1 Hz).

  It recovered from 8 of 10 pushes in 0.68–0.85 s and fell on the two
  hardest (400–640 deg/s).
- **The simulator reproduces this** with the fitted motor (model A) at about
  **2° of slack and ≤ 15 ms of delay**: 6.2–6.8 deg/s, 1500 mean |PWM|.
  With 3–4° of slack, or 20 ms of delay, the simulated PID falls into a
  10 Hz limit cycle (33–90 deg/s) that the car does not show. That bounds
  the car's slack near 2°, which agrees with the hand measurement of
  2–3.5°.
- **At those settings the policies are still 3× too calm in simulation.**
  Runs 9 and 11 give about 19–21 deg/s, against 68 and 57.5 on the car.
  The motor, slack and delay now agree with a controller we fully know, so
  **the policies' extra wobble on the car comes from the policy side**:
  - the inputs they see;
  - how they are computed on the car;
  - or behaviour the PID does not exercise (frequent motor reversals).

  A wrong pitch zero was tested and is not it: ±2° changes the simulated
  wobble by 0–5 deg/s.

## 1. Firmware: `car_firmware_pidlog`

`C:\Users\USER\Desktop\car_firmware_pidlog` is `car_firmware_motortest`
(run 11 weights and the `mt` commands) plus a 200 Hz stream.
`firmware/pidlog_stream.patch` holds the change, against
`car_firmware_motortest`; apply it with `patch -p1 --binary`.

**What changed:**
- `TUNE_Sample()` is called by the stock ISR at the end of every tick, in
  all modes; it was an empty stub. It now copies one record into a
  128-tick ring. The stub left `removed_modes21_27_stubs.c`.
- `LOG_Poll()` prints the ring from the main loop. TUNE_Poll calls it in
  mode 28, and one line added to `main.c` calls it in mode 1.
- In mode 28 the `p` command toggles the stream; it is on by default.

**Line format:** `P <k> <angle*100> <gyro> <L> <R> <encL> <encR> <accY>
<accZ>`:
- the stock Angle_Balance (Kalman, deg);
- Gyro_Balance (raw, −gyro X);
- the PWM that reached the motors (0 when they are cut);
- the encoder deltas;
- the raw accelerometer Y and Z.

**Build:** 0 errors and the usual 4 warnings. The image ends at
0x0800FE78, 392 bytes under 64 KB.

**Only about every second tick arrives in mode 1.** The stock mode-1 code
reads the IMU one byte at a time, which leaves the main loop too little
time to print 200 lines a second. The ring stays full and every other
record is dropped: tick steps are 2 for 85 % of samples, 1 for 13 %, and 3
for 2 %. That is effectively 100 Hz, regular, and enough for 1–10 Hz
statistics. A full 200 Hz would need a binary format or DMA output.

## 2. The test

The log is `car_logs/2026-10-08/pid_mode1.txt`, recorded with
`sim_tools/stream_log.py`.

- Battery: 12.48 V before and 12.46 V after (multimeter).
- Mode 1, on the floor. It stood about 58 s untouched, then took 10
  pushes, alternating direction.

**Results** (`sim_tools/pid_sim.py`):
- **Standing:** pitch-rate sd 7.6 deg/s, mean |PWM| 1461, in the dead zone
  (|PWM| < 1395) 26 %, PWM sign flips 1 %. The angle had a mean of +0.98°
  and an sd of 1.26°. The spectral peak sits at the 1 Hz band edge, so the
  motion is a slow sway.
- **Pushes:**
  - 8 recovered: peak rates of 184–287 deg/s and peak angles of 16–30°,
    back within 2° in 0.68–0.85 s, with the PWM at or near 2800;
  - 2 fell: 407 and 639 deg/s.
- **Gyro offset:** Gyro_Balance averages +39.8 counts while standing. That
  is the MPU's offset, the same as `cal`'s 0.0427 rad/s. The stock Kalman
  filter estimates and removes it; the PID's D term does not, and the
  velocity integral cancels it.

**The copied controller matches the car:**
- Over a slow stretch, logged PWM minus the copied `Balance_PD` (96 per
  degree, 0.75 per gyro count) is a smoothly varying velocity term.
- Replaying the copied `Velocity_PI` on the logged encoders tracks that
  term with correlation 0.95 and slope 1.15, plus an offset of −141. The
  offset is the integral's unknown start; the slope is above 1 because the
  missing ticks are interpolated.

## 3. The stock PID in the simulator (`pid_sim.py --sim`)

**Setup:**
- Controller: the same PID, copied from `pid_control.c`, `KF.c` (KF_X),
  `app_motor.c` (PWM_Ignore) and `app_control.c` (Get_Angle).
- Sensing:
  - the accelerometer is MuJoCo's chassis acceleration (gravity included),
    moved to an IMU 4 cm above the axle;
  - the gyro is the body pitch rate;
  - both in raw MPU6050 counts, with the gyro offset of +40;
  - noise of 120 counts on the accelerometer and 5 on the gyro, the
    motors-off levels on the car.
- The Kalman filter runs 10 s at rest first. The PWM is applied through
  PWMCommandWrapper, so the delay queue, slack, motor and armature are as
  in training.
- 30 s standing after 3 s, sampled every second tick like the car.

| standing | pitch-rate sd | peak | mean \|PWM\| | dead zone | sign flips |
|---|---|---|---|---|---|
| **car** | **7.6** | ≤ 1.0 Hz | 1461 | 26 % | 1 % |
| nominal motor, any of 2–4° / 5–20 ms | 12.0–13.4 | 1.7–2.7 Hz | 1533–1547 | 17–21 % | 3–5 % |
| fitted A, 2° / 5–15 ms and 3° / 5 ms | **6.2–6.8** | 1.6–2.2 Hz | 1494–1500 | 10–11 % | 1 % |
| fitted A, 2° / 20 ms, 3° / ≥ 10 ms, 4° | 33–90 | 9.2–10.6 Hz | 1648–2162 | 1–4 % | 18–21 % |

Full table: `session-logs/2026-10-08-pid-sim-output.txt`.

**Reading:**
- The fitted motor fixes the PID's level: the nominal motor makes it
  wobble 60–75 % more than the car.
- The calm regime only exists at small slack and short delay. That rules
  out the 4–6° and 10–20 ms the policy-only fit preferred
  (`2026-10-08-run9-run10-delay-and-gear-slack.md` §5), which was a fit
  that had no motor model.
- The car spends more time inside the dead zone than the simulator (26 %
  against 10 %), and sways a little slower. That is a residual pointer to
  the low-PWM behaviour the step test found.

## 4. The policies at the PID-consistent settings

With the fitted motor A at 2° and 10 ms (`motor_fit.py`'s harness, 2
seeds):
- run 8: 55.7 deg/s, 2011 mean |PWM| (car 95.5 / 2170);
- run 9: 20.1 deg/s, 1531 (car 68.2 / 1886);
- run 11: 20.3 deg/s, 1531 (car 57.5 / 1800).

**Pitch-zero error** (`env._pitch_off`, ±2°, same setting):

| pitch-zero error | −2° | −1° | 0° | +1° | +2° |
|---|---|---|---|---|---|
| run 8 | 54.9 | 55.2 | 55.7 | 56.3 | 61.9 |
| run 9 | 19.2 | 19.2 | 20.1 | 19.6 | 19.5 |
| run 11 | 24.7 | 21.4 | 20.3 | 21.5 | 23.9 |

Not the cause.

## 5. What is left, and how to test it

The simulated hardware is now checked against a known controller, so the
candidates are what differs when a policy runs:

1. **Attitude filter.** On the car the policy's pitch comes from
   `rl_mode.c`'s complementary filter: gyro integration plus 1 % per tick
   of accelerometer angle, gated to ±15 % of 1 g. The accelerometer also
   sees the wheel's acceleration. During a 4–7 Hz wobble that feeds back
   a lagged error of roughly 0.5°. The simulator gives the policy the true
   pitch plus 0.002 rad of noise.
   **Test (simulation only):** give the policy the car's filter, fed with
   the simulated accelerometer as `pid_sim.py` does.
2. **Motor reversals under load.** The PID flips sign on 1 % of ticks, the
   policies on 3–7 %, and the step test never reversed a turning wheel.
   **Test (car, motor-test firmware plus a two-level step):** for example
   +2000 → −1600.
3. **The int16 network.** The int16 network on the car (`policy_q.c`) was
   not re-run in closed loop for run 11.
   **Test (simulation):** run the quantised weights in the simulator.
4. **More policy data.** The `pidlog` firmware also streams in mode 28.
   Mode 28 has less main-loop time than mode 1, so expect more gaps, but
   it is still far more than the E lines' 0.6 s per run.

## Files

| file | what |
|---|---|
| `firmware/pidlog_stream.patch` | `car_firmware_pidlog` against `car_firmware_motortest` |
| `sim_tools/stream_log.py` | records the stream with PC time |
| `sim_tools/pid_sim.py` | car log analysis, port check, stock PID in the simulator |
| `car_logs/2026-10-08/pid_mode1.txt` | the mode-1 test |
| `session-logs/2026-10-08-pid-sim-output.txt` | `pid_sim.py --sim` output |

The car now runs `car_firmware_pidlog` (run 11 weights in mode 28). To go
back, flash `car_firmware_run11`'s hex.
