# Deploying the trained policy on the Yahboom balance car

For whoever flashes and tests the car. Do Steps 1–9 in order. The Reference
section at the end explains the why, and covers porting to other firmware.

**Status (2026-10-08):** run 8 and run 9 have run on the car with these
steps. Both balance (17–51 s per run) but wobble at about 7 Hz; run 10 is
being trained against the cause, sense-to-act delay and gearbox slack
([session log](../session-logs/2026-10-08-run9-run10-delay-and-gear-slack.md)).
`models/best_real/best_model.zip` is run 9. Run 7 fell within 0.2–2 s; run 8
was rebuilt from that failure
([diagnosis](../session-logs/2026-10-06-hardware-mode28-diagnosis.md)). Treat
every new policy's first session as a supervised test, with a hand ready to
catch.

On this car the `cal done` pitch zero comes out at about **3.2–3.5°** when the
car is held at its balance point. If yours lands far from that, `trim` to it
(Step 4's commands).

## What you need

| | |
|---|---|
| Car | Yahboom STM32F103 balance car with a charged battery (the stock code cuts the motors below about 10 V), and a clear floor |
| PC software | Keil MDK 5 with ARMCC 5.06; FlyMCU; Python with this repo's `venv`, plus `pip install pyserial` (and `matplotlib` for plots) |
| Phone | the Yahboom Bluetooth app used for mode 1, to drive the car |
| From this repo | `firmware/policy.c`, `policy.h`, `policy_weights.h`, `policy_weights_q.h`, and `firmware/check_car_log.py` |
| From the team | their mode-28 Keil project at its **v6** state (burst IMU read, 4 KB stack), and their folder `rl_mode28_20261003/`, which has `rl_trim_helper.py`. Without the project: `firmware/assemble_mode28_project.py` builds it from the stock Yahboom project and the folder's `src/` (Step 1 included) |

You do not need STM32CubeMX, an ST-Link or a motor datasheet.

The team's project is the stock Yahboom firmware with the policy added as
mode 28. It already contains the two fixes the car needed, a full 200 Hz loop
and a 4 KB stack. Run 8 drops into it and nothing else changes.

## Step 1 — Put run 8 into the Keil project

1. Copy these four files from `3Dsim/firmware/` into the project's `APP\RL\`
   folder (the one with `rl_mode.c`), replacing the old ones:
   `policy.c`, `policy.h`, `policy_weights.h`, `policy_weights_q.h`.
   Leave `policy_q.c`, `policy_q.h`, `odom.c` and `odom.h` as they are.
2. Open `APP\RL\rl_mode.c`, find the `policy_build_obs_rp(` call (about line
   297), and delete the two wheel-angle arguments:

   ```c
   /* before */
   policy_build_obs_rp(obs, roll, pitch, od.wheel_angle_l, od.wheel_angle_r, od.v_forward,
                       gyro, od.wheel_vel_l, od.wheel_vel_r, v, w);
   /* after */
   policy_build_obs_rp(obs, roll, pitch, od.v_forward,
                       gyro, od.wheel_vel_l, od.wheel_vel_r, v, w);
   ```

3. In Keil, open **Project → Options for Target → C/C++**. Add
   `POLICY_NO_FLOAT_INFER` to the **Define** box so that it reads
   `STM32F10X_HD,USE_STDPERIPH_DRIVER,POLICY_NO_FLOAT_INFER`.
   This leaves out the float network; the car runs the int16 one in
   `policy_q.c`. The team's old `policy.c` had this define as its first line,
   and the repo's copy does not.
4. Change nothing else. In particular, keep all of these:
   - `MOTOR_IGNORE_PULSE` and `POLICY_PWM_DEADBAND` at **1300** (Reference B
     explains why);
   - `Stack_Size EQU 0x00001000` in `startup_stm32f10x_hd.s`;
   - the burst IMU read in `app_control.c`;
   - the project's optimisation settings, because the timing in Reference E
     was measured with them.

## Step 2 — Build the hex

Choose **Project → Build Target** (F7). Expect `0 Error(s)`. The output is
`OBJ\stm32_Balance_Car_L.hex`.

If `rl_mode.c` reports `#140: too many arguments in function call`, step 1.2
was missed. If step 1.3 is missed, the build still works but may carry the
unused float network, up to about 26 KB of flash.

## Step 3 — Flash with FlyMCU

1. Close the logger, or anything else using the car's COM port.
2. Connect the car's USB port. In FlyMCU, select that COM port (the CH340
   entry in Device Manager) and the hex file. Tick **编程前重装文件**
   (reload the file before programming) so a rebuilt hex is always used.
3. Leave **编程到FLASH时写选项字节** (write option bytes) **unticked**.
4. In the drop-down at the bottom, choose **DTR的低电平复位，RTS高电平进BootLoader**
   (DTR low resets, RTS high enters the bootloader). Yahboom boards use these
   lines to enter the bootloader by themselves.
5. Click **开始编程** (start programming) and wait for the success message.
6. If FlyMCU cannot connect, flash by hand: set BOOT0 high, press reset,
   flash, then set BOOT0 low and press reset again.
7. **Close FlyMCU.** It keeps the port open.

## Step 4 — Start the logger

In the team's `rl_mode28_20261003/` folder:

1. Rename the old `rl_serial_log.txt` and `rl_runs_summary.txt`. The logger
   appends to them, and they hold the run-7 tests.
2. Run `python rl_trim_helper.py COM3`, using your COM port.
3. **Type `auto off` into `cmd.txt` and save it.** The logger starts with
   auto-trim on (`auto = True` in `main()`, despite the team's README). Its
   0.035 m/s-per-degree gain comes from run 7, so if left on it would change
   the pitch zero between runs.

To send a command, type it into `cmd.txt` and save. The logger sends it and
empties the file.

| command | what it does |
|---|---|
| `cal` | recalibrate (Step 5) |
| `st` | print the state, calibration and PWM |
| `s` | turn the 10 Hz `D` stream on or off |
| `trim <deg>` | add `<deg>` to the pitch zero, at most ±5° per command |
| `auto off` / `auto on` | switch the logger's own auto-trim; not sent to the car |

The logger writes two files: `rl_serial_log.txt`, with every line the car
printed and the PC time, and `rl_runs_summary.txt`, with one line per run.

## Step 5 — Power on, select mode 28, calibrate

1. Hold the car and switch it on. The OLED shows `1.Standard Mode`.
2. Turn either wheel by hand about a fifth of a turn to step to mode 28 (this
   build has only modes 1 and 28). Press **KEY1** to confirm.
3. Calibrate: hold the car upright at its balance point, completely still,
   until the logger prints `cal done`. That takes about 5 s (3 s to settle,
   2 s to average), and any movement restarts it. The OLED shows `CAL hold`
   while it runs. If it does not start, write `cal` into `cmd.txt`.
4. When the OLED asks for the key (`put down key start!`), press **KEY1**
   again. The motors are now enabled, **and they stay enabled until you
   switch the car off.** In mode 28, KEY1 does nothing after this.

From now on, the policy **starts by itself** whenever the car is held upright
(pitch within 5.7°, roll within 11.5°) and still for 1 s. The OLED then shows
the left and right PWM. A run ends only when one of these happens:

| what happens | reason code in the log |
|---|---|
| pitch or roll passes 40° (0.70 rad) | 1 |
| the stock code's own 40° check fires first, or the battery is low | 2 |
| a wheel stays above 25 rad/s for 0.5 s (the car was lifted) | 3 |

After a run ends, the car waits with its calibration kept, and **starts again
as soon as it is upright and still for 1 s**. Keep hold of it while you stand
it back up. Only the power switch turns the motors off.

## Step 6 — In-hand test, wheels off the ground

1. Hold the car upright and still in the air. It starts after 1 s.
2. Tilt it slightly forward: both wheels must turn forward. Tilt it back:
   both must turn backward.
3. The PWM values on the OLED must keep changing. They must not stay at
   −1923 / −1466, which was run 7's frozen output.
4. End the run by tipping the car past 40°, or let the wheels spin up
   (reason 3).

If a wheel turns the wrong way, stop. Something other than the policy has
changed (see the sign checks in Reference C).

## Step 7 — Floor runs

Do 3–5 runs, with a hand ready to catch:

1. Hold the car upright at its balance point on the floor, still. It starts
   after 1 s. Let go gently.
2. Leave some runs standing still for about 10 s. Drive others with the app:
   - forward / back: ±0.25 m/s;
   - left / right: turn at ±0.5 rad/s, and the diagonal buttons combine both;
   - releasing the buttons makes it stand still.

   Note which runs used the app. The log does not record commands.
3. End each run by catching the car and tipping it past 40°. The car then
   prints its flight recording.

## Step 8 — Check the log

From `3Dsim/`:

```
python firmware/check_car_log.py ../rl_mode28_20261003/rl_serial_log.txt --plot
```

`--last N` checks only the last N runs. `--plot` saves each run's last 0.6 s
(pitch, pitch rate, speed and PWM) as PNG files in `car_log_plots/` next to the
log; it needs matplotlib. Run on the team's run-7 log, the checker reports
exactly the faults found there.

Run 8 passes if the log shows the following:

| check | read it from | run 7 | run 8 should show |
|---|---|---|---|
| Stays up | `R` line | fell in 0.2–2 s | lasts until you tip it, 5 s or more |
| Output not frozen | `E` PWM columns | stuck at −1923 / −1466 | changes every tick |
| Plausible speed | `E` speed column | one-tick spikes of tens of m/s | under 1500 mm/s, never jumps more than 200 mm/s in one tick |
| Control rate | `T` line | — | 0 late ticks, longest gap about 5000 µs, longest interrupt under 5000 µs |
| PWM sign flips | consecutive `E` lines | about every tick (in simulation) | about 0–27 % of ticks |
| Standing drift | `D` speed, no app input | — | near 0 (simulation: up to ±0.02 m/s) |
| Forward button | `D` speed | — | about 0.22 m/s (0.25 commanded; simulation tracks about 90 %) |

Because a run ends with a tip, its `E` lines (the last 0.6 s) show the tip, not
the balancing. The balancing is in the 10 Hz `D` lines. Keep both log files:
they are what the simulation gets compared against.

## Step 9 — What to do next

Work down the table from the top. If an earlier row applies, the rows below it
don't mean anything yet.

| what you see | likely cause | what to do |
|---|---|---|
| Any `FAIL` for frozen output, impossible speed or timing | the firmware integration, not the policy. Run 7 had all three (a stack overflow and a half-rate loop) | Don't trim or retrain. Check that Step 1 was done on the v6 project, then send the log and plots |
| Falls within 1–2 s, with no firmware `FAIL` | the simulator still differs from the car | Redo `cal`, repeat Step 6, then send the log and plots |
| Stays up but creeps one way with no input (`D` mean above about 0.03 m/s) | the pitch zero is off. In simulation, 2° of error gives about 2 cm/s of creep | `trim -0.5` if it creeps forward, `trim 0.5` if it creeps back. If the creep gets worse, go the other way. If it needs more than ±2°, recalibrate instead |
| Stays up but shakes or buzzes, or the PWM sign flips on more than 50 % of ticks | the motor dead zone or gearbox backlash differs from the model | bench motor sweep, then retrain |
| The forward button gives far from about 0.22 m/s | the motor strength differs from the model | bench motor sweep, then retrain |
| Stays up, holds still, drives and turns | run 8 works on the car | Try longer runs, held commands and a 1 kg load, then slopes and other floors. Add a watchdog (Reference E) before any unsupervised use |

To compare with the simulation, run `python enjoy_drive.py` and give it the
same command with the sliders or arrow keys. In simulation, run 8 holds still
to within 1 mm/s and drives at about 90 % of the commanded speed.

### Bench motor sweep

The model's dead zone (1460 counts, randomised over 1350–1600) is fitted to one
free-spin log plus the parameter sheet. To measure it properly:

1. Lift the car so both wheels spin freely.
2. For each wheel and each direction, step the raw PWM from 0 to 2800 in steps
   of 50. Hold each step for 1 s and record the steady encoder speed.
3. The dead zone is the PWM at which the wheel starts to turn.

Compare the speeds with the model (nominal motor, wheels in the air):

| raw PWM | 1500 | 1705 | 1985 | 2400 | 2800 |
|---|---|---|---|---|---|
| model wheel speed (rad/s) | 0.8 | 5.9 | 12.8 | 23.1 | 32.9 |

The car's one free-spin log (left at PWM about 1985, right at about 1705)
averaged 9.5 rad/s; the model gives 9.35 for that pair. The spec, 333 RPM at
12 V, is 34.9 rad/s.

**The mode-28 build has no raw-PWM command yet.** `rl_mode.c` needs a serial
command that sets a fixed PWM while the policy is idle and prints the encoder
speed.

If the dead zone is outside 1350–1600, or the speeds are off by more than
about 10 %, edit `motor_model.py` and retrain:

| constant | set it to |
|---|---|
| `MOTOR_DEAD_ZONE` | the mean measured start-up PWM |
| `MOTOR_DEAD_ZONE_RANGE` | the measured spread, plus about 100 counts on each side |
| `TAU_SCALE_RANGE`, `KV_SCALE_RANGE` | widen them if the measured speeds fall outside the model; free-spin speed scales with the ratio of the two |

### Retrain and re-flash

Run these from `3Dsim/` with the venv active:

1. Back up run 8: copy `models/best_real/` to `models/best_real_RUN8/`.
   Training overwrites it.
2. Change `"logs/run8"` in `train_real_robot.py` to a new folder name, then run
   `python train_real_robot.py`. 30 M steps took about 2.5 h for run 8. The
   best checkpoint goes to `models/best_real/best_model.zip`, the final one to
   `models/ppo_real_robot.zip`.
3. Watch the result in simulation: `python enjoy_drive.py`.
4. Export the weights. `python export_stm32.py` writes the float header and
   test vectors; `python firmware/quantize_weights.py` writes the int16 header.
5. Check the export on the PC (Reference F).
6. Copy `policy_weights.h` and `policy_weights_q.h` into `APP\RL\`, rebuild,
   flash, and repeat Steps 4–8.

### Python files

| file | where | used for | needs |
|---|---|---|---|
| `rl_trim_helper.py` | team folder `rl_mode28_20261003/` | logging the car, sending commands | `pip install pyserial` |
| `firmware/check_car_log.py` | this repo | checking each run in a log | standard library; matplotlib for `--plot` |
| `enjoy_drive.py` | this repo | running the same commands in simulation | `requirements.txt` |
| `motor_model.py` | this repo | the motor constants a bench sweep would change | — |
| `train_real_robot.py` | this repo | retraining | `requirements.txt` |
| `export_stm32.py` | this repo | float weight header and test vectors | `requirements.txt` |
| `firmware/quantize_weights.py` | this repo | int16 weight header for `policy_q.c` | `requirements.txt` |
| `firmware/check_obs_builder.py` | this repo | firmware inputs vs training inputs, tick by tick | `requirements.txt`, gcc |

---

# Reference

## A. What the car prints

| line | when | fields |
|---|---|---|
| `D` | 10 times a second | time (ms), state (0 waiting, 1 calibrating, 2 running), pitch (°), speed (m/s), left PWM, right PWM, pitch zero (°) |
| `R` | a run ended | run length (ms), reason code (Step 5), largest pitch (°) |
| `E`, up to 120 | after `R` | the last 0.6 s at 200 Hz, one line per tick: index, pitch ×100 (°), pitch rate ×100 (°/s), speed (mm/s), left PWM, right PWM |
| `R end` | after the `E` lines | end of the dump |
| `T` | after `R end` | in µs: longest interrupt, stock part, RL part, network, longest gap between ticks; then late ticks and total ticks |

Never printed: roll, yaw rate, raw encoder counts, the commands and the 34
inputs. Full-rate data covers only the last 0.6 s. Streaming at 200 Hz is not
possible, because the interrupt already uses 4.6 ms of every 5 ms. To see
more, enlarge `FR_N` (120 now) in `rl_mode.c` and add fields, after checking
the free RAM in the Keil map file. The recording is printed after the run, so
it costs nothing while the car balances.

## B. Action to PWM, and why the dead band stays 1300

The network outputs one value per wheel in [−1, 1]. It is a PWM command
before dead-band compensation, not a torque, which is why no motor constant,
winding resistance or datasheet is needed:

```
pwm = sign(a) * (|a| * 1500 + 1300), clamped to ±2800        (a = 0 gives 0)
```

In repo terms, `policy_action_to_pwm()` gives `a × 1500`
(`POLICY_PWM_USABLE` = 2800 − 1300), and the stock `PWM_Ignore()` adds the 1300
(`MOTOR_IGNORE_PULSE`). Mode 28's `action_to_pwm()` does both in one step and
skips `PWM_Ignore()`. Never add the 1300 twice.

1300 is the firmware's compensation, not the motor's dead zone. The real
gearmotor gives almost nothing below about 1460. The parameter sheet measured
1480 forward and 1455 reverse, and the car's free-spin log fits about 1460.
Runs 1–7 were trained as if the 1300 offset already produced torque, which is
why run 7 fell. Run 8 trains with the 1300 compensation in front of a motor
whose dead zone is randomised over 1350–1600 per wheel and direction, so the
policy covers the gap itself. Raising 1300 to a measured value would change
the mapping the network learned. If the motor turns out different, change
`motor_model.py` and retrain instead.

## C. Hardware facts

| part | consequence |
|---|---|
| STM32F103RCT6: Cortex-M3, 72 MHz, no FPU, 256 KB flash, 48 KB RAM | every float operation is a library call, so the car runs the int16 network (`policy_q.c`) |
| MPU6050, no magnetometer | yaw drifts without bound, so the policy gets yaw pinned to 0 (`policy_build_obs_rp`) |
| AT8236 H-bridge, no current sensing | cannot close a torque loop, hence PWM actions |
| JGB37-520, 12 V, 333 RPM, 1:30 gearbox, 11 PPR, 4× quadrature | 1320 counts per wheel revolution (210 per rad). At 0.3 m/s the motor turns 1719 RPM, 17 % of no-load |

Pin map, from the stock `BSP/Motor/motor.h` and `BSP/Enconder/encoder.c`:

| function | signal | port | peripheral |
|---|---|---|---|
| Left motor IN1 / IN2 | `L_PWMA` / `L_PWMB` | PC6 / PC7 | `TIM8->CCR1` / `CCR2` |
| Right motor IN1 / IN2 | `R_PWMA` / `R_PWMB` | PC8 / PC9 | `TIM8->CCR3` / `CCR4` |
| Left encoder A/B | `H1A/H1B` | PA6 / PA7 | `TIM3`, `TIM_EncoderMode_TI12` (4×) |
| Right encoder A/B | `H2A/H2B` | PB6 / PB7 | `TIM4`, `TIM_EncoderMode_TI12` (4×) |
| MPU6050 | SCL/SDA + INT | I2C2 pins | INT drives the 200 Hz loop |

Both AT8236 inputs take PWM: one pin gets the magnitude and the other 0 (fast
decay). **Left and right are mirrored**, because the motors are handed. Copy
this exactly, or one wheel will drive backwards:

```c
if (motor_left  > 0) { L_PWMB = |v|; L_PWMA = 0; } else { L_PWMA = |v|; L_PWMB = 0; }
if (motor_right > 0) { R_PWMA = |v|; R_PWMB = 0; } else { R_PWMB = |v|; R_PWMA = 0; }
```

The stock `APP/app_motor.h` confirms the numbers: `Control_Frequency 200.0`,
`Diameter_67 67.0` (wheel radius 0.0335 m), `EncoderMultiples 4.0`,
`Encoder_precision 11.0`, `Reduction_Ratio 30.0`. Some JGB37-520s have 13 PPR,
so turn a wheel one revolution by hand and check that it counts 1320
(`POLICY_ENC_COUNTS_PER_REV`).

Signs the team checked on the car:
- both encoders count positive going forward;
- positive PWM drives forward;
- `ML` is the left wheel;
- pitch is positive leaning forward;
- yaw rate is positive turning left.

The encoders are required: the wheel speeds and the forward speed come from
them, and the motor driver gives no feedback.

## D. How the firmware builds the 34 inputs

Run 8 trains on exactly this computation, and `check_obs_builder.py`
(Reference F) shows the two agree. Mode 28 does the same with the team's
`odom.c`. With the repo's API:

```c
/* On arming (upright and still, motors off) and after every fall: */
policy_reset_state(pitch, gyro_xyz[1], POLICY_WHEEL_RADIUS * gyro_xyz[1] * cosf(pitch));

/* Every 5 ms tick, starting the tick after arming: */
policy_odom_update(dcount_l, dcount_r,      /* encoder counts since last tick, + = forward */
                   pitch, gyro_xyz[1],       /* rad, rad/s */
                   &wheel_vel_l, &wheel_vel_r, &v_forward);
policy_build_obs_rp(obs, roll, pitch, v_forward,
                    gyro_xyz,                /* rad/s: roll, pitch, yaw (+ = turning left) */
                    wheel_vel_l, wheel_vel_r,
                    cmd_forward, cmd_turn);  /* m/s, rad/s */
policy_infer(obs, action);                   /* or policy_infer_q() on the car */
```

The 34 inputs are:
- **17 base inputs:** height, the attitude quaternion from roll and pitch with
  yaw 0, two wheel angles, forward speed, lateral and vertical speed, gyro x,
  y and z, both wheel speeds, and both commands.
- **15 history inputs:** pitch, pitch rate and forward speed, each 2, 5, 11,
  23 and 47 ticks back, reaching 235 ms.
- **Position integral:** τ 2 s, clipped to ±0.5 m, scaled ×10.
- **Heading integral:** τ 30 s, clipped to ±0.2 rad, scaled ×5.

Inputs are clamped: speed to ±2 m/s, wheel speed to ±40 rad/s, and NaN becomes
0.

Five inputs are constants, which training feeds too:

| index | input | value | why |
|---|---|---|---|
| 0 | chassis height | 0.0334 | not measurable; the axle sits one wheel radius up |
| 5, 6 | wheel angles | 0 | dropped in run 8: physically meaningless, and run 7 had learned to depend on them |
| 8 | lateral speed | 0 | a two-wheeled robot has none |
| 9 | vertical speed | 0 | zero except in a fall |

Yaw is pinned to 0 because an earlier policy coped with a yaw error of 10°,
30° and 90° but fell after 138 steps at 180°. Every other input is
body-referenced, so pinning yaw costs almost nothing.

Largest first-layer weights in run 8, as a mean |weight| per input:

| input | weight |
|---|---|
| heading integral | 0.63 |
| speed 235 ms ago | 0.51 |
| turn command | 0.46 |
| yaw gyro | 0.40 |
| forward command | 0.39 |
| pitch | 0.36 |
| pitch 10 and 25 ms ago | 0.34 each |
| the other 21 inputs, on average | 0.18 |

The heading path matters most, so check the gyro-z sign and bias calibration
end to end. Run 8 was trained with ±0.003 rad/s of gyro bias and ±2° of
pitch-zero error.

## E. Timing, memory and safety

The network is 34 → 64 → 64 → 2 with tanh, 6,530 parameters: 25.5 KB of flash
as floats and 512 B of RAM. `export_stm32.py` prints these numbers for each
checkpoint. It must run at exactly **200 Hz**: the history taps and the
integral decay constants (`POLICY_POS_DECAY`, `POLICY_YAW_DECAY`) are defined
at 200 Hz.

Measured by the team on the car with the DWT cycle counter (mode-28 build,
ARMCC 5.06):

| | before their fix | after (v6) | budget |
|---|---|---|---|
| whole control interrupt | 6.7 ms | **4.6 ms** | 5 ms |
| stock part (IMU read, Kalman, stock PIDs) | 3.67 ms | 1.7 ms | — |
| RL part / network alone | 3.06 / 2.83 ms | 2.94 / 2.70 ms | — |
| longest gap between ticks | 10.2 ms | 5.2 ms | 5 ms |
| late ticks (gap over 7.5 ms) | every tick | 0 | 0 |

The fix replaced 14 one-byte software-I2C reads with one 14-byte burst read.
An overrun doesn't just slow the loop: it skips the next MPU6050 interrupt and
halves the rate, which silently doubles every time-based input. The margin is
thin. If anything is added, the stock Kalman filter and stock PIDs are the
first things to drop in mode 28, since the policy doesn't use them.

The stack must be **4 KB** (`0x1000`). With the stock 1 KB, the main loop's
float `printf` plus the control interrupt overflowed into `odom.c`'s buffer
and produced impossible speeds. The worst case measured is about 2.2 KB.

The policy has no idea of failure, so safety sits outside the network:

| safety item | status in mode 28 |
|---|---|
| tilt cut-out at 0.70 rad (training's fall limit) | yes |
| start only upright and still, calling `policy_reset_state()` at that moment | yes (auto-arm) |
| watchdog: cut the motors if a control tick is missed | **no** |
| current or thermal limit: the policy may hold a saturated PWM indefinitely | **no** |

## F. Checking a new export on the PC

The network:

```
cd firmware
gcc -O2 -o test_policy test_policy.c policy.c -lm && ./test_policy
```

The expected result (fast tanh, the default) is
`worst error: 2.198e-02 (tolerance 5.0e-02)`, then `PASS`. The tolerance is
the measured approximation error of the current checkpoint plus headroom (run
7: 0.011; run 8: 0.020 on realistic inputs, 0.031 on random ones), so
re-measure it per checkpoint. With `-DPOLICY_FAST_TANH=0` the worst error
should be about 1e-7. Anything above about 1e-5 is a porting bug, usually
row/column-major confusion.

The input pipeline (odometry, history, integrals and clamps), compared with
the training code tick by tick:

```
gcc -O2 -shared -o test_obs_builder.dll test_obs_builder.c policy.c -lm
..\venv\Scripts\python.exe check_obs_builder.py
```

Expect a worst difference of about 1e-6, then `PASS`. It is built as a DLL
because Windows Application Control on this PC blocks freshly built `.exe`
files.

The car's int16 network was checked in closed loop: `policy_q.c` with the
run-8 header balanced the simulated car on the nominal and worst-case
settings. `quantize_weights.py --check <header>` confirms that the quantiser
reproduces a given header exactly; it reproduces the team's run-7 header.

## G. Porting into a different firmware

Without the team's mode-28 project, each 200 Hz tick needs, in order: an IMU
read with roll/pitch fusion, the tilt check, odometry, the inputs (Reference D),
the network, then the PWM (Reference B), with the mirrored output from
Reference C.

```c
void control_tick_200hz(void)               /* from a hardware interrupt, not a delay loop */
{
    /* 1. sensors: roll, pitch (rad), gyro_xyz (rad/s), enc_l/enc_r (counts since last tick) */
    /* 2. safety first */
    if (fabsf(roll) > POLICY_FALL_ANGLE_LIMIT || fabsf(pitch) > POLICY_FALL_ANGLE_LIMIT) {
        armed = 0; TIM8->CCR1 = TIM8->CCR2 = TIM8->CCR3 = TIM8->CCR4 = 0; return;
    }
    if (!armed) return;
    /* 3. inputs and network */
    policy_odom_update(enc_l, enc_r, pitch, gyro_xyz[1], &wl, &wr, &v);
    policy_build_obs_rp(obs, roll, pitch, v, gyro_xyz, wl, wr, cmd_forward, cmd_turn);
    policy_infer(obs, action);
    /* 4. PWM, then the mirrored output */
    int pwm_l = PWM_Ignore(policy_action_to_pwm(action[0]));
    int pwm_r = PWM_Ignore(policy_action_to_pwm(action[1]));
    if (pwm_l > 0) { TIM8->CCR2 = pwm_l; TIM8->CCR1 = 0; } else { TIM8->CCR1 = -pwm_l; TIM8->CCR2 = 0; }
    if (pwm_r > 0) { TIM8->CCR3 = pwm_r; TIM8->CCR4 = 0; } else { TIM8->CCR4 = -pwm_r; TIM8->CCR3 = 0; }
}
```

Checklist for a new port:
- a 4 KB stack, built at `-O2`;
- arm with `policy_reset_state()`;
- before closing the loop, command a fixed PWM with the policy bypassed and
  check that each wheel turns the expected way;
- measure the loop time with a GPIO toggle and a scope, or with the DWT
  counter.

On the F103, `policy_infer()` (float, fast tanh) has not been measured. Its
128 tanh calls cost more than the 6,400 multiply-adds. The int16 `policy_q.c`
measured 2.7 ms. A blocking 14-byte MPU6050 read costs about 1.5 ms at
100 kHz and about 0.4 ms at 400 kHz, so use a burst read at 400 kHz, or DMA.

X-CUBE-AI could import an ONNX export instead. For 6,530 parameters, though,
it adds a code generator and a runtime to replace about 40 lines of verified
C. It also still needs the history and integrals (`finish_obs()`) written by
hand. If you want ONNX anyway:

```python
import torch
from stable_baselines3 import PPO
m = PPO.load("models/best_real/best_model.zip", device="cpu")
torch.onnx.export(m.policy, torch.zeros(1, 34), "policy.onnx",
                  input_names=["obs"], output_names=["action"], opset_version=11)
```

This exports the value head too; prune it in Cube.AI. Only the actor is used
to act, which is why `export_stm32.py` exports only the actor.

## H. Evidence so far, and known limits

Simulation and PC checks, run 8:

| check | result |
|---|---|
| Motor model vs the car's free-spin log | 9.35 vs 9.5 rad/s (old model: 22.3) |
| Nominal simulated car, 6 commands × 3 seeds × 10 s | 18/18 held, tracking 89–100 % |
| Each uncertain setting at its extreme (dead zone 1350/1600, 2-tick delay, motor −15 %, pitch zero ±2°, gyro bias, noise, 1 kg) | 54/54 held |
| All the worst settings at once | 6/6 held |
| Firmware inputs vs training inputs, tick by tick | at most 9e-7 to 1.2e-6 on all 34 |
| The C code (float fast-tanh, and the int16 `policy_q.c`) driving the simulated car | held 10 s on the nominal and worst-case car, same tracking as PyTorch |

Simulated behaviour, 3 seeds × 10 s, steady state after the first 2 s. The
worst-case car has dead zone 1600, a 2-tick delay, a 15 % weaker motor, +2°
pitch zero and noise:

| behaviour | nominal car | worst-case car |
|---|---|---|
| hold station | drift −0.001 m/s | −0.020 m/s |
| forward +0.15 / +0.30 m/s | +0.136 / +0.267 (91 % / 89 %) | +0.126 at +0.15 |
| reverse −0.15 m/s | −0.137 (91 %) | — |
| turn +0.50 rad/s | +0.500 (100 %) | — |
| +0.15 m/s with +0.30 rad/s | +0.136 / +0.300 | — |
| 1.0 kg load | holds; +0.135 at +0.15 | — |
| pitch zero ±2° | creeps at ∓0.020 m/s | — |

Forward tracking is about 90 % rather than run 7's about 100 %. That is the
price of a motor that gives nothing inside its dead zone.

Still open:
- the on-car test (Steps 5–8);
- the bench motor sweep, since the dead zone rests on one log;
- gearbox backlash, which is not modelled;
- the watchdog;
- friction and slope sensitivity for run 8.

Run 7's slope sweep is kept in
`session-logs/2026-10-07-car-test-data-collection.md` for its method.
