# Deploying the policy to STM32 — step by step

Written for: whoever does the firmware port.

**Updated for run 8** (`train_real_robot.py`, `real_robot.xml`): 34 inputs, PWM
output, 200 Hz control, trained against a motor model and sensor pipeline
corrected from the car's own logs. Run 7 is superseded: on the car it fell
within 0.2–2 s, and the cause turned out to be the simulator, not the port
(`session-logs/2026-10-06-hardware-mode28-diagnosis.md`).

The trained network is 34 → 64 → 64 → 2 with tanh activations: **6,530
parameters, 25.5 KB of flash, 512 bytes of RAM** (re-run `export_stm32.py` and
read its own printed summary if you export a different checkpoint — these
numbers are checkpoint-specific, not fixed). Against a 256 KB / 48 KB budget
the network is a non-issue. The work is in the observation pipeline (stateful
— encoder odometry, a 235 ms history buffer and two leaky integrals) and the
safety layer, not the maths.

## Readiness check before you start

**This is ready for a careful, instrumented on-car test — not for an
unsupervised deployment.** An earlier version of this section said nothing
about the policy's simulated behaviour was in question and that retraining
could not help. The car proved that wrong: the motor model treated the
firmware's dead-band compensation as useful torque, and run 7 learned a control
law for an actuator that does not exist. Run 8 fixes that and trains on exactly
the inputs the firmware computes. What it has been checked against:

| check | result |
|---|---|
| Corrected motor model vs the car's own free-spin log | 9.35 vs 9.5 rad/s (old model: 22.3) |
| Nominal simulated car, 6 commands × 3 seeds × 10 s | 18/18 held, tracking 89–100 % |
| Each uncertain parameter at its extreme (dead zone 1350/1600, 2-tick delay, motor −15 %, pitch zero ±2°, gyro bias, sensor noise, 1 kg) | 54/54 held |
| All of the worst at once (dead zone 1600, 2-tick delay, motor −15 %, +2°, noise) | 6/6 held |
| Firmware observation pipeline vs the training wrapper, tick by tick | ≤ 9e-7 on all 34 inputs (Step 2) |
| The actual C code (float fast-tanh, and the team's int16 `policy_q.c`) driving the simulated robot in closed loop | held 10 s on the nominal and worst-case car, same tracking as PyTorch |

Still open — none of it simulated away:

| gap | what settles it |
|---|---|
| Zero real-hardware runs of run 8 | the on-car test in Step 9 |
| The motor dead zone is fitted to one free-spin log plus the parameter sheet, not swept | 30-min bench sweep: raw PWM 0–2800 per wheel and direction, wheels in the air. If it falls outside 1350–1600, widen `MOTOR_DEAD_ZONE_RANGE` and retrain |
| Gearbox backlash and motor electrical dynamics are not modelled | on-car behaviour; the policy no longer dithers every tick (sign flips on 0–27 % of ticks vs ~100 % for run 7), which makes backlash matter less |
| Timing: the team measured 4.6 ms worst case of a 5 ms tick with the burst IMU read | already fits; keep the 4 KB stack and the burst read from their v6 build |
| Friction sensitivity, and slope on run 8 | not characterised; the slope sweep below is run 7's |
| Watchdog / overrun disarm | not yet implemented (Step 7) |

Steps 8–9 below have the safety checkpoints built in at the points they
actually matter, rather than as a disclaimer at the end.

### Essential apps

| tool | what it's for here |
|---|---|
| **STM32CubeMX** | confirm/add one peripheral (the 200 Hz control-loop timer) on top of the stock project |
| **Keil MDK (µVision)** | add the policy files, build the `.hex` |
| **FlyMCU** | flash over the UART bootloader - **no ST-Link needed** |
| A serial terminal (any UART terminal) | debug prints during bring-up; not optional given the timing risk above |
| Python (`3Dsim/venv`) | re-export weights if you ever retrain (Step 1) |

You do **not** need a motor datasheet, Kt, or winding resistance for this
policy (Step 6) - that's one of the things the PWM-output design removed.

## Target hardware

| part | implication for this port |
|---|---|
| **STM32F103RCT6** | Cortex-M3, 72 MHz, **no FPU**. Every float is a library call. Use the fast-tanh build (§3). |
| **MPU6050** | 6-axis, no magnetometer → **yaw drifts without bound**. Use `policy_build_obs_rp()`, which pins yaw to zero (§4). |
| **AT8236** | voltage-mode H-bridge, **no current sensing** → cannot close a torque loop itself. Needs the feed-forward mapping in §6. |
| **JGB37-520**, 12 V, 333 RPM | built-in hall quadrature encoder → all of the base 17 inputs available (the rest of the 34 are derived from these and the IMU, not separately sensed). 333 RPM at 12 V implies a **1:30 gearbox** (bare 520 runs ~10,000 RPM; 10000/30 = 333). |

### Pin map — read from the stock firmware, not inferred

Source: `4.Balanced_Car_base/*/BSP/Motor/motor.h`, `BSP/Enconder/encoder.c`.

| function | signal | port | peripheral |
|---|---|---|---|
| Left motor IN1 | `L_PWMA` | **PC6** | `TIM8->CCR1` |
| Left motor IN2 | `L_PWMB` | **PC7** | `TIM8->CCR2` |
| Right motor IN1 | `R_PWMA` | **PC8** | `TIM8->CCR3` |
| Right motor IN2 | `R_PWMB` | **PC9** | `TIM8->CCR4` |
| Left encoder A/B | `H1A/H1B` | **PA6 / PA7** | `TIM3`, `TIM_EncoderMode_TI12` |
| Right encoder A/B | `H2A/H2B` | **PB6 / PB7** | `TIM4`, `TIM_EncoderMode_TI12` |
| MPU6050 | SCL/SDA + INT | `I2C2` | INT drives the 200 Hz loop |

`TIM_EncoderMode_TI12` is 4x quadrature, which is why `EncoderMultiples = 4.0`.

**Both AT8236 inputs are PWM-capable** — this is a 2-PWM-per-motor topology,
not PWM+direction. The stock firmware drives one pin with the magnitude and
holds the other at 0 (fast decay):

```c
if (motor_left  > 0) { L_PWMB = |v|; L_PWMA = 0; } else { L_PWMA = |v|; L_PWMB = 0; }
if (motor_right > 0) { R_PWMA = |v|; R_PWMB = 0; } else { R_PWMB = |v|; R_PWMA = 0; }
```

Note the **left and right conventions are mirrored** — left uses PWMB for
forward, right uses PWMA. The motors are physically handed. Copy this exactly
or one wheel will drive backwards.

### Firmware constants confirm every earlier inference

From `APP/app_motor.h`:

```c
#define Control_Frequency  200.0   /* Hz  - matches PARAMS.md          */
#define Diameter_67        67.0    /* mm  -> wheel radius 0.0335 m     */
#define EncoderMultiples   4.0     /* 4x quadrature                    */
#define Encoder_precision  11.0    /* 11 ppr                           */
#define Reduction_Ratio    30.0    /* 1:30 - as inferred from 333 RPM  */
#define Perimeter          210.4867 /* mm = pi * 67                    */
```

1320 counts per wheel revolution = 4 x 11 x 30, exactly as the parameter sheet
states.

### Dead band: partly compensated in firmware, the rest learned

```c
#define MOTOR_IGNORE_PULSE (1300)
int PWM_Ignore(int pulse) {
    if (pulse > 0) return pulse + MOTOR_IGNORE_PULSE;
    if (pulse < 0) return pulse - MOTOR_IGNORE_PULSE;
    return pulse;
}
```

The offset is added **after** the controller, so a policy's output is a
pre-compensation command. The motor's *actual* dead zone is higher than this
1300: the parameter sheet measured 1480 forward / 1455 reverse, and the car's
own free-spin data fits ~1460 (`session-logs/2026-10-06-hardware-mode28-diagnosis.md`).
Below it the gearmotor produces essentially no torque. Runs 1–7 were trained
on a motor model that treated the 1300-count offset as useful torque, which is
why run 7 fell within 0.2–2 s on the car. Run 8 trains with the firmware's 1300
compensation in front of a motor whose real dead zone is randomised over
1350–1600 counts per wheel and direction, so the policy covers the gap itself.
**Keep `MOTOR_IGNORE_PULSE` / `POLICY_PWM_DEADBAND` at 1300.**

### Confirmed parameters for this build

```c
#define GEAR_RATIO      30.0f    /* inferred from 333 RPM @ 12V - CONFIRM on the gearbox */
#define ENCODER_PPR     11.0f    /* typical for JGB37-520 - CONFIRM, some are 13 */
#define ENCODER_QUAD    4.0f     /* TIM_ENCODERMODE_TI12 */
```

Still needed: they fix the 1320 counts per revolution that
`POLICY_ENC_COUNTS_PER_REV` and training both assume. **No longer
needed at all**, since the switch to a PWM-output policy (Step 6): gearbox
efficiency, supply voltage, motor Kt and winding resistance. Those were only
ever inputs to the torque-to-duty conversion this document used to describe,
and that conversion is gone — one fewer thing to chase down a datasheet for.

Derived: **1320 encoder counts per wheel revolution** (210 counts/rad), motor
shaft at **1719 RPM** when the wheel is at the policy's maximum 0.3 m/s — 17%
of no-load, so speed headroom is ample.

### Is the motor big enough? Yes, with margin (measured on the earlier policy)

The table below predates run 7 and the PWM action space — it was measured
against the torque-output policy and the earlier 0.6 N·m driver assumption.
`real_robot.xml`'s actual driver limit is **0.4 N·m** at the wheel (motor
stalls at 0.5679 N·m; see `motor_model.py`'s `TAU_DRIVER_LIMIT`), and torque
now arrives indirectly through the PWM->torque relationship the motor model
defines (`tau = TAU_STALL * duty - KV * omega`), not as a direct policy
output. The qualitative conclusion (margin is ample) has not changed, but
these specific numbers have not been re-measured against run 7 and should not
be quoted as current:

Measured torque the policy actually commands, across all 20 eval conditions:

| | per wheel | at the motor | % of the 0.6 Nm limit |
|---|---|---|---|
| mean | 0.053 Nm (0.54 kg·cm) | 2.5 mNm | 8.9% |
| p99 | 0.148 Nm (1.51 kg·cm) | 7.0 mNm | 24.6% |
| peak | 0.442 Nm (4.50 kg·cm) | 21.0 mNm | 73.6% |

It is above half the torque limit **0.04% of the time** and never above 80%.
Continuous demand is ~0.5 kg·cm, well inside a 1:30 JGB37-520's rating, and
the 4.5 kg·cm peaks are transients far shorter than the motor's thermal time
constant. Sizing is not a concern.

Do check the **AT8236 current rating** against peak draw once Kt is known:
peak current is 21 mNm / Kt, which for a typical small-motor Kt of
0.01–0.02 Nm/A lands around 1–2 A.

---

## Step 1 — Export the weights

```bash
python export_stm32.py
```

Produces two headers in `firmware/`:

| file | contents |
|---|---|
| `policy_weights.h` | every weight as `static const float` (lands in flash) |
| `policy_testvectors.h` | 4 reference input/output pairs for verifying the port |

To export a different checkpoint:

```bash
python export_stm32.py --model models/best_real_RUN6_station/best_model.zip --out firmware/policy_weights.h
```

That example is still a run-3-through-6-lineage checkpoint (33 inputs, no
heading integral) — the exporter will label it correctly and print a warning
that `policy.c` as currently written expects 34, since `policy_build_obs()`
is not generic across input counts. **Do not** point this at anything under
`models/best_their/`: that is the superseded 17-input torque policy on the
wrong plant entirely, and nothing past Step 1 in this document applies to it.

The exporter emits only the **actor**. PPO also trains a value network, but it
exists purely to compute advantages during learning and has no role in
choosing an action.

## Step 2 — Verify the C port on your PC first

Do this before touching hardware. It isolates "did I port the maths correctly"
from every other problem you will hit later.

```bash
cd firmware
gcc -O2 -o test_policy test_policy.c policy.c -lm
./test_policy
```

Expected (default build, `POLICY_FAST_TANH=1`):

```
mode: fast tanh (approximation, for STM32F103 / no FPU)
tolerance: 5.0e-02
...
worst error: 2.198e-02   (tolerance 5.0e-02)
PASS
```

The fast-tanh tolerance is the measured approximation error of the current
network plus headroom, and has to be re-measured per checkpoint (run 7: 0.011;
run 8: 0.020 on realistic inputs, 0.031 on random ones). The exact build below
is the porting check.

Add `-DPOLICY_FAST_TANH=0` to check the matrix maths in isolation from the
tanh approximation — expect worst error down around 1e-7 (float32 rounding).
Anything above ~1e-5 on that build means a real porting bug, almost always
row/column-major confusion in the matrix loops.

**This only checks `policy_infer()` — the network — on isolated vectors.** It
does not exercise the observation pipeline (encoder odometry, history buffer,
leaky integrals, input clamps), which depends on a *sequence* of calls. Check
that against the training wrapper itself:

```bash
gcc -O2 -shared -o test_obs_builder.dll test_obs_builder.c policy.c -lm
..\venv\Scripts\python.exe check_obs_builder.py
```

`check_obs_builder.py` rolls the actual training wrapper forward — the nominal
car and a randomised, noisy one — records the raw signals the firmware would
get each tick (encoder count deltas, fused roll/pitch, gyro, commands), pushes
them through `policy_reset_state()` + `policy_odom_update()` +
`policy_build_obs_rp()`, and compares all 34 observations tick by tick.
Expected: worst disagreement ~1e-7 (float rounding), then `PASS`. A wrong tap
index, a missing clamp or an off-by-one in a ring buffer shows up here and
nowhere else — `policy_infer()` alone cannot see it, because a wrong
observation still produces *a* plausible-looking action, just not the trained
one. (Built as a DLL rather than an `.exe` because this machine's Windows
Application Control policy blocks freshly compiled executables.)

## Step 2b — Toolchain notes (Keil + CubeMX + FlyMCU)

**STM32CubeMX** — open the **stock project's `.ioc`**, don't start a fresh
project (you'd lose the already-working AT8236/encoder/MPU6050 init). What
this policy needs, matching the pin map in §"Target hardware" above exactly
(an earlier revision of this table disagreed with that pin map on which
timers do what — this one is the one sourced from the actual firmware):

| peripheral | purpose | notes |
|---|---|---|
| `TIM3` / `TIM4` in **Encoder Mode** | left / right wheel angle + velocity | gives you 4 of the 34 inputs directly; the rest are derived from them and the IMU |
| `TIM8`, **PWM**, 4 channels | AT8236 drive, `PC6/7/8/9` | one timer, four channels — not two timers with direction GPIOs |
| `I2C2` | MPU6050 | **set to 400 kHz fast mode** if it is currently 100 kHz — see the budget note below |
| one spare timer (e.g. `TIM6`), **5 ms period**, update interrupt enabled | 200 Hz control tick | this one is new — the stock firmware ran its control loop at a different rate for a different controller; run the loop from this interrupt, not a delay |

Generate code. This regenerates init calls but leaves your existing working
drivers alone as long as you don't rename handles.

**Keil MDK** — the code is plain C99 with no dependencies, so it drops straight
in. Two settings matter:

- **Optimization `-O2`** (Options for Target → C/C++ → Level 2). At `-O0` the
  matrix loops are several times slower and the timing budget gets tight.
- **Increase the stack to at least 2 KB.** `policy_infer()` uses 512 bytes of
  local activations. The default in `startup_stm32f10x_hd.s` is often
  `Stack_Size EQU 0x00000400` (1 KB) — half of it consumed by one function
  invites a hard fault that looks random. Change it to `0x00000800`.
- There is **no FPU option to set** on the F103; ignore any guide that mentions
  one.

**FlyMCU** — flashes over the UART bootloader, so you do not need an ST-Link.
Nothing policy-specific here:

1. Set **BOOT0 high** (jumper/switch), then power-cycle or reset — this drops
   the chip into the bootloader instead of running whatever is already flashed.
2. Open FlyMCU, select the correct **COM port** and baud rate for your board's
   USB-serial adapter.
3. Browse to the `.hex` Keil produced (Step 3).
4. If your adapter supports it, check the DTR/RTS auto-reset boxes so FlyMCU
   can reset the chip into and out of the bootloader itself; otherwise you will
   toggle BOOT0/reset by hand before and after flashing.
5. Click **下载 (Download)**, wait for completion.
6. **BOOT0 back low**, reset — boots into the new firmware from flash.

### Watch the I2C budget

At 200 Hz you have **5 ms** per cycle, a quarter of what the earlier 80 Hz
plant budgeted, while inference on the F103 (Cortex-M3, no FPU) is still
roughly 4 ms with fast tanh (see Step 5 — the tanh call count didn't grow,
only the smaller matrix-multiply portion did). That leaves on the order of
1 ms for everything else: the sensor read, encoder processing, the history
and integral bookkeeping in `finish_obs()`, and the PWM update. This is
**tight, not comfortable**, and unlike the 80 Hz case there is little slack
to absorb a slow blocking I2C transfer.

A blocking MPU6050 read at 100 kHz costs about 1.5 ms for 14 bytes; at
400 kHz about 0.4 ms. Use 400 kHz, and prefer DMA or interrupt-driven reads
over blocking `HAL_I2C_Mem_Read()` so the transfer overlaps the rest of the
loop — at this budget it is closer to a requirement than a nicety.
**Measure the real loop time with a GPIO toggle and a scope** before trusting
any of these numbers; if it does not fit, `POLICY_FAST_TANH`'s rational
approximation is already the fast path; there is not much further headroom to
find without either a faster part or trimming the tick rate, and the latter
changes what the trained integrals mean (see Step 5).

## Step 3 — Add the files to your STM32 project

```
firmware/policy.c              inference + observation assembly
firmware/policy.h              API and robot constants
firmware/policy_weights.h      generated, do not edit
```

Requirements:
- `-lm` / libm
- Compile with `-O2`
- **Leave `POLICY_FAST_TANH` at its default of 1.** The F103 is a Cortex-M3
  with no FPU, so every float is a software library call, and the 128 `tanhf()`
  calls cost more than all the multiply-accumulates combined (see Step 5)
  because `tanhf` pulls in `expf`. The fast path replaces it with a rational
  approximation using only multiply, add and divide.

  Originally validated over 2000 random observations against the earlier
  torque policy: worst-case action error 0.0113 against a 0.6 N·m limit
  (1.9%), mean 0.0025, every commanded behaviour surviving a full episode in
  simulation. On run 8 the worst case is 0.020 of the ±1.0 action range on
  realistic inputs (mean 0.003). The check that matters is closed-loop: run 8
  through this fast-tanh build, driving the simulated robot, balanced and
  tracked the same as PyTorch on the nominal and worst-case cars. Set
  `POLICY_FAST_TANH=0` only on a target with an FPU.

There is **no FPU setting to enable** on this MCU — if you are following a
guide that mentions FPv4-SP-D16, that applies to Cortex-M4F parts, not the
F103.

## Step 4 — Build the observation each control cycle

The observation is **stateful**: encoder odometry over a 4-tick window, a
235 ms history buffer and two leaky integrals, all carried between calls (see
`policy.h`). Run 8 trains on exactly this computation, from emulated sensors,
and `check_obs_builder.py` (Step 2) proves the two agree. Call
`policy_reset_state()` when the controller arms, and again every time the
robot is re-armed after a fall — otherwise state carries across an event the
policy was never trained to see across.

```c
/* On arming (robot upright and still, motors off), and after every fall: */
policy_reset_state(pitch, gyro_xyz[1], POLICY_WHEEL_RADIUS * gyro_xyz[1] * cosf(pitch));

/* Then every control tick, starting the tick AFTER arming: */
float wheel_vel_l, wheel_vel_r, v_forward;
float obs[POLICY_N_OBS], action[POLICY_N_ACT];

policy_odom_update(dcount_l, dcount_r,     /* encoder counts since last tick, + = forward */
                   pitch, gyro_xyz[1],      /* rad, rad/s */
                   &wheel_vel_l, &wheel_vel_r, &v_forward);
policy_build_obs_rp(obs,
                    roll, pitch,            /* rad, from your gyro+accel fusion */
                    v_forward,
                    gyro_xyz,               /* rad/s, body frame: roll, pitch, yaw */
                    wheel_vel_l, wheel_vel_r,
                    cmd_forward, cmd_turn); /* your setpoints */

policy_infer(obs, action);
/* action[0] = left wheel PWM duty fraction in [-1, 1], action[1] = right -
 * pre-deadband-compensation. Feed each through policy_action_to_pwm() (Step 6)
 * before it goes anywhere near a timer register. This is NOT a torque - the
 * earlier torque-output policy this file used to describe is superseded. */
```

### Why roll/pitch rather than a full quaternion

The MPU6050 has no magnetometer, so roll and pitch are absolutely referenced
against gravity but **yaw is not observable and drifts without bound**.

Measured tolerance of the trained policy to a yaw error: fine at 10°, 30° and
90°, but it **fell after 138 steps at 180°**. Left to drift, the robot would
work for several minutes and then fall for no visible reason — a genuinely
nasty field failure.

`policy_build_obs_rp()` pins yaw to zero, removing the failure mode entirely.
It costs almost nothing because every other input is already body-referenced:
velocity tracking went 0.278 → 0.276 m/s and turn 0.401 → 0.429 rad/s.

### Encoders are required

Both wheel velocities come from the encoders, and forward velocity is derived
from them too. **The AT8236 is only a driver and provides no feedback.** If
the motors have no encoders, this policy cannot be deployed as trained.

### Five of the 34 inputs are constants

| index | input | value | why |
|---|---|---|---|
| 0 | chassis height | `0.0334` | not measurable; the axle sits one wheel radius up |
| 5, 6 | wheel angles | `0.0` | dropped in run 8 — physically meaningless, and run 7 had learned to depend on them |
| 8 | lateral velocity | `0.0` | a differential-drive robot has none by construction |
| 9 | vertical velocity | `0.0` | zero except during a fall |

Since run 8, training feeds exactly these constants too, so they are not
approximations any more — they are what the policy saw in every episode.
`finish_obs()` fills them. The remaining inputs are either sensed (attitude,
gyro, encoder speeds) or computed from sensed values inside `policy.c`
(forward speed, history taps, both leaky integrals).

### The input that matters most

First-layer weight magnitudes, run 8's checkpoint specifically, over the 29
inputs that are not constants (recompute per checkpoint — this shifts every
time the policy is retrained):

| input | mean \|weight\| |
|---|---|
| `yaw_err` (heading integral) | **0.63** |
| `hist_v_forward` @ 235 ms back | 0.51 |
| `cmd_turn` | 0.46 |
| `gyro_yaw` | 0.40 |
| `cmd_forward` | 0.39 |
| `quat_y` (pitch) | 0.36 |
| `hist_pitch` @ 10 ms / 25 ms back | 0.34 / 0.34 |
| the other 21, mean | 0.18 |

No single input dominates. The heading path (integral, yaw gyro, turn command)
is the largest group, so verify the gyro-z sign and bias calibration
end-to-end before closing the loop; pitch is spread across the current reading
and the history taps. Run 8 was trained with ±0.003 rad/s gyro bias and ±2°
pitch-zero error, so it tolerates ordinary calibration slop — at +2° pitch zero
it creeps at ~2 cm/s instead of holding perfectly still.

## Step 5 — Match the control rate

The policy was trained and evaluated at **200 Hz** (`POLICY_CONTROL_HZ`),
matching the stock firmware's MPU6050-interrupt loop — this is not a target
with headroom below it, it is the rate the leaky integrals' decay constants
and the history taps' tap spacing were both derived from (`policy.h`'s
`POLICY_POS_DECAY`/`POLICY_YAW_DECAY` are baked in *at* 200 Hz; running at any
other rate silently changes what those integrals mean without changing the
constants that define them). Run the loop from a hardware timer interrupt,
not a delay loop.

(An earlier PID-gain sweep on the superseded 80 Hz plant found controllability
fell off sharply below its own control rate — 91/120 gain sets stable at
80 Hz, 24/120 at 40 Hz, 0/120 at 25 Hz. That result is about a different plant
and a different controller and does not transfer here; it is not a substitute
for the point above.)

Timing estimate: roughly 6,400 multiply-accumulates (the hidden layers are
still 64 and 64 wide, so this only grew a little over the 17-input policy's
~5,300 despite the input layer nearly doubling) plus 128 `tanhf` calls — the
tanh count is unchanged, since it depends on hidden width, not input width.
On a Cortex-M4F expect roughly 0.2–0.5 ms, comfortably inside the **5 ms**
budget at 200 Hz (a quarter of the 80 Hz plant's old 12.5 ms). Note the
**tanh calls still dominate**, not the matrix maths — `POLICY_FAST_TANH`
(Step 3) is what actually buys the margin. Measure with a GPIO toggle and a
scope rather than trusting this estimate, and remember the sensor read (§2b's
I2C budget note) now has a tighter window to fit in too.

## Step 6 — Send the action to the motors

The policy outputs **PWM directly**, pre-deadband-compensation, in [-1, 1] —
not torque. This was a deliberate design change from the earlier policy: the
team has no motor Kt or winding resistance, so a torque-commanding policy
could not be converted to a motor drive at all. Commanding PWM removes both
unknowns; **you do not need the motor datasheet for this step**, unlike the
torque-output policy this document used to describe.

```c
int pwm_l = policy_action_to_pwm(action[0]);
int pwm_r = policy_action_to_pwm(action[1]);
```

`policy_action_to_pwm()` implements exactly what the simulator's
`motor_model.py:action_to_pwm()` did during training:
`pwm = clip(action, -1, 1) * POLICY_PWM_USABLE`, where `POLICY_PWM_USABLE`
(1500 counts, `policy.h`) is the span left over after `PWM_LIMIT` (2800) minus
`PWM_DEADBAND` (1300). That result is a **pre-compensation** command — feed it
into the stock firmware's own deadband-add step (`PWM_Ignore()` in the
reference firmware, §0 above) exactly as you would any other controller's
output, and drive the H-bridge from there. **Do not** add the deadband twice,
and do not route this through a Kt/R torque conversion — `policy_torque_to_duty()`
no longer exists in `policy.h` for this reason; it belongs to the superseded
policy and would silently double-convert a value that is already PWM.

**Keep `POLICY_PWM_DEADBAND` (and the stock `MOTOR_IGNORE_PULSE`) at 1300.**
It is the firmware's *compensation*, part of the action mapping the network
learned — not the motor's dead zone. (Earlier versions of this guide said to
re-tune it to your measured dead band; for run 8 that would be wrong.) The
motor's actual dead zone — ~1460 counts on this car, randomised over
1350–1600 in training — is handled by the policy itself. If a bench sweep puts
your motors' dead zone outside 1350–1600, update `MOTOR_DEAD_ZONE_RANGE` in
`motor_model.py` and retrain rather than touching this constant.

**Bench-verify before closing the loop anyway**: command a known PWM value,
confirm the wheel spins the expected direction at roughly the expected speed.
Left and right are physically handed (§0's pin-map note) — get this backwards
and the robot drives in a circle instead of straight, which looks like a much
more mysterious bug than a swapped sign.

## Step 7 — Safety layer before the first untethered run

The policy has no notion of failure. Add, outside the network:

1. **Tilt cutoff.** Kill the motors beyond ±0.70 rad (~40°, `POLICY_FALL_ANGLE_LIMIT`)
   — the fall threshold used in training, matching the firmware's own real
   cut-out. Past it the policy is extrapolating. (The earlier torque policy
   used ±0.4 rad / 23° here — do not reuse that number with this checkpoint.)
2. **Watchdog.** If a control cycle is missed, cut the motors. A stalled loop
   with PWM still applied is the dangerous failure.
3. **Current/thermal limit.** The policy will happily hold a saturated PWM
   command indefinitely if that is what balancing calls for.
4. **Startup gate.** Only enable once the IMU has converged and the robot is
   within a few degrees of upright — and call `policy_reset_state()` (Step 4)
   as part of this gate, not before it, so the history buffer primes on a
   genuinely settled reading.

## Step 8 — Putting it together: the control-loop ISR

Steps 4, 6 and 7 are three pieces of one function. This is the reference shape
— handle names (`TIM6`, `TIM8`) match the pin map above but may need adjusting
to whatever your CubeMX config actually generated:

```c
static volatile int g_policy_armed = 0;
static float g_cmd_forward = 0.0f, g_cmd_turn = 0.0f;   /* from your RC/app link */

/* Call once, after the IMU has converged and the robot is confirmed upright
   by hand - NOT blindly at power-on (Step 7, item 4). Call again after every
   fall recovery, once re-armed. */
void arm_policy(float pitch0, float pitch_rate0, float v_forward0)
{
    policy_reset_state(pitch0, pitch_rate0, v_forward0);
    g_policy_armed = 1;
}

void TIM6_IRQHandler(void)
{
    if (!(TIM6->SR & TIM_SR_UIF)) return;
    TIM6->SR &= ~TIM_SR_UIF;

    /* 1. Sensors - reuse your existing MPU6050 read + fusion + encoder code. */
    float roll, pitch;                 /* your fused estimate, radians */
    float gyro_xyz[3];                 /* rad/s: roll, pitch, yaw rate */
    long enc_l, enc_r;                 /* counts since last tick */
    /* ... fill these from your existing drivers ... */

    /* 2. Safety layer FIRST, unconditionally (Step 7, items 1 and 4). Items 2
     *    (watchdog) and 3 (current/thermal limit) are NOT shown here - they
     *    depend on hardware specifics outside this repo's scope and still
     *    need adding separately. */
    if (fabsf(roll) > POLICY_FALL_ANGLE_LIMIT || fabsf(pitch) > POLICY_FALL_ANGLE_LIMIT) {
        g_policy_armed = 0;
        TIM8->CCR1 = TIM8->CCR2 = TIM8->CCR3 = TIM8->CCR4 = 0;
        return;
    }
    if (!g_policy_armed) return;

    /* 3. Odometry, observation, infer (Step 4). */
    float wheel_vel_l, wheel_vel_r, v_forward;
    float obs[POLICY_N_OBS], action[POLICY_N_ACT];
    policy_odom_update(enc_l, enc_r, pitch, gyro_xyz[1], &wheel_vel_l, &wheel_vel_r, &v_forward);
    policy_build_obs_rp(obs, roll, pitch, v_forward, gyro_xyz,
                        wheel_vel_l, wheel_vel_r, g_cmd_forward, g_cmd_turn);
    policy_infer(obs, action);

    /* 4. PWM out (Step 6) - no torque conversion, this IS the conversion. */
    int pwm_l = PWM_Ignore(policy_action_to_pwm(action[0]));
    int pwm_r = PWM_Ignore(policy_action_to_pwm(action[1]));

    /* 5. Mirrored L/R convention (Target hardware, §"Both AT8236 inputs...")
     *    - copy exactly, do not "simplify". */
    if (pwm_l > 0) { TIM8->CCR2 = pwm_l; TIM8->CCR1 = 0; } else { TIM8->CCR1 = -pwm_l; TIM8->CCR2 = 0; }
    if (pwm_r > 0) { TIM8->CCR3 = pwm_r; TIM8->CCR4 = 0; } else { TIM8->CCR4 = -pwm_r; TIM8->CCR3 = 0; }
}
```

## Step 9 — First power-on: restrained, not free-standing

1. **Wheels off the ground.** Power up, arm the policy, watch debug-UART
   output of `action`/PWM values before any wheel touches a surface.
2. **Bench-verify PWM direction** (bypass the policy, command a known PWM
   value directly): confirm each wheel spins the expected way. Swapped L/R
   looks like a baffling balance bug, not an obviously wrong sign.
3. **Measure the real loop time with a GPIO toggle and a scope** (Step 5 /
   §"Watch the I2C budget"). This is the single biggest unverified risk in
   this whole document — if inference plus sensor read doesn't fit in 5 ms,
   the control loop falls behind and nothing else here matters.
4. Only then set it down, hand hovering to catch it.

## Known limitations to expect on hardware

Run 8, simulated on the corrected plant (dead-zone motor, 1-tick delay, encoder
odometry), 3 seeds × 10 s each, steady state after the first 2 s:

| behaviour | nominal car | worst-case car (dead zone 1600, 2-tick delay, motor −15 %, +2° pitch zero, noise) |
|---|---|---|
| Hold station | drift −0.001 m/s | −0.020 m/s |
| Forward +0.15 / +0.30 m/s | +0.136 / +0.267 (91 % / 89 %) | +0.126 at +0.15 |
| Reverse −0.15 m/s | −0.137 (91 %) | — |
| Turn +0.50 rad/s | +0.500 (100 %) | — |
| +0.15 m/s with +0.30 rad/s | +0.136 / +0.300 | — |
| Payload 1.0 kg | holds; +0.135 at +0.15 | — |
| Pitch zero ±2° | creeps at ∓0.020 m/s | — |

Forward tracking is ~90 % rather than run 6/7's ~100 % — the price of a motor
that delivers nothing in its dead zone. None of this is hardware-measured.
Friction sensitivity and slope have not been characterised for run 8.

### Slope sweep (run 7 — superseded policy, kept for the method)

**Caveat first, because it changes how to read every number below:**
`apply_slope()` (`enjoy_drive.py`) tilts gravity, not the floor — correct
physics in the slope's own frame, but the observation and reward are
world-referenced, which overstates difficulty. A previous policy's measured
drift at a nominal 5° was ~5x worse under this method than under a true
floor-tilt (`session-logs/2026-09-23-slope-run3-invalid.md`). Read the angles
below as a **stress-test ordering**, not a calibrated real-incline rating —
the real breaking point is almost certainly higher than 8°, just not
precisely known by how much without a true floor-tilt test.

| slope | hold: steps / v_fwd | +0.15 cmd: steps / v_fwd | +0.30 cmd: steps / v_fwd |
|---|---|---|---|
| 0° | 2000 / +0.002 | 2000 / +0.147 (98%) | 2000 / +0.270 (90%) |
| 2° | 2000 / −0.105 | 2000 / +0.064 | 2000 / +0.187 |
| 4° | 2000 / −0.213 | 2000 / −0.061 | 2000 / +0.107 |
| 6° | 2000 / −0.358 | 2000 / −0.177 | 2000 / +0.037 |
| **8°** | **838 / −0.565** (falls) | 2000 / −0.334 | 2000 / −0.135 |
| 10° | 291 / −0.545 (falls fast, high variance) | 973 / −0.520 (falls on 2/3 seeds) | 2000 / −0.300 |
| 12.5° | 115 / −0.518 (falls) | 145 / −0.549 (falls) | 732 / −0.512 (bimodal: 2/3 full, 1/3 falls @110) |
| 15–20° | <110 steps on every command | — | — |

Reading this: up to 6° the robot never falls, it just drifts downhill at a
roughly linear rate (~0.06 m/s per degree while holding station) and forward
commands get biased toward downhill — expected for a controller with no
integral action against a sustained external disturbance (the leaky
integrals exist for *tracking* error, not for rejecting a constant force).
**8° is the first angle where holding station alone fails** while driving
still doesn't — a commanded forward motion apparently gives the balance
controller more to work with than standing still against the same slope.
Above 15° nothing survives more than half a second regardless of command.

Not tested: friction sweep (same gap, not yet closed), and whether payload
interacts with slope tolerance (plausible it would, given payload raises the
COM and was shown elsewhere in this project to change the dynamics
materially — untested combination).

## If you would rather use ST's tooling

X-CUBE-AI (STM32Cube.AI) can ingest an ONNX export instead. It is the right
choice for large convolutional models, but for 6,530 parameters it adds a code
generator, a runtime library and a build dependency to replace roughly 40 lines
of C that already verify bit-exact. The hand-written path is recommended here.

Either way, ONNX only gets you the network (`policy_infer()`'s job) — you
would still need to hand-write the history buffer and both leaky integrals
(`finish_obs()` in `policy.c`) yourself, since none of that is part of the
exported graph. This is not a shortcut around Step 4.

If you do want ONNX:

```python
import torch
from stable_baselines3 import PPO
m = PPO.load("models/best_real/best_model.zip", device="cpu")
torch.onnx.export(m.policy, torch.zeros(1, 34), "policy.onnx",
                  input_names=["obs"], output_names=["action"], opset_version=11)
```

Note this exports the full policy including the value head; prune it in the
Cube.AI import step or the value network wastes flash.
