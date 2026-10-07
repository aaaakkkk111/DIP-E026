# Deploying the policy to STM32 — step by step

Written for: whoever does the firmware port.

**Updated for run 7** (`train_real_robot.py`, `real_robot.xml`): 34 inputs, PWM
output, 200 Hz control. If you are looking at an older checkout of this file,
it described a 17-input, torque-output, 80 Hz policy on a different (wrong,
by 3.5x COM height and 12x inertia) plant — that policy is superseded and
`policy.c` no longer implements it.

The trained network is 34 → 64 → 64 → 2 with tanh activations: **6,530
parameters, 25.5 KB of flash, 512 bytes of RAM** (re-run `export_stm32.py` and
read its own printed summary if you export a different checkpoint — these
numbers are checkpoint-specific, not fixed). Against a 256 KB / 48 KB budget
the network is a non-issue. The work is in the observation pipeline (now
stateful — a 235 ms history buffer plus two leaky integrals) and the safety
layer, not the maths.

## Readiness check before you start

**This is ready for a careful, instrumented first bench test — not for a
confident unsupervised deployment.** Nothing about the policy's simulated
behaviour is in question (it converged cleanly, a specific known defect was
measured and fixed, and the firmware port is bit-verified against it — see
Step 2). What's still open is entirely on the hardware-integration side, and
none of it is fixed by retraining:

| gap | why more training doesn't help |
|---|---|
| Zero real-hardware runs so far | sim-to-real gap is unverified until the first physical test |
| The 200 Hz / 5 ms timing budget on the actual F103 is an *estimate* (§"Watch the I2C budget"), not measured | firmware/MCU question, not a policy question |
| `PWM_DEADBAND`/motor constants are per-build, measured on a different unit than yours | needs a bench measurement on your specific motors |
| Friction sensitivity was never characterised for this plant (slope now has been - see "Known limitations" below) | sim characterisation gap, same caveat: a sim sweep still isn't a hardware measurement |
| The 180°-yaw-drift fall mode was only measured on the superseded torque policy | needs re-verification, but a measurement, not training |
| Your sensor-fusion roll/pitch code (outside this repo) hasn't been checked against what the policy expects | integration correctness, not policy quality |

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

### Deadband is compensated in firmware, not learned

```c
#define MOTOR_IGNORE_PULSE (1300)
int PWM_Ignore(int pulse) {
    if (pulse > 0) return pulse + MOTOR_IGNORE_PULSE;
    if (pulse < 0) return pulse - MOTOR_IGNORE_PULSE;
    return pulse;
}
```

The offset is added **after** the controller, so a policy's output is a
pre-compensation command. Note this build uses **1300** while the parameter
sheet quotes 1480 forward / 1455 reverse — the value is per-build and was
re-measured. Use whichever matches the firmware you actually flash.

### Confirmed parameters for this build

```c
#define GEAR_RATIO      30.0f    /* inferred from 333 RPM @ 12V - CONFIRM on the gearbox */
#define ENCODER_PPR     11.0f    /* typical for JGB37-520 - CONFIRM, some are 13 */
#define ENCODER_QUAD    4.0f     /* TIM_ENCODERMODE_TI12 */
```

Still needed, for `policy_counts_to_rad()` / `policy_odometry()`. **No longer
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
tolerance: 2.0e-02
...
worst error: 1.156e-02   (tolerance 2.0e-02)
PASS
```

Add `-DPOLICY_FAST_TANH=0` to check the matrix maths in isolation from the
tanh approximation — expect worst error down around 1e-7 (float32 rounding).
Anything above ~1e-5 on that build means a real porting bug, almost always
row/column-major confusion in the matrix loops.

**This only checks `policy_infer()` — the network — on isolated vectors.** It
does not exercise `policy_build_obs()`'s history buffer or leaky integrals,
since those depend on a *sequence* of calls, not one observation at a time.
Check that separately:

```bash
gcc -O2 -o test_obs_builder test_obs_builder.c policy.c -lm
./test_obs_builder > obs_c.txt
```

then run the Python reference in the comment at the bottom of
`test_obs_builder.c` (it imports the real decay/scale constants from
`train_real_robot.py` rather than hand-copying them, so it can't silently
drift the way a hand-copied reference could) and diff the two outputs.
Expect agreement to within ~1e-7 — pure float32-vs-float64 rounding, not
logic. This is what actually caught (in the sense of would have caught, had
it existed sooner) the kind of bug a tap-index or decay-constant typo
produces: `policy_infer()` alone cannot see it, because a wrong observation
still produces *a* plausible-looking action, just not the trained one.

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
  simulation. Re-checked against the current run-7 checkpoint's own test
  vectors (`test_policy.c`): worst case 1.16e-02 against a +/-1.0 action
  limit, still comfortably under the 2% bound. Set `POLICY_FAST_TANH=0` only
  on a target with an FPU.

There is **no FPU setting to enable** on this MCU — if you are following a
guide that mentions FPv4-SP-D16, that applies to Cortex-M4F parts, not the
F103.

## Step 4 — Build the observation each control cycle

The observation is now **stateful**: `policy_build_obs_rp()` carries a 235 ms
history buffer and two leaky integrals between calls (see `policy.h`'s
comment above `policy_reset_state()` for why). Call `policy_reset_state()`
once at controller start-up, and again every time the robot is re-armed after
a fall — otherwise the integrals and history buffer carry state across an
event they were never trained to see across.

```c
/* Once, before the control loop starts (and again after every fall recovery,
 * once the robot is back upright and re-armed): */
policy_reset_state(pitch, gyro_xyz[1], policy_odometry(enc_vel_l, enc_vel_r));

/* Then every control tick, with an MPU6050 use the roll/pitch entry point: */
float obs[POLICY_N_OBS], action[POLICY_N_ACT];

policy_build_obs_rp(obs,
                    roll, pitch,       /* rad, from your gyro+accel fusion */
                    enc_angle_l, enc_angle_r,               /* rad, accumulated */
                    policy_odometry(enc_vel_l, enc_vel_r),  /* m/s */
                    gyro_xyz,          /* rad/s, body frame */
                    enc_vel_l, enc_vel_r,                   /* rad/s */
                    cmd_forward, cmd_turn);                 /* your setpoints */

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

Four inputs — both wheel angles and both wheel velocities — come from
encoders, and forward velocity is derived from them too. **The AT8236 is only
a driver and provides no feedback.** If the motors have no encoders, this
policy cannot be deployed as trained; it would need retraining with an
observation restricted to what the IMU alone can supply.

### Three of the 34 inputs the hardware cannot measure

| index | input | substitute | why it is safe |
|---|---|---|---|
| 0 | chassis height | `0.05` (nominal) | simulator-only quantity; near-constant while upright |
| 8 | lateral velocity | `0.0` | a differential-drive robot has none by construction |
| 9 | vertical velocity | `0.0` | zero except during a fall |

`policy_build_obs()` already fills these (measured on the earlier torque
policy: substituting the constants moved velocity tracking from +0.278 to
+0.276 m/s with every command still holding a full episode — not re-measured
against run 7 specifically, but the same three quantities, same substitution).

The remaining 17 of the 34 inputs (history taps, both leaky integrals) are not
"unmeasurable" in the same sense — they are computed *from* the other sensor
readings, inside `policy_build_obs()`/`policy_build_obs_rp()` themselves. See
`policy.c`'s `finish_obs()`.

### The input that matters most

First-layer weight magnitudes, run 7's checkpoint specifically (recompute per
checkpoint — this is not architectural, it will shift every time the policy is
retrained):

| input | mean \|weight\| |
|---|---|
| `yaw_err` (heading integral) | **1.08** |
| `hist_pitch` @ 10 ms back | 0.65 |
| `quat_y` (pitch) | 0.63 |
| `hist_pitch` @ 25 ms back | 0.61 |
| `hist_pitch` @ 55 ms back | 0.50 |
| `cmd_turn` | 0.50 |
| everything else, mean | 0.18 |

Pitch is still important, but it no longer dominates 13x the way it did on
the earlier 17-input policy — the heading integral now weighs more than raw
pitch does, and three of the top six entries are history taps, not
instantaneous readings. Get the sensor-fusion filter right, but also verify
the heading-integral bookkeeping (Step 4) end-to-end before closing the loop;
on this checkpoint it is not a minor term.

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

`POLICY_PWM_LIMIT`/`POLICY_PWM_DEADBAND` are per-build, same caveat as
`PWM_Ignore()` itself (§0): the parameter sheet quotes 1480 fwd / 1455 rev for
a different unit than the 1300 used in simulation. Re-measure your own build's
deadband and override the macro if it differs; a mismatch here does not cause
a crash, it just means the policy's dithering assumption (the smallest
non-zero command is 64% of the driver limit) is calibrated against the wrong
number.

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

    static float wheel_angle_l = 0.0f, wheel_angle_r = 0.0f;
    float dth_l = policy_counts_to_rad(enc_l, ENCODER_PPR, GEAR_RATIO, ENCODER_QUAD);
    float dth_r = policy_counts_to_rad(enc_r, ENCODER_PPR, GEAR_RATIO, ENCODER_QUAD);
    wheel_angle_l += dth_l;  wheel_angle_r += dth_r;
    float wheel_vel_l = dth_l * POLICY_CONTROL_HZ, wheel_vel_r = dth_r * POLICY_CONTROL_HZ;
    float v_forward = policy_odometry(wheel_vel_l, wheel_vel_r);

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

    /* 3. Build observation, infer (Step 4). */
    float obs[POLICY_N_OBS], action[POLICY_N_ACT];
    policy_build_obs_rp(obs, roll, pitch, wheel_angle_l, wheel_angle_r, v_forward,
                        gyro_xyz, wheel_vel_l, wheel_vel_r, g_cmd_forward, g_cmd_turn);
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

The table below is from the superseded 17-input torque policy on the earlier
plant and no longer applies — left only so a stale table isn't silently
deleted without a pointer to where the current numbers actually live. For
run 7, in simulation:

| behaviour | status | source |
|---|---|---|
| Balancing, idle | station-keeping drift ~0.001 m/s, heading drift bounded to ~0.35° over 60 s | `session-logs/2026-09-25-run7-yaw-integral.md` |
| Forward to ±0.3 m/s | tracks 101–104% of command (run 6, unchanged by run 7's yaw-only change) | `session-logs/2026-09-24-...md`, run 6 section |
| Turning to ±0.5 rad/s | tracks ~96–100% of command (was 105–112% overshoot before the heading integral) | `2026-09-25-run7-yaw-integral.md` |
| Payload to 1.0 kg | flat and accurate across the whole range (run 6) | same |
| Slopes | **characterised below** — holds to ~6°, degrades gracefully, breaks down above ~15° | this session, see below |
| Friction changes | not re-characterised on this plant | — |

None of this is hardware-measured — it is simulation only, same caveat as
everything else in this document.

### Slope sweep (run 7, `models/best_real/best_model.zip`, unloaded, 3 seeds)

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
