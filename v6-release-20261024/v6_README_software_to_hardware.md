# Balance Car Adaptive Control Model v6: A Complete Guide from Software Design to Hardware Flashing
# 平衡小车自适应控制模型 v6：从软件设计到硬件烧录的完整说明

Version date / 版本日期: 2026-10-24
Firmware / 对应固件: `stm32_Balance_Car_L_v6.hex` (this folder / 本文件夹)
SHA-256: `355e30cd001edd9612df92b7494ff9a5958eaf2fb26d2289c9761eda72184532`
Image end / 镜像结束地址: 0x0800 E5C4 (6716 bytes free below 64 KB / 64 KB 内剩余 6716 字节)
Base / 基于: v4 release `v4_release_20261003` → v5 `v5_20261021` → v5.2 real-car test 1–4 / v5.2 实车测试 1–4

> **v6 in one paragraph.** v6 keeps v4's idea: the factory cascade PID balances the car, and a small explainable controller chooses, every 5 ms, a gear `f` between the factory Normal and Weight gains. On top of that it adds:
> - large-tilt recovery with a 45° cut-off;
> - a bounded get-up from the floor, with the car geometry taken from the STEP CAD model;
> - a protection lock;
> - **frequency-based gear shifting**: low-frequency sway (a high load on the top plate) shifts up, high-frequency chatter (gains too stiff) shifts down quickly in three layers, and a calm car does not shift.
>
> The last four changes were made directly from real-car tests on 2026-10-24.
>
> **v6 一段话概括。** v6 沿用 v4 的思路：平衡由原厂串级 PID 负责，一个可解释的小控制器每 5 ms 在原厂「正常档」和「负重档」两套增益之间选一个档位 `f`。在此之上新增：
> - 45° 切电机的大角度回正；
> - 有时间和次数上限的倒地自起，车的几何以 STEP 三维模型为准；
> - 保护锁；
> - **按频率换档**：低频摆动（负重放在顶板上、重心高）升档，高频抖动（增益太硬）分三层快速降档，车稳时不换档。
>
> 最后四项改动是 2026-10-24 根据实车测试直接做出的。

**How this document is organised / 本文结构**: Part I is the complete document in English; Part II is the same document in Chinese, with the same section numbers. 第一部分是完整的英文说明；第二部分是同一份说明的中文版，节号相同。

## Contents / 目录

| § | Part I — English | 第二部分 — 中文 |
|---|---|---|
| 0 | What is in this folder, how to flash and use it | 文件夹里有什么，怎么烧录、怎么用 |
| 1 | What v6 adds to v4, and why | v6 在 v4 基础上加了什么、为什么 |
| 2 | Glossary | 术语表 |
| 3 | Architecture: what runs every 5 ms | 架构：每 5 ms 运行什么 |
| 4 | Feature 1 — automatic gear shifting | 功能一：自动换档 |
| 5 | Feature 2 — large-tilt recovery and protection | 功能二：大角度回正和保护 |
| 6 | Feature 3 — get-up from the floor | 功能三：倒地自起 |
| 7 | Feature 4 — motor compensation module | 功能四：电机补偿模块 |
| 8 | Flash budget | Flash 预算 |
| 9 | The digital twin and the STEP geometry | 数字孪生和 STEP 几何 |
| 10 | Source walkthrough | 源码说明 |
| 11 | Serial commands | 串口命令 |
| 12 | Results (with factory baselines) | 结果（带原厂基线） |
| 13 | Development history 10-03 → 10-24 | 开发过程 10-03 → 10-24 |
| 14 | Known limitations and real-car calibration | 已知局限和实车标定 |
| 15 | How to rebuild | 怎么重新编译 |
| 16 | File locations | 相关文件位置 |
| 17 | Division of work | 分工 |

---

# Part I — English

## 0. What is in this folder, and how to flash and use it

### 0.1 File list

| File / folder | Contents |
|---|---|
| `stm32_Balance_Car_L_v6.hex` | **The firmware to flash.** It is built from the complete Keil project (`e026 keil\v4_work_20261003`). Only mode 27 is left in it, and the screen shows "27.Adapt V6". Apart from that screen label, the code is the same as the real-car test build `v5_2_test4` (2026-10-24). |
| `v6_README_software_to_hardware.md` | This document. |
| `CAR_TEST_PLAN.md` | Step-by-step real-car checks for v6, and a quick reference of the serial commands. |
| `source_snapshot/core/` | The pure-logic modules, which never touch hardware: `v4_core.*` (gear decisions), `rec_core.*` (large-tilt recovery, protection lock, get-up from the floor), `mc_core.*` (motor dead-band compensation, off by default). Also the glue that connects them to the factory firmware: `v4_adapt.*` and `mc_adapt.*`. |
| `source_snapshot/firmware_hooks/` | The factory-firmware files that were changed to call the modules, plus the Keil project file: `app_control.c`, `app_motor.c`, `app_mode.c`, `oled_show.c`, `main.c`, `pid_control.c`, `tune_io.c/.h`, `removed_modes_v5_stubs.c`, `stm32_Balance_Car.uvprojx`. |
| `source_snapshot/twin_scripts/` | The digital-twin scripts used to design and check v6, and the C wrappers that compile the **same** firmware C files into a Windows DLL for the twin (`v51_dll.c`, `rec_dll.c`, `v4rel_dll.c`). `step_geometry.py` reads the STEP CAD model; `stm32_step_geom.py` is its output. |
| `twin_results/` | The logs that the result tables in Section 12 are taken from. |
| `previous_builds/` | Earlier hex files, kept for A/B comparison: v4 release, dead-band test, recovery test, v5, v5.1, and v5.2 test 1–4. |
| `blog_2026-10-10_to_10-24.md` | Two-week progress summary (English, then Chinese). |

### 0.2 Flashing and use

1. Flash `stm32_Balance_Car_L_v6.hex` into the STM32F103RCT6 with your usual tool (ST-Link, a serial download tool, etc.). The hex starts at 0x0800 0000, like the factory firmware. The image ends at 0x0800 E5C4, so 6716 bytes of the reliable 64 KB remain free. (This board only works reliably below 0x0801 0000. An image past that line does not run at all, not even mode 1.)
2. Power on. The screen shows **"27.Adapt V6"**. Turning a wheel no longer changes the mode: every other mode was removed to free flash (Section 8). Press **KEY1** to confirm.
3. The screen shows "put down key start!". Put the car on the ground and press **KEY1** again. The car starts balancing. Starting it upright is the normal case. If you start it while it is lying on the floor, it will try to get up by itself (Section 6).
4. Drive it with the Bluetooth phone app as usual. Bluetooth, the app protocol and the serial console are all kept. There is no "Normal / Weight" mode to choose: v6 judges the load and changes gears by itself.
5. Line 3 of the screen shows the gear state as `f0.00L osc 1.2 u0`:
   - `f` is the current gear (0 = factory Normal gains, 1 = factory Weight gains);
   - `L` means latched (a gear has been chosen) and `P` means still probing;
   - `osc` is the slow 8–16 Hz chatter meter in deg/s;
   - `u` counts how many times a large tilt has sent the gear back to 1 to re-probe.
6. Optional: open the serial console (115200 baud) and send `?` to read the full state. Section 11 lists all commands. Every command in this document is something you send yourself; nothing is sent automatically.

---

## 1. What v6 adds to v4, and why

v4 (2026-10-03) was a gain-shifting controller. It sits on top of the factory cascade PID and chooses, every 5 ms, a gear `f` between the two factory gain sets. In the twin it scored 31.83 against 111.45 / 115.41 for the factory Normal / Weight gains (lower is better). Over the following three weeks the real car showed what it still could not do. v6 is the answer to each item:

| Real-car problem (date found) | Cause found | What v6 does |
|---|---|---|
| A car leaning at 40° could not get back up; the motors simply stopped (10-20) | The firmware cut the motors at 40°. The motors themselves could still lift the car. | Cut-off moved to 45°. A "recovery" state holds the speed-loop integral at zero while the car lifts itself (Section 5). |
| A car that fell over just lay there, sometimes with the wheels still spinning (10-21) | The factory firmware had no get-up and no lock. | A bounded push–coast–push get-up from the floor, and a lock that switches the motors off when the car is stuck at a large tilt (Sections 5–6). |
| The car's real shape was guessed as a box (10-21) | — | All geometry now comes from the STEP CAD file. The body first touches the floor at 78.5°, not 44.5° (Section 9). |
| With a small load the car shook more and more, calmed, then shook again, and sometimes fell (10-21) | Several gear rules fought each other (v5.1 fixes) | v5.1 rules (bits 2 and 16 of `v4_opt`) are kept. |
| With a load on the top plate the body swayed visibly a few times a second and the sway grew until it fell (10-24) | The load's centre of mass sits high, 0.15–0.25 m above the axle. The gear the probe picked (0.4) is too soft for such a high load. The old gear rules only reacted to 8–16 Hz chatter, so they could not see a 1–2 Hz sway. | **Shift by frequency**: a low-frequency sway (0.5–6 Hz) that is big or growing shifts UP; 8–16 Hz chatter shifts DOWN; a calm car does not shift (Section 4.6–4.7). |
| Empty car: chatter would not go away; the gear hung at 0.1, or the car chattered at 0.0 (10-24, test 1) | Real chatter leaked into the new sway meter, so the sway rule shifted up while the chatter rule shifted down. A ceiling bug let the sway rule undo each downshift. | Two-pole sway filter. No sway upshift while there is any chatter. The ceiling after a downshift equals the new gear. A bottom sweep sends the gear straight to 0 (test 2). |
| Load taken off: the gear stuck at 0.2 while chattering (10-24, test 2) | Taking the load off counted as an "event". The event state could not end, because the balance angle had moved with the load and the chatter kept the raw rate above 60°/s. The downshift was not allowed during that state. | Downshift works in every state. The event state ends on a low-passed rate, or after 3 s, and then re-centres its reference angle (test 3). |
| Small-amplitude high-frequency oscillation was not removed (10-24) | Below 15 deg/s nothing reacted above gear 0.15 | A new small-chatter rule: ≥ 7.5 deg/s (serial `ad`) for 0.3 s → down 0.1, at any gear (test 4 = v6). |

The design rules did not change from v4. The decision logic is plain, explainable C that never touches hardware. Every rule exists because a measured problem required it. Every threshold can be changed over the serial port without reflashing.

---

## 2. Glossary

| Term | Meaning |
|---|---|
| **Gear `f`** | A number from 0 to 1. The six PID gains are a straight-line mix of the factory Normal set (f = 0) and the factory Weight set (f = 1): `gain = GN + (GH − GN) · f`. |
| **GN / GH** | Factory Normal / Weight gains, in firmware units (×100): balance Kp 9600 / 19200, balance Kd 48 / 150, speed Kp 6200 / 9450, speed Ki 31 / 47.25, turn Kp 1700 / 1400, turn Kd 20 / 20. |
| **Probe** | After boot or after a large-tilt event the gear jumps to f = 1 and the car "listens" to how much it chatters there. An empty car chatters strongly on the stiff gains; a loaded car does not. The probe ends in a **latch**: 0 (light), 0.2 (middle) or 0.4 (heavy). |
| **Event / `rec` (v4 recovery state)** | A tilt (predicted 0.1 s ahead) bigger than the threshold: 3–8° standing, 16° driving. The gear jumps to 1 and re-probes. Do not confuse this with rec_core's large-tilt recovery (Section 5). |
| **Chatter meter `osc` / `oscf`** | RMS of the gyro rate in the 8–16 Hz band, in deg/s. `osc` averages over about 0.3 s, `oscf` over about 0.1 s. High-frequency chatter is the direct sign that the gains are too stiff for the load. |
| **Sway meter `sw`** | RMS of the tilt angle in the 0.5–6 Hz band, in degrees, averaged over about 0.5 s. `sw_old` is its value 0.5 s earlier. A growing sway is the sign that the gains are too soft. |
| **Reversal rate `swhz`** | How often the 0.5–6 Hz tilt changes sign (with 0.3° hysteresis), in Hz. A push or a brake reverses once; a sway keeps reversing. |
| **Floor / ceiling** | The floor is the lowest gear a re-probe may latch, remembered after a sway upshift. The ceiling is the highest gear allowed for 10 s after a chatter downshift. |
| **Boost (slow escape)** | A temporary addition to `f` while the car leans for a long time. It decays with a 1.5 s time constant. |
| **Dead band** | The PWM below which the motor does not turn: 1480 forward and 1455 reverse on this car. The firmware adds it to every non-zero command. |
| **Cut-off** | The tilt beyond which the factory code switches the motors off: 40° in the factory firmware, 45° in v6 mode 27. |
| **Lock** | rec_core's protection: tilt above 40° for 1 s outside a get-up attempt → motors off until the tilt is below 10°. |
| **Digital twin** | A MuJoCo simulation of this car (motor model with dead band and gear backlash, IMU noise, STEP geometry), in `e026\balance_bot_windows\balance_bot`. It runs the firmware's own C code through a DLL. |
| **STEP model** | The manufacturer's 3D CAD file `STM32_Balance_V2.STEP`. Since 2026-10-21 it is the authority for the car's shape and collision. |
| **`v4_opt`** | A bit field that switches gear rules on or off (serial `ax`). v6 default is 50 = bits 2 + 16 + 32. |

---

## 3. Architecture: what runs every 5 ms

The factory firmware runs its control loop in the MPU6050 data-ready interrupt at 200 Hz (`APP/app_control.c`). v6 adds three calls to that loop and keeps everything else:

```
read IMU  ->  Angle_Balance (deg), Gyro_Balance (raw, 16.4 LSB per deg/s)
read encoders  ->  Encoder_Left, Encoder_Right
 |
 |- V4_Tick(enc_l, enc_r)                         [v4_adapt.c]  (mode 27 only)
 |     |- REC_Step(...)      large-tilt recovery / lock / get-up      [rec_core.c]
 |     |- Turn_Off(...) == 1 ?  -> motors are off: hold v4 at its boot state, return
 |     |- V4_Step(angle, gyro, moving)  -> gear f                     [v4_core.c]
 |     \- apply(f): write the 6 PID gains (bumpless on the speed integral)
 |
 |- Balance_PD  = Kp/100 * (angle - Mid_Angle) + Kd/100 * gyro      [factory]
 |- Velocity_PI = -bias*Vkp/100 - Encoder_Integral*Vki/100           [factory]
 |- Turn_PD                                                          [factory]
 |- Motor = Balance + Velocity -/+ Turn, limited to +-2800
 |- V4_RecOverride(&L, &R)   recovery: speed integral := 0; get-up: write the push
 |- MC_FwOutput(&L, &R)      dead band: factory 1480/1455 (compensation off by default)
 \- PWM out (or 0 if Turn_Off)
```

The layers stay strictly apart:

- **The factory PID does the balancing.** v6 never computes a motor command itself, except during a bounded get-up push.
- **`v4_core` decides the gains only.** It sees three inputs: the tilt angle, the gyro rate, and whether a drive command is non-zero. It has no other access to the hardware.
- **`rec_core` handles the extremes.** These are the large tilt, the floor, and the stuck car.
- **`mc_core` sits on the motor output.** It is off unless the serial command `a1` turns it on.

Because the cores are pure C with no hardware access, the twin compiles the very same `.c` files into a DLL. A twin result is therefore a result for the code that is on the car, not for a Python copy of it.

---

## 4. Feature 1 — automatic gear shifting (`v4_core.c`)

### 4.1 The gear ladder

`V4_GainsFor(f)` mixes the two factory sets in a straight line. `apply()` in `v4_adapt.c` writes the six gains into the factory variables every tick. The speed loop outputs `−Encoder_Integral · Ki/100`, so changing Ki alone would make the motor command jump by up to about 1280 PWM. To avoid that, the integral is rescaled by `Ki_old / Ki_new` whenever Ki changes. This keeps the change bumpless.

Why stay between the two factory sets? Both are known to work on this car: Normal for an empty car, Weight for a heavy one. Any point in between is a smooth mix of two stable controllers. Going outside the range would need new tuning on the real car, and the twin under-reads chatter (Section 9).

### 4.2 The three meters

Every tick `V4_Step` updates three meters from the IMU:

1. **Chatter (8–16 Hz), from the gyro rate.** An 8 Hz one-pole high-pass, then a 16 Hz one-pole low-pass, then the energy is averaged. Two averages are kept:
   - `osc = sqrt(EMA_0.3s(bp²)) / 16.4`: slow and steady. The probe and the screen use it.
   - `oscf = sqrt(EMA_0.1s(bp²)) / 16.4`: fast. The downshift uses it, so it reacts in about 0.15 s and stops stepping as soon as the chatter dies.
2. **Sway (0.5–6 Hz), from the tilt angle.** A 0.5 Hz one-pole high-pass, then **two** 6 Hz one-pole low-passes in series, then the energy is averaged over about 0.5 s: `sw = sqrt(EMA(swl²))`. A value is stored every 0.5 s as `sw_old`. The second low-pass pole was added after real-car test 1. The real chatter (about 54 deg/s at about 14 Hz, which is about 0.6° of tilt) leaked through a single pole. Two poles cut 14 Hz to about 1/7 in amplitude.
3. **Reversal rate `swhz`.** It counts the sign changes of the 0.5–6 Hz tilt, with 0.3° hysteresis, averaged over about 0.5 s. It tells a sway, which keeps reversing, from a single push or brake.

### 4.3 Probe and latch (inherited from v4/v5, unchanged in v6)

- The car boots on f = 1, the safe end for a heavy load, and listens.
- After 1.2–3 s standing, once the `osc` reading has settled, it latches:
  - `osc ≥ 25` (`ak`) → light, f = 0;
  - `osc ≥ 20` (`aj`) → middle, f = 0.2;
  - otherwise → heavy, f = 0.4.
- After an event the whole 0.3 s window must agree before the car is judged light ("light only when sure"). A light car judged heavy only chatters; a heavy car judged light can fall.
- **Fast light:** if every reading in the last 0.3 s is past the light split, the car latches light at once instead of waiting.
- **Probe abort** (bit 2): 8–16 Hz chatter ≥ 15 for 0.2 s on f = 1 ends the probe early, so the car does not chatter for 3 s. In v6 this is skipped while the car is swaying (`sw ≥ 1.5`). A big sway leaks into the 8–16 Hz meter, and in the twin it was read as chatter.
- **Floor:** a probe never latches below the remembered sway floor, except when the car is judged empty.
- After a latch the downshift waits 0.2 s, because the chatter meter still holds the f = 1 reading.

### 4.4 Upshift on a large tilt (inherited from v4)

- **Prediction.** `th_pred = (angle − th_ref) + 0.1 s · rate`. Here `th_ref` is the slowly learned balance angle.
- **Thresholds.** A tilt past the threshold sends the gear to f = 1 and starts a re-probe:
  - standing: 3° by default, rising with the car's own noise level (`6 · σ`, capped at 8°);
  - driving: 16°.
- **What prevents false upshifts:**
  - a 1.5 s grace window after any key press, where the rate term is ignored;
  - a stop must settle before the 3° standing threshold applies;
  - a re-arm is needed after each event;
  - a quick revert: if the peak in the 0.3 s after an upshift stays under 6°, the old gear comes back without a re-probe.
- **Chatter is not a push.** While chattering (`osc ≥ 15`) a tilt under 8° does not upshift.

### 4.5 Downshift on high-frequency chatter (v6: three layers, every state)

High-frequency chatter is the direct sign that the gains are too stiff. It means the derivative gain amplifies sensor noise and backlash at the loop's own resonance. The cure is a softer gear, as fast as possible.

| Fast meter `oscf` (deg/s) | Holds for | Step down |
|---|---|---|
| ≥ 30 | 0.15 s | 0.3 |
| 22.5 – 30 | 0.15 s | 0.2 |
| 15 – 22.5 | 0.15 s | 0.1 |
| **7.5 (`ad`) – 15** | **0.3 s** | **0.1** (small-chatter rule, new in v6) |
| ≥ 7.5 while f ≤ 0.15 | 0.5 s | straight to 0 (bottom sweep) |

- After each step the rule waits 0.2 s for the meter to catch up with the new gear, then looks again.
- The temporary boost is cleared on each step.
- The timer restarts whenever `oscf` drops under 7.5.
- **In every state.** Up to test 2 the downshift was gated by "not in the event state". That gate caused the stuck-at-0.2 bug, so v6 removes it. Chatter is chatter whatever the state.
- **Not while swaying hard.** At `sw ≥ 2 × 1.5 = 3°` RMS the 8–16 Hz reading comes from the sway itself, so the downshift does not count it.
- Why 7.5? On the real car an empty car at f = 0 reads 4.5–7.4 when calm, and 54–58 when the gains are too stiff (records of 2026-10-03). 7.5 sits just above the calm level. It is a serial setting because loaded cars have not been measured yet.

### 4.6 Upshift on low-frequency sway (new in v5.2/v6, bit 32)

A load on the top plate puts its centre of mass 0.15–0.25 m above the axle. In the twin, a gear that is too soft for such a load does not chatter. Instead it sways at about 1–2 Hz, and the sway grows until the car falls (`sway_probe.log`: f 0–0.4 sway and fall; f 0.7–1 hold). This matches what the user saw on the real car.

**Conditions:**

- `f_latched < 1`.
- **No chatter at all:** `osc < 15` and `oscf < 7.5` (the same `ad` threshold as the small-chatter rule). This makes the two rules disjoint.
- **Reversing:** `swhz ≥ 1 Hz`, so it is a sway and not a lean.
- **Swaying,** one of:
  - big and not dying out: `sw ≥ 1.5°` (`au`) and `sw ≥ 0.9 · sw_old`;
  - small but growing: `sw ≥ 0.75°` and `sw ≥ 1.2 · sw_old`.
- **When,** one of:
  - standing (no drive command for 1 s);
  - in the 3 s after a probe latch, because the latched gear may be too soft;
  - driving with a big and growing sway (`sw ≥ 1.5` and `sw ≥ sw_old`).
- **Not blocked:**
  - not within 1 s of a chatter downshift;
  - not above the ceiling (unless `sw ≥ 3°`).

**Action:**

- After 0.1 s of all conditions, `f_latched` goes up by:
  - 0.1 for a small sway;
  - 0.2 when `sw ≥ 1.5`;
  - 0.3 when `sw ≥ 3`.
- Then the rule waits 0.3 s.
- The new gear is remembered as the floor, so a later re-probe cannot drop below it.

**Event while swaying:** if the car was already swaying when a large-tilt event fires, the gear that failed was too soft. The floor becomes that gear + 0.3.

**Why not simply always use f = 1?** f = 1 makes an empty or light car chatter hard (the real car reads 54–58 deg/s). The sway rule climbs only as high as the sway requires.

### 4.7 How the up and down rules avoid fighting

| Situation | Rule that acts | Rule that is blocked, and how |
|---|---|---|
| Any chatter (`oscf ≥ 7.5`) | Downshift | Sway upshift needs `oscf < 7.5` |
| Big sway (`sw ≥ 3°`) | Sway upshift | Downshift ignores the 8–16 Hz reading, which is sway leakage |
| Just shifted down | — | Sway upshift paused 1 s; ceiling = new gear for 10 s (escape only at tilt > 12° or `sw ≥ 3°`) |
| Calm (`oscf < 7.5`, `sw` small or shrinking) | none | **No gear change** |
| Chatter and a tilt < 8° | — | Tilt upshift blocked (chatter is not a push) |
| New load (event → re-probe) | Probe | The old ceiling is cleared |

### 4.8 Slow escape (inherited)

A sustained lean, measured on the signed and low-passed tilt (bit 4 is off in v6, so the absolute value is used), above 7.1° adds 0.022 to `f` per tick as a temporary boost. The boost decays with a 1.5 s time constant once the lean is gone. It is not active during the event state.

### 4.9 Why frequency is the right signal

A two-wheel balancing car under PD + PI control has two typical failure modes:

- **Gains too stiff.** The derivative term amplifies IMU noise and gear backlash near the loop's resonance, about 8–16 Hz on this car. The result is a fast, small chatter.
- **Gains too soft for the load.** With a heavy, high load the body's own pendulum frequency drops, and the cascade with the speed loop becomes poorly damped. The result is a slow sway of 1–2 Hz that grows.

So the frequency of the oscillation tells you which way to shift. The amplitude alone does not.

The twin gain sweep (`hgain_sweep.log`, 10-24) found a further point. At CoM ≥ 0.25 m the real culprit is the **speed loop**:

- with Weight balance gains and the speed loop × 0.5, the car is stable for 1–3 kg at 0.20, 0.25 and 0.30 m;
- with the speed loop × 1.5, almost every case falls.

Moving up the ladder also raises the speed gains, so at 0.25 m and above "more gear" alone cannot hold the car. v6 does **not** yet scale the speed loop separately (Section 14).

---

## 5. Feature 2 — large-tilt recovery and protection (`rec_core.c`, `app_motor.c`)

**Finding (twin, 10-20).** The old "40° cannot be recovered" limit was the firmware cut-off, not the motors. With the cut-off at 45° the v4 balance loop lifts the car from 40° in about 0.26 s. What failed after that was the **speed loop's integral**. During the 30 cm of travel while lifting, the integral wound up as a position error. Once upright, the car then drove off to "return" and fell again.

**Implementation:**

- **Cut-off.** In mode 27 `Turn_Off()` switches the motors off at |tilt| > 45° (factory: 40°), at a battery under 9.6 V, before the second KEY1, or when rec_core reports a lock.
- **Recovery state.** It starts when |tilt| > 20°. While it is active, `V4_RecOverride` sets `Encoder_Integral = 0` every tick, so the lift cannot wind up a position error. It ends when |tilt| < 2°, rate < 30 deg/s and wheel speed < 4 counts per tick, all held for 0.1 s.
- **v4 keeps deciding gears during recovery.** Pausing it failed 0/4 when the car was started at a tilt. The car boots on the stiff loaded gains and must be allowed to relax them.
- **Lock.** If |tilt| > 40° for 1 s outside a get-up attempt, the motors stay off until |tilt| < 10°. It covers a car that is held, stuck, or lying with a heavy load. Wheels do not spin on a fallen car.
- **Serial.** `ae0` / `ae1` switches the recovery off or on; the lock and the 45° cut-off stay on either way.

**Limit (physics, `impulse_cut.log`).** Largest tilt the car can catch without touching the floor, by fall rate:

| Fall rate at the start | 0 | 100 | 200 | 300 | 400 deg/s |
|---|---|---|---|---|---|
| Largest angle caught (4/4) | 40° | 35° | 25° | 20° | 10° |

A cut-off of 60° or 75° gives identical numbers, so the limit is motor torque, not the cut-off. Beyond it the car falls, and Feature 3 gets it up again: in the twin all 140 cases ended upright.

---

## 6. Feature 3 — get-up from the floor (`rec_core.c`)

**Geometry.** The STEP model shows a boat-shaped chassis. The body first touches flat ground at 78.5° nose-down and 78.5° tail-down, and a fallen car rests at about 79°. The old box model said 44.5°, which would have made a get-up look impossible.

**Why push–coast–push.** Pushing flat out saturates the wheel speed (about 1 m/s) in 0.2 s, before the body is up, and the body falls back: 0–2 of 4 in the twin. The fix is to push for 0.2 s and then coast for 0.15 s. While coasting the body keeps rocking up and the wheels slow down, which gives back speed margin. A second push then lifts the car: 8/8 at 12 V and 8/8 at 10.5 V, upright in about 0.9 s.

**State machine** (`REC_Step`):

1. **Arming.** The get-up is armed only by a fall from upright (|tilt| crossing 45°), or by starting the car while it lies on the floor (`REC_Stopped` counts the motors-off time as "upright").
2. **Wait.** The car must lie still (|tilt| > 60°, rate < 30 deg/s for 0.5 s) within 5 s of the fall.
3. **Push 1:** full push (1320 PWM above the dead band) toward the lean for 40 ticks (0.2 s; `ap`).
4. **Coast:** motors at 0 for 30 ticks (0.15 s; `aw`).
5. **Push 2:** full push for up to 120 ticks, until |tilt| < 30°. Then the balance controller takes over, inside the recovery state.
6. **Limits.** If it fails, the motors are off for 1 s and it retries, at most 3 tries. After that it locks.

**Safety.** The push is time-limited (at most 0.2 + 0.15 + 0.6 s per try, 3 tries). It only happens right after a fall or a lying start. `as0` disables it completely, and with it off the wheels spin for 0 s on a fallen car. The twin measured how far the car travels while getting up: about 50 cm on average, 66 cm at most. Keep that space clear.

**Limits.** A car with a 1 kg load does not get up (motor torque). It tries 3 times, about 2.4 s in total, then locks. A "flywheel" variant (`sr_rev`, first push away from the lean) was worse: 1–4 of 8. It is kept off.

---

## 7. Feature 4 — motor compensation module (`mc_core.c`, off by default)

Built on 10-12 for the dead band, which changes with battery voltage, temperature and wear. It is **off by default**. With it off the output is the factory `PWM_Ignore` (1480 / 1455), bit for bit. Turning it on with `a1` gives:

- **Voltage feed-forward:** the compensation is scaled by 12 V / battery voltage every tick.
- **Wheels-in-the-air self-calibration (`ac`):** each wheel and direction is ramped from 1300 PWM until the encoder moves, and the start-up PWM is stored at 12 V.
- **Optional small-command hold (`ah`):** |u| < hold → 0, so small commands do not cross the gear backlash.

The on-line estimator is compiled out (`MC_ONLINE 0`) to save flash. In the twin it ran the wrong way: while balancing, a wheel that does not move is often held by the body, so "did not move" does not measure friction. Twin results from 10-12: dead band +100 → falls 6 → 0; dead band −80 → chatter down to 1/5. Normal conditions are unchanged.

---

## 8. Flash budget

The board is only reliable below 64 KB. v5 (get-up) overflowed by 48 bytes, so the following was done:

- Every mode except 27 was removed from the build: line tracking, CCD, electromagnetic, K210, lidar, follow, and the older load modes 22–26. `removed_modes_v5_stubs.c` provides empty versions of the functions the rest of the firmware still references.
- `Mode_select` shows mode 27 and no longer switches.
- The motor-compensation on-line estimator was compiled out.
- `rec_core.c` is compiled with `#pragma Ospace`.
- Bluetooth, the app protocol, the OLED and the serial console are kept.

Result: Code 52332 + RO 6304 + RW 944 bytes, and the image ends at 0x0800 E5C4, leaving 6716 bytes free.

---

## 9. The digital twin and the STEP geometry

- **Engine and model.** MuJoCo. Motor torque limit, dead band (1480/1455), gear backlash, friction, IMU noise and a 200 Hz control tick are all modelled. Parameters come from real-car measurements (`e026 keil\realcar\PARAMS.md`).
- **Geometry from STEP (10-21).**
  - `step_geometry.py` reads `STM32_Balance_V2.STEP` with cascadio + trimesh and writes `stm32_step_geom.py`: wheel radius 33.5 mm, track 167 mm, body box, the body's convex hull and the 78.5° contact angles.
  - The twin's collision body is that convex hull.
  - The top plate is 105.2 mm above the axle. A real load sits **on** it, so its centre of mass is higher: the v5.2 work used 0.15–0.30 m.
- **Same code.** `v51_dll.c` and `rec_dll.c` wrap the firmware's `v4_core.c` and `rec_core.c` into DLLs. The Python twin calls them every tick, so the twin runs the shipped C code.
- **Known gaps.**
  - The twin's chatter is about 30× weaker than the real car's, so chatter thresholds must be set on the real car.
  - At the default load height (0.105 m) the twin does not show the growing sway at all. The high-CoM runs reproduce it.
  - Battery voltage is fixed in most runs.

---

## 10. Source walkthrough

### 10.1 Files and functions

| File | Function | What it does |
|---|---|---|
| `v4_core.c` | `V4_GainsFor(f, g)` | Mixes GN and GH at gear f. |
| | `V4_Init(s)` | Sets the boot state: f = 1, probing, all meters at 0. |
| | `V4_CmdEdge(s)` | A key changed: starts the 1.5 s upshift grace window. |
| | `rearm(s)` | Event: f = 1, re-probe, clear the ceiling. |
| | `revert(s, f)` | Quick revert to the previous gear without a re-probe. |
| | `V4_Step(s, angle, gyro, moving)` | The whole decision for one tick, in this order: (1) tilt prediction, thresholds, event / recovery-state exit; (2) the meters; (3) probe, fast-light, probe-abort and move-latch; (4) latched: chatter downshift, small-chatter rule, bottom sweep, sway upshift, slow escape, ceiling. Returns f. |
| `v4_adapt.c` | `V4_Tick(enc_l, enc_r)` | Runs `REC_Step`, checks `Turn_Off` (motors off → hold the boot state), feeds key edges and the "moving" flag, calls `V4_Step` and applies the gains. |
| | `apply(f)` | Writes the six gains; bumpless rescale of the speed integral. |
| | `V4_RecOverride(&l, &r)` | During recovery: `Encoder_Integral = 0`. During a get-up: write the push. |
| | `cmd_moving()` | "Is there a drive or turn command?", using exactly the rules the factory `Velocity_PI` / `Turn_PD` use. |
| `rec_core.c` | `REC_Step(...)` | Get-up state machine, lock, large-tilt recovery. Returns NORMAL / RECOVER / DRIVE. |
| | `REC_Stopped(s)` | Motors off by the user: count as upright, so a lying start arms a get-up. |
| `app_motor.c` | `Turn_Off()` | Mode 27: ±45°, lock, and the get-up bypass. |
| `tune_io.c` | serial console | Status, recording, and the `a…` commands of Section 11. |

### 10.2 Constants and why they have their values

| Constant | Value | Reason |
|---|---|---|
| `DOWN_SPLIT` | 15 deg/s | Chatter split, set in the twin. Also the probe-abort level. |
| `v4_dnsmall` (`ad`) | 7.5 deg/s | Just above the real car's calm reading at f = 0 (4.5–7.4). |
| `DOWN_N` / `SM_N` | 0.15 s / 0.3 s | Fast enough that chatter is gone in about 1 s; the small rule is slower to avoid noise. |
| `DOWN_WAIT` | 0.2 s | The fast meter (0.1 s) has settled on the new gear by then. |
| `LOW_F` / `LOW_N` | 0.15 / 0.5 s | Real car: the gear hung at 0.1 with chatter under 15. |
| `v4_swup` (`au`) | 1.5° RMS | The real car's own idle limit cycle (4.3 Hz) is about 0.47° RMS and must not trigger it. |
| `SW_GROW_K`, `SW_GROW_R` | 0.5 (= 0.75°), ×1.2 per 0.5 s | Catch a growing sway early. Twin: 2 kg at 0.20 m grew from 0.3° to 4° in 1.4 s. |
| `SU_N` / `SU_WAIT` | 0.1 s / 0.3 s | As above: waiting for 1.5° was too late. |
| `PL_N` | 3 s | Watch window after a probe latch, valid while driving too. |
| `SW_FAIL_UP` | +0.3 | An event that hits a swaying car: the gear it was on was too soft. |
| `SU_AFTER_DN` | 1 s | No sway upshift right after a chatter downshift. |
| `CEIL_N` / `CEIL_ESC` | 10 s / 12° | v5.1: stop the up–down cycle; escape if the car is really falling. |
| `REC_STALE_N` | 3 s | The event state ends and re-centres if the balance angle moved (load changed). |
| `v4_light_split` / `v4_mid_split` | 25 / 20 | Twin sweep (`split_sweep.log`). |
| `F_LIGHT` / `V4_F_MID` / `F_HEAVY` | 0 / 0.2 / 0.4 | 0: real car, 0.12 still chattered. 0.4: v4 benchmark optimum. |
| rec: `on_deg` / exit | 20° / 2°, 30 deg/s, 4 counts | Twin `rec_final_checks.log`. |
| rec: lock | > 40° for 1 s, unlock < 10° | Covers held, stuck, or loaded-and-lying. |
| get-up: push / coast / push2 / hand-over | 0.2 s / 0.15 s / ≤ 0.6 s / 30° | Only 40×30 and 50×20 succeed at both voltages (scan). |

---

## 11. Serial commands (115200 baud; each command ends with Enter)

| Command | Effect |
|---|---|
| `?` | Print the state. The `#mc` line ends with `re=` recovery on / entries / locks, `sr=` get-up on / attempts / successes / push1 / coast, `sp=` light / middle split, `op=` `v4_opt` / wide-band threshold / sway threshold ×10 / small-chatter threshold ×10. |
| `r` / `s` | Start / stop recording a 200 Hz CSV (`seq,gyro,ang_c,ml,mr,el,er,inj,osc_c,k_c,lvl,cs,tv`). `lvl` = f×100, +1000 latched, +2000 event state, +4000 recovery, +8000 lock, +16000 get-up push. |
| `ae0` / `ae1` | Large-tilt recovery off / on. |
| `as0` / `as1` | Get-up off / on. |
| `ap<n>` / `aw<n>` | Get-up first push / coast, in ticks of 5 ms. |
| `ak<x>` / `aj<x>` | Probe light / middle split (deg/s). |
| `ax<n>` | Gear-rule bits. v6 = 50. `ax18` = without the sway upshift. `ax30` = the v5.1 bit set (the v6 downshift changes stay). |
| `au<x>` | Sway-upshift threshold, deg RMS (1.5). |
| `ad<x>` | Small-chatter downshift threshold, deg/s (7.5). |
| `ay<x>` | Wide-band oscillation threshold (only used with bit 8). |
| `a1` / `a0`, `ac`, `ah<n>`, … | Motor compensation (Section 7). |
| `p q v b t y<x>` | Overwrite one gain directly (balance Kp / Kd, speed Kp / Ki, turn Kp / Kd). v4 overwrites the gains again on the next tick in mode 27. |

Selecting the mode again (power cycle) restores all defaults.

---

## 12. Results

All twin results include the factory baselines. **Lower penalty is better.** Each result names the build it was measured on. The last three builds (v5.2 test 2–4 = v6) were changed **from real-car feedback** at the user's request, and were not re-scored in the twin.

### 12.1 Benchmark (stop-go-stop course, 5 conditions × 3 seeds, weighted)

| Controller | Total | Empty ×2 | 1 kg ×2 | 2 kg ×1 | 4 kg ×1.5 | Empty + 3 N push ×1 |
|---|---|---|---|---|---|---|
| Factory Normal | 111.45 | 3.74 | 12.61 | 30.00 (fell) | 30.00 (fell) | 3.75 |
| Factory Weight | 115.41 | 23.85 | 17.36 | 3.74 | 3.59 | 23.85 |
| v4 release | 31.83 | 3.81 | 5.93 | 3.54 | 3.34 | 3.80 |
| v5.1 | 27.09 | 3.66 | 3.75 | 3.54 | 3.34 | 3.70 |
| v5.2 (10-24, build "d") | 26.97 | 3.66 | 3.40 | 3.54 | 3.34 | 4.30 |

The scoring rules are unchanged from v4. Each condition drives to 0.9 m, back to 0.3 m, then to 1.2 m, and must stop for 5 s within 5 cm each time. Penalty = settle time + 0.5 × chatter. A fall costs 30 × the share of segments not done.

### 12.2 High centre of mass (load on the top plate), load put on a standing car, 15 s, 3 seeds

Cell = falls / sway amplitude in the last 3 s (°) / final gear / gear changes in the last 5 s.

| CoM 0.20 m | 0.5 kg | 1 kg | 2 kg | 3 kg |
|---|---|---|---|---|
| Factory Normal | fell 3/3 | fell 3/3 | fell 3/3 | fell 3/3 |
| Factory Weight | 0 / 0.12 / 1.0 / 0 | 0 / 0.15 / 1.0 / 0 | 0 / 0.20 / 1.0 / 0 | 0 / 0.26 / 1.0 / 0 |
| v4 release | 0 / 0.19 / 0.4 / 0 | 0 / 1.56 / 0.4 / 0 | 0 / 1.71 / 0.4 / 2 | fell 3/3 |
| v5.1 | 0 / 0.22 / 0.4 / 0 | 0 / 0.45 / 0.4 / 0 | 0 / 1.88 / 0.4 / 2 | fell 3/3 |
| v5.2 (build "d") | 0 / 0.22 / 0.4 / 0 | 0 / 0.45 / 0.4 / 0 | **0 / 0.20 / 1.0 / 0** | fell 3/3 |

On the course with the load at 0.20 m (3 seeds, falls):

| Load | Factory Weight | v4 | v5.1 | v5.2 build "d" | v5.2 build "e" (faster sway upshift, kept in v6) |
|---|---|---|---|---|---|
| 1 kg | 0 | 3 | 3 | **0** | — |
| 2 kg | 0 | 3 | 3 | 2 | **0 of the 2 traced seeds** |

At 0.25 m and above, even the factory Weight gains fall from 1 kg (`v52_hmatrix1.log`). The cause is the speed loop (Section 4.9).

### 12.3 Chatter removal (empty car forced to f = 1, 3 seeds)

| | v4 | v5.1 | v5.2 build "d" |
|---|---|---|---|
| Time until the fast meter < 5 deg/s | never (stays at f = 1) | 3.81 s | **1.21 s** |
| Final gear | 1.0 | 0.3 | 0.1–0.2 |

v6 adds the small-chatter rule and the bottom sweep on top of this. Those two were made for the real car.

### 12.4 Fixed loads and load swaps (CoM at the plate)

| | Factory Normal | Factory Weight | v4 | v5.1 | v5.2 |
|---|---|---|---|---|---|
| 0–4 kg every 0.5 kg, falls | 2/3 at 1 kg, all from 1.5 kg | 0 (chatters at 0–1 kg) | 2 (at 0.5 kg) | 0 | 0 |
| Swaps between 0/1/2/3/4 kg (60) | — | — | 0 falls | 0 falls | 0 falls |

Two bugs that made v5.2 shake longer after a swap (1 → 3 kg and 1 → 4 kg, about 8 s) were found by trace (`v52_tswap13.log`) and fixed: the ceiling now clears on a re-probe, and the downshift waits after a latch. After the fix the same swaps settle on 0.4 with 0.15–0.2° sway (`v52_tswap_f.log`).

### 12.5 Large tilt and get-up (empty, 12 V)

| | Factory / v4 | v6 |
|---|---|---|
| Released from rest at 40° | lost (cut at 40°) | 4/4, upright in 0.26 s, settled in 1.5 s |
| Pressed to 42° while balancing, then released | lost | 4/4 |
| Lying on the floor (≈ 79°), 12 V / 10.5 V | stays down | 8/8 / 8/8, up about 1.9 s after the fall |
| Started while lying (second KEY1) | stays down | 8/8 / 8/8 |
| 100 shock starts (5 angles × 5 rates × 4): caught without touching the floor / finally upright | 42 / 42 (factory), 51 / 51 (v4) | 56 / **100** |
| Wheel spin on a fallen car with `as0` | 0 s | 0 s |

### 12.6 Real car (2026-10-21 … 10-24, reported by the user)

| Build | Observation | Action |
|---|---|---|
| v5 | Get-up works. Loaded behaviour worse than before. | v5.1 |
| v5.1 | Still a growing sway with loads; less calm than the factory Normal gains on an empty car. | v5.2 (shift by frequency) |
| v5.2 test 1 | Empty: chatter would not go away; gear hung at 0.1, or chattered at 0.0. | test 2 (sway/chatter separation) |
| v5.2 test 2 | Fine with a load. After taking it off, the gear stuck at 0.2. | test 3 (downshift in every state) |
| v5.2 test 3 | Question: small-amplitude high-frequency oscillation? | test 4 = **v6** (small-chatter rule) |

---

## 13. Development history, 2026-10-03 → 10-24

1. **10-03 to 10-07: v4 release and follow-up.**
   - The v4 release was documented. Real-car CEM tuning was tried while the car stood.
   - It showed that the moments of near-perfect stillness came from the dead band, not from the PID.
2. **10-12: dead-band and backlash compensation** (`v4_mc_20261012`).
   - The dead band depends on battery voltage, so the design is a voltage feed-forward plus a wheels-in-the-air self-calibration, with a small-command hold against backlash.
   - It was built and checked in the twin while the car was not available, and a real-car test plan was written for when it is. It is shipped switched off.
3. **10-20: large-tilt recovery** (`v4_rec_20261020`).
   - The 40° limit turned out to be the firmware cut-off. Cut-off 45°, plus an integral hold during the lift.
   - 40° from rest: 4/4 in the twin. Daily benchmark unchanged (31.83).
4. **10-21: get-up and STEP geometry.**
   - The user made the STEP file the authority. The contact angle is 78.5°, not 44.5°.
   - Push–coast–push get-up, 8/8 at both voltages. A lying start works too.
   - Lock and `as0` for safety.
   - The shock envelope showed that a larger cut-off does not help, because the limit is physics.
   - Released as **v5**.
5. **10-21: load gears, v5.1.**
   - Real car: v5 was worse with loads, and small loads cycled shake → calm → shake.
   - The v5.1 bits (graded upshift, probe abort, signed lean, wide-band detector, ceiling) were tested one by one in the twin, and three were kept.
   - All modes but 27 were removed to free flash.
6. **10-24: high centre of mass, v5.2.**
   - Real car: a visible, growing sway with a load on the top plate.
   - The twin reproduced it only with the load CoM raised to 0.15–0.25 m.
   - The user's rule became the design: low-frequency sway → up, high-frequency chatter → down, calm → no change, no conflict.
   - The twin traces then exposed and fixed: a late upshift, a probe that re-latched too soft, sway leaking into the chatter meter, a stale ceiling after a swap, and a re-downshift right after a latch.
   - A gain sweep found the speed-loop cause at ≥ 0.25 m.
7. **10-24: real-car iteration test 1 → 4.**
   - The user stopped twin scoring ("judge by the real car").
   - Three real-car reports led to three fixes the same day (Section 12.6).
   - Test 4 was declared final → **v6**.

---

## 14. Known limitations and real-car calibration

1. **Chatter thresholds were set in the twin.** These are 15 deg/s and the small-chatter level 7.5 (`ad`); the twin under-reads chatter about 30×. If a calm loaded car keeps shifting down, raise `ad` (10–12). If small chatter stays, lower it (6). Read `?` and a short recording first.
2. **Sway threshold `au` (1.5° RMS).** If a calm car shifts up, raise it (2). If it sways visibly and does not shift up, lower it (1).
3. **CoM ≥ 0.25 m.** v6 can still fall: the twin shows the speed loop must be scaled down there, and the gear ladder raises it instead. Next step if needed: a second gear axis that scales the speed-loop gains (0.5–0.6×) when the sway persists at f = 1.
4. **Get-up** needs about 50–66 cm of clear floor, and does not work with a load.
5. **Twin re-scoring of v6** was not done (user decision). The last twin-scored build is v5.2 "d/e/f"; test 2–4 changes were driven by the real car.
6. **Battery.** Below 9.6 V the motors are cut, as in the factory firmware. The get-up was checked at 10.5 V and 12 V.

---

## 15. How to rebuild

1. Keil MDK 5: `UV4.exe -b USER\stm32_Balance_Car.uvprojx -j0 -o build.log` in `e026 keil\v4_work_20261003` (or open the project and press F7). Expect 0 errors. The two warnings in `bsp.c` are from the factory code.
2. Check the image end. In `OBJ\stm32_Balance_Car_L.hex` the highest data address must stay below 0x0801 0000.
3. Twin DLL (w64devkit gcc), from `balance_bot\scripts\rl`:
   `gcc -O2 -shared -I<APP\PID> -o v52core.dll v51_dll.c <APP\PID>\v4_core.c -lm`.
   `rec_core` is built the same way from `rec_dll.c`.

---

## 16. File locations

| What | Where |
|---|---|
| Build project (all source) | `e026 keil\v4_work_20261003` |
| Backups of each change | `e026 keil\v4_backup\before_*` |
| Earlier release folders | `v4_release_20261003`, `v4_mc_20261012`, `v4_rec_20261020`, `v5_20261021` |
| Digital twin | `e026\balance_bot_windows\balance_bot` (scripts `scripts\rl`, logs `runs_e2e`) |
| STEP model | `Downloads\STM322\10.attch\3Dmodel\STM32_Balance_V2.STEP` |
| Real-car measurement tools | `e026 keil\realcar` |

---

## 17. Division of work

- **The user (human)** set every goal and every constraint. These included: the 40° recovery and the 45° cut-off; get-up from the floor with no wheel spin after a fall; the STEP file as the geometry authority; finer low-load gears; the rule "low-frequency sway up, high-frequency chatter down, calm no change, no conflict"; "high-frequency small oscillation must go fast"; keeping Bluetooth when removing modes; "judge by the real car". The user also ran every real-car test and reported each failure precisely: what the car did, at which gear, under which load.
- **The large language model (Claude)** turned each goal into a mechanism. It wrote the C and Python code, built the twin experiments (STEP import, high-CoM runs, gain sweeps, traces), compiled and size-checked every hex, and explained each cause found.


---

# 第二部分 — 中文

## 0. 文件夹里有什么，怎么烧录、怎么用

### 0.1 文件清单

| 文件 / 文件夹 | 内容 |
|---|---|
| `stm32_Balance_Car_L_v6.hex` | **要烧录的固件**。由完整 Keil 工程（`e026 keil\v4_work_20261003`）编译得到。固件里只保留模式 27，屏幕显示「27.Adapt V6」。除了这行屏幕文字，代码和 2026-10-24 实车测试版 `v5_2_test4` 完全相同。 |
| `v6_README_software_to_hardware.md` | 本说明。 |
| `CAR_TEST_PLAN.md` | v6 实车检查步骤和串口命令速查。 |
| `source_snapshot/core/` | 纯逻辑模块（不碰硬件）：`v4_core.*`（换档决策）、`rec_core.*`（大角度回正、保护锁、倒地自起）、`mc_core.*`（电机死区补偿，默认关闭）；以及把它们接进原厂固件的胶水层 `v4_adapt.*`、`mc_adapt.*`。 |
| `source_snapshot/firmware_hooks/` | 为调用这些模块而改动过的原厂文件，加上 Keil 工程文件：`app_control.c`、`app_motor.c`、`app_mode.c`、`oled_show.c`、`main.c`、`pid_control.c`、`tune_io.c/.h`、`removed_modes_v5_stubs.c`、`stm32_Balance_Car.uvprojx`。 |
| `source_snapshot/twin_scripts/` | 设计和检验 v6 用的数字孪生脚本，以及把**同一份**固件 C 代码编译成 Windows DLL 给孪生调用的包装文件（`v51_dll.c`、`rec_dll.c`、`v4rel_dll.c`）。`step_geometry.py` 读取 STEP 三维模型，`stm32_step_geom.py` 是它的输出。 |
| `twin_results/` | 第 12 节各结果表对应的原始日志。 |
| `previous_builds/` | 以前的 hex，留作对比：v4 正式版、死区测试版、回正测试版、v5、v5.1、v5.2 test1–4。 |
| `blog_2026-10-10_to_10-24.md` | 两周进展总结（先英文，后中文）。 |

### 0.2 烧录和使用

1. 用平时的方式（ST-Link、串口下载工具等）把 `stm32_Balance_Car_L_v6.hex` 烧进 STM32F103RCT6。起始地址 0x0800 0000，和原厂相同。镜像结束于 0x0800 E5C4，在可靠的 64 KB 内还剩 6716 字节。（这块板子只有 0x0801 0000 以下可靠；镜像超过这条线，连模式 1 都跑不起来。）
2. 上电，屏幕显示 **「27.Adapt V6」**。为了腾出 Flash，其他模式都删了（第 8 节），所以转车轮已经不会切换模式。按 **KEY1** 确认。
3. 屏幕提示「put down key start!」。把车放到地上，再按一次 **KEY1**，开始自平衡。正常是竖着启动；如果启动时车躺在地上，它会尝试自己爬起来（第 6 节）。
4. 照常用手机蓝牙 App 操控。蓝牙、App 协议、串口控制台都保留。不需要再选「正常 / 负重」模式，v6 自己判断负载、自己换档。
5. 屏幕第 3 行显示档位状态，格式 `f0.00L osc 1.2 u0`：
   - `f`：当前档位（0 = 原厂正常档增益，1 = 原厂负重档增益）；
   - `L`：已锁定档位；`P`：还在探测；
   - `osc`：慢速 8–16 Hz 抖动计读数，单位 °/s；
   - `u`：大倾角事件让档位回到 1 重新探测的次数。
6. 可选：打开串口（115200 波特率），发送 `?` 读取完整状态。全部命令见第 11 节。文中所有命令都要你自己发，固件不会自动发出任何命令。

---

## 1. v6 在 v4 基础上加了什么、为什么

v4（2026-10-03）是一个换档控制器：它套在原厂串级 PID 外面，每 5 ms 在原厂两套增益之间选一个档位 `f`。孪生跑分 31.83，原厂正常档 / 负重档分别是 111.45 / 115.41（越低越好）。之后三周里，实车暴露了 v4 还做不到的事，v6 逐条给出解决办法：

| 实车问题（发现日期） | 查到的原因 | v6 的做法 |
|---|---|---|
| 车斜到 40° 起不来，电机直接停（10-20） | 固件在 40° 切电机；电机本身其实抬得起来 | 切电机角改到 45°；回正状态下把速度环积分保持为 0，让车自己抬起来（第 5 节） |
| 车倒地后就躺着，有时轮子还在转（10-21） | 原厂没有自起，也没有保护锁 | 有时间和次数上限的「推—滑行—推」倒地自起；车卡在大角度时用保护锁关电机（第 5–6 节） |
| 车的真实外形一直按长方体估算（10-21） | — | 几何全部改用 STEP 三维模型：车身在 78.5° 才第一次碰地，不是 44.5°（第 9 节） |
| 放小负载时越抖越大、平稳、再抖，偶尔摔倒（10-21） | 几条换档规则互相打架（v5.1 修复） | 保留 v5.1 的规则（`v4_opt` 的位 2 和位 16） |
| 负重放在顶板上，车身肉眼可见地每秒前后摆几次，越摆越大直到摔倒（10-24） | 负重重心高，在轮轴上方 0.15–0.25 m；探测选出的档位（0.4）对这么高的负重太软；旧规则只对 8–16 Hz 抖动有反应，看不见 1–2 Hz 的摆动 | **按频率换档**：低频摆动（0.5–6 Hz）大或在变大 → 升档；8–16 Hz 抖动 → 降档；车稳 → 不换（第 4.6–4.7 节） |
| 空载高频抖振下不来，f 卡在 0.1，或 0.0 时仍抖（10-24，test1） | 实车抖振漏进新加的摆动计，摆动升档和抖动降档互相打架；还有一个上限 bug，让摆动升档抵消了降档 | 摆动计改二阶低通；有任何抖动都不做摆动升档；降档后上限 = 新档位；底档清扫直接降到 0（test2） |
| 取下负载后档位卡在 0.2 一直抖（10-24，test2） | 取下负载被当成「事件」；平衡角随负载变了，抖振又让原始角速度一直超过 60°/s，事件状态永远结束不了；而降档在这个状态下被禁止 | 降档在任何状态都执行；事件状态用低通角速度判断退出，3 s 后强制退出并重设参考角（test3） |
| 小幅高频振荡没有被消除（10-24） | 档位 0.15 以上时，读数低于 15 °/s 就没有任何规则响应 | 新增小幅抖动规则：快表 ≥ 7.5 °/s（串口 `ad`）持续 0.3 s → 任意档位降 0.1（test4 = v6） |

设计原则和 v4 一样：
- 决策逻辑是不碰硬件、可解释的纯 C 代码；
- 每条规则都对应一个实测发现的问题；
- 每个阈值都能通过串口修改，不用重新烧录。

---

## 2. 术语表

| 术语 | 含义 |
|---|---|
| **档位 `f`** | 0 到 1 的数。六个 PID 增益是原厂正常档（f=0）和负重档（f=1）的线性插值：`增益 = GN + (GH − GN) · f`。 |
| **GN / GH** | 原厂正常 / 负重增益（固件里存的是 ×100 的值）：平衡 Kp 9600 / 19200，平衡 Kd 48 / 150，速度 Kp 6200 / 9450，速度 Ki 31 / 47.25，转向 Kp 1700 / 1400，转向 Kd 20 / 20。 |
| **探测** | 开机或大倾角事件后，档位跳到 f=1，「听」车在这个档位抖得多厉害。空车在硬档上抖得很厉害，带载车不会。探测以**锁定**结束：0（轻）、0.2（中）或 0.4（重）。 |
| **事件 / `rec`（v4 的事件状态）** | 预测 0.1 s 后的倾角超过阈值（站立时 3–8°，行驶时 16°）。档位跳到 1 并重新探测。注意和 rec_core 的大角度回正（第 5 节）不是一回事。 |
| **抖动计 `osc` / `oscf`** | 陀螺角速度在 8–16 Hz 频段的均方根，单位 °/s。`osc` 约 0.3 s 平均，`oscf` 约 0.1 s 平均。高频抖动直接说明增益相对负载太硬。 |
| **摆动计 `sw`** | 倾角在 0.5–6 Hz 频段的均方根，单位度，约 0.5 s 平均。`sw_old` 是 0.5 s 前的值。摆动越来越大，说明增益太软。 |
| **反向频率 `swhz`** | 0.5–6 Hz 倾角改变方向的频率（带 0.3° 回差），单位 Hz。推一下或刹一下只反向一次，摆动会一直反向。 |
| **下限 / 上限** | 下限：摆动升档后记住的档位，重新探测不能锁到它下面。上限：抖动降档后 10 s 内允许的最高档位。 |
| **临时加档（慢逃逸）** | 车长时间倾斜时临时加在 `f` 上的量，倾斜消失后按 1.5 s 时间常数衰减。 |
| **死区** | 电机开始转动所需的 PWM，本车正转 1480、反转 1455。固件给每个非零指令都加上这个值。 |
| **切电机角** | 超过这个倾角就关电机：原厂 40°，v6 模式 27 为 45°。 |
| **保护锁** | rec_core 的保护：不在自起过程中、倾角超过 40° 持续 1 s → 关电机，直到倾角回到 10° 以内。 |
| **数字孪生** | 用 MuJoCo 搭的这辆车的仿真（含死区和齿隙的电机模型、IMU 噪声、STEP 几何），在 `e026\balance_bot_windows\balance_bot`。它通过 DLL 直接运行固件自己的 C 代码。 |
| **STEP 模型** | 厂家的三维 CAD 文件 `STM32_Balance_V2.STEP`。从 2026-10-21 起，车的外形和碰撞都以它为准。 |
| **`v4_opt`** | 打开 / 关闭各条换档规则的位开关（串口 `ax`）。v6 默认 50 = 位 2 + 16 + 32。 |

---

## 3. 架构：每 5 ms 运行什么

原厂固件在 MPU6050 数据就绪中断里以 200 Hz 运行控制循环（`APP/app_control.c`）。v6 在这个循环里加了三处调用，其余全部保留：

```
读 IMU    ->  Angle_Balance（度）、Gyro_Balance（原始值，16.4 LSB = 1 °/s）
读编码器  ->  Encoder_Left、Encoder_Right
 |
 |- V4_Tick(enc_l, enc_r)                          [v4_adapt.c]（仅模式 27）
 |     |- REC_Step(...)      大角度回正 / 保护锁 / 倒地自起          [rec_core.c]
 |     |- Turn_Off(...) == 1 ?  -> 电机关着：v4 保持开机状态，返回
 |     |- V4_Step(angle, gyro, moving)  -> 档位 f                     [v4_core.c]
 |     \- apply(f)：写入 6 个 PID 增益（速度积分无扰切换）
 |
 |- Balance_PD  = Kp/100 *（角度 - Mid_Angle）+ Kd/100 * 角速度      [原厂]
 |- Velocity_PI = -偏差*Vkp/100 - Encoder_Integral*Vki/100           [原厂]
 |- Turn_PD                                                          [原厂]
 |- 电机 = 平衡 + 速度 -/+ 转向，限幅 ±2800
 |- V4_RecOverride(&L, &R)   回正时：速度积分清零；自起时：写入推力
 |- MC_FwOutput(&L, &R)      死区：原厂 1480/1455（补偿默认关闭）
 \- 输出 PWM（Turn_Off 时为 0）
```

各层严格分开：
- **平衡由原厂 PID 负责**。除了有时限的自起推力，v6 自己从不计算电机指令。
- **`v4_core` 只决定增益**。它只有三个输入：倾角、角速度、行驶指令是否非零，不接触其他硬件。
- **`rec_core` 处理极端情况**：大倾角、倒地、车卡住。
- **`mc_core` 作用在电机输出上**。不用串口命令 `a1` 打开就不起作用。

几个核心模块是不碰硬件的纯 C，所以孪生把**同一份** `.c` 文件编译成 DLL 来跑。孪生的结果就是车上这份代码的结果，而不是某个 Python 仿写版本的结果。

---

## 4. 功能一：自动换档（`v4_core.c`）

### 4.1 档位线

`V4_GainsFor(f)` 把原厂两套增益做线性插值，`v4_adapt.c` 里的 `apply()` 每个控制周期把六个增益写进原厂变量。速度环输出是 `−Encoder_Integral · Ki/100`，只改 Ki 会让电机指令一下子跳最多约 1280 PWM。所以每次 Ki 变化时，把积分乘以 `Ki旧 / Ki新`，做到无扰切换。

为什么只在原厂两套增益之间取值？两套都已经在这辆车上验证过（空车用正常档，重载用负重档），中间任何一点都是两个稳定控制器的平滑混合。超出这个范围就要在实车上重新整定，而孪生的抖振读数偏低（第 9 节）。

### 4.2 三个测量量

`V4_Step` 每个周期从 IMU 更新三个量：

1. **抖动（8–16 Hz，用角速度）**：8 Hz 一阶高通 → 16 Hz 一阶低通 → 能量平均。保留两个平均：
   - `osc = sqrt(0.3 s 平均(bp²)) / 16.4`：慢、稳，探测和屏幕用；
   - `oscf = sqrt(0.1 s 平均(bp²)) / 16.4`：快，降档用。约 0.15 s 起反应，抖动一消失就停止降档。
2. **摆动（0.5–6 Hz，用倾角）**：0.5 Hz 一阶高通 → **两级**串联的 6 Hz 一阶低通 → 约 0.5 s 能量平均，`sw = sqrt(平均(swl²))`；每 0.5 s 存一次 `sw_old`。第二级低通是实车 test1 之后加的：实车抖振（约 14 Hz、54 °/s，折合约 0.6° 倾角）能穿过一级低通，两级把 14 Hz 的幅值压到约 1/7。
3. **反向频率 `swhz`**：0.5–6 Hz 倾角的过零次数（0.3° 回差），约 0.5 s 平均。用来区分摆动（一直反向）和一次推车或刹车。

### 4.3 探测与锁定（继承 v4/v5，v6 未改）

- 开机时档位为 f=1（对重载最安全的一端），开始「听」。
- 站立 1.2–3 s、`osc` 读数稳定后锁定：
  - `osc ≥ 25`（`ak`）→ 轻，f=0；
  - `osc ≥ 20`（`aj`）→ 中，f=0.2；
  - 否则 → 重，f=0.4。
- 事件之后，要 0.3 s 窗口内的读数全部同意才判轻（「有把握才判轻」）。轻车判成重只会抖；重车判成轻会摔。
- **快速判轻**：最近 0.3 s 内每个读数都超过轻载分界，就立刻判轻，不再等。
- **提前结束探测**（位 2）：在 f=1 上 8–16 Hz 抖动 ≥ 15 持续 0.2 s，就提前结束探测，免得车抖 3 s。v6 里车在摆动（`sw ≥ 1.5`）时跳过这一条：大摆动会漏进 8–16 Hz 表，孪生里就被当成过抖动。
- **下限**：探测不会锁到记住的摆动下限以下（判为空载时除外）。
- 锁定后降档先等 0.2 s，因为抖动表里还留着 f=1 时的读数。

### 4.4 大倾角升档（继承 v4）

- **预测**：`th_pred =（倾角 − th_ref）+ 0.1 s × 角速度`，`th_ref` 是慢慢学习的平衡角。
- **阈值**：超过阈值就把档位升到 1 并重新探测：
  - 站立：默认 3°，随车自身噪声上调（`6·σ`，上限 8°）；
  - 行驶：16°。
- **防误升档**：
  - 按键变化后 1.5 s 宽限期内不看角速度项；
  - 停车要先稳下来，才开始用 3° 的站立阈值；
  - 每次事件后要重新进入准备状态；
  - 快速回退：升档后 0.3 s 内峰值低于 6°，就恢复原档，不重新探测。
- **抖动不是推车**：正在抖（`osc ≥ 15`）时，小于 8° 的倾角不升档。

### 4.5 高频抖动降档（v6：三层，任何状态）

高频抖动直接说明增益太硬：微分项在回路自身的谐振频率附近放大了传感器噪声和齿隙。办法是换软一点的档，越快越好。

| 快表 `oscf`（°/s） | 持续 | 降档 |
|---|---|---|
| ≥ 30 | 0.15 s | 0.3 |
| 22.5 – 30 | 0.15 s | 0.2 |
| 15 – 22.5 | 0.15 s | 0.1 |
| **7.5（`ad`）– 15** | **0.3 s** | **0.1**（v6 新增的小幅抖动规则） |
| 档位 ≤ 0.15 时 ≥ 7.5 | 0.5 s | 直接到 0（底档清扫） |

- 每降一步等 0.2 s，让读数跟上新档位，再看一次。
- 每次降档都清掉临时加档。
- `oscf` 一旦低于 7.5，计时就重新开始。
- **任何状态都降**：到 test2 为止，降档要求「不在事件状态」，正是这个限制导致了卡在 0.2 的 bug，v6 去掉了它。抖动就是抖动，与状态无关。
- **大摆动时不算**：`sw ≥ 2 × 1.5 = 3°` RMS 时，8–16 Hz 读数来自摆动本身，降档不计入。
- 为什么是 7.5？实车空载在 f=0 平稳时读数 4.5–7.4，增益太硬时 54–58（2026-10-03 记录）。7.5 刚好在平稳读数之上。带载车还没有实测过，所以做成串口可调。

### 4.6 低频摆动升档（v5.2/v6 新增，位 32）

负重放在顶板上，重心在轮轴上方 0.15–0.25 m。孪生里，对这种负重太软的档位不会高频抖，而是以约 1–2 Hz 摆动，越摆越大直到摔倒（`sway_probe.log`：f 0–0.4 摆动并摔倒，f 0.7–1 站得住）。这和用户在实车上看到的一致。

**条件**：
- `f_latched < 1`。
- **完全没有抖动**：`osc < 15` 且 `oscf < 7.5`（和小幅抖动规则用同一个 `ad` 阈值，两条规则因此互斥）。
- **在反向**：`swhz ≥ 1 Hz`，是摆动而不是倾斜。
- **在摆**，满足其一：
  - 大且没在变小：`sw ≥ 1.5°`（`au`）且 `sw ≥ 0.9 · sw_old`；
  - 小但在变大：`sw ≥ 0.75°` 且 `sw ≥ 1.2 · sw_old`。
- **时机**，满足其一：
  - 站立（1 s 内没有行驶指令）；
  - 探测锁定后 3 s 内（锁定的档位可能太软）；
  - 行驶中且摆动大并在变大（`sw ≥ 1.5` 且 `sw ≥ sw_old`）。
- **没有被挡住**：
  - 不在抖动降档后 1 s 内；
  - 不超过上限（除非 `sw ≥ 3°`）。

**动作**：
- 条件全部满足 0.1 s 后，`f_latched` 上调：
  - 小摆动 0.1；
  - `sw ≥ 1.5` 时 0.2；
  - `sw ≥ 3` 时 0.3。
- 然后等 0.3 s。
- 新档位记为下限，以后重新探测不会锁到它下面。

**摆动中发生事件**：如果大倾角事件发生时车已经在摆，说明当时的档位太软，下限设为该档位 + 0.3。

**为什么不干脆一直用 f=1？** f=1 会让空车和轻载车剧烈抖振（实车读数 54–58 °/s）。摆动规则只升到压住摆动所需的档位。

### 4.7 升档和降档怎么不打架

| 情况 | 起作用的规则 | 被挡住的规则及方式 |
|---|---|---|
| 有任何抖动（`oscf ≥ 7.5`） | 降档 | 摆动升档要求 `oscf < 7.5` |
| 大摆动（`sw ≥ 3°`） | 摆动升档 | 降档不计 8–16 Hz 读数（那是摆动漏进来的） |
| 刚降过档 | — | 摆动升档暂停 1 s；10 s 内上限 = 新档位（只有倾角 > 12° 或 `sw ≥ 3°` 才能突破） |
| 平稳（`oscf < 7.5`，`sw` 小或在变小） | 无 | **不换档** |
| 有抖动且倾角 < 8° | — | 倾角升档被挡（抖动不是推车） |
| 换了负载（事件 → 重新探测） | 探测 | 旧的上限被清除 |

### 4.8 慢逃逸（继承）

持续倾斜超过 7.1° 时（v6 未开位 4，所以用倾角绝对值），每个周期给 `f` 临时加 0.022；倾斜消失后按 1.5 s 时间常数衰减。事件状态中不起作用。

### 4.9 为什么用频率判断

PD + PI 控制的两轮平衡车有两种典型失败：
- **增益太硬**：微分项在回路谐振附近（本车约 8–16 Hz）放大 IMU 噪声和齿隙，表现为快速的小幅抖动。
- **增益对负载太软**：负重又重又高时，车身自身的摆频降低，和速度环串起来以后阻尼变差，表现为 1–2 Hz、越来越大的摆动。

所以，振荡的频率告诉你该往哪个方向换档，只看幅度做不到这一点。

孪生增益扫描（`hgain_sweep.log`，10-24）还发现：重心 ≥ 0.25 m 时，真正的原因是**速度环**：
- 负重档的平衡增益配上速度环 × 0.5，在 0.20、0.25、0.30 m 下 1–3 kg 全部站稳；
- 速度环 × 1.5 时几乎全倒。

沿档位线升档会同时加大速度环增益，所以在 0.25 m 以上，单靠「升档」压不住。v6 **还没有**单独缩小速度环（第 14 节）。

---

## 5. 功能二：大角度回正和保护（`rec_core.c`、`app_motor.c`）

**发现（孪生，10-20）**：原来「40° 救不回来」的限制是固件切电机角，不是电机能力。把切电机角改到 45° 后，v4 的平衡环能在约 0.26 s 内把车从 40° 抬起来。之后出问题的是**速度环积分**：抬车过程中车走了 30 cm，积分把这段当成位置误差累积起来；车立直后又开回去「找位置」，再次摔倒。

**实现**：
- **切电机**：模式 27 下 `Turn_Off()` 在以下情况关电机：|倾角| > 45°（原厂 40°）、电池 < 9.6 V、第二次按 KEY1 之前、rec_core 报保护锁。
- **回正状态**：|倾角| > 20° 时进入。期间 `V4_RecOverride` 每个周期把 `Encoder_Integral` 清零，抬车时就不会累积位置误差。|倾角| < 2°、角速度 < 30 °/s、车轮速度 < 4（每周期编码器计数）同时保持 0.1 s 后退出。
- **回正期间 v4 照常换档**：暂停它的话，斜着启动时 0/4 成功，因为车开机用的是负重档的硬增益，必须允许它放软。
- **保护锁**：不在自起过程中、|倾角| > 40° 持续 1 s，电机就一直关着，直到 |倾角| < 10°。用于车被手扶着、卡住、或带重载躺在地上的情况。倒地的车轮子不会转。
- **串口**：`ae0` / `ae1` 关 / 开回正；保护锁和 45° 切电机始终有效。

**极限（物理，`impulse_cut.log`）**：不碰地能接住的最大倾角，与倒下的角速度有关：

| 起始倒下角速度 | 0 | 100 | 200 | 300 | 400 °/s |
|---|---|---|---|---|---|
| 能接住的最大角度（4/4） | 40° | 35° | 25° | 20° | 10° |

切电机角改成 60° 或 75°，结果完全一样，所以限制来自电机力矩，不是切电机角。超出这个范围车会倒地，再由功能三把它扶起来：孪生里 140 种情况最终全部立直。

---

## 6. 功能三：倒地自起（`rec_core.c`）

**几何**：STEP 模型显示底盘是船底形。车身向前、向后都要到 78.5° 才第一次碰到平地，倒地后停在约 79°。旧的长方体模型算出来是 44.5°，按那个数字看自起几乎不可能。

**为什么要「推—滑行—推」**：一直全力推，0.2 s 内轮速就饱和（约 1 m/s），车身还没起来就倒回去了，孪生 0–2/4 成功。改成：先推 0.2 s，再滑行 0.15 s（车身继续往上翘，轮子同时减速，腾出速度余量），然后第二次推。结果 12 V 8/8、10.5 V 8/8 成功，约 0.9 s 立起。

**状态机**（`REC_Step`）：
1. **触发**：只在从直立状态倒下（|倾角| 越过 45°）后，或车躺着时启动（`REC_Stopped` 把关电机期间算作「直立」）。
2. **等待**：倒下后 5 s 内，车必须躺稳（|倾角| > 60°，角速度 < 30 °/s，持续 0.5 s）。
3. **第一推**：朝倾倒方向全力推（死区之上 1320 PWM），40 个周期（0.2 s，`ap`）。
4. **滑行**：电机输出 0，30 个周期（0.15 s，`aw`）。
5. **第二推**：全力推，最多 120 个周期，直到 |倾角| < 30°，然后交给平衡控制器（处于回正状态）。
6. **上限**：失败就关电机 1 s 再试，最多 3 次，然后上锁。

**安全**：
- 推力有时限：每次最多 0.2 + 0.15 + 0.6 s，最多 3 次。
- 只在刚倒下或躺着启动时触发。
- `as0` 可完全关闭自起；关闭后倒地的车轮子转动 0 s。
- 孪生里自起过程车会移动：平均约 50 cm，最多 66 cm，要留出空地。

**局限**：带 1 kg 负载起不来（电机力矩不够），试 3 次共约 2.4 s 后上锁。「飞轮」方案（`sr_rev`，第一推反方向）更差（1–4/8），默认关闭。

---

## 7. 功能四：电机补偿模块（`mc_core.c`，默认关闭）

10-12 做的，针对随电池电压、温度和磨损变化的死区。**默认关闭**：关闭时输出和原厂 `PWM_Ignore`（1480 / 1455）逐位相同。用 `a1` 打开后：
- **电压前馈**：每个周期把补偿乘以 12 V / 电池电压。
- **架空自标定（`ac`）**：轮子离地，每个轮子、每个方向从 1300 PWM 慢慢加，编码器一动就记下起转值（折算到 12 V）。
- **可选的小指令保持（`ah`）**：|u| < 设定值时输出 0，小指令就不会穿过齿隙。

在线估计已编译掉（`MC_ONLINE 0`）以节省 Flash。孪生里它往反方向走：平衡时轮子不转，常常是车身顶着，所以「没转」量不出摩擦。10-12 孪生结果：死区升高 100 时摔倒 6 → 0；死区降低 80 时抖动降到 1/5；正常情况下不变。

---

## 8. Flash 预算

这块板子只有 64 KB 以内可靠。v5（自起）超了 48 字节，于是做了以下处理：
- 除模式 27 外，其余模式全部移出编译：巡线、CCD、电磁、K210、雷达、跟随，以及旧的负重模式 22–26。`removed_modes_v5_stubs.c` 为固件其余部分仍引用的函数提供空实现。
- `Mode_select` 只显示模式 27，不再切换。
- 电机补偿的在线估计编译掉。
- `rec_core.c` 用 `#pragma Ospace` 编译。
- 蓝牙、App 协议、OLED、串口控制台全部保留。

结果：代码 52332 + 只读数据 6304 + 读写数据 944 字节，镜像结束于 0x0800 E5C4，剩余 6716 字节。

---

## 9. 数字孪生和 STEP 几何

- **引擎和模型**：MuJoCo。电机力矩上限、死区（1480/1455）、齿隙、摩擦、IMU 噪声、200 Hz 控制周期都在模型里。参数来自实车测量（`e026 keil\realcar\PARAMS.md`）。
- **几何来自 STEP（10-21）**：
  - `step_geometry.py` 用 cascadio + trimesh 读取 `STM32_Balance_V2.STEP`，输出 `stm32_step_geom.py`：轮半径 33.5 mm、轮距 167 mm、车身包围盒、车身凸包、78.5° 碰地角。
  - 孪生的碰撞体就用这个凸包。
  - 顶板在轮轴上方 105.2 mm。实际负重**放在**顶板上，重心更高，所以 v5.2 的测试用了 0.15–0.30 m。
- **同一份代码**：`v51_dll.c` 和 `rec_dll.c` 把固件的 `v4_core.c`、`rec_core.c` 包成 DLL，Python 孪生每个周期调用它们，所以孪生跑的就是要烧进车里的 C 代码。
- **已知差距**：
  - 孪生的抖振比实车弱约 30 倍，所以抖动相关阈值必须在实车上定；
  - 默认负重高度（0.105 m）下孪生完全看不到「越摆越大」，要把重心抬高才能复现；
  - 大部分测试里电池电压固定。

---

## 10. 源码说明

### 10.1 文件和函数

| 文件 | 函数 | 作用 |
|---|---|---|
| `v4_core.c` | `V4_GainsFor(f, g)` | 按档位 f 插值 GN 和 GH。 |
| | `V4_Init(s)` | 开机状态：f=1、探测中、各测量量清零。 |
| | `V4_CmdEdge(s)` | 按键变化：开始 1.5 s 升档宽限期。 |
| | `rearm(s)` | 事件：f=1、重新探测、清除上限。 |
| | `revert(s, f)` | 快速回退到原档位，不重新探测。 |
| | `V4_Step(s, angle, gyro, moving)` | 一个周期的全部决策，顺序：(1) 倾角预测、阈值、事件 / 事件状态退出；(2) 各测量量；(3) 探测、快速判轻、提前结束探测、行驶中判轻；(4) 已锁定时：抖动降档、小幅抖动规则、底档清扫、摆动升档、慢逃逸、上限。返回 f。 |
| `v4_adapt.c` | `V4_Tick(enc_l, enc_r)` | 运行 `REC_Step`；检查 `Turn_Off`（电机关着就保持开机状态）；送入按键变化和「是否在行驶」；调用 `V4_Step` 并写入增益。 |
| | `apply(f)` | 写入六个增益；速度积分无扰缩放。 |
| | `V4_RecOverride(&l, &r)` | 回正时 `Encoder_Integral = 0`；自起时写入推力。 |
| | `cmd_moving()` | 「有没有行驶或转向指令」，判定规则和原厂 `Velocity_PI` / `Turn_PD` 完全相同。 |
| `rec_core.c` | `REC_Step(...)` | 自起状态机、保护锁、大角度回正。返回 NORMAL / RECOVER / DRIVE。 |
| | `REC_Stopped(s)` | 用户关着电机时算作直立，躺着启动也能触发自起。 |
| `app_motor.c` | `Turn_Off()` | 模式 27：±45°、保护锁、自起时绕过。 |
| `tune_io.c` | 串口控制台 | 状态、录制、第 11 节的 `a…` 命令。 |

### 10.2 常数和取值理由

| 常数 | 值 | 理由 |
|---|---|---|
| `DOWN_SPLIT` | 15 °/s | 孪生定的抖动分界，也是提前结束探测的门槛。 |
| `v4_dnsmall`（`ad`） | 7.5 °/s | 刚好高于实车 f=0 平稳读数（4.5–7.4）。 |
| `DOWN_N` / `SM_N` | 0.15 s / 0.3 s | 足够快，约 1 s 内消除抖动；小幅规则慢一点，避免被噪声触发。 |
| `DOWN_WAIT` | 0.2 s | 快表（0.1 s）到那时已跟上新档位。 |
| `LOW_F` / `LOW_N` | 0.15 / 0.5 s | 实车上档位卡在 0.1，抖动又低于 15。 |
| `v4_swup`（`au`） | 1.5° RMS | 实车自身 4.3 Hz 怠速极限环约 0.47° RMS，不能触发升档。 |
| `SW_GROW_K`、`SW_GROW_R` | 0.5（= 0.75°）、每 0.5 s ×1.2 | 尽早抓住变大的摆动。孪生：0.20 m 放 2 kg，1.4 s 内从 0.3° 涨到 4°。 |
| `SU_N` / `SU_WAIT` | 0.1 s / 0.3 s | 同上：等到 1.5° 已经太晚。 |
| `PL_N` | 3 s | 探测锁定后的观察窗口，行驶中也有效。 |
| `SW_FAIL_UP` | +0.3 | 摆动中发生事件，说明当时的档位太软。 |
| `SU_AFTER_DN` | 1 s | 抖动降档后马上不做摆动升档。 |
| `CEIL_N` / `CEIL_ESC` | 10 s / 12° | v5.1：防止来回升降；车真的要倒时允许突破。 |
| `REC_STALE_N` | 3 s | 平衡角变了（换了负载）时，事件状态退出并重设参考角。 |
| `v4_light_split` / `v4_mid_split` | 25 / 20 | 孪生扫描（`split_sweep.log`）。 |
| `F_LIGHT` / `V4_F_MID` / `F_HEAVY` | 0 / 0.2 / 0.4 | 0：实车用 0.12 仍会抖；0.4：v4 基准最优。 |
| 回正：`on_deg` / 退出 | 20° / 2°、30 °/s、4 计数 | 孪生 `rec_final_checks.log`。 |
| 回正：保护锁 | > 40° 持续 1 s，< 10° 解锁 | 覆盖手扶、卡住、带载躺地。 |
| 自起：推 / 滑行 / 第二推 / 交接 | 0.2 s / 0.15 s / ≤ 0.6 s / 30° | 扫描结果：只有 40×30 和 50×20 两组在两种电压下都成功。 |

---

## 11. 串口命令（115200 波特率，每条命令以回车结束）

| 命令 | 作用 |
|---|---|
| `?` | 打印状态。`#mc` 行末尾：`re=` 回正开关 / 回正次数 / 上锁次数；`sr=` 自起开关 / 尝试 / 成功 / 第一推 / 滑行；`sp=` 轻 / 中分界；`op=` `v4_opt` / 宽频阈值 / 摆动阈值×10 / 小幅抖动阈值×10。 |
| `r` / `s` | 开始 / 停止录制 200 Hz CSV（`seq,gyro,ang_c,ml,mr,el,er,inj,osc_c,k_c,lvl,cs,tv`）。`lvl` = f×100，锁定 +1000，事件状态 +2000，回正 +4000，保护锁 +8000，自起推 +16000。 |
| `ae0` / `ae1` | 大角度回正关 / 开。 |
| `as0` / `as1` | 倒地自起关 / 开。 |
| `ap<n>` / `aw<n>` | 自起第一推 / 滑行时长，单位：5 ms 周期数。 |
| `ak<x>` / `aj<x>` | 探测的轻 / 中分界（°/s）。 |
| `ax<n>` | 换档规则位开关。v6 = 50；`ax18` = 去掉摆动升档；`ax30` = v5.1 的开关组合（v6 的降档改动仍然生效）。 |
| `au<x>` | 摆动升档阈值，单位 ° RMS（默认 1.5）。 |
| `ad<x>` | 小幅抖动降档阈值，单位 °/s（默认 7.5）。 |
| `ay<x>` | 宽频振荡阈值（只在开位 8 时使用）。 |
| `a1` / `a0`、`ac`、`ah<n>` 等 | 电机补偿（第 7 节）。 |
| `p q v b t y<x>` | 直接改某个增益（平衡 Kp / Kd、速度 Kp / Ki、转向 Kp / Kd）。模式 27 下 v4 会在下一个周期把增益改回去。 |

重新上电会恢复全部默认值。

---

## 12. 结果

所有孪生结果都带原厂基线，**罚分越低越好**。每个结果都标明了用哪个版本测的。最后三个版本（v5.2 test2–4 = v6）是按用户要求**根据实车反馈**修改的，没有在孪生里重新跑分。

### 12.1 基准（停—走—停路线，5 种工况 × 3 个随机种子，加权）

| 控制器 | 总分 | 空载 ×2 | 1 kg ×2 | 2 kg ×1 | 4 kg ×1.5 | 空载 + 推 3 N ×1 |
|---|---|---|---|---|---|---|
| 原厂正常档 | 111.45 | 3.74 | 12.61 | 30.00（摔） | 30.00（摔） | 3.75 |
| 原厂负重档 | 115.41 | 23.85 | 17.36 | 3.74 | 3.59 | 23.85 |
| v4 正式版 | 31.83 | 3.81 | 5.93 | 3.54 | 3.34 | 3.80 |
| v5.1 | 27.09 | 3.66 | 3.75 | 3.54 | 3.34 | 3.70 |
| v5.2（10-24，"d" 版） | 26.97 | 3.66 | 3.40 | 3.54 | 3.34 | 4.30 |

评分规则和 v4 相同：每种工况依次开到 0.9 m、退回 0.3 m、再开到 1.2 m，每段都要在 5 cm 内停住 5 s。罚分 = 变稳时间 + 0.5 × 抖动；摔倒罚 30 × 未完成段数占比。

### 12.2 高重心（负重放在顶板上），在站着的车上放负重，15 s，3 个随机种子

每格 = 摔倒次数 / 最后 3 s 摆动幅度（°） / 最终档位 / 最后 5 s 换档次数。

| 重心 0.20 m | 0.5 kg | 1 kg | 2 kg | 3 kg |
|---|---|---|---|---|
| 原厂正常档 | 摔 3/3 | 摔 3/3 | 摔 3/3 | 摔 3/3 |
| 原厂负重档 | 0 / 0.12 / 1.0 / 0 | 0 / 0.15 / 1.0 / 0 | 0 / 0.20 / 1.0 / 0 | 0 / 0.26 / 1.0 / 0 |
| v4 正式版 | 0 / 0.19 / 0.4 / 0 | 0 / 1.56 / 0.4 / 0 | 0 / 1.71 / 0.4 / 2 | 摔 3/3 |
| v5.1 | 0 / 0.22 / 0.4 / 0 | 0 / 0.45 / 0.4 / 0 | 0 / 1.88 / 0.4 / 2 | 摔 3/3 |
| v5.2（"d" 版） | 0 / 0.22 / 0.4 / 0 | 0 / 0.45 / 0.4 / 0 | **0 / 0.20 / 1.0 / 0** | 摔 3/3 |

负重在 0.20 m 时跑路线（3 个随机种子，摔倒次数）：

| 负重 | 原厂负重档 | v4 | v5.1 | v5.2 "d" 版 | v5.2 "e" 版（摆动升档更快，v6 保留） |
|---|---|---|---|---|---|
| 1 kg | 0 | 3 | 3 | **0** | — |
| 2 kg | 0 | 3 | 3 | 2 | **跟踪的 2 个种子 0 次** |

0.25 m 及以上，从 1 kg 起连原厂负重档也会摔（`v52_hmatrix1.log`），原因是速度环（第 4.9 节）。

### 12.3 消除抖动（空车强制 f=1，3 个随机种子）

| | v4 | v5.1 | v5.2 "d" 版 |
|---|---|---|---|
| 快表降到 5 °/s 以下的时间 | 不会（一直在 f=1） | 3.81 s | **1.21 s** |
| 最终档位 | 1.0 | 0.3 | 0.1–0.2 |

v6 在此基础上又加了小幅抖动规则和底档清扫，这两条是针对实车做的。

### 12.4 固定负载和交换负载（重心在顶板高度）

| | 原厂正常档 | 原厂负重档 | v4 | v5.1 | v5.2 |
|---|---|---|---|---|---|
| 0–4 kg 每 0.5 kg，摔倒 | 1 kg 摔 2/3，1.5 kg 起全摔 | 0（0–1 kg 一直抖） | 2（0.5 kg） | 0 | 0 |
| 0/1/2/3/4 kg 两两互换（60 次） | — | — | 0 摔 | 0 摔 | 0 摔 |

v5.2 换负载后抖得更久（1→3 kg、1→4 kg，约 8 s）的两个 bug 通过跟踪找到并修复（`v52_tswap13.log`）：重新探测时清除上限；锁定后降档先等待。修复后同样的交换稳定在 0.4，摆动 0.15–0.2°（`v52_tswap_f.log`）。

### 12.5 大倾角和自起（空载，12 V）

| | 原厂 / v4 | v6 |
|---|---|---|
| 静止斜放 40° 后松手 | 救不回（40° 切电机） | 4/4，0.26 s 立起，1.5 s 停稳 |
| 平衡中压到 42° 再松手 | 救不回 | 4/4 |
| 躺在地上（约 79°），12 V / 10.5 V | 一直躺着 | 8/8 / 8/8，倒下后约 1.9 s 立起 |
| 躺着时启动（第二次按 KEY1） | 一直躺着 | 8/8 / 8/8 |
| 100 种冲击起始状态（5 个角度 × 5 个角速度 × 4 次）：不碰地接住 / 最终立直 | 原厂 42 / 42，v4 51 / 51 | 56 / **100** |
| `as0` 时倒地车轮子转动时间 | 0 s | 0 s |

### 12.6 实车（2026-10-21 至 10-24，用户反馈）

| 版本 | 现象 | 处理 |
|---|---|---|
| v5 | 自起有效；带负重比以前差 | v5.1 |
| v5.1 | 带负重仍越晃越大；空车不如原厂正常档平稳 | v5.2（按频率换档） |
| v5.2 test1 | 空载抖振下不来，f 卡 0.1，或 0.0 也抖 | test2（摆动 / 抖动分离） |
| v5.2 test2 | 加负载没问题；取下后档位卡在 0.2 | test3（任何状态都降档） |
| v5.2 test3 | 用户提问：小幅高频振荡呢？ | test4 = **v6**（小幅抖动规则） |

---

## 13. 开发过程，2026-10-03 → 10-24

1. **10-03 至 10-07：v4 发布和后续**。
   - 整理 v4 正式版文档；尝试实车站立 CEM 调参。
   - 发现车「几乎完全静止」的瞬间来自死区，不是 PID。
2. **10-12：死区和齿隙补偿**（`v4_mc_20261012`）。
   - 死区随电池电压变化，所以方案是电压前馈 + 架空自标定，另加小指令保持来应对齿隙。
   - 没车期间在孪生里完成并检验，写好拿到车就能做的实车测试计划。默认关闭发布。
3. **10-20：大角度回正**（`v4_rec_20261020`）。
   - 发现 40° 的限制是固件切电机角。改为 45° 切电机，抬车期间保持积分为 0。
   - 孪生：静止 40° 4/4；日常基准不变（31.83）。
4. **10-21：自起和 STEP 几何**。
   - 用户指定以 STEP 为准；碰地角 78.5°，不是 44.5°。
   - 推—滑行—推自起，两种电压都 8/8；躺着启动也能起来。
   - 保护锁和 `as0` 保证安全。
   - 冲击包络表明：加大切电机角没有帮助，限制来自物理。
   - 发布 **v5**。
5. **10-21：负重档位，v5.1**。
   - 实车：v5 带负重变差，小负载「抖 → 平稳 → 再抖」循环。
   - 在孪生里逐项测试 v5.1 的几个开关位（分级升档、提前结束探测、带符号倾斜、宽频检测、上限），保留了其中三个。
   - 删掉模式 27 以外的所有模式，腾出 Flash。
6. **10-24：高重心，v5.2**。
   - 实车：负重放在顶板上，出现肉眼可见、越来越大的摆动。
   - 孪生只有把负重重心抬到 0.15–0.25 m 才能复现。
   - 用户定下的规则成为设计：低频摆动 → 升档，高频抖动 → 降档，平稳 → 不换，两者不冲突。
   - 孪生跟踪随后发现并修复了：升档太晚、探测重新锁到太软的档、摆动漏进抖动计、换负载后残留的上限、锁定后马上又降档。
   - 增益扫描找到 ≥ 0.25 m 时的速度环原因。
7. **10-24：实车迭代 test1 → 4**。
   - 用户叫停孪生跑分（「以实车为准」）。
   - 三次实车反馈，当天三次修复（第 12.6 节）。
   - test4 定为最终版 → **v6**。

---

## 14. 已知局限和实车标定

1. **抖动阈值是在孪生里定的**，即 15 °/s 和小幅抖动 7.5（`ad`）；而孪生的抖振读数偏低约 30 倍。如果带载车平稳时档位还一直往下降，就调高 `ad`（10–12）；如果小幅抖动还消不掉，就调低（6）。调之前先发 `?`，再录一小段数据。
2. **摆动阈值 `au`（1.5° RMS）**：平稳时也升档就调高（2）；明显在摆却不升档就调低（1）。
3. **重心 ≥ 0.25 m**：v6 仍可能摔倒。孪生表明那里要缩小速度环，而档位线反而把它加大。需要时的下一步：加第二个档位维度，在 f=1 仍持续摆动时把速度环增益缩到 0.5–0.6 倍。
4. **自起**需要约 50–66 cm 空地，带负载时不起作用。
5. **v6 没有在孪生里重新跑分**（用户决定）。最后一个跑过分的是 v5.2 "d/e/f" 版；test2–4 的改动是根据实车做的。
6. **电池**：低于 9.6 V 切电机，和原厂一样。自起在 10.5 V 和 12 V 下检验过。

---

## 15. 怎么重新编译

1. Keil MDK 5：在 `e026 keil\v4_work_20261003` 下运行 `UV4.exe -b USER\stm32_Balance_Car.uvprojx -j0 -o build.log`（或打开工程按 F7）。应为 0 错误；`bsp.c` 的两个警告来自原厂代码。
2. 检查镜像结束地址：`OBJ\stm32_Balance_Car_L.hex` 里最高数据地址必须低于 0x0801 0000。
3. 孪生 DLL（w64devkit gcc），在 `balance_bot\scripts\rl` 下：
   `gcc -O2 -shared -I<APP\PID> -o v52core.dll v51_dll.c <APP\PID>\v4_core.c -lm`。
   `rec_core` 用 `rec_dll.c` 同样编译。

---

## 16. 相关文件位置

| 内容 | 位置 |
|---|---|
| 编译工程（全部源码） | `e026 keil\v4_work_20261003` |
| 每次修改前的备份 | `e026 keil\v4_backup\before_*` |
| 以前的发布文件夹 | `v4_release_20261003`、`v4_mc_20261012`、`v4_rec_20261020`、`v5_20261021` |
| 数字孪生 | `e026\balance_bot_windows\balance_bot`（脚本在 `scripts\rl`，日志在 `runs_e2e`） |
| STEP 模型 | `Downloads\STM322\10.attch\3Dmodel\STM32_Balance_V2.STEP` |
| 实车测量工具 | `e026 keil\realcar` |

---

## 17. 分工

- **用户（人）**提出了全部目标和约束，包括：40° 回正和 45° 切电机；倒地自起，且倒地后轮子不能空转；以 STEP 文件为几何依据；低负载档位要更细；「低频摆动升档、高频抖动降档、平稳不换、两者不冲突」；「高频小幅振荡要快速消除」；删模式时保留蓝牙；「以实车为准」。用户还完成了全部实车测试，并准确报告每一次失败：车做了什么、在哪个档位、带什么负载。
- **大语言模型（Claude）**把每个目标变成具体机制：编写 C 和 Python 代码；搭建孪生实验（STEP 导入、高重心测试、增益扫描、过程跟踪）；编译每个 hex 并检查大小；解释查到的每个原因。
