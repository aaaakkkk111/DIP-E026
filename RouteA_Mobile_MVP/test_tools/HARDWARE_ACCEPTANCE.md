# Route A firmware v1.5 / Android v1.4 hardware acceptance

Record phone model, Android version, battery voltage, firmware/APK hashes, and
the complete app log. Keep both wheels clear of the table during stages A and B.

## A. Link and independent motor channels

1. Power the car, rotate a wheel until the OLED shows `21.TEST MODE`, and press
   Key1 once to select it. Do **not** press the balance-start key.
2. Connect the app. Pass requires `BLE UART ready`, an automatic GET response,
   firmware `RA1.5-TEST-MODE`, live PID `96/48/62/31/17/20`, and balance state
   `R0`. The status panel must show the balance, training lock, firmware state,
   and phone transaction state.
3. Press each diagnostic button once and record whether the named wheel moved,
   its direction, EL/ER, and C1/C2/C3/C4.

| Command | Required active CCR pair | Other pair | Physical result |
|---|---:|---:|---|
| Left + | C1=0, C2=1500 | C3=C4=0 | left wheel turns |
| Left - | C1=1500, C2=0 | C3=C4=0 | left wheel reverses |
| Right + | C3=1500, C4=0 | C1=C2=0 | right wheel turns |
| Right - | C3=0, C4=1500 | C1=C2=0 | right wheel reverses |

The active wheel encoder must become nonzero in at least one telemetry sample;
the inactive wheel should remain near zero. Each pulse must stop automatically
within 0.8 s.

If the expected CCR value is present but that wheel does not move, report it as
a motor-driver/wiring/channel failure. Do not attempt PID tuning. If the CCR
mapping is wrong, report the raw frame and button pressed as a firmware failure.

## B. TEST MODE baseline (official StandardMode parameters)

1. Only continue after all four motor diagnostics pass. Put the car on a safety
   tether, hold it near upright, and press Key1 to start balance.
2. Pass requires `R1`, both wheels responding in the correcting direction,
   and no wheel remaining stationary while its command/CCR is nonzero.
3. Require 10 continuous seconds first, then 30 continuous seconds without a
   fall, motor shutdown, or sustained PWM saturation. Use these reporting
   thresholds: peak absolute angle below 10 degrees, battery above 9.6 V, and
   no repeated `ML` or `MR` values at ±2600.
4. Read PID again. It must still be `96/48/62/31/17/20` unless you explicitly
   applied a manual pre-training change.

## C. Manual tuning and training lock

1. Before training, change AP by at most 1%, apply, and read it back. Pass
   requires an MSET ACK and matching GET response.
2. Restore the baseline, then press **开始训练并锁定人工调参**.
3. Pass requires `ACK,TRAIN_START`, `L1`, disabled manual PID controls, and
   rejection of legacy PID writes with `$TrainingLocked#`.

## D. Harness/LLM transaction loop

1. Collect at least 10 s of baseline data, then finish the baseline.
2. Request the Simulation Harness proposal or a DeepSeek proposal. A valid
   result must contain all six PID values, change only AP/AD, and stay within
   5% of the champion.
3. PREP must produce `ACK,PREP` while GET still reports the champion.
4. APPLY must produce trial state `S2`; heartbeat must continue every 500 ms.
5. ROLLBACK must restore all six champion values. In a separate safe run,
   ACCEPT must promote all six candidate values.
6. During a trial, close the app. Pass requires local rollback and commanded
   motion stop within 2.5 s.

## Report template

```text
Firmware HEX hash:
APK hash:
Phone / Android:
Battery:
GET PID and R/L/S fields:
Left+: moved/direction, EL/ER, C1/C2/C3/C4:
Left-: moved/direction, EL/ER, C1/C2/C3/C4:
Right+: moved/direction, EL/ER, C1/C2/C3/C4:
Right-: moved/direction, EL/ER, C1/C2/C3/C4:
10 s / 30 s balance result:
Peak angle, PWM saturation, fall description:
MSET/readback result:
TRAIN lock result:
PREP/APPLY/ROLLBACK result:
PREP/APPLY/ACCEPT result:
Full app log attached: yes/no
```
