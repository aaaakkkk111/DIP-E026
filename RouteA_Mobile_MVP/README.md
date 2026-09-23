# Route A Mobile MVP

This deliverable adds a guarded, phone-driven PID experiment loop to the
Yahboom STM32 balance car. Nothing under
`D:\DIP\STM32平衡车_V2` was modified.

## What is implemented

The firmware baseline is a direct copy of the official Large Program standard
library project at
`D:\DIP\STM32平衡车_V2\10.附件\源码汇总\0.Large program\1.standard libraries(keil)\stm32_Balance_Car_L`.
The original source directory remains untouched. Firmware v1.5 preserves the
official mode numbers and behavior for modes 1 through 20, then appends mode 21
`TEST MODE`. Route A initialization, diagnostics, telemetry, manual PID writes,
training lock and LLM transactions run only in `TEST MODE`. Its baseline calls
the same official setup functions and uses the same PID values as StandardMode.

The existing UART5/JDY-23 link remains 9600 8N1 with BLE FFE0/FFE1, and the
legacy Yahboom `$...#` control and PID packets remain available. Route A adds:

* CRC-protected `P1` commands and versioned `T1` telemetry;
* an interrupt-driven 512-byte UART TX queue so telemetry never busy-waits in
  the 5 ms control path;
* PREPARE, APPLY, ACCEPT, ROLLBACK, GET, STOP and HEARTBEAT transactions;
* `MSET` manual six-value tuning before training and a firmware-enforced
  `TRAIN START/STOP` lock that also blocks legacy PID writes;
* atomic six-value PID updates at a 5 ms control boundary, while only AP/AD may
  differ from the champion;
* 5% firmware-side AP/AD change limit, 1–30 s candidate TTL, 2 s heartbeat
  timeout, 25 degree trial guard, low-voltage guard, saturation guard and local
  rollback;
* a pre-balance, time-limited motor diagnostic for each TIM8 output pair.
  The Android display reports controller commands, encoder counts and the
  actual CCR1–CCR4 register values;
* Large Program Normal-mode startup values AP=96, AD=48, VP=62, VI=31,
  TP=17, TD=20, with the original `Turn_Off` protection unchanged;
* a native Kotlin Android app with BLE scan/connect, official-compatible
  20-byte FFE1 writes, notification stream reassembly, live plot, metrics, a fixed
  Harness suggestion, DeepSeek suggestion, manual review, and transaction UI.

The v1.5 firmware is compiled with ARMCC optimization level 1 and constrains
the complete Flash load image to `0x08000000..0x0800FFFF`. The reported A/B
hardware tests indicate that this board cannot reliably use the relevant high
Flash range: the earlier v1.3 load image ended at `0x08011843`, while v1.5 ends
at `0x0800DA57`. The Keil linker
now fails the build if future changes cross that range. This is a board-specific
compatibility constraint; it is not a general STM32F103RC Flash limit.

The app defaults to a configurable DeepSeek-compatible endpoint and model.
The API key is encrypted with Android Keystore AES-GCM. The app and firmware
independently enforce the change limit. No model output is sent
until the user presses PREPARE, and APPLY is disabled by state until PREP ACK.
The mobile LLM boundary follows the Simulation Route A v4 schema: strict
`propose/hold/rollback`, balance-stage isolation, a complete six-value
candidate, expected effect, requested test and confidence. The real firmware
keeps the stricter 5% AP/AD per-candidate cap.

## First run

1. Flash `releases/routea_mobile_mvp_v1.5.hex` with the same Keil/J-Link
   procedure used for the original project. Put the car on a stand with both
   wheels free.
2. Power the car. Rotate a wheel until the OLED shows `21.TEST MODE`, then press
   Key1 to select it. The other 20 choices retain their official behavior.
   Install `releases/routea-mobile-mvp-v1.4-debug.apk`, grant Nearby devices, scan,
   select `YahBoom_BL`, and connect.
3. A successful notification subscription automatically sends GET. Confirm
   that a PID response appears; this verifies phone-to-car writes and
   car-to-phone notifications. Put down the car and press Key1 to start balance.
4. Keep both wheels raised and do not press the balance-start key yet. Run
   **左轮 +**, **左轮 −**, **右轮 +**, and **右轮 −**. Each pulse lasts about
   0.6 s. Use the acceptance table in `test_tools/HARDWARE_ACCEPTANCE.md`.
5. Before training, edit the
   six PID fields and press **应用人工 PID**, then read back the values.
6. Press **开始训练并锁定人工调参**. Both the controls and STM32 write path
   remain locked until **停止训练并解锁**.
7. Collect a baseline. Use **Simulation Harness 建议** for the deterministic
   Simulation-compatible rule, or configure DeepSeek and request a proposal.
8. Review the proposal, press PREPARE, wait for `ACK,PREP`, then press APPLY.
   The app sends heartbeat every 500 ms. ACCEPT promotes the candidate;
   ROLLBACK restores the prior champion.

Start with the wheels off the ground. Phone disconnect is detected indirectly
by heartbeat loss, so rollback can take up to about 2 seconds. TTL and the
original firmware hard shutdown remain local and do not depend on Android or
the cloud.

## Verification

Run `test_tools/run_tests.bat` for stream, CRC, legacy coexistence and
transaction tests. Android JVM tests cover the same parser, strict LLM JSON
schema, percent limits, and state machine. See `BUILD.md` for builds and
`releases/build-info.txt` for recorded outputs and checksums.

Run `python test_tools/check_hex_range.py releases/routea_mobile_mvp_v1.5.hex`
to independently verify that the HEX contains no data at or above
`0x08010000`.

The Keil build and offline tests are machine-verifiable. A successful build
does not prove radio operation, sensor correctness, balance stability, or safe
motor behavior. Those require the staged hardware checklist in
`test_tools/HARDWARE_ACCEPTANCE.md`; this repository does not claim those tests
were performed.
