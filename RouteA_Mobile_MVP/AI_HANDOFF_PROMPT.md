# AI handoff prompt

Copy the prompt below into another AI together with a link to this branch.

```text
You are taking over development and verification of a Yahboom STM32F103
two-wheel balance car project. Read the complete repository before suggesting
changes. Treat the C implementation and generated protocol frames as the
source of truth when comments or older documents disagree.

Repository layout:
- firmware/: Keil MDK ARMCC 5.06u7 STM32 project based on Yahboom's official
  Large Program standard-library project.
- android/: native Kotlin Android BLE application.
- protocol/: Route A CRC-framed BLE protocol documentation.
- test_tools/: protocol, transaction, Flash-range and mode-isolation tests.
- releases/: tested build artifacts and recorded hashes.

Hardware and transport:
- STM32F103 balance-car controller.
- JDY-23 BLE module connected to STM32 UART5 at 9600 8N1.
- BLE service UUID 0000FFE0-0000-1000-8000-00805F9B34FB.
- BLE characteristic UUID 0000FFE1-0000-1000-8000-00805F9B34FB.
- FFE1 is used for writes and notifications.
- BLE writes are split into 20-byte chunks; received bytes require streaming
  reassembly because frames can be fragmented or coalesced.

Firmware design:
- The 20 official modes retain their original enum numbers and behavior.
- Mode 21 is Test_Mode and is shown on the OLED as "21.TEST MODE".
- Route A initialization, pre-balance motor diagnostics, telemetry, manual PID
  setting, training lock, candidate transactions and rollback execute only in
  Test_Mode. Do not add Route A effects to modes 1-20.
- Test_Mode starts from the official StandardMode baseline:
  AP=96, AD=48, VP=62, VI=31, TP=17, TD=20 in protocol display units.
- Internal firmware scaling is AP/VP/TP x100; AD/VI/TD are used directly.
- Control runs at 5 ms. Route A PID changes are applied atomically at a control
  boundary.
- Training supports GET, MSET, TRAIN START/STOP, PREP, APPLY, ACCEPT, ROLLBACK,
  HEARTBEAT and STOP. During training, manual PID writes are locked.
- Only AP and AD may change during the current neural/LLM balance stage; all
  proposals contain six PID values and the firmware enforces a 5% AP/AD cap.
- Trial rollback is local and is triggered by TTL/heartbeat timeout, excessive
  angle, low voltage or sustained PWM saturation.
- Fast telemetry includes angle, gyro, encoders, computed motor commands,
  Route A state, lock/balance flags and TIM8 CCR1-CCR4.
- Slow telemetry includes battery, loop components, all six live PID values,
  dropped-TX count, lock/balance flags and firmware version.

Flash constraint:
- This physical controller showed unreliable behavior when the custom image
  extended into the problematic high Flash range.
- The project therefore uses ARMCC optimization level 1 and deliberately caps
  the complete load region at 0x08000000..0x0800FFFF.
- Firmware v1.5 ends at 0x0800DA57. Never remove the 64 KiB linker guard.
- Run test_tools/check_hex_range.py on every generated HEX.

Android design:
- Package com.example.routeamobilemvp, Android app version 1.4.
- Uses Core Android BLE GATT logic, not Bluetooth Classic serial ports.
- Scans/connects to YahBoom_BL, discovers FFE0/FFE1, enables notifications and
  sends an automatic GET to prove bidirectional communication.
- Displays live six-value PID feedback separately from editable PID inputs.
- Displays balance state, training lock, firmware transaction state and phone
  transaction state in real time.
- Allows manual PID tuning only before training.
- Implements baseline collection, deterministic Simulation Harness proposals,
  optional DeepSeek proposals, review, PREP/APPLY/ACCEPT/ROLLBACK and 500 ms
  heartbeats.
- API keys are stored with Android Keystore and are never committed.

Safety and development constraints:
- Keep the official modes 1-20 logically unchanged.
- Keep Route A isolated to Test_Mode.
- Do not change UART5 baud rate or BLE UUIDs without hardware evidence.
- Do not equate a transmitted command with an applied PID; require firmware
  ACK/state and readback.
- Preserve the local rollback and motor-stop safety behavior.
- Do not run autonomous LLM-to-motor updates without explicit user review.
- Do not claim hardware verification unless results were obtained on the car.
- Do not modify the vendor source directory referenced in the documentation;
  work only in this copied project.

Before making a change:
1. Read README.md, BUILD.md, protocol/route_a_protocol.md,
   FLASH_BOUNDARY_FIX.md and test_tools/HARDWARE_ACCEPTANCE.md.
2. Inspect firmware/USER/main.c, USER/myenum.h, APP/mode/app_mode.c,
   APP/app_control.c, APP/RouteA/route_a.c, BSP/Timer/bsp_timer.c,
   BSP/Bluetooth and BSP/Motor.
3. Inspect Android MainActivity.kt, BleClient.kt, Protocol.kt, Experiment.kt
   and DeepSeekClient.kt.
4. State which official path or Test_Mode path will change and why.

Required validation after firmware changes:
- Full Keil rebuild using ARMCC 5.06u7.
- Zero compile/link errors.
- HEX maximum address below 0x08010000.
- Python protocol and mode-isolation tests.
- Confirm official mode enum values 0-19 remain unchanged.

Required validation after Android changes:
- Debug and release JVM unit tests.
- assembleDebug.
- Verify no API key, local.properties or signing private key is committed.
- Real BLE/motor/balance behavior must remain marked unverified until tested on
  the physical car.

First summarize the architecture and invariants you found. Then explain the
smallest safe implementation plan for the requested change. If asked to make
the change, implement it, run the applicable tests, report exact artifacts and
hashes, and clearly separate machine verification from physical-car results.
```
