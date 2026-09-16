# Route A: LLM-assisted PID auto-tuning in MuJoCo

This package implements a runnable, review-first Route A loop:

`MuJoCo plant -> 5 ms virtual STM32 PID -> 9600 baud byte stream -> PC metrics -> LLM proposal -> deterministic Harness -> candidate trial -> accept/shrink/rollback`

The LLM is an adviser only. It cannot produce PWM, command motors, change safety
limits, bypass the Harness, or accept its own proposal. In Manual Review mode a
human must approve the proposal before the candidate PID is transmitted. The
post-trial accept/rollback decision is always deterministic.

This is a simulation and migration tool. Passing its tests or improving its
MuJoCo score does **not** prove that a real robot is safe or stable.

## Quick start on this PC

From PowerShell:

```powershell
Set-Location D:\DIP\Simulation
& 'C:\Users\stato\AppData\Local\Programs\Python\Python313\python.exe' -m pip install -r requirements-route-a.txt
& 'C:\Users\stato\AppData\Local\Programs\Python\Python313\python.exe' -m route_a.main
```

The control UI and a separate MuJoCo 3D Viewer open automatically. The 3D window
is separate because `mujoco.viewer.launch_passive` is a native GLFW window; use
**Reopen MuJoCo 3D viewer** if it was closed, and use `--no-viewer` only for
headless UI diagnostics. The UI defaults to `MockLLM`; no API key or network connection is needed. The
same launch is available as:

```powershell
.\start_route_a.bat
```

Run the complete offline command-line loop:

```powershell
& 'C:\Users\stato\AppData\Local\Programs\Python\Python313\python.exe' -m route_a.main --offline-demo --duration 4
```

Run tests:

```powershell
& 'C:\Users\stato\AppData\Local\Programs\Python\Python313\python.exe' -m unittest discover -s route_a\tests -v
```

## Architecture and trust boundaries

| Layer | Responsibility | Data boundary |
|---|---|---|
| `plant.py` | MuJoCo rigid-body/contact model, motor response, load, slope, friction, pushes and sensor truth | Produces sensor pins for the virtual MCU; Oracle stays here |
| `virtual_stm32.py` | Exact 200 Hz balance/velocity/turn control semantics from `04.bluetooth_control` | Consumes IMU, encoder, voltage and command state; returns only PWM and protocol bytes |
| `protocol.py` | Official `$...#` control/PID frames and versioned checked telemetry | Enforces ASCII and the 80-byte receive boundary |
| `transport.py` | `MuJoCoTransport` UART serialization and future `SerialTransport` interface | PC never reads virtual MCU Python attributes |
| `telemetry.py` | PC frame parsing and loss/staleness accounting | Contains deployable fields only |
| `metrics.py` | Stage-specific metrics and scores | Uses deployable telemetry, never Oracle |
| `deepseek_client.py` | bounded DeepSeek call, strict JSON parse, Mock/manual alternatives | Receives summarized windows only |
| `safety_harness.py` | ranges, per-stage locks, per-round change limits, safety gates, champion persistence | Deterministic and independent of LLM judgement |
| `experiment_runner.py` | matched-seed baseline/candidate experiments and records | Sends/query PID through bytes only |
| `orchestrator.py` | explicit Manual/Automatic state machine | Handles approval, candidate trial, shrink, accept and rollback |
| `ui.py` | controls, environment settings, live telemetry, metrics, raw JSON and Oracle-labelled plots | UI monitor tap duplicates bytes; it cannot steal protocol bytes |

`llm_step_optimizer.py` is not imported. Its prior LQR/free-control behavior is
not a Route A controller. Existing Route B and neural-policy files remain intact.

The balance-stage prompt prioritizes near-zero steady-state pitch error and
reduced post-disturbance oscillation. It permits decisive AP/AD proposals up to
the deterministic per-round Harness limits when telemetry is complete. This
does not relax PID bounds, stage locks, firmware protection, safety gates, or
the Harness-owned accept/rollback decision.

Stage trials are isolated for identifiability: `balance_recovery` holds the
motion command at stop and applies the configured forward push; `velocity_step`
applies the firmware forward command without an added push; `turn_step` uses a
right-turn phase to identify TP followed by a forward straight-line phase to
identify TD, without an added push. This mirrors the firmware behavior: TD is
disabled during explicit left/right commands and enabled during forward/backward
commands. Baseline and candidate repeat the same stage scenario and random seed.

## Firmware source trace

The sole firmware baseline is read-only:

```text
D:\DIP\STM32平衡车_V2\10.附件\源码汇总\4.Balanced_Car_base\04.bluetooth_control
```

| Simulated behavior | Executed firmware source |
|---|---|
| `Mid_Angle = 1` | `USER/main.c` |
| AP=96, AD=48, VP=62, VI=31, TP=14, TD=20 and Movement/Turn targets | `APP/PID/pid_control.c` |
| balance PD, speed PI, turn PD equations | `APP/PID/pid_control.c` |
| 5 ms control ordering and final left/right PWM sums | `APP/app_control.c` |
| ±1300 dead-zone compensation, ±2600 clamp, 40° and 9.6 V protection | `APP/app_motor.c` |
| AP/VP/TP ×100 scaling, ACKs, duplicate query and reset behavior | `BSP/Bluetooth/app_bluetooth.c` |
| UART5, 9600 baud, 8N1, no parity/flow control | `BSP/Bluetooth/bsp_bluetooth.c` |
| 67 mm wheel and `11 × 4 × 30 = 1320` counts/rev | `APP/app_motor.h` and `APP/app_motor.c` |
| command table and examples | `Bluetooth remote control communication protocol.xls` |

Protocol-display values are used by the PC, UI and LLM. Scaling occurs once,
inside `VirtualSTM32.internal_pid`.

Actual firmware movement semantics are preserved:

- forward/back: add `+25/-25` to `Encoder_Integral` each 5 ms cycle;
- left/right: `Turn_Target=-30/+30`;
- pivot left/right: `Turn_Target=-50/+50`.

These values are not replaced by a target velocity such as ±0.15 m/s. The
velocity scoring reference is a separately labelled `calibration_required`
analysis assumption and is never fed into the controller.

### Source/protocol conflicts recorded

Executed C takes precedence:

1. The spreadsheet names the voltage tag `VI`; C transmits `VT`.
2. The spreadsheet does not show that PID query replies twice; C calls the send
   function twice.
3. The auto-report comment says two seconds, while its actual flag path and
   blocking UART do not establish that rate reliably.
4. Encoder speed comments say mm/s, but the formula divides a mm circumference
   by 10 and is numerically consistent with cm/s.
5. AD/TD comments say 0-2 but executed defaults are 48/20 and the receive code
   does not enforce those commented maxima. Route A therefore labels its bounds
   as PC Harness limits rather than firmware limits.
6. The original telemetry builder may retransmit a stale buffer after a field
   range failure. Route A extended frames include ticks and checksums so the PC
   can detect staleness/corruption.

## Protocol and telemetry

The full compatible update frame is:

```text
$0,0,0,0,1,1,1,AP96.00,AD48.00,VP62.00,VI31.00,TP14.00,TD20.00#
```

Each selected PID pair produces one `$OK#`; updating all pairs produces three.
A query produces two identical PID reports. Invalid receive/query operations
produce `$ReceivePackError#`/`$GetPIDError#`.

Extended telemetry remains `$...#` framed and uses XOR checksums:

- `E1F` at 10 Hz: MCU tick, measured pitch/rates, encoder counts accumulated
  over the preceding 100 ms producer window, three loop
  outputs, final PWM, flags and current command.
- `E1S` at 2 Hz: MCU tick, measured battery, RX errors, TX drops and overruns.

The tested worst representative rate is below 960 bytes/s, the practical 9600
baud 8N1 payload ceiling. Frames remain below 80 bytes. The fast/slow producers
can be ported to a real STM32 using the existing Kalman/MPU6050 variables,
encoder counts, PID local outputs, PWM values, battery ADC and status flags.
They contain no contact force, exact friction, exact centre of mass, noiseless
angle, exact motor torque or known push force.

## Hardware-faithful and Exploration profiles

Hardware-faithful fixes 1 ms physics stepping, 5 ms MCU control, UART/framing,
PWM limit, 1300 dead zone and firmware protection behavior. The UI still permits
environment and uncertain-model changes.

Exploration permits alternate dead zone and wider model studies. Its results do
not represent the original firmware.

Values directly sourced from firmware/hardware material:

- control/UART periods and framing;
- PID defaults and scaling;
- Movement/Turn targets;
- wheel diameter and encoder count;
- nominal/cutoff voltage;
- PWM frequency, clamp and dead-zone value;
- stop angle.

Estimated or `calibration_required`:

- base mass distribution and inertias from the existing simplified model;
- friction/contact coefficients;
- motor stall torque, physical breakaway PWM, mechanical friction and motor time constant;
- left/right motor gain mismatch and actuator delay;
- IMU noise, bias and delay; the firmware Kalman equations and 5 ms update are
  reproduced, but numerical agreement with a physical MPU6050 is not yet established;
- load box inertia and exact mounting geometry;
- external-force application height;
- the scoring-only expected speed/yaw references.

The balance plant intentionally does not constrain the axle to a fixed world
position. The firmware balance loop first moves the wheels under the centre of
mass; the velocity filter damps that motion; and `Encoder_Integral` provides a
soft return toward the previous region. No artificial fore/aft oscillation
controller is added. The PWM value added by firmware as dead-zone compensation
is removed by the plant's physical breakaway model before motor torque is
calculated, so `±1300` is not interpreted as an immediate 45% drive command.

## UI operation

Use **Reset training** to discard the active champion and in-memory tuning
history and generate independent two-decimal initial gains over the full Harness
bounds: AP 20-200, AD 5-100, VP 10-120, VI 1-80, TP 1-80 and TD 0-60. The
manual-success, iteration, API and no-improvement counters are reset to zero.
Saved run directories are retained as an audit trail. The random initial values
are not screened for stability; firmware fall/voltage/PWM protections remain.

If a complete balance baseline falls, triggers protection or exceeds the 40 deg
pitch envelope, the cycle enters **Bootstrap recovery**. DeepSeek is instructed
to propose a decisive AP/AD rescue candidate instead of holding merely because
the random seed is unstable. The Harness temporarily permits up to 60% relative
or 40 absolute AP/AD movement (with a 10-point minimum allowance), while still
enforcing absolute PID bounds, two-decimal protocol precision and stage locking.
The candidate must eliminate hard safety events to become champion; otherwise it
is rolled back. Normal small-step limits resume automatically after recovery.

1. Launch the UI. Optionally click **Open MuJoCo viewer** and enable **Camera
   follow**.
2. Use Forward, Backward, Left, Right, Stop, Pivot left and Pivot right. All are
   transmitted through the official control frame.
3. Select push force and use the four external-force buttons.
4. Edit environment values and click **Apply environment**. Use **Reset robot
   pose** to return the MuJoCo body to its initial position with zero velocity;
   PID values, champion, counters, history and pending review state are preserved.
5. Select `balance`, `velocity`, or `turn` and a trial duration.
6. Select Adviser:
   - `MockLLM`: deterministic, offline;
   - `DeepSeek`: reads `route_a.env`;
   - `Manual JSON`: parses the JSON in the tab through the same strict schema.
7. In `manual-review`, click **Start**. Inspect raw JSON, Harness-adjusted
   candidate, rejection reasons, current/champion/candidate PID and metrics.
8. Click **Approve** to run the candidate or **Reject** to leave the champion
   unchanged. Approval never implies acceptance; the scorer may still shrink or
   roll back.
9. **Pause** interrupts a running trial. **Rollback** writes and queries the
   champion. **Emergency Stop** stops motion, sets the virtual MCU stop flag and
   interrupts the trial.

Telemetry and Oracle plots are explicitly separated. Oracle is saved for
research validation and plotting only; it is excluded from metrics, Harness and
LLM summaries.

## Automatic mode

Automatic mode is locked until the configured number of accepted Manual Review
trials has been completed (default 3). The count and champion are stored
atomically in `route_a/state/champion.json`.

After unlocking:

1. choose `automatic`;
2. set maximum rounds for this run;
3. click **Start**;
4. use Pause, Emergency Stop or Rollback at any time.

Global limits are in `route_a/config/default.json`. Harness defaults also cap
total iterations, API calls and consecutive no-improvement outcomes.

## DeepSeek setup

Create this local file:

```text
D:\DIP\Simulation\route_a.env
```

with:

```dotenv
DEEPSEEK_API_KEY=我自己的密钥
DEEPSEEK_BASE_URL=https://api.deepseek.com
DEEPSEEK_MODEL=deepseek-flash
```

The model name follows the official JSON Output example selected for this
project. The client never silently retries a 400/401/402/422. It uses finite
exponential backoff for 429/500/503, connection timeouts, the documented
occasional empty JSON Output response, and JSON that fails the strict local
schema. Invalid output is never promoted to a candidate.

Confirm the secret file is ignored:

```powershell
git check-ignore -v route_a.env
git status --short -- route_a.env
```

The first command must identify `.gitignore`; the second must print nothing.
Do not paste an API key into source, screenshots, logs or reports.

Run the explicit real-API smoke test only when intended:

```powershell
& 'C:\Users\stato\AppData\Local\Programs\Python\Python313\python.exe' -m route_a.api_smoke_test --run-real-api
```

Unit tests never call the real API.

## Experiment records

Each round creates `route_a/runs/<timestamp>_round_<n>_<stage>/` containing:

- baseline/candidate/shrink telemetry CSV, Oracle CSV, summary JSON and curves;
- environment, PID, random seed, metrics and score;
- compact LLM input and raw response;
- Schema/Harness result and reasons;
- actual PID readback and protocol frames;
- final accept/shrink/rollback decision;
- model/provider, prompt version, token usage when supplied, and Git commit.

`route_a.env` and generated `runs/state` contents are ignored. The API key is
never included in these records.

## Parts not validated on a real robot

- simplified mass, centre of mass and inertias;
- wheel/ground friction and MuJoCo contact settings;
- motor electrical/mechanical dynamics and dead-zone symmetry;
- actuator, IMU and encoder delay/noise distributions;
- battery sag and motor gain asymmetry;
- extended telemetry firmware implementation and its ISR/main-loop timing;
- real UART/Bluetooth loss/latency behavior;
- numerical agreement between virtual and physical Kalman/MPU6050 signals;
- all Harness ranges, score weights and acceptance thresholds;
- any PID candidate produced by MockLLM or DeepSeek.

Before hardware actuation, compare Python and firmware outputs on identical
recorded inputs, perform shadow runs, verify execution timing, retain physical
emergency controls, and begin with conservative parameter changes.
