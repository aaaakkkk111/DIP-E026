# mistral_tzejun_rewardcode_v3

LLM-assisted tuning for an underactuated two-wheel balancing robot, where the LLM
(**mistral:latest**, local, offboard) **writes the reward function as code** each
iteration — not just tuning eight weights as the ollama project did. PPO then
fine-tunes the pinned tzejun actor on that reward. A matched fixed-reward condition
isolates what the LLM-written code contributes.

This project is a self-contained duplicate of `ollama_tzejun_reward_v2`; it never
modifies the original project, checkpoint or firmware.

## What is new vs the ollama project

| | ollama_tzejun_reward_v2 | this project |
|---|---|---|
| LLM | llama3.2:3b | **mistral:latest** |
| LLM output | 8 reward weights (sum→1) | **a reward-function body (Python code)** |
| Safety of LLM output | range/sum validation | **AST allowlist + no-builtins sandbox + self-test** |
| New: robot logging | — | **`robot_logger.py`** saves real-robot behaviour to feed back |

## How the LLM writing reward code is kept safe

The LLM's code is **never executed as free Python**. `mistral_reward_code.py`:
1. Parses it with `ast` and rejects anything not on an allowlist — no imports,
   attributes, subscripts, loops, comprehensions, lambdas, def/class, dunder names,
   or calls to anything but whitelisted math helpers (`abs min max clip sqrt exp
   tanh sin cos pow2`). Node count and exponent magnitude are capped.
2. Compiles once and evaluates in a namespace with **no builtins**, only those
   helpers and the fixed per-step variables.
3. Self-tests every proposal on 400 random + 3 extreme inputs; any non-finite or
   out-of-range output rejects it.
4. On any failure, keeps the previous safe reward and marks the iteration
   fallback-only (no LLM contribution claimed). The immutable fall penalty (-25)
   is applied by the env, outside the LLM code.

`python -E mistral_reward_code.py` runs the sandbox self-test (blocks 10 escape
attempts, accepts the default reward).

## Model, gains, reward, randomisation

- **RL network (actor):** `34 → 64 (tanh) → 64 (tanh) → 2 (linear, clipped to ±1)`,
  6,530 parameters. PPO (lr 3e-5, n_steps 2048, clip 0.1, target_kl 0.01).
- **Gains:** the RL policy has **no PID gains** — it is a neural net. PID gains exist
  only in the stock-PID baseline used for comparison (balance Kp 9600 / Kd 48,
  velocity 6200/31, turn 1700/20). Firmware adds a +1300 deadband, clips PWM at 2600.
- **Reward/penalties:** the LLM writes a cost over per-step physical variables
  (pitch, pitch_rate, roll, yaw_rate, v_forward, speed_err, yaw_err, position_error,
  effort, slew, …). Higher reward = better. Immutable −25 fall penalty applied by
  the env.
- **Randomisation (what it is optimised to tolerate):** each training episode
  randomises friction 0.5–1.2, payload 0–0.25 kg, chassis mass ±10%, motor armature
  0.0018–0.0027, and gives a movement command 10% of episodes. So it **is** trained
  for friction, payload and disturbance. Sensing **latency is not** randomised here
  (that was the ollama project's latency-robust variant; same follow-up applies).

## Is it ready?

- **MuJoCo sim:** ready. View the fine-tuned vs base policy:
  ```
  & 'C:\Users\Sridevi\Desktop\sridevi-DIP-recess-week\.venv\Scripts\python.exe' -E 'C:\Users\Sridevi\Desktop\sridevi-DIP-recess-week\mistral_tzejun_rewardcode_v3\viewer_v2.py'
  ```
  (Keys: 1 fine-tuned, 2 base, WASD drive, arrows push, `[` `]` payload, `;` `'`
  friction, R reset. Shows the zero-latency env — not where hardware latency lives.)
- **Keil project:** present (`stm32_Balance_Car_L`), but **not yet staged or built**.
  No HEX exists until you build. To deploy a trained model and build:
  1. Stage the selected actor into this project's Keil tree:
     ```
     & '...\.venv\Scripts\python.exe' -E '...\mistral_tzejun_rewardcode_v3\deploy_to_keil.py' --run '...\mistral_tzejun_rewardcode_v3\runs\<run>\run_summary.json'
     ```
  2. Open and **Rebuild All in the uVision GUI**:
     ```
     & 'C:\Keil_v5\UV4\UV4.exe' '...\mistral_tzejun_rewardcode_v3\stm32_Balance_Car_L\USER\stm32_Balance_Car.uvprojx'
     ```
  3. **HEX location after build:** `...\mistral_tzejun_rewardcode_v3\stm32_Balance_Car_L\OBJ_PURE_TZEJUN\yahboom_pure_tzejun.hex`
- **FlyMcu flash:** open FlyMcu → select that `.hex` → port COM5 → "DTR low on reset,
  RTS high on boot" → Start.
- **Run on the robot:** first KEY1 selects the controller, second KEY1 arms (KEY1
  after that is STOP). Note the firmware's gyro-calibration arming gate needs the
  robot held dead still at startup.

## Logging real-robot behaviour (for improving the model)

`robot_logger.py` is **read-only** — it never writes to the serial port, so it cannot
arm the robot. While you run the flashed model, it saves `robot_logs/<label>/`:
`telemetry.csv` (every streamed sample) and `summary.json` with a `feedback_for_model`
block (pitch RMS, yaw-rate RMS, net displacement, dominant failure). Run:
```
& '...\.venv\Scripts\python.exe' -E '...\mistral_tzejun_rewardcode_v3\robot_logger.py' --port COM5 --seconds 120 --label mistral_trial1
```
`python -E robot_logger.py --self-test` decodes a synthetic frame with no serial.
That summary is the real-world gap to feed back into the next training round.

## Training

```
& '...\.venv\Scripts\python.exe' -E '...\mistral_tzejun_rewardcode_v3\finetune_mistral.py' --condition mistral --threads 1
```
Single-core by default (quiet). `--condition fixed` runs the matched control. Outputs
land in `runs/<condition>_seed00_<stamp>/` with `run_summary.json`, `reward_design_log.json`
(every proposed reward code + its accept/fallback status), `selected_policy.zip`, and
`export/` headers. Nothing here runs on the robot; only the gated Keil build + flash does.

## Honest status

Simulation only; no model has been flashed or tested on hardware. A reward change
does not alter a deployed actor — only PPO changes the network, and only the export +
a fresh Keil build + the gated hardware procedure reach the robot. The full training
run has been built and smoke-verified but not yet executed end-to-end.
