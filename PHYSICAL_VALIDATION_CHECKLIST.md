# Physical validation operator checklist

This checklist is for validation only. Do not use the physical robot as a reward-search environment.

1. Freeze the known-good PID configuration and record its version in `configs/physical_validation_development.json`.
2. Train/select the two PPO artifacts in simulation only: one baseline reward and one best LLM reward. Do not select by training reward alone.
3. Create one manifest per PPO artifact with model hash, `balancebot-v1` observation schema, `normalized-bidirectional-v1` action scaling, reward specification, and `validated: true` only after bench validation.
4. Keep `configs/hardware_unverified.json` unchanged until independently verified Yahboom protocol, sensor mapping, motor scaling, motor-disabled telemetry, action direction, E-stop, watchdog, and safe-stop have been bench-tested. The program will otherwise remain disarmed.
5. Use the same marked surface, battery range, fixed duration, start procedure, and pre-built mechanical disturbance fixture for every scheduled trial. Do not use uncontrolled pushing.
6. Before each trial: measure battery, inspect wheels/chassis, confirm E-stop, start with motors disabled, confirm fresh finite pitch/rate/encoder telemetry and command scaling, then follow the next randomized order from `--plan`.
7. Stop immediately for pitch limit, wheel-speed limit, stale data, communication loss, NaN/Inf, watchdog expiry, or E-stop. Preserve the raw telemetry even for failures.
8. After each trial, save raw JSONL under `results/<experiment-id>/raw/`, record it through `--record`, and do not edit the frozen experiment config. The runner checkpoints every record.
9. After completion, produce `--report`. Treat simulation and physical evidence separately; do not claim physical improvement without the measured physical comparison.
