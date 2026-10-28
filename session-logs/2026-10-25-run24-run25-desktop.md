# 2026-10-25 — runs 24, 25 and 26 (desktop, simulator only)

Every number here is from the MuJoCo simulator, 2 deg gear slack unless noted. Nothing in this log was run on the car.

## Run 24 — run 22 (8 s memory) + random load placement, 30M from scratch

Spec (laptop): load placed randomly, fore/aft ±3 cm, sideways ±2 cm, 10.5–12 cm above the axle; off-centre 2 kg should move < 50 cm.

| | best (23.2M) | 26M | 27M | 28M | 30M |
|---|---|---|---|---|---|
| Wobble 2° (deg/s) | 4.3–11.7 | 3.6–8.0 | 2.4–4.4 | 3.5–5.1 | 5.8–14.8 |
| Empty 2 N pushes | 14/18 | 17/18 | 16/18 | 16/18 | 18/18 |

- 2 kg at ±2 cm (26–30M): standing holds but drifts 0.8–1.9 m; driving holds 81–98 %. Best (23.2M) falls driving at +2 cm. ±3 cm mostly falls.
- 60 s creep: worse than runs 22 and 23 — up to 1.0–1.7 cm/s, ends 60–108 cm away on +1/+2° slopes (run 23: ≤ 0.3 cm/s, within 17 cm).
- **Result: partial / FAIL against the spec.** Better than run 22 with off-centre loads, but drifts 1–2 m and creep got worse. Not exported. Reading: the 8 s leak forgets the steady lean an off-centre load needs.

## Run 25 — run 23 (full memory) + run 24's load placement, fine-tuned 10M from run 23

- `POSITION_TAU_S = inf`, `POS_ERR_RANDOM_START = True`.
- Faster convergence: `--init models/best_real_RUN23/best_model.zip` (existing fine-tune path: LR 1e-4, full command range from the start, payload ramps over the run). 10M steps instead of 30M, about 1/3 of the time.
- Best model saved to `models/best_real_RUN25/`; committed `models/best_real` (run 23) untouched. `firmware/policy.h` back to `1.0f` (matches runs 23 and 25).

### Per-1M watch (standing wobble deg/s, 3 delays)

| ckpt | empty | 2 kg |
|---|---|---|
| 1M | 5.3–9.0 | 4.6–5.9 |
| 3M | 7.2–8.2 | 6.5–8.1 |
| 4M | 9.2–17.6 | 6.2–7.8 |
| 5M | 5.8–8.7 | 6.8–8.1 |
| 7M | 5.7–7.2 | 5.0–5.7 |
| 8M | 6.5–16.3 | 4.9–6.4 |
| 9M | 7.1–9.6 | 5.1–5.4 |
| 10M | 9.8–17.2 | 4.9–5.2 |

### Final comparison

| | run 23 | 25 best (eval) | 25 @7M | 25 @9M | 25 @10M |
|---|---|---|---|---|---|
| Wobble 2° (deg/s) | **3.7–4.2** | 4.3–9.0 | 5.7–7.2 | 7.1–11.9 | 8.9–17.2 |
| Wobble 4° (deg/s) | **5.6–9.9** | 11.8–15.9 | 7.2–15.7 | 16.0–23.6 | 15.5–31.6 |
| Reversals 2° (/s/wheel) | **1.8–3.4** | 2.9–7.6 | 5.5–7.2 | 7.0–12.0 | 8.2–17.1 |
| Empty pushes 2 N / 3 N | 13 / 0 | 13 / 0 | 17 / 1 | **18 / 3** | **18 / 4** |
| 2 kg −2 cm stand / drive | falls | falls | ≤ 2.0 m / holds | ≤ 0.6 m / holds | **≤ 0.44 m / holds** |
| 2 kg +2 cm stand / drive | falls | falls | ≤ 0.9 m / falls | ≤ 1.0 m / holds | **≤ 0.41 m / holds 84–86 %** |
| 2 kg −3 cm stand / drive | falls | falls | 4.5–5.8 m / holds | 2.7–3.7 m / holds | 2.0–2.9 m / holds |
| 2 kg +3 cm stand / drive | falls | falls | falls | 2.5–3.5 m / falls | 1.1–1.8 m / falls |
| 60 s creep (slopes, trim, 0/2 kg) | within 17 cm | within 16 cm | within 15 cm | within 37 cm | within 22 cm |
| 4 kg drive held | 2/3 | 2/3 | 3/3 | 3/3 | 3/3 |

- "25 best (eval)" is essentially the 1M model: the eval reward peaked early, so it has none of the off-centre skill.
- Run 23 itself falls with a 2 kg load 2 cm off-centre (fits the laptop's run 23 car log: ±20 cm swing with 2 kg).
- **Result: partial.** 10M meets the ±2 cm spec (moves < 50 cm standing, and drives) — the first run to do so — and is the first with any empty 3 N push survival; creep stays at run 23's level. But empty standing shakes 9–17 deg/s (run 23: 4), the shake the car punishes. +3 cm still falls driving.
- Trade-off: 7M is calmer but only half-handles ±2 cm; 9M is in between.
- Not exported, not pushed. Final model backed up as `models/ppo_real_robot_RUN25_10M.zip`; progress in `train_real_robot_RUN25_finetune_progress.csv`.

## Run 26 — run 25 with stronger shake charges, fine-tuned 5M from run 25 at 10M

- `OSC_PENALTY_REF_DPS` 10 → 5 (same osc costs 4x, cap still 1.0 below the +2 alive reward), `PWM_STEP_WEIGHT` 0.3 → 0.5.
- `--init models/ckpt_run25/rl_model_9999890_steps.zip --steps 5000000 --best-dir models/best_real_RUN26`.
- Goal: keep run 25's ±2 cm load skill and push gains, bring empty standing back to run 23's ~4 deg/s.

| | run 23 | 25 @10M | 26 @2M | 26 @3M | 26 @4M | 26 @5M |
|---|---|---|---|---|---|---|
| Wobble 2°, empty (deg/s) | **3.7–4.2** | 8.9–17.2 | 4.4–9.0 | 2.3–9.0 | 5.3–9.3 | 7.0–8.2 |
| Reversals 2° (/s/wheel) | **1.8–3.4** | 8.2–17.1 | 4.5–10.9 | 0.5–11.4 | 4.0–13.8 | 8.1–12.4 |
| Wobble standing 2 kg / 4 kg | 3.6–4.0 / 6.4–7.9 | 4.9–5.2 / 5.8–7.3 | 7.1–8.6 / 10.8–13.2 | 8.1–9.3 / 12.2–14.0 | 7.9–9.5 / 12.7–14.4 | 8.5–9.8 / 12.7–14.4 |
| Empty pushes 2 N / 3 N | 13 / 0 | 18 / 4 | 18 / 5 | 18 / 4 | 18 / 2 | 18 / 3 |
| 2 kg −2 cm stand / drive | falls | ≤ 0.44 m / holds | ≤ 0.39 m / holds | ≤ 0.37 m / holds | ≤ 0.36 m / holds | ≤ 0.34 m / holds |
| 2 kg +2 cm stand / drive | falls | **≤ 0.41 m / holds** | ≤ 3.2 m / falls | ≤ 1.8 m / falls | ≤ 1.2 m / falls | ≤ 1.8 m / falls |
| 60 s creep (slopes, trim, 0/2 kg) | within 17 cm | within 22 cm | within 16 cm | within 11 cm | within 10 cm | within 12 cm |
| Tilt sd with 2 kg (deg) | 0.2–0.9 | 0.15–0.8 | 1.2–1.4 | 1.3–1.5 | 1.3–1.5 | 1.4–1.5 |

- **Result: FAIL.** The stronger shake charges halved the empty shake (best 2.3 at 3M, but trial-to-trial 2–9) without reaching run 23's steady 4, and moved it into the loaded case: 2 kg wobble 5 → 8–10 deg/s, 4 kg 6–7 → 13–14, tilt sd with 2 kg 0.2–0.8 → 1.3–1.5. It also lost run 25's +2 cm (load forward of centre) skill: driving falls again. Creep improved slightly (within 10–16 cm), pushes kept.
- Reading: the policy has a fixed budget of stiffness; charging the shake harder bought calm empty by going softer, which is exactly what an off-centre forward load cannot afford. A shake charge cannot fix this alone.
- Not exported. Final model `models/ppo_real_robot_RUN26_5M.zip`; progress in `train_real_robot_RUN26_finetune_progress.csv`. `train_real_robot.py` left at run 26 settings (uncommitted).

## Slope driving: runs 22, 23, 25 (first climb test for 22 and 23)

`slope_drive.py`: 1 s stand, then 5 s at 0.25 m/s; gravity-relative pitch; 2° slack, 5/10/15 ms. Speed as % of command, trials held of 3. Negative = downhill. Run 22 with its own 8 s memory.

| Slope | 22 empty | 22 2 kg | 23 empty | 23 2 kg | 25 @10M empty | 25 @10M 2 kg |
|---|---|---|---|---|---|---|
| −10° | 87 %, 3/3 | 50 %, 3/3 | 79 %, 1/3 | 75 %, 3/3 | 86 %, 3/3 | 79 %, 3/3 |
| −5° | 100 %, 3/3 | 66 %, 3/3 | 166 %, 1/3 | 75 %, 3/3 | 100 %, 3/3 | 80 %, 3/3 |
| 0° | 87 %, 3/3 | 89 %, 3/3 | 107 %, 3/3 | 104 %, 3/3 | 96 %, 3/3 | 95 %, 3/3 |
| +5° | 82 %, 3/3 | 92 %, 3/3 | 98 %, 3/3 | 109 %, 3/3 | 95 %, 3/3 | 100 %, 3/3 |
| +10° | 54 %, 3/3 | 86 %, 2/3 | 49 %, 3/3 | falls 0/3 | 77 %, 3/3 | 98 %, 3/3 |
| +15°, +20° | falls | falls | falls | falls | falls | falls |

- Run 22 climbs 10° empty and loaded (run 21 fell at 5° with 2 kg). Run 23 falls at +10° with 2 kg and on empty descents. Run 25 @10M is the best on slopes. Nobody does 15°.

## Run 27 — run 22 + load placement, tilt graded against the balance point, fine-tuned 10M from run 22 @24M

- `_balance_tilt()`: forward angle of the whole robot's centre of mass (from the axle) away from "up" along gravity. Replaces `pitch` in the `-3|tilt|` reward, the posture shaping, and `upright_gate` (2 kg 2 cm off-centre balances at 9.5°, past the 8.6° gate, so the calm charges used to switch off there). Self-check `check_balance_tilt.py` (scratchpad): 0° empty level, ±9.48° for 2 kg ±2 cm, +14.06° at +3 cm, −2.00° on a +2° slope.
- Run 22 settings otherwise: 8 s memory, zero start, `OSC_PENALTY_REF_DPS` 10, `PWM_STEP_WEIGHT` 0.3 (run 26's values removed).

| | run 22 @24M | 27 @5M | 27 @7M | 27 @9M | 27 @10M |
|---|---|---|---|---|---|
| Wobble 2° empty (deg/s) | 4.3–11.4 | **4.3–7.7** | 5.5–12.1 | 7.5–14.4 | 10.4–20.8 |
| Wobble 4° empty (deg/s) | 7.1–23.5 | 13.9–23.1 | 19.7–31.9 | 21.1–30.8 | 32.1–36.1 |
| Standing 2 kg / 4 kg (deg/s) | 5.1–5.6 / 7.8–9.1 | 6.2–7.4 / 10.6–15.8 | 5.4–6.0 / 8.3–10.0 | 5.2–6.5 / 8.0–9.2 | 4.9–5.1 / 6.6–7.8 |
| Empty pushes 2 N / 3 N | 17 / 0 | 18 / 0 | 18 / 0 | 18 / 5 | 18 / **10** |
| 2 kg −2 cm stand / drive | 5.3–6.2 m / holds | 1.0–1.6 m / holds | 2.2–3.5 m / holds | 0.5–0.6 m / holds | **0.50–0.55 m / holds** |
| 2 kg +2 cm stand / drive | 1.4–5.2 m / falls | 1.1 m / holds | 0.6–0.9 m / holds | 0.4–0.5 m / holds | **0.32–0.38 m / holds** |
| 2 kg −3 cm stand / drive | falls / mostly falls | falls / holds 70–77 % | falls / holds | 2.5–3.8 m / holds | **1.0–1.3 m / holds** |
| 2 kg +3 cm stand / drive | falls / falls | 5.4–6.4 m / falls | 5.3–6.0 m / falls | 1.0–1.1 m / 1 of 4 | **0.7–0.9 m / holds 72–80 %** |
| 60 s creep (slopes, trim; 0 / 2 kg) | within 37 / 41 cm | 38 / 38 cm | 38 / 52 cm | 35 / 52 cm | 30 / 47 cm |
| 4 kg drive held | 3/3 | 2/3 | 3/3 | 3/3 | 3/3 |
| Climb +10° 2 kg | 86 %, 2/3 | 83 %, 2/3 | 88 %, 3/3 | 89 %, 3/3 | 89 %, 3/3 |
| Descend −10° 2 kg | 50 %, 3/3 | 49 %, 3/3 | 21 %, 1/3 | 35 %, 3/3 | 38 %, 3/3 |

- **Result: partial — the balance-point reward works for the load, the empty shake comes back late.** @10M is the best off-centre run so far: ±2 cm within 32–55 cm (spec < 50 cm: met ahead of centre, just over behind), the first to hold and drive ±3 cm, and the best empty 3 N push survival (10/18) of any run. But empty standing shakes 10–21 deg/s. @5M is calmer than run 22 empty and halves its ±2 cm drift, but only to 1–1.6 m. Creep stays at run 22's 8 s-memory level (30–50 cm in 60 s).
- Same late empty shake as run 25 (fine-tunes from 23 and 22, different rewards). Likely cause, not tested: payload mass is drawn uniform(0, cap), so while the cap ramps up early every episode is near-empty, and by the end only ~15 % are under 0.3 kg - empty standing stops being practised. Candidate fix: draw 0 kg with a fixed probability (as `STAND_CMD_PROB` does for commands).
- Not exported, not pushed. Final model `models/ppo_real_robot_RUN27_10M.zip`; progress in `train_real_robot_RUN27_finetune_progress.csv`. `train_real_robot.py` at run 27 settings (uncommitted). Run 22's empty 2° wobble measured 4.3–11.4 in this batch (4.0–5.4 on 2026-10-22, different seeds/batch).

## Run 28 — run 27 + 30 % empty episodes, fine-tuned 5M from run 27 @10M (simulator)

- Change: `EMPTY_PAYLOAD_PROB = 0.3` — 30 % of training episodes get 0 kg, the rest uniform(0, cap) as before. Everything else as run 27 (8 s memory, balance-point tilt, random load placement). Training only: firmware as run 22.
- `--init models/ckpt_run27/rl_model_9999890_steps.zip --steps 5000000`, 22 min, no crash.

Per-1M watcher (2° slack, standing, deg/s):

| Ckpt | Wobble 0 kg | Wobble 2 kg | Creep +1° |
|---|---|---|---|
| 1M | 6.3–16.0 | 4.6–5.1 | held, ~1 cm/s |
| 2M | 5.3–6.6 | 5.6–6.5 | held |
| 3M | 6.0–6.6 | 7.2–9.1 | held |
| 4M | 5.0–7.6 | 6.8–7.8 | held |
| 5M | 6.8–7.8 | 5.7–7.5 | held |

Full comparison (sim), run 28 @5M vs run 27 @10M:

| Check | Run 27 @10M | Run 28 @5M | Run 28 @3M |
|---|---|---|---|
| ±2 cm 2 kg, stand drift | 32–55 cm | 36–57 cm | 55–83 cm |
| ±3 cm 2 kg, stand / drive | 69–133 cm / 4/4 | 74–119 cm / 2/4 (+3 cm drive 1/2 each height) | 113–401 cm, one fall / +3 cm drive FELL |
| Empty 3 N pushes | 10/18 | **15/18** (best of any run) | — |
| Empty wobble 2° slack | 10.4–20.8 | **6.7–9.3** | 6.0–6.6 |
| Empty wobble 4° slack | 32.1–36.1 | 20.8–36.0 | 7.7–23.9 |
| Reversals 2° (/s per wheel) | 6.9–24.2 | 6.8–10.5 | 6.1–13.5 |
| Load sweep stand 0/1/2/4 kg | 10–21 / 4.5–6 / 5 / 7–8 | 7–8 / 5–6 / 6–7.5 / 9–10 | — |
| Drive empty wobble | 16.3 | 15.1 | — |
| 60 s creep (worst end) | 47 cm | 61 cm (2 kg, −1° slope) | — |
| Climb +10° 2 kg | 89 %, 3/3 | 95 %, 3/3 | — |
| Descend −10° 2 kg | 38 %, 3/3 | 42 %, 3/3 | — |

- **Result: mostly pass — the 0 kg draw fixed the late empty shake without losing the ±2 cm load skill.** Empty 2° wobble back near run 22 level, pushes best ever. Costs: +3 cm drive falls 1/2 (run 27 held), 4° slack still shaky at 5M (3M much calmer there but loses ±3 cm), 2 kg creep on −1° slope 61 cm.
- Not exported, not pushed. Final model `models/ppo_real_robot_RUN28_5M.zip`; progress `train_real_robot_RUN28_finetune_progress.csv`. `train_real_robot.py` at run 28 settings (uncommitted). Export needs `POLICY_POS_DECAY 0.99937520f` (8 s).
