# v6 Real-Car Test Plan / v6 实车测试步骤

Firmware / 固件: `stm32_Balance_Car_L_v6.hex` (screen "27.Adapt V6")
Tools / 工具: a serial terminal at 115200 baud (or `e026 keil\realcar\mc_carkit.py`). Send one command at a time and wait for its `#…` reply before the next.
工具：115200 波特率串口终端（或 `e026 keil\realcar\mc_carkit.py`）。每次只发一条命令，等它回 `#…` 后再发下一条。

---

## Part I — English

### 0. Before each session
1. Charge the battery: above 11 V is best, and the motors are cut below 9.6 V.
2. Clear about 1 m of floor around the car; a get-up can travel up to 66 cm.
3. Power on, press KEY1, put the car down, and press KEY1 again.
4. Send `?` and keep the `#mc` line. The defaults are `op=50/40/15/75`, `re=1/…`, `sr=1/…/40/30`.

### 1. Empty car: chatter must go fast, then nothing should change
1. Start the car empty and let it stand for 10 s. Expected: the screen shows `f0.00L` within about 1–2 s, with no audible chatter.
2. Push it lightly or tap a key, then let it stand. Expected: f may jump to 1 briefly, then returns to 0 within about 1 s, and the chatter stops.
3. Record 20 s of standing (`r`, wait, `s`) and send the file. Look at `osc` on the screen: a calm car should read under about 7.

### 2. Load on the top plate (the case that used to sway and fall)
1. Put the usual load on the top plate of a standing car.
2. Expected:
   - any sway stops growing within a few seconds;
   - f climbs in steps of 0.1–0.3 and then stays put;
   - there are no repeated up/down jumps.
3. Wait 30 s and record f and the visible sway. Then drive forward, stop, and drive back.
4. If f reaches 1.0 and the car still sways and grows, stop the test and report it. That is the high-CoM speed-loop case (README §4.9).

### 3. Take the load off
1. Lift the load off a standing car.
2. Expected: within about 1–3 s f goes down to 0–0.2, and the chatter stops. It must **not** stick at 0.2 with chatter (the test-2 bug).

### 4. Small-amplitude high-frequency oscillation
1. On any gear above 0, a small fast buzz should disappear within about 1 s. The gear steps down 0.1 every 0.5 s.
2. If a calm, loaded car keeps stepping down and then begins to sway: raise the threshold with `ad10` (or `ad12`).
3. If a small buzz remains: lower it with `ad6`.
4. Send `?` before and after each change.

### 5. Large tilt and get-up
1. Hold the car at about 40° with the motors running, then let go. Expected: it rights itself.
2. Push the empty car over onto the floor. Expected: about 0.5 s still, then push, coast, push, and it is up, all within about 2 s of the fall.
3. Lay the car down, then press the second KEY1. Expected: it gets up.
4. Send `as0`, then push it over. Expected: the wheels do not turn at all. Send `as1` to restore.

### 6. Regression (daily use must not be worse)
Drive forward, back and turn for 2 minutes, empty and with a load. Report any chatter, or any gear change while driving calmly.

### 7. A/B switches (no reflash needed)
| Command | Meaning |
|---|---|
| `ax50` | v6 (default) |
| `ax18` | v6 without the sway upshift |
| `ax30` | the v5.1 bit set (the v6 downshift changes stay) |
| `au2` / `au1` | sway threshold higher / lower |
| `ad10` / `ad6` | small-chatter threshold higher / lower |

Send the recordings and the `?` lines to Claude after the session.

---

## 第二部分 — 中文

### 0. 每次开始前
1. 电池充电：11 V 以上最好，低于 9.6 V 会切电机。
2. 车周围清出约 1 m 的空地，自起时车最多会移动 66 cm。
3. 上电，按 KEY1，把车放到地上，再按 KEY1。
4. 发送 `?`，保存 `#mc` 那一行。默认值：`op=50/40/15/75`、`re=1/…`、`sr=1/…/40/30`。

### 1. 空车：抖动要快速消失，之后不再换档
1. 空车启动，站 10 s。预期：约 1–2 s 内屏幕显示 `f0.00L`，听不到抖振声。
2. 轻推一下或按一下按键，再让它站着。预期：f 可能短暂跳到 1，约 1 s 内回到 0，抖动停止。
3. 录 20 s 站立数据（`r`，等待，`s`），把文件发过来。同时看屏幕上的 `osc`：平稳时应低于约 7。

### 2. 负重放在顶板上（以前会越摆越大并摔倒的情况）
1. 车站稳后，把平时的负重放到顶板上。
2. 预期：
   - 如果有摆动，几秒内停止变大；
   - f 按 0.1–0.3 一步往上升，然后停住；
   - 不会反复升降。
3. 等 30 s，记下 f 和肉眼看到的摆动。然后前进、停车、后退各一次。
4. 如果 f 已经到 1.0，车还在越摆越大，就停止测试并告诉我。这是高重心速度环的情况（README §4.9）。

### 3. 取下负载
1. 车站着时把负载拿下来。
2. 预期：约 1–3 s 内 f 降到 0–0.2，抖动停止。**不能**再卡在 0.2 一直抖（test2 的 bug）。

### 4. 小幅高频振荡
1. 任何高于 0 的档位上，轻微的快速嗡嗡抖动应在约 1 s 内消失（每 0.5 s 降 0.1）。
2. 如果带载车平稳时档位还一直往下降，之后开始摆：用 `ad10`（或 `ad12`）调高阈值。
3. 如果小幅抖动一直不消失：用 `ad6` 调低。
4. 每次修改前后都发一次 `?`。

### 5. 大倾角和自起
1. 电机运行时把车扶到约 40° 再松手。预期：车自己回正。
2. 把空车推倒在地上。预期：静止约 0.5 s → 推 → 滑行 → 推 → 立起，倒下后约 2 s 内完成。
3. 让车躺着，然后按第二次 KEY1。预期：车自己起来。
4. 发送 `as0`，再推倒车。预期：轮子完全不转。测完发 `as1` 恢复。

### 6. 回归（日常使用不能变差）
空车和带载各前进、后退、转弯 2 分钟。如果出现抖振，或者平稳行驶时换档，请记下来。

### 7. A/B 开关（不用重新烧录）
| 命令 | 含义 |
|---|---|
| `ax50` | v6（默认） |
| `ax18` | v6 去掉摆动升档 |
| `ax30` | v5.1 的开关组合（v6 的降档改动仍然生效） |
| `au2` / `au1` | 摆动阈值调高 / 调低 |
| `ad10` / `ad6` | 小幅抖动阈值调高 / 调低 |

测完把录的文件和 `?` 回传的那几行发给 Claude。
