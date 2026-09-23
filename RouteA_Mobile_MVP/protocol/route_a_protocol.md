# Route A wire protocol v1

The JDY-23 remains at UART5 9600 8N1 and exposes BLE service/characteristic
`FFE0`/`FFE1`. FFE1 is used for both writes and notifications, matching the
official Android app. Every application frame is ASCII, starts with `$`, ends
with `#`, and is split by the phone into 20-byte BLE writes.

Route A frames use `CRC16-CCITT-FALSE` (poly `0x1021`, init `0xFFFF`). The CRC
covers the bytes between `$` and the final `,Cxxxx#`, excluding both delimiters.
Legacy Yahboom frames have no CRC and remain accepted unchanged.

Commands:

* `P1,PREP,<trial>,<ttl_ms>,AP,AD,VP,VI,TP,TD`
* `P1,APPLY,<trial>` / `P1,ACCEPT,<trial>` / `P1,ROLLBACK,<trial>`
* `P1,HB,<trial>` / `P1,STOP,<trial>` / `P1,GET`
* `P1,MSET,AP,AD,VP,VI,TP,TD` — atomic manual setup before training
* `P1,TRAIN,START` / `P1,TRAIN,STOP` — firmware-enforced manual tuning lock
* `P1,DIAG,L+`, `L-`, `R+`, `R-`, or `STOP` — pre-balance motor
  diagnostic. A wheel command applies raw PWM 1500 for 0.6 s and is rejected
  after balance has started.

This build explicitly selects the unmodified Large Program Normal/StandardMode
control path. Its startup values are AP 96, AD 48, VP 62, VI 31,
TP 17 and TD 20. Before `TRAIN,START`, `MSET` may replace all six values within
the protection ranges. Once training starts, both MSET and legacy PID
update/restore frames are rejected. During training only AP/AD may differ from the
current champion. VP/VI/TP/TD must match, and AP/AD must each remain within 5%
of the champion. `PREP` validates and stages; `APPLY` changes all six values at
one 5 ms control boundary. A 2 s heartbeat lapse, TTL expiry, angle above 25
degrees, low battery, or sustained PWM saturation requests local rollback and
stops commanded motion. The original 40 degree hard shutdown remains intact.

Fast telemetry is emitted every 200 ms (`T1,F`) and slow telemetry every 1 s.
Fast frames include `C1`–`C4`, the actual TIM8 CCR register values.
Worst-case design traffic stays below the 960-byte/s UART payload capacity; the
interrupt-driven TX ring reports dropped frames in slow telemetry.
