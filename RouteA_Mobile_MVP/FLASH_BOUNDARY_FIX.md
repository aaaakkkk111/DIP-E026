# Flash boundary fix (firmware v1.4)

## Independent assessment

The reported A/B/A experiment is strong evidence that image placement, rather
than the left-motor control equations, caused the one-wheel failure. The local
artifacts confirm the decisive part of that hypothesis:

| Image | Load/HEX end | Result |
| --- | ---: | --- |
| Official Large Program build | HEX `0x0801008F` | reported working |
| Route A v1.3 (`-O0`) | HEX `0x08011843` | observed left wheel stopped |
| Route A v1.4 (`-O1`) | HEX `0x0800D933` | hardware baseline passed |
| Route A v1.5 TEST MODE (`-O1`) | HEX `0x0800DA57` | awaiting mode-isolation test |

ARM scatter loading places the compressed initial values for nonzero global
variables after code and read-only data. In v1.3, `RW_IRAM1` had a Flash load
base of `0x08011784`, so startup depended on reading Flash well above the
observed problem boundary. Corruption there can affect initialized state such
as `GET_Angle_Way` and `Stop_Flag`; it can also affect PID state before the
mode setup overwrites some parameters.

This evidence does not establish a general `STM32F103RC` 64 KiB limit. A
genuine STM32F103RC has more Flash. The likely board-specific causes include a
different or compatible MCU, incorrect capacity identification, or unreliable
high-address Flash. Reading the MCU ID/Flash-size registers and doing a
program/read-back test would distinguish them, but that diagnosis is not
required for the conservative fix.

One detail in the supplied explanation is too strong: `myTurn_Kd = 0` normally
belongs to zero-initialized RAM rather than the nonzero RW initialization
payload. It may still become wrong if startup code/data is corrupted, but it is
not direct proof that the RW payload alone caused the steering term.

## Applied modification

1. Set Keil/ARMCC C optimization to level 1 (`<Optim>2</Optim>` in the project).
2. Limit the complete linker load region to `0x10000` bytes from
   `0x08000000`. Future growth beyond the boundary now fails the link.
3. Add `test_tools/check_hex_range.py` for an independent Intel HEX range
   check.
4. Keep the application protocol, PID logic, motor mapping, Android app and
   diagnostic commands unchanged.

The v1.5 full rebuild reports `Code=49620`, `RO-data=6108`, `RW-data=964`, and an
Intel HEX maximum address of `0x0800DA57`, leaving 9,640 bytes below
`0x08010000`.

## Hardware acceptance order

Flash `releases/routea_mobile_mvp_v1.4.hex`. Keep the wheels raised and repeat
the four pre-balance motor diagnostics first. Only if both motors and encoders
pass should the car be placed on the ground for the 10-second and 30-second
baseline tests. Continue to PID/LLM training only after the baseline passes.
