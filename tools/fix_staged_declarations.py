from pathlib import Path


replacements = (
    (
        Path(r"D:\DIP\Simulation\stm32_test_migration\BSP\bsp.c"),
        b'bluetooth_send_string("Hello Yahboom!\\n");',
        b'UART5_Send_Char("Hello Yahboom!\\n");',
    ),
    (
        Path(r"D:\DIP\Simulation\stm32_test_migration\BSP\Motor\motor.h"),
        b"void Balance_PWM_Init(u16 arr,u16 psc);",
        b"void BalanceCar_PWM_Init(u16 arr,u16 psc);",
    ),
    (
        Path(r"D:\DIP\Simulation\stm32_test_migration\BSP\Motor\motor.h"),
        b"void Balance_Motor_Init(void);",
        b"void BalanceCar_Motor_Init(void);",
    ),
)

for path, old, new in replacements:
    data = path.read_bytes()
    if data.count(old) != 1:
        raise RuntimeError(f"expected one occurrence in {path}: {old!r}")
    path.write_bytes(data.replace(old, new, 1))

print("fixed existing declarations and Bluetooth call")
