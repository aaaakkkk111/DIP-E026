"""Digital twin of the Yahboom STM32 two-wheeled balance car.

Everything in this subpackage is derived from the shipped firmware, not
invented.  Each constant carries the file it came from so you can check it:

    sourcecode/6.LQR/Matlab/parameter_LQR.m            physical parameters
    sourcecode/6.LQR/STM32_code/APP/app_control.c      LQR loop + gains
    sourcecode/6.LQR/STM32_code/APP/app_motor.{c,h}    PWM, dead zone, encoder
    sourcecode/6.LQR/STM32_code/APP/PID/pid_control.c  cascade PID gains
    sourcecode/6.LQR/STM32_code/BSP/bsp.c              PWM timer setup

The point of the twin is to give the *real firmware's* behaviour as a
baseline number in the same simulator the RL policies are measured in, so
"the learned controller survives N steps" means something.
"""

from .motor import Yahboom370Motor, MotorCalibration          # noqa: F401
from .controllers import (STM32_LQR, STM32_CascadePID,        # noqa: F401
                          FirmwareState, LQR_GAINS, PID_GAINS,
                          PID_GAIN_SETS, PID_DEFAULT_SET, pid_gains,
                          LQR_YAW_GAINS_BY_MODE, FIRMWARE_COMMANDS)
from .imu import (MPU6050Kalman,            # noqa: F401
                  accel_pitch_angle, GYRO_LSB_PER_RAD_S)
from .robot import (STM32_CAR, STM32_FIRMWARE_CONST,          # noqa: F401
                    stm32_sim_params)

__all__ = [
    "Yahboom370Motor", "MotorCalibration",
    "STM32_LQR", "STM32_CascadePID", "FirmwareState",
    "LQR_GAINS", "PID_GAINS", "PID_GAIN_SETS", "PID_DEFAULT_SET",
    "pid_gains", "LQR_YAW_GAINS_BY_MODE", "FIRMWARE_COMMANDS",
    "MPU6050Kalman", "accel_pitch_angle",
    "GYRO_LSB_PER_RAD_S",
    "STM32_CAR", "STM32_FIRMWARE_CONST", "stm32_sim_params",
]
