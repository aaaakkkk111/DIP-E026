#ifndef YB_HYBRID_H
#define YB_HYBRID_H
#include <stdint.h>

enum { YB_STOCK_PID=0, YB_OLLAMA_PID=1, YB_TZEJUN_NN=2 };
enum { YB_FAULT_SENSOR=1, YB_FAULT_TILT=2, YB_FAULT_DEADLINE=4,
       YB_FAULT_CONFIG=8, YB_FAULT_CADENCE=32, YB_FAULT_PHYSICAL_STOP=128 };
/* These are hardware adaptation constants, NOT network retraining. */
#define YB_NN_SPEED_M_S 0.15f
#define YB_NN_YAW_RAD_S 0.45f
#define YB_GYRO_LSB_PER_RAD_S 939.65078f /* MPU6050 FS=2000, 16.4 LSB/(deg/s) */
#define YB_PITCH_SIGN 1.0f
#define YB_ROLL_SIGN 1.0f
#define YB_YAW_SIGN 1.0f
#define YB_NN_PWM_LIMIT 2600 /* preserve this board's factory output ceiling */
#define YB_CONTROL_BUDGET_US 5000U
#define YB_PREFLIGHT_TICKS 200U /* 1 s of non-actuating, real-policy work */
#define YB_TRACE_ROWS 256U
#define YB_TRACE_ROW_BYTES 48U
#define YB_ENCODER_WINDOW 1U /* reject unverified delayed filter; raw 5ms encoder input */
#define YB_GYRO_CAL_TICKS 400U /* 2s stationary, stopped; never adapt while armed */
#define YB_GYRO_CAL_MAX_STD 0.01f /* rad/s, per axis */
#define YB_GYRO_CAL_MAX_BIAS 0.05f /* rad/s; larger offset needs investigation */
/* Read-only diagnostics: 0x15(index:u16) retrieves an armed trace row;
 * 0x16() retrieves current stopped NN forecast; 0x17() stationary calibration.
 * None of these can actuate/arm. */

extern float Yb_Roll_Rate_Raw;
void Hybrid_Init(void);              /* before MPU6050 EXTI is enabled */
void Hybrid_ConfigureInterrupts(void); /* AFTER vendor EXTI init sets priority */
void Hybrid_BeginTick(void);
void Hybrid_SensorsReady(void);      /* timestamp actual IMU acquisition */
void Hybrid_SensorFault(void);       /* failed coherent MPU transaction */
void Hybrid_Control(int encoder_l, int encoder_r, int *pwm_l, int *pwm_r);
void Hybrid_EndTick(int actual_pwm_l, int actual_pwm_r);
void Hybrid_Poll(void);              /* foreground ONLY: protocol + telemetry */
void Hybrid_RxByte(uint8_t byte);     /* USART1 ISR: enqueue ONLY */
int Hybrid_IsNN(void);
int Hybrid_HasFault(void);
int Hybrid_CanArm(void);             /* physical KEY1 cannot bypass preflight */
void Hybrid_PhysicalStop(void);      /* K1 stop-only; reset required */
#endif
