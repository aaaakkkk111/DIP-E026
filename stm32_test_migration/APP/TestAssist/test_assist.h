#ifndef __TEST_ASSIST_H_
#define __TEST_ASSIST_H_

#include <stdint.h>

typedef enum
{
    TEST_PID_NORMAL = 0,
    TEST_PID_WEIGHT = 1
} TestPidProfile;

typedef struct
{
    uint32_t inference_cycles;
    uint32_t maximum_inference_cycles;
    uint32_t inference_count;
    int16_t base_pwm_left;
    int16_t base_pwm_right;
    int16_t residual_pwm_left;
    int16_t residual_pwm_right;
    int16_t assisted_pwm_left;
    int16_t assisted_pwm_right;
    int16_t commanded_pwm_left;
    int16_t commanded_pwm_right;
    uint8_t weight_profile;
    uint8_t residual_applied;
} TestAssistTelemetry;

extern volatile TestAssistTelemetry g_test_assist_telemetry;

void TestAssist_Init(void);
void TestAssist_ControlStep(float angle,
                            float gyro_balance,
                            float gyro_turn,
                            int encoder_left,
                            int encoder_right,
                            int *motor_left,
                            int *motor_right);

#endif
