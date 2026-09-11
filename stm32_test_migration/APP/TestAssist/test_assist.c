#include "AllHeader.h"
#include "test_assist.h"
#include "neural_actor.h"

/* Keep this disabled until shadow inference, timing and parity are accepted. */
#define TEST_ASSIST_APPLY_RESIDUAL 0

#define TEST_SWITCH_TO_WEIGHT_DEG 10.0f
#define TEST_SWITCH_TO_NORMAL_DEG 7.0f
#define TEST_GYRO_COUNTS_PER_DEGREE_SECOND 16.4f
#define TEST_RESIDUAL_PWM_LIMIT 350.0f
#define TEST_PWM_LIMIT 2600
#define TEST_WHEEL_RADIUS_M 0.0335f
#define TEST_CONTROL_PERIOD_S 0.005f
#define TEST_ENCODER_COUNTS_PER_REVOLUTION 1320.0f
#define TEST_PI_F 3.14159265f

#define TEST_DEMCR (*(volatile uint32_t *)0xE000EDFCUL)
#define TEST_DWT_CTRL (*(volatile uint32_t *)0xE0001000UL)
#define TEST_DWT_CYCCNT (*(volatile uint32_t *)0xE0001004UL)
#define TEST_DEMCR_TRCENA (1UL << 24)
#define TEST_DWT_CYCCNTENA (1UL << 0)

volatile TestAssistTelemetry g_test_assist_telemetry;

static TestPidProfile test_pid_profile;
static float test_encoder_bias;
static float test_encoder_integral;

static float TestAssist_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float TestAssist_ClipFloat(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static int TestAssist_RoundFloat(float value)
{
    int integer = (int)value;
    float fraction = value - (float)integer;

    if (fraction > 0.5f) return integer + 1;
    if (fraction < -0.5f) return integer - 1;
    if (fraction == 0.5f && (integer & 1) != 0) return integer + 1;
    if (fraction == -0.5f && (integer & 1) != 0) return integer - 1;
    return integer;
}

static float TestAssist_MovementTarget(void)
{
    if (g_newcarstate == enRUN ||
        g_newcarstate == enps2Fleft ||
        g_newcarstate == enps2Fright)
    {
        return Car_Target_Velocity;
    }
    if (g_newcarstate == enBACK ||
        g_newcarstate == enps2Bleft ||
        g_newcarstate == enps2Bright)
    {
        return -Car_Target_Velocity;
    }
    if (g_newcarstate == enAvoid) return -10.0f;
    if (g_newcarstate == enFollow) return 10.0f;
    return Move_X;
}

static float TestAssist_TurnTarget(void)
{
    if (g_newcarstate == enLEFT ||
        g_newcarstate == enps2Fleft ||
        g_newcarstate == enps2Bleft)
    {
        return -Car_Turn_Amplitude_speed;
    }
    if (g_newcarstate == enRIGHT ||
        g_newcarstate == enps2Fright ||
        g_newcarstate == enps2Bright)
    {
        return Car_Turn_Amplitude_speed;
    }
    if (g_newcarstate == enTLEFT) return -50.0f;
    if (g_newcarstate == enTRIGHT) return 50.0f;
    return 0.0f;
}

static void TestAssist_UpdatePidProfile(float angle)
{
    float magnitude = TestAssist_AbsFloat(angle);
    TestPidProfile next_profile = test_pid_profile;

    if (test_pid_profile == TEST_PID_NORMAL && magnitude >= TEST_SWITCH_TO_WEIGHT_DEG)
    {
        next_profile = TEST_PID_WEIGHT;
    }
    else if (test_pid_profile == TEST_PID_WEIGHT && magnitude <= TEST_SWITCH_TO_NORMAL_DEG)
    {
        next_profile = TEST_PID_NORMAL;
    }

    if (next_profile != test_pid_profile)
    {
        test_pid_profile = next_profile;
        /* Match the simulation supervisor when its PID profile changes. */
        test_encoder_bias = 0.0f;
        test_encoder_integral = 0.0f;
    }
}

static void TestAssist_BasePid(float angle,
                               float gyro_balance,
                               float gyro_turn,
                               int encoder_left,
                               int encoder_right,
                               int *base_left,
                               int *base_right,
                               float *movement,
                               float *turn_target)
{
    float balance_kp;
    float balance_kd;
    float velocity_kp;
    float velocity_ki;
    float turn_kp;
    float turn_kd;
    float balance_scale;
    float velocity_scale;
    float turn_scale;
    float encoder_least;
    float balance_pwm;
    float velocity_pwm;
    float turn_pwm;
    float selected_turn_kd;

    if (test_pid_profile == TEST_PID_WEIGHT)
    {
        balance_kp = 9600.0f;
        balance_kd = 75.0f;
        velocity_kp = 7000.0f;
        velocity_ki = 35.0f;
        turn_kp = 1400.0f;
        turn_kd = 20.0f;
        balance_scale = 2.0f;
        velocity_scale = 1.35f;
        turn_scale = 1.0f;
    }
    else
    {
        balance_kp = 9600.0f;
        balance_kd = 48.0f;
        velocity_kp = 6200.0f;
        velocity_ki = 31.0f;
        turn_kp = 1700.0f;
        turn_kd = 20.0f;
        balance_scale = 1.0f;
        velocity_scale = 1.0f;
        turn_scale = 1.0f;
    }

    *movement = TestAssist_MovementTarget();
    *turn_target = TestAssist_TurnTarget();

    balance_pwm = (balance_kp * angle / 100.0f) +
                  (balance_kd * gyro_balance / 100.0f);
    balance_pwm *= balance_scale;

    encoder_least = (float)(-(encoder_left + encoder_right));
    test_encoder_bias = test_encoder_bias * 0.84f + encoder_least * 0.16f;
    test_encoder_integral += test_encoder_bias + *movement;
    test_encoder_integral = TestAssist_ClipFloat(test_encoder_integral, -8000.0f, 8000.0f);
    velocity_pwm = -test_encoder_bias * velocity_kp / 100.0f -
                   test_encoder_integral * velocity_ki / 100.0f;
    velocity_pwm *= velocity_scale;

    if (g_newcarstate == enRUN || g_newcarstate == enBACK)
    {
        selected_turn_kd = turn_kd;
    }
    else
    {
        selected_turn_kd = 0.0f;
    }
    turn_pwm = *turn_target * turn_kp / 100.0f +
               gyro_turn * selected_turn_kd / 100.0f + Move_Z;
    turn_pwm *= turn_scale;

    *base_left = (int)balance_pwm + (int)velocity_pwm + (int)turn_pwm;
    *base_right = (int)balance_pwm + (int)velocity_pwm - (int)turn_pwm;
    *base_left = PWM_Ignore(*base_left);
    *base_right = PWM_Ignore(*base_right);
    *base_left = PWM_Limit(*base_left, TEST_PWM_LIMIT, -TEST_PWM_LIMIT);
    *base_right = PWM_Limit(*base_right, TEST_PWM_LIMIT, -TEST_PWM_LIMIT);

    if (angle < -40.0f || angle > (float)angle_max || battery < 9.6f || Stop_Flag == 1)
    {
        test_encoder_integral = 0.0f;
    }
}

static void TestAssist_BuildObservation(float angle,
                                        float gyro_balance,
                                        float gyro_turn,
                                        int encoder_left,
                                        int encoder_right,
                                        int base_left,
                                        int base_right,
                                        float movement,
                                        float turn_target,
                                        float observation[NEURAL_ACTOR_INPUTS])
{
    float target_speed;
    float target_yaw_rate;

    target_speed = movement * TEST_PI_F * TEST_WHEEL_RADIUS_M /
                   (TEST_CONTROL_PERIOD_S * TEST_ENCODER_COUNTS_PER_REVOLUTION);
    target_yaw_rate = -turn_target * 0.8f / 36.0f;

    observation[0] = angle / 30.0f;
    observation[1] = (gyro_balance / TEST_GYRO_COUNTS_PER_DEGREE_SECOND) / 400.0f;
    observation[2] = ((Velocity_Left + Velocity_Right) / 200.0f) / 1.0f;
    observation[3] = ((gyro_turn / TEST_GYRO_COUNTS_PER_DEGREE_SECOND) *
                      TEST_PI_F / 180.0f) / 5.0f;
    observation[4] = (float)encoder_left / 60.0f;
    observation[5] = (float)encoder_right / 60.0f;
    observation[6] = test_encoder_bias / 60.0f;
    observation[7] = test_encoder_integral / 8000.0f;
    observation[8] = target_speed / 0.5f;
    observation[9] = target_yaw_rate / 1.0f;
    observation[10] = (float)base_left / 2600.0f;
    observation[11] = (float)base_right / 2600.0f;
    observation[12] = (test_pid_profile == TEST_PID_WEIGHT) ? 1.0f : 0.0f;

    {
        int i;
        for (i = 0; i < NEURAL_ACTOR_INPUTS; ++i)
        {
            observation[i] = TestAssist_ClipFloat(observation[i], -5.0f, 5.0f);
        }
    }
}

void TestAssist_Init(void)
{
    test_pid_profile = TEST_PID_NORMAL;
    test_encoder_bias = 0.0f;
    test_encoder_integral = 0.0f;

    g_test_assist_telemetry.inference_cycles = 0U;
    g_test_assist_telemetry.maximum_inference_cycles = 0U;
    g_test_assist_telemetry.inference_count = 0U;
    g_test_assist_telemetry.base_pwm_left = 0;
    g_test_assist_telemetry.base_pwm_right = 0;
    g_test_assist_telemetry.residual_pwm_left = 0;
    g_test_assist_telemetry.residual_pwm_right = 0;
    g_test_assist_telemetry.assisted_pwm_left = 0;
    g_test_assist_telemetry.assisted_pwm_right = 0;
    g_test_assist_telemetry.commanded_pwm_left = 0;
    g_test_assist_telemetry.commanded_pwm_right = 0;
    g_test_assist_telemetry.weight_profile = 0U;
    g_test_assist_telemetry.residual_applied = 0U;

    TEST_DEMCR |= TEST_DEMCR_TRCENA;
    TEST_DWT_CYCCNT = 0U;
    TEST_DWT_CTRL |= TEST_DWT_CYCCNTENA;
    NeuralActor_Init();
}

void TestAssist_ControlStep(float angle,
                            float gyro_balance,
                            float gyro_turn,
                            int encoder_left,
                            int encoder_right,
                            int *motor_left,
                            int *motor_right)
{
    float observation[NEURAL_ACTOR_INPUTS];
    float action[NEURAL_ACTOR_OUTPUTS];
    float movement;
    float turn_target;
    int base_left;
    int base_right;
    int residual_left;
    int residual_right;
    int combined_left;
    int combined_right;
    uint32_t cycle_start;
    uint32_t elapsed_cycles;

    TestAssist_UpdatePidProfile(angle);
    TestAssist_BasePid(angle,
                       gyro_balance,
                       gyro_turn,
                       encoder_left,
                       encoder_right,
                       &base_left,
                       &base_right,
                       &movement,
                       &turn_target);
    TestAssist_BuildObservation(angle,
                                gyro_balance,
                                gyro_turn,
                                encoder_left,
                                encoder_right,
                                base_left,
                                base_right,
                                movement,
                                turn_target,
                                observation);

    cycle_start = TEST_DWT_CYCCNT;
    NeuralActor_Predict(observation, action);
    elapsed_cycles = TEST_DWT_CYCCNT - cycle_start;

    residual_left = TestAssist_RoundFloat(action[0] * TEST_RESIDUAL_PWM_LIMIT);
    residual_right = TestAssist_RoundFloat(action[1] * TEST_RESIDUAL_PWM_LIMIT);
    combined_left = PWM_Limit(base_left + residual_left, TEST_PWM_LIMIT, -TEST_PWM_LIMIT);
    combined_right = PWM_Limit(base_right + residual_right, TEST_PWM_LIMIT, -TEST_PWM_LIMIT);

#if TEST_ASSIST_APPLY_RESIDUAL
    *motor_left = combined_left;
    *motor_right = combined_right;
    g_test_assist_telemetry.residual_applied = 1U;
#else
    *motor_left = base_left;
    *motor_right = base_right;
    g_test_assist_telemetry.residual_applied = 0U;
#endif

    g_test_assist_telemetry.inference_cycles = elapsed_cycles;
    if (elapsed_cycles > g_test_assist_telemetry.maximum_inference_cycles)
    {
        g_test_assist_telemetry.maximum_inference_cycles = elapsed_cycles;
    }
    g_test_assist_telemetry.inference_count++;
    g_test_assist_telemetry.base_pwm_left = (int16_t)base_left;
    g_test_assist_telemetry.base_pwm_right = (int16_t)base_right;
    g_test_assist_telemetry.residual_pwm_left = (int16_t)residual_left;
    g_test_assist_telemetry.residual_pwm_right = (int16_t)residual_right;
    g_test_assist_telemetry.assisted_pwm_left = (int16_t)combined_left;
    g_test_assist_telemetry.assisted_pwm_right = (int16_t)combined_right;
    g_test_assist_telemetry.commanded_pwm_left = (int16_t)(*motor_left);
    g_test_assist_telemetry.commanded_pwm_right = (int16_t)(*motor_right);
    g_test_assist_telemetry.weight_profile =
        (test_pid_profile == TEST_PID_WEIGHT) ? 1U : 0U;
}
