/* deadband_test.c -- see deadband_test.h
 *
 * HOW TO USE
 *   1. Prop the car up so BOTH WHEELS HANG FREE.
 *   2. In APP/app_motor.c set  #define MOTOR_IGNORE_PULSE (0)
 *      The whole point is to measure without the compensation.
 *   3. In USER/main.c, as the FIRST statement inside while(1):
 *          DB_Test();
 *          while (1) { }
 *      The second line parks the CPU so the balance loop never runs.
 *   4. Flash, watch the wheels, note db_pwm when a wheel first turns.
 *
 * Left and right may differ.  If so, use the LARGER value.
 * Put everything back afterwards: remove these two lines and set
 * MOTOR_IGNORE_PULSE per the table in README_how_to_measure.md.
 */

#include "deadband_test.h"

extern void Set_Pwm(int motor_left, int motor_right);
extern void delay_ms(unsigned short nms);

int db_pwm = 0;

#define DB_STEP    25      /* counts added per step */
#define DB_HOLD_MS 500     /* how long each step is held */
#define DB_MAX     2600    /* the firmware's own PWM limit */

void DB_Test(void)
{
    int n;

    for (n = 0; n <= DB_MAX; n += DB_STEP)
    {
        db_pwm = n;
        Set_Pwm(n, n);
        delay_ms(DB_HOLD_MS);
    }

    db_pwm = 0;
    Set_Pwm(0, 0);
}
