/* deadband_test.h -- measure this car's real motor dead band.
 *
 * The factory firmware assumes 1300 counts (MOTOR_IGNORE_PULSE in
 * APP/app_motor.c) and its own comment says the value must be trimmed per
 * car.  Which gain set you should flash depends on the real value, so
 * measure it once.  Takes about 20 minutes.
 * See README_how_to_measure.md next to this file.
 */
#ifndef __DEADBAND_TEST_H
#define __DEADBAND_TEST_H

/* Ramps both wheels from 0 upward, then stops.  Watch the wheels and read
 * db_pwm (Keil Watch window) at the moment a wheel first turns. */
void DB_Test(void);

/* Current PWM being applied.  Put this in the Watch window. */
extern int db_pwm;

#endif /* __DEADBAND_TEST_H */
