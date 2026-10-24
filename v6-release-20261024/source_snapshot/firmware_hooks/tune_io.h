/* tune_io.h -- USART1 telemetry + live parameter tuning.
 *
 * Streams the control loop at 200 Hz as CSV, and accepts single-letter
 * commands so thresholds can be changed without reflashing.
 *
 * Wiring (three call sites):
 *   main()            TUNE_Init();            once, after bsp_init()
 *   main() while(1)   TUNE_Poll();            drains the ring, runs commands
 *   control ISR       TUNE_Sample(...);       one row per tick
 *
 * Commands (type them + Enter in any serial terminal, 115200 8N1):
 *   r          start recording, prints a self-describing header
 *   s          stop, prints the dropped-sample count
 *   m 0|1|2    gain select: 0 = detector decides, 1 = lock NORMAL,
 *              2 = lock HEAVY.  The detector keeps running either way,
 *              so ld_rms stays valid while a set is locked.
 *   l <num>    set LD_LOADED_BELOW   e.g.  l 12.3
 *   e <num>    set LD_EMPTY_ABOVE    e.g.  e 31.5
 *   ?          print current state
 *
 * CSV columns:  seq,gyro,ang_c,ml,mr,el,er
 *   seq    sample counter.  A GAP IN seq MEANS SAMPLES WERE DROPPED --
 *          check for this before trusting an FFT.
 *   gyro   Gyro_Balance, raw MPU6050 LSB (16.4 LSB per deg/s)
 *   ang_c  Angle_Balance in centi-degrees (divide by 100)
 *   ml,mr  commanded PWM after limiting; 0 when Turn_Off() cut the output
 *   el,er  encoder counts this tick
 */
#ifndef __TUNE_IO_H
#define __TUNE_IO_H

void TUNE_Init(void);
void TUNE_Poll(void);
void TUNE_Sample(float gyro, float angle, int ml, int mr, int el, int er);
void TUNE_RxByte(unsigned char c);

/* Disturbance injection, added equally to both wheels (a pure pitch
 * torque, no yaw).  Called once per control tick; returns 0 when idle. */
int  TUNE_Inject(void);
int  TUNE_Injecting(void);
void TUNE_Fell(void);        /* control loop reports a cut-out */
int  TUNE_Logging(void);

#endif /* __TUNE_IO_H */
