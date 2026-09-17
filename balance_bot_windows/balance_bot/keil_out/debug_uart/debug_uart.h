/* debug_uart.h -- live telemetry and on-line tuning over USART1 (115200).
 *
 * WHY
 *   Threshold calibration needs to read ld_rms while the car balances freely.
 *   Doing that through the Keil Watch window means the car is tethered to a
 *   debugger.  Over the serial port it is wireless and takes minutes.
 *   On-line tuning removes the compile-flash-reset cycle from every step of
 *   the bring-up list.
 *   Binary logging captures the real car's 200 Hz gyro trace, which is what
 *   the simulation still cannot reproduce.
 *
 * COST
 *   Nothing is formatted or sent inside the interrupt.  DBG_Tick() only
 *   writes bytes into a ring buffer; DBG_Poll() drains it from the main
 *   loop.  Worst case in the ISR is a small sprintf once every 20 ticks.
 *
 * WIRING (three lines, see README_debug_uart.md)
 *   app_control.c, in the 200 Hz interrupt, after LD_Tick():   DBG_Tick();
 *   main.c, inside while(1):                                   DBG_Poll();
 *   usart.c, in USART1_IRQHandler, after the read:             DBG_RxByte(Rx1_Temp);
 *
 *   OPTIONAL fourth line, needed only for the osc/rev readings -- next to
 *   DBG_Tick(), pass the summed wheel position:
 *       LD_Pos((float)(Encoder_Left + Encoder_Right));
 *   Without it osc and rev read 0 and the slow-rocking complaint has no
 *   number attached to it.
 */
#ifndef __DEBUG_UART_H
#define __DEBUG_UART_H

/* Call once at start-up, after uart_init(115200). */
void DBG_Init(void);

/* Call once per 200 Hz control tick, after LD_Tick(). Never blocks. */
void DBG_Tick(void);

/* Call from the main while(1) loop.  Sends at most one byte per call. */
void DBG_Poll(void);

/* Call from USART1_IRQHandler with each received byte. */
void DBG_RxByte(unsigned char c);

/* Telemetry mode, also settable at runtime with the "t" command.
 *   0 = off
 *   1 = text, 10 Hz:  "<heavy> <rms> <osc> <rev> <pitch>"
 *   2 = binary, 200 Hz raw log for the PC-side capture script */
extern int dbg_mode;

/* Forcing a gain set is ld_force in load_ctrl.h, set with the "m" command.
 * It lives there because LD_Tick() has to honour it when it picks the set --
 * overriding afterwards only gets one gain and leaves the rest mismatched. */

#endif /* __DEBUG_UART_H */
