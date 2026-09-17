/* debug_uart.c -- see debug_uart.h
 *
 * COMMANDS (type them in any serial terminal at 115200, end with Enter)
 *   ?            print every tunable and its current value
 *   p 28800      Balance_Kp    ) these four edit the gain SET in effect
 *   d 48         Balance_Kd    ) right now -- NOT the PID globals.
 *   v 8200       Velocity_Kp   ) LD_Tick() rewrites the globals every tick,
 *   i 69         Velocity_Ki   ) so assigning them directly does nothing.
 *   l 7.5        LD_LOADED_BELOW   (detector threshold)
 *   e 21.9       LD_EMPTY_ABOVE    (detector threshold)
 *   m 0          gain set: 0 = detector decides, 1 = hold NORMAL, 2 = hold HEAVY
 *                Hold a set before tuning it, or the detector can switch
 *                under you and the next 'p' lands in the other table.
 *   t 0          telemetry: 0 = off, 1 = text 10 Hz, 2 = binary 200 Hz
 *
 * Values are the firmware's x100 integers, same as pid_control.c, except
 * l and e which are in deg/s and accept a decimal point.
 *
 * Nothing here is persistent.  A reset restores whatever was compiled in.
 * That is deliberate: you cannot brick the car by typing a bad number.
 */

#include <stdio.h>
#include <string.h>
#include "debug_uart.h"
#include "load_ctrl.h"

/* From the firmware. */
extern float Balance_Kp, Balance_Kd;
extern float Velocity_Kp, Velocity_Ki;
extern float Angle_Balance, Gyro_Balance;
extern int   Motor_Left, Motor_Right;

/* Which set the next p/d/v/i edits: whichever one is in effect. */
static LD_Gains *active(void)
{
    if (ld_force == 1) return &ld_g_normal;
    if (ld_force == 2) return &ld_g_heavy;
    return ld_heavy ? &ld_g_heavy : &ld_g_normal;
}

/* ---- transmit ring -------------------------------------------------- */
/* 512 is four text lines or fifty binary records; the main loop drains it
 * far faster than 10 Hz text or 200 Hz binary fills it. */
#define TX_SIZE 512

static unsigned char tx_buf[TX_SIZE];
static int tx_head;          /* written by DBG_Tick   */
static int tx_tail;          /* read by DBG_Poll      */

int dbg_mode = 0;

static void tx_byte(unsigned char c)
{
    int n = tx_head + 1;
    if (n >= TX_SIZE) n = 0;
    if (n == tx_tail) return;       /* full: drop, never block the ISR */
    tx_buf[tx_head] = c;
    tx_head = n;
}

static void tx_str(const char *s)
{
    while (*s) tx_byte((unsigned char)*s++);
}

/* ---- per-tick ------------------------------------------------------- */

void DBG_Init(void)
{
    tx_head = 0;
    tx_tail = 0;
    tx_str("\r\nload_ctrl debug ready, type ? for help\r\n");
}

void DBG_Tick(void)
{
    static int div_cnt = 0;
    char line[64];
    int g, a;

    /* Forcing now happens inside LD_Tick().  The first version patched
     * Balance_Kd here instead, which left Velocity_Kp and Velocity_Ki at the
     * OTHER set's values -- a mixture that was never tested.  Nothing in
     * this function writes a gain any more. */

    if (dbg_mode == 1)
    {
        if (++div_cnt < 20) return;  /* 200 Hz / 20 = 10 Hz */
        div_cnt = 0;
        /* heavy  rms(26..46Hz)  osc(0.3..3Hz)  rev/s  pitch
         * rms is the buzz; osc and rev are the slow fore/aft rocking.
         * "It wobbles too much" is a complaint about osc and rev -- rms
         * barely moves with it, so reading rms alone misses it entirely. */
        sprintf(line, "%d %.2f %.1f %.2f %.2f\r\n",
                ld_heavy, ld_rms, ld_osc, ld_rev, Angle_Balance);
        tx_str(line);
    }
    else if (dbg_mode == 2)
    {
        /* 10 bytes per tick at 200 Hz = 2000 B/s, well under 115200.
         * Sync word first so the PC side can re-align after a drop. */
        g = (int)Gyro_Balance;
        a = (int)(Angle_Balance * 100.0f);
        tx_byte(0xA5);
        tx_byte(0x5A);
        tx_byte((unsigned char)(g & 0xFF));
        tx_byte((unsigned char)((g >> 8) & 0xFF));
        tx_byte((unsigned char)(a & 0xFF));
        tx_byte((unsigned char)((a >> 8) & 0xFF));
        tx_byte((unsigned char)(Motor_Left & 0xFF));
        tx_byte((unsigned char)((Motor_Left >> 8) & 0xFF));
        tx_byte((unsigned char)(Motor_Right & 0xFF));
        tx_byte((unsigned char)((Motor_Right >> 8) & 0xFF));
    }
}

/* ---- drain, from the main loop --------------------------------------- */

extern void USART1_Send_U8(unsigned char ch);

void DBG_Poll(void)
{
    if (tx_tail == tx_head) return;
    USART1_Send_U8(tx_buf[tx_tail]);
    tx_tail++;
    if (tx_tail >= TX_SIZE) tx_tail = 0;
}

/* ---- receive --------------------------------------------------------- */

#define RX_SIZE 24
static char rx_buf[RX_SIZE];
static int  rx_len;

static float parse_num(const char *s)
{
    float whole = 0.0f;
    float frac = 0.0f;
    float scale = 1.0f;
    int neg = 0;

    while (*s == ' ') s++;
    if (*s == '-') { neg = 1; s++; }
    while (*s >= '0' && *s <= '9')
    {
        whole = whole * 10.0f + (float)(*s - '0');
        s++;
    }
    if (*s == '.')
    {
        s++;
        while (*s >= '0' && *s <= '9')
        {
            scale *= 10.0f;
            frac = frac * 10.0f + (float)(*s - '0');
            s++;
        }
    }
    whole += frac / scale;
    return neg ? -whole : whole;
}

static void report(void)
{
    char line[88];
    LD_Gains *a = active();

    sprintf(line, "NORMAL  p %.0f  d %.0f  v %.0f  i %.0f%s\r\n",
            ld_g_normal.kp, ld_g_normal.kd, ld_g_normal.vkp, ld_g_normal.vki,
            (a == &ld_g_normal) ? "   <- editing" : "");
    tx_str(line);
    sprintf(line, "HEAVY   p %.0f  d %.0f  v %.0f  i %.0f%s\r\n",
            ld_g_heavy.kp, ld_g_heavy.kd, ld_g_heavy.vkp, ld_g_heavy.vki,
            (a == &ld_g_heavy) ? "   <- editing" : "");
    tx_str(line);
    sprintf(line, "l %.2f  e %.2f  rms %.2f  heavy %d\r\n",
            LD_LOADED_BELOW, LD_EMPTY_ABOVE, ld_rms, ld_heavy);
    tx_str(line);
    sprintf(line, "osc %.1f  rev %.2f  (both 0 = LD_Pos not wired up)\r\n",
            ld_osc, ld_rev);
    tx_str(line);
    sprintf(line, "m %d (0=auto 1=NORMAL 2=HEAVY)  t %d (0=off 1=text 2=bin)\r\n",
            ld_force, dbg_mode);
    tx_str(line);
}

static void exec_line(void)
{
    char c = rx_buf[0];
    float v = parse_num(rx_buf + 1);
    LD_Gains *a = active();

    switch (c)
    {
    case '?': report();                        return;
    case 'p': a->kp           = v;             break;
    case 'd': a->kd           = v;             break;
    case 'v': a->vkp          = v;             break;
    case 'i': a->vki          = v;             break;
    case 'l': LD_LOADED_BELOW = v;             break;
    case 'e': LD_EMPTY_ABOVE  = v;             break;
    case 'm': ld_force        = (int)v;        break;
    case 't': dbg_mode        = (int)v;        break;
    default:  tx_str("?\r\n");                 return;
    }
    if (dbg_mode != 2) tx_str("ok\r\n");
}

void DBG_RxByte(unsigned char c)
{
    if (c == '\r' || c == '\n')
    {
        if (rx_len > 0)
        {
            rx_buf[rx_len] = 0;
            exec_line();
            rx_len = 0;
        }
        return;
    }
    if (rx_len < RX_SIZE - 1) rx_buf[rx_len++] = (char)c;
}
