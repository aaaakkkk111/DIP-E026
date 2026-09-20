/* tune_io.c -- see tune_io.h */

#include "tune_io.h"
#include "AllHeader.h"
#include <math.h>

/* The six gains live in pid_control.c and are not in AllHeader.h; the rest
 * of the project externs them locally too (see app_mode.c). */
extern float Balance_Kp, Balance_Kd;
extern float Velocity_Kp, Velocity_Ki;
extern float Turn_Kp, Turn_Kd;

/* ---- ring buffer ------------------------------------------------------
 * The control ISR must never block, so it only copies six numbers in here.
 * All formatting and all UART waiting happens in TUNE_Poll(), on the main
 * loop, where blocking costs nothing.
 *
 * 128 slots is 640 ms of slack at 200 Hz -- far more than the main loop
 * ever needs, so an overflow means the UART itself could not keep up.
 * At 115200 the CSV needs about 8200 byte/s of the 11520 available; if
 * drops do show up, raise the baud rate in bsp.c (uart_init) and in the
 * terminal, 230400 doubles the headroom.  */
#define LOG_N 320        /* 1.6 s of slack at 200 Hz; 128 was overflowing */

typedef struct {
    unsigned short seq;
    short gyro, ang, ml, mr, el, er, inj, s_m, kg_c, lvl;
} LogSample;

static LogSample log_buf[LOG_N];
static volatile unsigned short log_head, log_tail, log_seq, log_drop;
static volatile int log_on;
/* Decimation.  The switching tests only need tens of Hz, and at 200 Hz
 * with eleven columns the UART was dropping a fifth of the rows -- which
 * corrupts exactly the switch timings those tests are trying to measure. */
static volatile unsigned char log_div = 1, log_cnt;

static char rx_line[24];
static volatile unsigned char rx_len, rx_ready;
/* Idle timeout in 5 ms control ticks.  A command is accepted on CR/LF OR
 * after this much silence, so it does not matter whether the terminal
 * appends a newline -- UartAssist does not by default. */
#define RX_IDLE_TICKS 10          /* 50 ms */
static volatile unsigned char rx_idle;

int TUNE_Logging(void) { return log_on; }

/* ---- disturbance injection -------------------------------------------
 * Tuning needs REPEATABLE trials.  A hand push is not repeatable, so the
 * firmware makes its own disturbance and logs exactly what it injected
 * (the 'inj' CSV column), which is what turns a recording into a system
 * identification run rather than just a noise sample.
 *
 *   k <amp>   one 100 ms pulse            -- settling / damping
 *   c <amp>   0.5 -> 50 Hz sweep, 20 s    -- frequency response
 *
 * The sweep is the valuable one: it measures this car's real response
 * across the whole band while the balance loop holds it up, so the
 * resonance and the chatter frequency come straight out of the data
 * instead of being assumed from the simulation. */
#define INJ_HZ        200.0f
#define IMPULSE_TICKS 20               /* 100 ms */
#define CHIRP_TICKS   4000             /* 20 s   */
#define CHIRP_F0      0.5f
#define CHIRP_F1      25.0f            /* measured 2026-09-16: this car is
                                        * deaf above ~22 Hz (gain 1/24 of the
                                        * 12 Hz peak), so sweeping to 50 Hz
                                        * spent half the run on nothing.
                                        * Halving the span doubles the dwell
                                        * per Hz where the dynamics live. */
#define INJ_AMP_MAX   900              /* hard cap, keeps it recoverable */

static volatile int   inj_mode;        /* 0 idle, 1 impulse, 2 chirp */
static volatile int   inj_amp;
static volatile int   inj_n;
static volatile int   inj_t;
static float          inj_ph;
static volatile short inj_last;

int TUNE_Injecting(void) { return inj_mode != 0; }

int TUNE_Inject(void)
{
    int out = 0;
    float f;

    if (inj_mode == 1)
    {
        if (inj_n > 0) { inj_n--; out = inj_amp; }
        else           { inj_mode = 0; }
    }
    else if (inj_mode == 2)
    {
        if (inj_t < CHIRP_TICKS)
        {
            f = CHIRP_F0 + (CHIRP_F1 - CHIRP_F0) *
                ((float)inj_t / (float)CHIRP_TICKS);
            inj_ph += 6.2831853f * f / INJ_HZ;
            if (inj_ph > 6.2831853f) inj_ph -= 6.2831853f;
            out = (int)((float)inj_amp * (float)sin((double)inj_ph));
            inj_t++;
        }
        else { inj_mode = 0; }
    }

    inj_last = (short)out;
    return out;
}

/* Reported once per cut-out so a failed trial is visible in the log
 * without waiting to be told. */
static volatile unsigned char fell_flag;
void TUNE_Fell(void) { fell_flag = 1; }

/* ---- output primitives ------------------------------------------------
 * Hand rolled instead of printf: this runs 200 times a second and printf
 * would pull the full formatter into the hot path for no benefit. */
static void tx(char c)
{
    /* 0x80 = TXE (data register empty), not 0x40 = TC (frame fully shifted
     * out).  The stock fputc waits on TC, which idles the line for a whole
     * stop bit between bytes; TXE lets the next byte queue immediately. */
    while ((USART1->SR & 0x80) == 0) { }
    USART1->DR = (unsigned char)c;
}

static void tx_s(const char *s)
{
    while (*s) tx(*s++);
}

static void tx_i(int v)
{
    char b[8];
    int n = 0;
    if (v < 0) { tx('-'); v = -v; }
    do { b[n++] = (char)('0' + (v % 10)); v /= 10; } while (v && n < 8);
    while (n) tx(b[--n]);
}

static void tx_f1(float f)
{
    int w, fr;
    if (f < 0.0f) { tx('-'); f = -f; }
    w  = (int)f;
    fr = (int)((f - (float)w) * 10.0f + 0.5f);
    if (fr > 9) { w++; fr = 0; }
    tx_i(w); tx('.'); tx_i(fr);
}

static void tx_f2(float f)
{
    int w, fr;
    if (f < 0.0f) { tx('-'); f = -f; }
    w  = (int)f;
    fr = (int)((f - (float)w) * 100.0f + 0.5f);
    if (fr > 99) { w++; fr = 0; }
    tx_i(w); tx('.');
    if (fr < 10) tx('0');
    tx_i(fr);
}

static void nl(void) { tx('\r'); tx('\n'); }

/* ---- status -----------------------------------------------------------
 * Printed at the top of every recording so each saved file says by itself
 * which condition it was taken under.  No separate note taking. */
static void status(const char *tag)
{
    tx_s(tag);
    tx_s(" mode=");    tx_i((int)mode + 1);
    tx_s(" prof=");    tx_i(ld_profile);
    tx_s(" force=");   tx_i(ld_force);
    tx_s(" gear=");    tx_s(ld_heavy ? "HEAVY" : "NORMAL");
    tx_s(" kp=");      tx_f1(Balance_Kp);
    tx_s(" kd=");      tx_f1(Balance_Kd);
    tx_s(" vkp=");     tx_f1(Velocity_Kp);
    tx_s(" vki=");     tx_f1(Velocity_Ki);
    tx_s(" lb=");      tx_f1(LD_LOADED_BELOW);
    tx_s(" ea=");      tx_f1(LD_EMPTY_ABOVE);
    tx_s(" rms=");     tx_f1(ld_rms);
    tx_s(" lvl=");     tx_i(la_level);
    tx_s("/");         tx_i(la_force);
    tx_s(" osc=");     tx_f2(la_osc);
    tx_s(" lean=");    tx_f2(la_lean);
    tx_s(" sat=");     tx_f2(la_sat);
    tx_s(" oHi=");     tx_f2(LA_OSC_HI);
    tx_s(" lHi=");     tx_f2(LA_LEAN_HI);
    tx_s(" oLo=");     tx_f2(LA_OSC_LO);
    tx_s(" pan=");     tx_f2(LA_ANG_PANIC);
    tx_s(" npan=");    tx_i(la_panic);
    tx_s(" boot=");    tx_i(LA_BOOT_LEVEL);
    tx_s(" k=");       tx_f2(la_k);
    tx_s(" kE=");      tx_f2(LA_K_EMPTY);
    tx_s(" kF=");      tx_f2(LA_K_FULL);
    tx_s(" div=");     tx_i((int)log_div);
    tx_s(" dead=");    tx_i(motor_ignore_pulse);
    tx_s("/");         tx_i(motor_ignore_rev);
    tx_s(" vbat=");    tx_f1(battery);
    nl();
}

/* ---- command parsing --------------------------------------------------- */
static float parse_num(const char *s)
{
    float v = 0.0f, f = 0.1f;
    int neg = 0;
    while (*s == ' ' || *s == '=') s++;
    if (*s == '-') { neg = 1; s++; }
    while (*s >= '0' && *s <= '9') { v = v * 10.0f + (float)(*s - '0'); s++; }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') { v += (float)(*s - '0') * f; f *= 0.1f; s++; }
    }
    return neg ? -v : v;
}

static void do_cmd(const char *s)
{
    char c = s[0];
    const char *arg = s + 1;
    int n;

    if (c == 'r')
    {
        log_head = 0; log_tail = 0; log_seq = 0; log_drop = 0;
        status("#start");
        tx_s("#seq,gyro,ang_c,ml,mr,el,er,inj,osc_c,k_c,lvl"); nl();
        log_on = 1;
    }
    else if (c == 's')
    {
        log_on = 0;
        tx_s("#end rows="); tx_i((int)log_seq);
        tx_s(" dropped="); tx_i((int)log_drop);
        nl();
        if (log_drop) { tx_s("#WARNING dropped samples, seq has gaps"); nl(); }
    }
    else if (c == 'm')
    {
        n = (int)parse_num(arg);
        if (n < 0 || n > 2) n = 0;
        ld_force = n;
        status("#ok");
    }
    else if (c == 'd')
    {
        n = (int)parse_num(arg);
        if (n < 0) n = 0;
        if (n > 2400) n = 2400;      /* leave the limiter some room to work */
        motor_ignore_pulse = n;
        motor_ignore_rev   = n - 25;   /* keep the measured asymmetry */
        status("#ok");
    }
    /* The six gains.  Only meaningful in a mode where nothing overwrites
     * them -- mode 1 is the right testbed; in modes 22-25 LD_Tick()
     * rewrites all six every tick and these are pointless. */
    else if (c == 'p') { Balance_Kp  = parse_num(arg); status("#ok"); }
    else if (c == 'q') { Balance_Kd  = parse_num(arg); status("#ok"); }
    else if (c == 'v') { Velocity_Kp = parse_num(arg); status("#ok"); }
    else if (c == 'b') { Velocity_Ki = parse_num(arg); status("#ok"); }
    else if (c == 't') { Turn_Kp     = parse_num(arg); status("#ok"); }
    else if (c == 'y') { Turn_Kd     = parse_num(arg); status("#ok"); }
    else if (c == 'k')
    {
        n = (int)parse_num(arg); if (n <= 0) n = 600;
        if (n > INJ_AMP_MAX) n = INJ_AMP_MAX;
        inj_amp = n; inj_n = IMPULSE_TICKS; inj_mode = 1;
        tx_s("#kick amp="); tx_i(n); nl();
    }
    else if (c == 'c')
    {
        n = (int)parse_num(arg); if (n <= 0) n = 300;
        if (n > INJ_AMP_MAX) n = INJ_AMP_MAX;
        inj_amp = n; inj_t = 0; inj_ph = 0.0f; inj_mode = 2;
        tx_s("#chirp amp="); tx_i(n);
        tx_s(" 0.5-25Hz 20s"); nl();
    }
    else if (c == 'x') { inj_mode = 0; tx_s("#inject off"); nl(); }
    /* mode 26 ladder, all live: n = force level (-1 = auto),
     * o = chatter threshold, w = lean threshold. */
    else if (c == 'n') { la_force   = (int)parse_num(arg); status("#ok"); }
    else if (c == 'o') { LA_OSC_HI  = parse_num(arg);      status("#ok"); }
    else if (c == 'w') { LA_LEAN_HI = parse_num(arg);      status("#ok"); }
    else if (c == 'f') { LA_OSC_LO  = parse_num(arg);      status("#ok"); }
    else if (c == 'g') { LA_ANG_PANIC = parse_num(arg);    status("#ok"); }
    else if (c == 'h') { LA_BOOT_LEVEL = (int)parse_num(arg); status("#ok"); }
    else if (c == 'j') { LA_K_EMPTY = parse_num(arg);      status("#ok"); }
    else if (c == 'u') { LA_K_FULL  = parse_num(arg);      status("#ok"); }
    else if (c == 'z')            /* log every n-th tick ('t' is Turn_Kp) */
    {
        n = (int)parse_num(arg);
        if (n < 1) n = 1;
        if (n > 20) n = 20;
        log_div = (unsigned char)n; log_cnt = 0;
        status("#ok");
    }
    else if (c == 'l') { LD_LOADED_BELOW = parse_num(arg); status("#ok"); }
    else if (c == 'e') { LD_EMPTY_ABOVE  = parse_num(arg); status("#ok"); }
    else if (c == '?') { status("#state"); }
    else
    {
        tx_s("#cmds: r s ? | p q v b t y=gains | k<amp> c<amp> x=inject | m l e d"); nl();
        tx_s("#      p=BalKp q=BalKd v=VelKp b=VelKi t=TurnKp y=TurnKd"); nl();
        tx_s("#      n o w f g = mode 26 ladder | z<n> = log every n-th tick"); nl();
    }
}

/* ---- ISR side ---------------------------------------------------------- */
void TUNE_Sample(float gyro, float angle, int ml, int mr, int el, int er)
{
    unsigned short h, s;

    /* runs every tick, logging or not -- also ages the command line */
    if (rx_len && rx_idle)
    {
        if (--rx_idle == 0) { rx_line[rx_len] = 0; rx_len = 0; rx_ready = 1; }
    }

    if (!log_on) return;

    s = log_seq++;                 /* advanced even on drop, so a gap shows */
    if (++log_cnt < log_div) return;
    log_cnt = 0;
    h = (unsigned short)((log_head + 1) % LOG_N);
    if (h == log_tail) { log_drop++; return; }

    log_buf[log_head].seq  = s;
    log_buf[log_head].gyro = (short)gyro;
    log_buf[log_head].ang  = (short)(angle * 100.0f);
    log_buf[log_head].ml   = (short)ml;
    log_buf[log_head].mr   = (short)mr;
    log_buf[log_head].el   = (short)el;
    log_buf[log_head].er   = (short)er;
    log_buf[log_head].inj  = inj_last;
    log_buf[log_head].s_m  = (short)(la_osc * 100.0f);
    log_buf[log_head].kg_c = (short)(la_k * 100.0f);
    log_buf[log_head].lvl  = (short)la_level;
    log_head = h;
}

void TUNE_RxByte(unsigned char c)
{
    if (c == '\r' || c == '\n')
    {
        rx_idle = 0;
        if (rx_len) { rx_line[rx_len] = 0; rx_len = 0; rx_ready = 1; }
        return;
    }
    if (rx_len < sizeof(rx_line) - 1) rx_line[rx_len++] = (char)c;
    rx_idle = RX_IDLE_TICKS;          /* restart the silence timer */
}

/* ---- main loop side ---------------------------------------------------- */
void TUNE_Poll(void)
{
    LogSample *p;
    int budget;

    if (rx_ready) { rx_ready = 0; do_cmd(rx_line); }

    if (fell_flag)
    {
        fell_flag = 0;
        inj_mode  = 0;                    /* stop driving a fallen car */
        tx_s("#FELL seq="); tx_i((int)log_seq);
        tx_s(" ang="); tx_f1(Angle_Balance); nl();
    }

    /* Bounded drain.  An unbounded `while (tail != head)` never returns once
     * the UART cannot keep up with the 200 Hz producer -- and since commands
     * are only dispatched above, the console goes deaf for the whole
     * recording.  That is exactly what killed the 2026-09-19 level sweep. */
    budget = 48;
    while (log_tail != log_head && budget-- > 0)
    {
        p = &log_buf[log_tail];
        tx_i(p->seq);  tx(',');
        tx_i(p->gyro); tx(',');
        tx_i(p->ang);  tx(',');
        tx_i(p->ml);   tx(',');
        tx_i(p->mr);   tx(',');
        tx_i(p->el);   tx(',');
        tx_i(p->er);   tx(',');
        tx_i(p->inj);  tx(',');
        tx_i(p->s_m);  tx(',');
        tx_i(p->kg_c); tx(',');
        tx_i(p->lvl);
        nl();
        log_tail = (unsigned short)((log_tail + 1) % LOG_N);

        if (rx_ready) { rx_ready = 0; do_cmd(rx_line); }   /* stay responsive */
    }
}

void TUNE_Init(void)
{
    log_on = 0; log_head = 0; log_tail = 0; log_seq = 0; log_drop = 0;
    rx_len = 0; rx_ready = 0; rx_idle = 0;
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);   /* uart_init leaves it off */
    nl();
    tx_s("#tune_io ready -- '?' for state, 'r' to record"); nl();
}
