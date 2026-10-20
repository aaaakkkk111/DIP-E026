#include "route_a.h"

#define RA_IDLE                 0
#define RA_PREPARED             1
#define RA_TRIAL                2
#define RA_PROFILE_NORMAL       0
#define RA_PROFILE_LOAD         1
#define RA_FW_VERSION           "RA3.3-DIRECT"
#define RA_HEARTBEAT_MS         2000UL
#define RA_MAX_TTL_MS           30000UL
#define RA_FEATURE_SAMPLES      200
#define RA_BALANCE_MASK         1
#define RA_VELOCITY_MASK        2
#define RA_TURN_MASK            4
#define RA_ALL_MASK             7

extern float Balance_Kp, Balance_Kd, Velocity_Kp, Velocity_Ki, Turn_Kp, Turn_Kd;
extern enCarState g_newcarstate;

typedef struct
{
    float ap, ad, vp, vi, tp, td;
} RouteAPid;

/* champion and candidate contain pre-scaled internal values, not wire values. */
static RouteAPid champion, candidate;
static volatile u8 candidate_mask = RA_ALL_MASK, candidate_switch_profile = 0;
static volatile u8 candidate_legacy = 0, applied_legacy_mask = 0;
static volatile u8 legacy_autoup = 0;
static float balance_scale = 1.0f, velocity_scale = 1.0f;
static volatile u8 state = RA_IDLE;
static volatile u8 apply_request = 0;
static volatile u8 manual_apply_request = 0;
static volatile u8 rollback_request = 0;
static volatile u8 unlock_after_rollback = 0;
static volatile u8 fast_due = 0, slow_due = 0;
static volatile u8 training_locked = 0, balance_started = 0;
static volatile u8 active_profile = RA_PROFILE_NORMAL;
static volatile u8 champion_profile = RA_PROFILE_NORMAL;
static volatile u8 candidate_profile = RA_PROFILE_NORMAL;
static volatile u8 manual_applied_notice = 0;
static volatile u8 rollback_applied_notice = 0, manual_rollback_notice = 0;
static volatile u32 now_ms = 0, deadline_ms = 0, heartbeat_ms = 0;
static volatile u16 tx_dropped = 0;
static u32 trial_id = 0;
/* Main-loop-only scratch buffers: never allocated under a nested control ISR. */
static char tx_output[320], task_body[280], status_body[180];
static char ack_body[80], error_body[64];
static char *rx_fields[16];
extern u32 __stack_limit[];
#define RA_STACK_WORDS 1024
#ifndef RA_HOST_TEST
#define RA_DWT_CTRL (*(volatile u32 *)0xE0001000UL)
#define RA_DWT_COUNT (*(volatile u32 *)0xE0001004UL)
#else
static volatile u32 RA_DWT_CTRL, RA_DWT_COUNT;
#endif
static volatile u32 control_enter_cycle = 0, control_last_cycle = 0;
static volatile u32 control_gap_max = 0, control_exec_max = 0;
static u32 clock_mhz = 72;
static u16 stack_free_min = 4096;
static volatile u8 memory_fault = 0, memory_fault_notice = 0;

static int snap_el, snap_er, snap_ml, snap_mr;
static volatile int diag_left = 0, diag_right = 0;
static volatile u16 diag_ticks = 0;
static u8 angle_danger_count = 0, saturation_danger_count = 0;

/* Feature extraction only. These values never select a profile or modify PID. */
static float feature_lean = 0.0f;
static float feature_osc = 0.0f;
static float feature_angle_sq = 0.0f;
static float feature_gyro_sq = 0.0f;
static float feature_peak = 0.0f;
static u16 feature_saturation = 0;
static u16 feature_count = 0;
static volatile float snap_angle_sq_mean = 0.0f;
static volatile float snap_gyro_sq_mean = 0.0f;
static volatile float snap_peak = 0.0f;
static volatile float snap_saturation = 0.0f;

static float ra_abs(float value)
{
    return value < 0.0f ? -value : value;
}

static int ra_numbers_ok(const RouteAPid *pid)
{
    return pid->ap == pid->ap && pid->ad == pid->ad &&
           pid->vp == pid->vp && pid->vi == pid->vi &&
           pid->tp == pid->tp && pid->td == pid->td;
}

static int ra_pid_safe(const RouteAPid *pid)
{
    if(!ra_numbers_ok(pid)) return 0;
    return pid->ap >= 20.0f && pid->ap <= 288.0f &&
           pid->ad >= 5.0f && pid->ad <= 200.0f &&
           pid->vp >= 10.0f && pid->vp <= 72.0f &&
           pid->vi >= 1.0f && pid->vi <= 100.0f &&
           pid->tp >= 1.0f && pid->tp <= 100.0f &&
           pid->td >= 1.0f && pid->td <= 100.0f;
}

/* Single coefficient-store path for P3, legacy updates and rollback.
   No parsing, scaling, filter/integral reset, profile changes or PWM writes here. */
static void ra_store_raw(const RouteAPid *raw, u8 mask)
{
    if(mask & RA_BALANCE_MASK) { Balance_Kp = raw->ap; Balance_Kd = raw->ad; }
    if(mask & RA_VELOCITY_MASK) { Velocity_Kp = raw->vp; Velocity_Ki = raw->vi; }
    if(mask & RA_TURN_MASK) { Turn_Kp = raw->tp; Turn_Kd = raw->td; }
}

static void ra_restore_normal(void)
{
    const RouteAPid normal = {9600.0f,48.0f,6200.0f,31.0f,1700.0f,20.0f};
    ra_store_raw(&normal, RA_ALL_MASK);
}

static int ra_raw_safe(const RouteAPid *raw)
{
    return ra_numbers_ok(raw) && raw->ap >= 2000.0f && raw->ap <= 28800.0f &&
           raw->ad >= 5.0f && raw->ad <= 200.0f &&
           raw->vp >= 1000.0f && raw->vp <= 7200.0f &&
           raw->vi >= 1.0f && raw->vi <= 100.0f &&
           raw->tp >= 100.0f && raw->tp <= 10000.0f &&
           raw->td >= 1.0f && raw->td <= 100.0f;
}

static u16 ra_crc16(const char *text, int length)
{
    u16 crc = 0xFFFF;
    int i, bit;
    for(i = 0; i < length; i++)
    {
        crc ^= (u16)((u8)text[i]) << 8;
        for(bit = 0; bit < 8; bit++)
            crc = (crc & 0x8000) ? (u16)((crc << 1) ^ 0x1021) : (u16)(crc << 1);
    }
    return crc;
}

static int ra_hex(char value)
{
    if(value >= '0' && value <= '9') return value - '0';
    if(value >= 'A' && value <= 'F') return value - 'A' + 10;
    if(value >= 'a' && value <= 'f') return value - 'a' + 10;
    return -1;
}

static int ra_verify(char *frame)
{
    char *crc_text;
    int i, nibble, expected = 0;
    if(frame[0] != '$') return 0;
    crc_text = strstr(frame, ",C");
    if(!crc_text || strlen(crc_text) != 7 || crc_text[6] != '#') return 0;
    for(i = 0; i < 4; i++)
    {
        nibble = ra_hex(crc_text[2 + i]);
        if(nibble < 0) return 0;
        expected = (expected << 4) | nibble;
    }
    return ra_crc16(frame + 1, (int)(crc_text - frame - 1)) == (u16)expected;
}

static void ra_send(const char *body)
{

    u16 crc = ra_crc16(body, strlen(body));
    sprintf(tx_output, "$%s,C%04X#", body, crc);
    if(!UART5_TxEnqueue(tx_output)) tx_dropped++;
}

static char ra_profile_char(u8 profile)
{
    return profile == RA_PROFILE_LOAD ? 'L' : 'N';
}

static int ra_parse_profile(char value, u8 *profile)
{
    if(value == 'N') { *profile = RA_PROFILE_NORMAL; return 1; }
    if(value == 'L') { *profile = RA_PROFILE_LOAD; return 1; }
    return 0;
}

static void ra_ack(const char *command)
{
    sprintf(ack_body, "P3,ACK,%s,%lu,%u,%u,P%c", command, trial_id, state,
            training_locked, ra_profile_char(active_profile));
    ra_send(ack_body);
}

static void ra_error(const char *code)
{
    sprintf(error_body, "P3,ERR,%s,%lu", code, trial_id);
    ra_send(error_body);
}

static void ra_read_raw(RouteAPid *raw)
{
    u32 mask = __get_PRIMASK();
    __disable_irq();
    raw->ap = Balance_Kp; raw->ad = Balance_Kd;
    raw->vp = Velocity_Kp; raw->vi = Velocity_Ki;
    raw->tp = Turn_Kp; raw->td = Turn_Kd;
    __set_PRIMASK(mask);
}

static void ra_read(RouteAPid *pid)
{
    ra_read_raw(pid);
    pid->ap *= 0.01f; pid->vp *= 0.01f; pid->tp *= 0.01f;
}

/* Main loop only: convert before publishing a control-boundary request. */
static void ra_encode(const RouteAPid *wire, RouteAPid *raw)
{
    *raw = *wire;
    raw->ap = wire->ap * 100.0f;
    raw->vp = wire->vp * 100.0f;
    raw->tp = wire->tp * 100.0f;
}

/* Strict bounded decimal parsing, matching the two-decimal wire format.
   No scanf/strtod, exponent, sign, NaN, infinity, or silent trailing text. */
static int ra_uint(const char *text, u32 *result)
{
    u32 value = 0;
    int count = 0, digit;
    if(!text || !*text) return 0;
    while(*text)
    {
        digit = *text++ - '0';
        if(digit < 0 || digit > 9 || ++count > 10) return 0;
        if(value > 429496729UL || (value == 429496729UL && digit > 5)) return 0;
        value = value * 10UL + (u32)digit;
    }
    *result = value;
    return 1;
}

static int ra_decimal(const char *text, float *result)
{
    unsigned int whole = 0, frac = 0;
    int digits = 0, places = 0;
    if(!text || !*text) return 0;
    while(*text >= '0' && *text <= '9')
    {
        if(++digits > 3) return 0;
        whole = whole * 10U + (unsigned int)(*text++ - '0');
    }
    if(!digits) return 0;
    if(*text == '.')
    {
        text++;
        while(*text >= '0' && *text <= '9')
        {
            if(++places > 2) return 0;
            frac = frac * 10U + (unsigned int)(*text++ - '0');
        }
        if(!places) return 0;
    }
    if(*text) return 0;
    if(places == 1) frac *= 10U;
    *result = (float)whole + (float)frac * 0.01f;
    return 1;
}

static int ra_tokenize(char *body)
{
    int count = 0;
    char *p = body;
    while(1)
    {
        if(count >= 16 || !*p) return 0;
        rx_fields[count++] = p;
        while(*p && *p != ',') p++;
        if(!*p) return count;
        *p++ = 0;
    }
}

static int ra_parse_pid(char **fields, RouteAPid *pid)
{
    return ra_decimal(fields[0], &pid->ap) && ra_decimal(fields[1], &pid->ad) &&
           ra_decimal(fields[2], &pid->vp) && ra_decimal(fields[3], &pid->vi) &&
           ra_decimal(fields[4], &pid->tp) && ra_decimal(fields[5], &pid->td);
}

static u16 ra_stack_free(void)
{
    u16 count = 0, free_bytes;
    while(count < RA_STACK_WORDS && __stack_limit[count] == 0xA5A5A5A5UL) count++;
    free_bytes = (u16)(count * 4);
    if(free_bytes < stack_free_min) stack_free_min = free_bytes;
    return stack_free_min;
}

void RouteA_ControlEnter(void)
{
    u32 cycle = RA_DWT_COUNT, gap;
    if(control_last_cycle)
    {
        gap = (u32)(cycle - control_last_cycle);
        if(gap > control_gap_max) control_gap_max = gap;
    }
    control_last_cycle = cycle;
    control_enter_cycle = cycle;
}

float RouteA_GetBalanceScale(void) { return balance_scale; }
float RouteA_GetVelocityScale(void) { return velocity_scale; }

static void ra_select_profile(u8 profile)
{
    active_profile = profile;
    balance_scale = profile == RA_PROFILE_LOAD ? 2.0f : 1.0f;
    velocity_scale = profile == RA_PROFILE_LOAD ? 1.35f : 1.0f;
}

static void ra_preset(u8 profile, RouteAPid *pid)
{
    pid->ap = 96.0f; pid->td = 20.0f;
    if(profile == RA_PROFILE_LOAD)
    { pid->ad = 75.0f; pid->vp = 70.0f; pid->vi = 35.0f; pid->tp = 14.0f; }
    else
    { pid->ad = 48.0f; pid->vp = 62.0f; pid->vi = 31.0f; pid->tp = 17.0f; }
}

static int ra_mask(const char *field, u8 *mask)
{
    if(!field || field[0] != 'M' || field[1] < '1' || field[1] > '7' || field[2]) return 0;
    *mask = (u8)(field[1] - '0');
    return 1;
}

static int ra_merge_validate(const RouteAPid *wire, u8 mask, RouteAPid *merged)
{
    ra_read(merged);
    if(mask & RA_BALANCE_MASK) { merged->ap = wire->ap; merged->ad = wire->ad; }
    if(mask & RA_VELOCITY_MASK) { merged->vp = wire->vp; merged->vi = wire->vi; }
    if(mask & RA_TURN_MASK) { merged->tp = wire->tp; merged->td = wire->td; }
    return ra_pid_safe(merged);
}

static void ra_stage(const RouteAPid *wire, u8 profile, u8 mask, u8 switch_profile, u8 legacy)
{
    RouteAPid raw;
    u32 irq;
    ra_encode(wire, &raw);
    irq = __get_PRIMASK();
    __disable_irq();
    candidate = raw;
    candidate_profile = profile;
    candidate_mask = mask;
    candidate_switch_profile = switch_profile;
    candidate_legacy = legacy;
    __set_PRIMASK(irq);
}

static void ra_reset_features(void)
{
    feature_lean = 0.0f;
    feature_osc = 0.0f;
    feature_angle_sq = 0.0f;
    feature_gyro_sq = 0.0f;
    feature_peak = 0.0f;
    feature_saturation = 0;
    feature_count = 0;
    snap_angle_sq_mean = 0.0f;
    snap_gyro_sq_mean = 0.0f;
    snap_peak = 0.0f;
    snap_saturation = 0.0f;
}

void RouteA_Init(void)
{
    ra_restore_normal();
    ra_read_raw(&champion);
    candidate = champion;
    ra_select_profile(RA_PROFILE_NORMAL);
    champion_profile = candidate_profile = RA_PROFILE_NORMAL;
    candidate_mask = RA_ALL_MASK; candidate_switch_profile = candidate_legacy = 0;
    applied_legacy_mask = legacy_autoup = 0;
    state = RA_IDLE;
    apply_request = manual_apply_request = rollback_request = 0;
    unlock_after_rollback = manual_applied_notice = 0;
    rollback_applied_notice = manual_rollback_notice = 0;
    training_locked = balance_started = 0;
    trial_id = now_ms = 0;
    fast_due = slow_due = 0;
    angle_danger_count = saturation_danger_count = 0;
    ra_reset_features();
    control_gap_max = control_exec_max = control_last_cycle = 0;
    memory_fault = memory_fault_notice = 0;
#ifndef RA_HOST_TEST
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    RA_DWT_COUNT = 0;
    RA_DWT_CTRL |= 1UL;
    clock_mhz = SystemCoreClock / 1000000UL;
#endif
    if(!clock_mhz) clock_mhz = 72;
}

void RouteA_BalanceStarted(void)
{
    diag_ticks = 0;
    diag_left = 0;
    diag_right = 0;
    Set_Pwm(0, 0);
    balance_started = 1;
    ra_send("P3,READY,1");
}

int RouteA_ManualLocked(void)
{
    return training_locked ? 1 : 0;
}

int RouteA_ProfileIsLoad(void)
{
    return active_profile == RA_PROFILE_LOAD ? 1 : 0;
}

int RouteA_IsFrame(const u8 *frame)
{
    return frame && frame[0] == '$' && frame[1] == 'P' &&
           frame[2] == '3' && frame[3] == ',';
}

void RouteA_HandleFrame(char *frame)
{
    char *crc_text;
    char *command, *action;
    u32 id = 0, ttl = 0;
    RouteAPid pid, merged;
    u8 profile, mask, preset_request;
    int fields;

    if(!ra_verify(frame)) { ra_error("CRC"); return; }
    crc_text = strstr(frame, ",C");
    *crc_text = 0;
    fields = ra_tokenize(frame + 1);
    if(fields < 2 || strcmp(rx_fields[0], "P3")) { ra_error("FORMAT"); return; }

    legacy_autoup = 0;
    command = rx_fields[1];
    if(!strcmp(command, "PROBE"))
    {
        if(fields != 2) { ra_error("FORMAT"); return; }
        ra_ack("PROBE");
        return;
    }
    if(!strcmp(command, "GET"))
    {
        if(fields != 2) { ra_error("FORMAT"); return; }
        ra_read(&pid);
        sprintf(status_body, "P3,STATUS,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,P%c,S%u,L%u,B%u",
                pid.ap, pid.ad, pid.vp, pid.vi, pid.tp, pid.td,
                ra_profile_char(active_profile), state, training_locked, balance_started);
        ra_send(status_body);
        return;
    }

    if(memory_fault && strcmp(command, "STOP") && strcmp(command, "TRAIN"))
    {
        ra_error("MEMORY_LATCH");
        return;
    }
    if(!strcmp(command, "DIAG"))
    {
        if(fields != 3) { ra_error("FORMAT"); return; }
        action = rx_fields[2];
        if(balance_started || training_locked || state != RA_IDLE) { ra_error("DIAG_STATE"); return; }
        diag_left = 0; diag_right = 0; diag_ticks = 0;
        if(!strcmp(action, "L+")) diag_left = 1500;
        else if(!strcmp(action, "L-")) diag_left = -1500;
        else if(!strcmp(action, "R+")) diag_right = 1500;
        else if(!strcmp(action, "R-")) diag_right = -1500;
        else if(strcmp(action, "STOP")) { ra_error("FORMAT"); return; }
        if(diag_left || diag_right) diag_ticks = 120;
        else Set_Pwm(0, 0);
        ra_ack(action);
        return;
    }

    if(!strcmp(command, "MSET") || !strcmp(command, "PRESET"))
    {
        preset_request = !strcmp(command, "PRESET");
        mask = RA_ALL_MASK;
        if((preset_request && fields != 3) ||
           (!preset_request && fields != 9 && fields != 10) ||
           strlen(rx_fields[2]) != 1 || !ra_parse_profile(rx_fields[2][0], &profile))
        { ra_error("FORMAT"); return; }
        if(training_locked) { ra_error("LOCKED"); return; }
        if(state != RA_IDLE || manual_apply_request || manual_applied_notice)
        { ra_error("BUSY"); return; }
        if(preset_request) ra_preset(profile, &pid);
        else
        {
            if(!ra_parse_pid(rx_fields + 3, &pid) ||
               (fields == 10 && !ra_mask(rx_fields[9], &mask)))
            { ra_error("FORMAT"); return; }
            if(profile != active_profile) { ra_error("PROFILE"); return; }
        }
        if(!ra_merge_validate(&pid, mask, &merged)) { ra_error("RANGE"); return; }
        ra_stage(&merged, profile, mask, preset_request, 0);
        manual_apply_request = 1;
        ra_ack("MSET_QUEUED");
        return;
    }

    if(!strcmp(command, "TRAIN"))
    {
        if(fields != 3) { ra_error("FORMAT"); return; }
        action = rx_fields[2];
        if(!strcmp(action, "START"))
        {
            if(!balance_started) { ra_error("NOTREADY"); return; }
            if(state != RA_IDLE || manual_apply_request) { ra_error("BUSY"); return; }
            ra_read_raw(&champion);
            champion_profile = active_profile;
            training_locked = 1;
            ra_ack("TRAIN_START");
            return;
        }
        if(!strcmp(action, "STOP"))
        {
            if(state != RA_IDLE)
            {
                unlock_after_rollback = 1;
                rollback_request = 1;
                g_newcarstate = enSTOP;
            }
            else training_locked = 0;
            ra_ack("TRAIN_STOP");
            return;
        }
        ra_error("FORMAT");
        return;
    }

    if(!strcmp(command, "PREP") || !strcmp(command, "PPRESET"))
    {
        preset_request = !strcmp(command, "PPRESET");
        mask = RA_ALL_MASK;
        if((preset_request && fields != 5) ||
           (!preset_request && fields != 11 && fields != 12) ||
           !ra_uint(rx_fields[2], &id) || !ra_uint(rx_fields[3], &ttl) ||
           strlen(rx_fields[4]) != 1 || !ra_parse_profile(rx_fields[4][0], &profile))
        { ra_error("FORMAT"); return; }
        if(!training_locked) { ra_error("NOTRAIN"); return; }
        if(state != RA_IDLE || manual_apply_request || manual_applied_notice)
        { ra_error("BUSY"); return; }
        if(ttl < 1000UL || ttl > RA_MAX_TTL_MS) { ra_error("TTL"); return; }
        if(preset_request) ra_preset(profile, &pid);
        else
        {
            if(!ra_parse_pid(rx_fields + 5, &pid) ||
               (fields == 12 && !ra_mask(rx_fields[11], &mask)))
            { ra_error("FORMAT"); return; }
            if(profile != active_profile) { ra_error("PROFILE"); return; }
        }
        if(!ra_merge_validate(&pid, mask, &merged)) { ra_error("RANGE"); return; }
        ra_stage(&merged, profile, mask, preset_request, 0);
        trial_id = id;
        deadline_ms = now_ms + ttl; heartbeat_ms = now_ms;
        state = RA_PREPARED;
        ra_ack("PREP");
        return;
    }

    if(fields != 3 || !ra_uint(rx_fields[2], &id) || id != trial_id)
    {
        ra_error("TRIAL");
        return;
    }
    if(!strcmp(command, "APPLY") && state == RA_PREPARED && !apply_request)
    {
        apply_request = 1;
        heartbeat_ms = now_ms;
        ra_ack("APPLY");
        return;
    }
    if(!strcmp(command, "HB") && (state == RA_PREPARED || state == RA_TRIAL))
    {
        heartbeat_ms = now_ms;
        ra_ack("HB");
        return;
    }
    if(!strcmp(command, "ACCEPT") && state == RA_TRIAL && !rollback_request)
    {
        ra_read_raw(&champion);
        champion_profile = active_profile;
        state = RA_IDLE;
        ra_ack("ACCEPT");
        trial_id = 0;
        return;
    }
    if(!strcmp(command, "ROLLBACK") && state != RA_IDLE)
    {
        rollback_request = 1;
        ra_ack("ROLLBACK");
        return;
    }
    if(!strcmp(command, "STOP"))
    {
        g_newcarstate = enSTOP;
        ra_ack("STOP");
        return;
    }
    ra_error("STATE");
}

static void ra_legacy_pid(void)
{
    RouteAPid pid;
    ra_read(&pid);
    sprintf(status_body, "$AP%.2f,AD%.2f,VP%.2f,VI%.2f,TP%.2f,TD%.2f#",
            pid.ap,pid.ad,pid.vp,pid.vi,pid.tp,pid.td);
    UART5_TxEnqueue(status_body);
}

void RouteA_HandleLegacy(char *frame)
{
    int fields, i, length;
    u8 mask = 0, restore;
    RouteAPid wire, merged;
    static const char *keys[6] = {"AP","AD","VP","VI","TP","TD"};
    if(!frame || frame[0] != '$') return;
    length = strlen(frame);
    if(length < 21 || length >= 80 || frame[length-1] != '#')
    { UART5_TxEnqueue("$ReceivePackError#"); return; }
    frame[length-1] = 0;
    fields = ra_tokenize(frame + 1);
    if(fields != 13) { UART5_TxEnqueue("$ReceivePackError#"); return; }
    for(i=0;i<7;i++)
    {
        if(strlen(rx_fields[i]) != 1 || rx_fields[i][0] < '0' ||
           rx_fields[i][0] > (i==0 ? '4' : (i<4 ? '2' : '1')))
        { UART5_TxEnqueue("$ReceivePackError#"); return; }
    }
    for(i=0;i<6;i++)
    {
        if(strncmp(rx_fields[7+i], keys[i], 2))
        { UART5_TxEnqueue("$ReceivePackError#"); return; }
        rx_fields[7+i] += 2;
    }
    if(!ra_parse_pid(rx_fields+7, &wire)) { UART5_TxEnqueue("$ReceivePackError#"); return; }
    switch(rx_fields[0][0])
    {
        case '1': g_newcarstate=enRUN; break;
        case '2': g_newcarstate=enBACK; break;
        case '3': g_newcarstate=enLEFT; break;
        case '4': g_newcarstate=enRIGHT; break;
        default: g_newcarstate=enSTOP; break;
    }
    if(rx_fields[1][0]=='1') g_newcarstate=enTLEFT;
    if(rx_fields[1][0]=='2') g_newcarstate=enTRIGHT;
    if(rx_fields[4][0]=='1') mask |= RA_BALANCE_MASK;
    if(rx_fields[5][0]=='1') mask |= RA_VELOCITY_MASK;
    if(rx_fields[6][0]=='1') mask |= RA_TURN_MASK;
    restore = rx_fields[2][0]=='2';
    if(mask || restore)
    {
        if(memory_fault || training_locked) { UART5_TxEnqueue("$TrainingLocked#"); return; }
        if(state != RA_IDLE || manual_apply_request || manual_applied_notice)
        { UART5_TxEnqueue("$Busy#"); return; }
        if(restore) { mask=RA_ALL_MASK; ra_preset(RA_PROFILE_NORMAL,&wire); }
        if(!ra_merge_validate(&wire,mask,&merged)) { UART5_TxEnqueue("$GetPIDError#"); return; }
        ra_stage(&merged,restore ? RA_PROFILE_NORMAL : active_profile,mask,restore,1);
        manual_apply_request=1;
    }
    if(rx_fields[2][0]=='1') ra_legacy_pid();
    if(rx_fields[3][0]=='1') { legacy_autoup=1; UART5_TxEnqueue("$OK#"); }
    if(rx_fields[3][0]=='2') { legacy_autoup=0; UART5_TxEnqueue("$OK#"); }
}

void RouteA_Tick10ms(void)
{
    static u16 fast_counter = 0, slow_counter = 0;
    now_ms += 10;
    if(++fast_counter >= 20) { fast_counter = 0; fast_due = 1; }
    if(++slow_counter >= 100) { slow_counter = 0; slow_due = 1; }
    if(state != RA_IDLE &&
       ((s32)(now_ms - deadline_ms) >= 0 || (u32)(now_ms - heartbeat_ms) > RA_HEARTBEAT_MS))
    {
        rollback_request = 1;
        g_newcarstate = enSTOP;
    }
}

void RouteA_ControlBegin(void)
{
    RouteAPid live;
    ra_read_raw(&live);
    if(memory_fault || !ra_raw_safe(&live) || !ra_raw_safe(&champion) ||
       ((manual_apply_request || apply_request) &&
        (!ra_raw_safe(&candidate) || !candidate_mask || candidate_mask > RA_ALL_MASK ||
         candidate_profile > RA_PROFILE_LOAD)) ||
       active_profile > RA_PROFILE_LOAD || state > RA_TRIAL ||
       !(balance_scale >= 1.0f && balance_scale <= 2.0f) ||
       !(velocity_scale >= 1.0f && velocity_scale <= 1.35f) ||
       __stack_limit[0] != 0xA5A5A5A5UL)
    {
        if(!memory_fault) memory_fault_notice = 1;
        memory_fault = 1; ra_restore_normal(); ra_select_profile(RA_PROFILE_NORMAL);
        state = RA_IDLE; trial_id = 0; training_locked = balance_started = 0;
        apply_request = manual_apply_request = rollback_request = 0;
        manual_applied_notice = rollback_applied_notice = 0;
        Stop_Flag = 1; Set_Pwm(0,0);
        return;
    }
    if(rollback_request)
    {
        ra_store_raw(&champion, RA_ALL_MASK);
        ra_select_profile(champion_profile);
        rollback_request = apply_request = manual_apply_request = 0;
        state = RA_IDLE; trial_id = 0;
        rollback_applied_notice = 1;
        angle_danger_count = saturation_danger_count = 0;
        if(unlock_after_rollback) { training_locked = 0; unlock_after_rollback = 0; }
    }
    else if(manual_apply_request && !training_locked && state == RA_IDLE)
    {
        ra_store_raw(&candidate, candidate_mask);
        if(candidate_switch_profile) ra_select_profile(candidate_profile);
        ra_read_raw(&champion); champion_profile = active_profile;
        applied_legacy_mask = candidate_legacy ? candidate_mask : 0;
        manual_apply_request = 0; manual_applied_notice = 1;
    }
    else if(apply_request && state == RA_PREPARED)
    {
        ra_store_raw(&candidate, candidate_mask);
        if(candidate_switch_profile) ra_select_profile(candidate_profile);
        apply_request = 0; state = RA_TRIAL;
        angle_danger_count = saturation_danger_count = 0;
    }
}

void RouteA_ControlCapture(int el, int er, int bp, int vp, int tp, int ml, int mr)
{
    float angle_abs, high_pass;
    (void)bp; (void)vp; (void)tp;
    snap_el = el; snap_er = er; snap_ml = ml; snap_mr = mr;

    feature_lean += (Angle_Balance - feature_lean) * 0.0025f;
    high_pass = Angle_Balance - feature_lean;
    feature_osc += (ra_abs(high_pass) - feature_osc) * 0.025f;
    angle_abs = ra_abs(Angle_Balance);
    feature_angle_sq += Angle_Balance * Angle_Balance;
    feature_gyro_sq += Gyro_Balance * Gyro_Balance;
    if(angle_abs > feature_peak) feature_peak = angle_abs;
    if(ra_abs((float)ml) >= 2595.0f || ra_abs((float)mr) >= 2595.0f) feature_saturation++;
    if(++feature_count >= RA_FEATURE_SAMPLES)
    {
        snap_angle_sq_mean = feature_angle_sq / (float)feature_count;
        snap_gyro_sq_mean = feature_gyro_sq / (float)feature_count;
        snap_peak = feature_peak;
        snap_saturation = (float)feature_saturation / (float)feature_count;
        feature_angle_sq = 0.0f;
        feature_gyro_sq = 0.0f;
        feature_peak = 0.0f;
        feature_saturation = 0;
        feature_count = 0;
    }

    if(state == RA_TRIAL)
    {
        if(angle_abs > 25.0f || battery < 9.6f)
        {
            if(angle_danger_count < 255) angle_danger_count++;
        }
        else angle_danger_count = 0;

        if(ra_abs((float)ml) >= 2595.0f || ra_abs((float)mr) >= 2595.0f)
        {
            if(saturation_danger_count < 255) saturation_danger_count++;
        }
        else saturation_danger_count = 0;

        if(angle_danger_count >= 3 || saturation_danger_count >= 40)
        {
            rollback_request = 1;
            g_newcarstate = enSTOP;
        }
    }
}

void RouteA_ControlEnd(void)
{
    u32 elapsed = (u32)(RA_DWT_COUNT - control_enter_cycle);
    if(elapsed > control_exec_max) control_exec_max = elapsed;
    if(memory_fault)
    {
        Stop_Flag = 1;
        Set_Pwm(0, 0);
        return;
    }
    if(diag_ticks && !balance_started && Stop_Flag == 1)
    {
        Set_Pwm(diag_left, diag_right);
        diag_ticks--;
        if(diag_ticks == 0)
        {
            diag_left = 0;
            diag_right = 0;
            Set_Pwm(0, 0);
        }
    }
}

void RouteA_Task(void)
{
    if(memory_fault_notice)
    {
        memory_fault_notice = 0;
        ra_error("PID_MEMORY");
    }

    if(manual_applied_notice)
    {
        manual_applied_notice = 0;
        if(applied_legacy_mask)
        {
            if(applied_legacy_mask & RA_BALANCE_MASK) UART5_TxEnqueue("$OK#");
            if(applied_legacy_mask & RA_VELOCITY_MASK) UART5_TxEnqueue("$OK#");
            if(applied_legacy_mask & RA_TURN_MASK) UART5_TxEnqueue("$OK#");
            applied_legacy_mask = 0;
        }
        else ra_ack("MSET_APPLIED");
    }
    if(rollback_applied_notice)
    {
        rollback_applied_notice = 0;
        if(manual_rollback_notice)
        {
            manual_rollback_notice = 0;
            ra_error("MSET_ROLLBACK");
        }
        ra_ack("ROLLED_BACK");
    }
    if(legacy_autoup)
    {
        if(fast_due)
        {
            fast_due=0;
            sprintf(task_body,"$LV%.2f,RV%.2f,AC%.2f,GY%.2f,CSB%.2f,VT%.2f#",
                    Velocity_Left,Velocity_Right,Acceleration_Z/100.0f,Gyro_Balance,
                    (float)g_distance/10.0f,battery);
            UART5_TxEnqueue(task_body);
        }
        slow_due=0;
        return;
    }
    if(fast_due)
    {
        fast_due = 0;
        sprintf(task_body, "T3,F,%lu,A%.2f,G%.1f,O%.3f,N%.3f,EL%d,ER%d,ML%d,MR%d,S%u,L%u,P%c,U%u",
                now_ms, Angle_Balance, Gyro_Balance, feature_osc, feature_lean,
                snap_el, snap_er, snap_ml, snap_mr, state, training_locked,
                ra_profile_char(active_profile), 0);
        ra_send(task_body);
    }
    if(slow_due)
    {
        RouteAPid pid;
        slow_due = 0;
        ra_read(&pid);
        sprintf(task_body, "T3,S,%lu,V%.2f,A2%.3f,G2%.3f,PK%.2f,X%.3f,AP%.2f,AD%.2f,VP%.2f,VI%.2f,TP%.2f,TD%.2f,P%c,D%u,B%u,%s,SF%u,DT%lu,CT%lu",
                now_ms, battery, snap_angle_sq_mean, snap_gyro_sq_mean, snap_peak,
                snap_saturation, pid.ap, pid.ad, pid.vp, pid.vi, pid.tp, pid.td,
                ra_profile_char(active_profile), tx_dropped, balance_started, RA_FW_VERSION,
                ra_stack_free(), control_gap_max / clock_mhz, control_exec_max / clock_mhz);
        ra_send(task_body);
    }
}



