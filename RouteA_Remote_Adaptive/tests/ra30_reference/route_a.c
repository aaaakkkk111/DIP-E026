#include "route_a.h"

#define RA_IDLE                 0
#define RA_PREPARED             1
#define RA_TRIAL                2
#define RA_PROFILE_NORMAL       0
#define RA_PROFILE_LOAD         1
#define RA_FW_VERSION           "RA3.0-REMOTE"
#define RA_HEARTBEAT_MS         2000UL
#define RA_MAX_TTL_MS           30000UL
#define RA_FEATURE_SAMPLES      200

extern float Balance_Kp, Balance_Kd, Velocity_Kp, Velocity_Ki, Turn_Kp, Turn_Kd;
extern enCarState g_newcarstate;

typedef struct
{
    float ap, ad, vp, vi, tp, td;
} RouteAPid;

static RouteAPid champion, candidate;
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
static volatile u32 now_ms = 0, deadline_ms = 0, heartbeat_ms = 0;
static volatile u16 tx_dropped = 0;
static u32 trial_id = 0;

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
    char output[192];
    u16 crc = ra_crc16(body, strlen(body));
    sprintf(output, "$%s,C%04X#", body, crc);
    if(!UART5_TxEnqueue(output)) tx_dropped++;
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
    char body[80];
    sprintf(body, "P3,ACK,%s,%lu,%u,%u,P%c", command, trial_id, state,
            training_locked, ra_profile_char(active_profile));
    ra_send(body);
}

static void ra_error(const char *code)
{
    char body[64];
    sprintf(body, "P3,ERR,%s,%lu", code, trial_id);
    ra_send(body);
}

static void ra_read(RouteAPid *pid)
{
    pid->ap = Balance_Kp / 100.0f;
    pid->ad = Balance_Kd;
    pid->vp = Velocity_Kp / 100.0f;
    pid->vi = Velocity_Ki;
    pid->tp = Turn_Kp / 100.0f;
    pid->td = Turn_Kd;
}

static void ra_write(const RouteAPid *pid)
{
    Balance_Kp = pid->ap * 100.0f;
    Balance_Kd = pid->ad;
    Velocity_Kp = pid->vp * 100.0f;
    Velocity_Ki = pid->vi;
    Turn_Kp = pid->tp * 100.0f;
    Turn_Kd = pid->td;
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
    RouteAPid official_normal;
    official_normal.ap = 96.0f;
    official_normal.ad = 48.0f;
    official_normal.vp = 62.0f;
    official_normal.vi = 31.0f;
    official_normal.tp = 17.0f;
    official_normal.td = 20.0f;
    ra_write(&official_normal);
    champion = official_normal;
    candidate = official_normal;
    active_profile = RA_PROFILE_NORMAL;
    champion_profile = RA_PROFILE_NORMAL;
    candidate_profile = RA_PROFILE_NORMAL;
    state = RA_IDLE;
    training_locked = 0;
    balance_started = 0;
    trial_id = 0;
    ra_reset_features();
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
    char command[12], action[12], profile_char;
    unsigned long id = 0, ttl = 0;
    RouteAPid pid;
    u8 profile;
    int parsed;

    if(!ra_verify(frame)) { ra_error("CRC"); return; }
    crc_text = strstr(frame, ",C");
    *crc_text = 0;
    command[0] = 0;
    if(sscanf(frame, "$P3,%11[^,]", command) != 1) { ra_error("FORMAT"); return; }

    if(!strcmp(command, "GET"))
    {
        char body[144];
        ra_read(&pid);
        sprintf(body, "P3,STATUS,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,P%c,S%u,L%u,B%u",
                pid.ap, pid.ad, pid.vp, pid.vi, pid.tp, pid.td,
                ra_profile_char(active_profile), state, training_locked, balance_started);
        ra_send(body);
        return;
    }

    if(!strcmp(command, "DIAG"))
    {
        if(sscanf(frame, "$P3,DIAG,%11[^,]", action) != 1) { ra_error("FORMAT"); return; }
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

    if(!strcmp(command, "MSET"))
    {
        parsed = sscanf(frame, "$P3,MSET,%c,%f,%f,%f,%f,%f,%f", &profile_char,
                        &pid.ap, &pid.ad, &pid.vp, &pid.vi, &pid.tp, &pid.td);
        if(parsed != 7 || !ra_parse_profile(profile_char, &profile)) { ra_error("FORMAT"); return; }
        if(training_locked) { ra_error("LOCKED"); return; }
        if(state != RA_IDLE) { ra_error("BUSY"); return; }
        if(!ra_pid_safe(&pid)) { ra_error("RANGE"); return; }
        candidate = pid;
        candidate_profile = profile;
        manual_apply_request = 1;
        ra_ack("MSET_QUEUED");
        return;
    }

    if(!strcmp(command, "TRAIN"))
    {
        if(sscanf(frame, "$P3,TRAIN,%11[^,]", action) != 1) { ra_error("FORMAT"); return; }
        if(!strcmp(action, "START"))
        {
            if(!balance_started) { ra_error("NOTREADY"); return; }
            if(state != RA_IDLE) { ra_error("BUSY"); return; }
            ra_read(&champion);
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

    if(!strcmp(command, "PREP"))
    {
        parsed = sscanf(frame, "$P3,PREP,%lu,%lu,%c,%f,%f,%f,%f,%f,%f", &id, &ttl,
                        &profile_char, &pid.ap, &pid.ad, &pid.vp, &pid.vi, &pid.tp, &pid.td);
        if(parsed != 9 || !ra_parse_profile(profile_char, &profile)) { ra_error("FORMAT"); return; }
        if(!training_locked) { ra_error("NOTRAIN"); return; }
        if(state != RA_IDLE) { ra_error("BUSY"); return; }
        if(ttl < 1000UL || ttl > RA_MAX_TTL_MS) { ra_error("TTL"); return; }
        if(!ra_pid_safe(&pid)) { ra_error("RANGE"); return; }
        candidate = pid;
        candidate_profile = profile;
        trial_id = id;
        deadline_ms = now_ms + ttl;
        heartbeat_ms = now_ms;
        state = RA_PREPARED;
        ra_ack("PREP");
        return;
    }

    if(sscanf(frame, "$P3,%*[^,],%lu", &id) != 1 || id != trial_id)
    {
        ra_error("TRIAL");
        return;
    }
    if(!strcmp(command, "APPLY") && state == RA_PREPARED)
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
    if(!strcmp(command, "ACCEPT") && state == RA_TRIAL)
    {
        ra_read(&champion);
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
    if(rollback_request)
    {
        ra_write(&champion);
        active_profile = champion_profile;
        PID_ResetDynamicState();
        rollback_request = 0;
        apply_request = 0;
        state = RA_IDLE;
        trial_id = 0;
        angle_danger_count = 0;
        saturation_danger_count = 0;
        if(unlock_after_rollback) { training_locked = 0; unlock_after_rollback = 0; }
    }
    else if(manual_apply_request && !training_locked && state == RA_IDLE)
    {
        ra_write(&candidate);
        active_profile = candidate_profile;
        champion = candidate;
        champion_profile = candidate_profile;
        PID_ResetDynamicState();
        manual_apply_request = 0;
        manual_applied_notice = 1;
    }
    else if(apply_request && state == RA_PREPARED)
    {
        ra_write(&candidate);
        active_profile = candidate_profile;
        PID_ResetDynamicState();
        apply_request = 0;
        angle_danger_count = 0;
        saturation_danger_count = 0;
        state = RA_TRIAL;
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
    char body[176];
    if(manual_applied_notice)
    {
        manual_applied_notice = 0;
        ra_ack("MSET_APPLIED");
    }
    if(fast_due)
    {
        fast_due = 0;
        sprintf(body, "T3,F,%lu,A%.2f,G%.1f,O%.3f,N%.3f,EL%d,ER%d,ML%d,MR%d,S%u,L%u,P%c",
                now_ms, Angle_Balance, Gyro_Balance, feature_osc, feature_lean,
                snap_el, snap_er, snap_ml, snap_mr, state, training_locked,
                ra_profile_char(active_profile));
        ra_send(body);
    }
    if(slow_due)
    {
        RouteAPid pid;
        slow_due = 0;
        ra_read(&pid);
        sprintf(body, "T3,S,%lu,V%.2f,A2%.3f,G2%.3f,PK%.2f,X%.3f,AP%.2f,AD%.2f,VP%.2f,VI%.2f,TP%.2f,TD%.2f,P%c,D%u,B%u,%s",
                now_ms, battery, snap_angle_sq_mean, snap_gyro_sq_mean, snap_peak,
                snap_saturation, pid.ap, pid.ad, pid.vp, pid.vi, pid.tp, pid.td,
                ra_profile_char(active_profile), tx_dropped, balance_started, RA_FW_VERSION);
        ra_send(body);
    }
}



