#include "route_a.h"

#define RA_IDLE                 0
#define RA_PREPARED             1
#define RA_TRIAL                2
#define RA_PROFILE_NORMAL       0
#define RA_PROFILE_LOAD         1
#define RA_FW_VERSION           "RA3.0-SEQ1"
#define RA_HEARTBEAT_MS         2000UL
#define RA_MAX_TTL_MS           30000UL
#define RA_FEATURE_SAMPLES      200

extern float Balance_Kp, Balance_Kd, Velocity_Kp, Velocity_Ki, Turn_Kp, Turn_Kd;
extern enCarState g_newcarstate;

typedef struct
{
    float ap, ad, vp, vi, tp, td;
} RouteAPid;

static RouteAPid champion;
static volatile u8 seq_active = 0, seq_finishing = 0, seq_apply = 0;
static volatile u8 seq_one_notice = 0, seq_done_notice = 0, rollback_notice = 0;
static u8 seq_manual = 0, seq_target_profile = 0, seq_step = 0, seq_index = 0;
static float seq_value = 0, seq_raw = 0;
static char seq_key[3] = "AP";
static u32 seq_last_id = 0;
static char tx_output[320], task_body[280], status_body[180], ack_body[80], error_body[64];
static char *rx_fields[8];

static volatile u8 state = RA_IDLE;
static volatile u8 rollback_request = 0;
static volatile u8 unlock_after_rollback = 0;
static volatile u8 fast_due = 0, slow_due = 0;
static volatile u8 training_locked = 0, balance_started = 0;
static volatile u8 active_profile = RA_PROFILE_NORMAL;
static volatile u8 champion_profile = RA_PROFILE_NORMAL;
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

    u16 crc = ra_crc16(body, strlen(body));
    snprintf(tx_output, sizeof(tx_output), "$%s,C%04X#", body, crc);
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

    snprintf(ack_body, sizeof(ack_body), "P3,ACK,%s,%lu,%u,%u,P%c", command, trial_id, state,
            training_locked, ra_profile_char(active_profile));
    ra_send(ack_body);
}

static void ra_error(const char *code)
{

    snprintf(error_body, sizeof(error_body), "P3,ERR,%s,%lu", code, trial_id);
    ra_send(error_body);
}

static void ra_read(RouteAPid *pid)
{
    u32 irq = __get_PRIMASK();
    __disable_irq();
    pid->ap = Balance_Kp; pid->ad = Balance_Kd;
    pid->vp = Velocity_Kp; pid->vi = Velocity_Ki;
    pid->tp = Turn_Kp; pid->td = Turn_Kd;
    __set_PRIMASK(irq);
    pid->ap /= 100.0f; pid->vp /= 100.0f; pid->tp /= 100.0f;
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

    active_profile = RA_PROFILE_NORMAL;
    champion_profile = RA_PROFILE_NORMAL;
    state = RA_IDLE;
    training_locked = 0;
    balance_started = 0;
    trial_id = seq_last_id = now_ms = 0;
    seq_active = seq_finishing = seq_apply = 0;
    seq_one_notice = seq_done_notice = rollback_notice = seq_step = 0;
    rollback_request = 0;
    fast_due = slow_due = 0;
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

/* Keep the original control-start path free of parameter guards/latches.
   This hook is retained for the existing interrupt call site. */
void RouteA_ControlEnter(void) { }
float RouteA_GetBalanceScale(void) { return active_profile == RA_PROFILE_LOAD ? 2.0f : 1.0f; }
float RouteA_GetVelocityScale(void) { return active_profile == RA_PROFILE_LOAD ? 1.35f : 1.0f; }

static int ra_uint(const char *s, u32 *out)
{
    u32 v = 0;
    int count = 0, d;
    if(!s || !*s) return 0;
    while(*s)
    {
        d = *s++ - '0';
        if(d < 0 || d > 9 || ++count > 10 || v > 429496729UL || (v == 429496729UL && d > 5)) return 0;
        v = v * 10UL + (u32)d;
    }
    *out = v; return 1;
}
static int ra_decimal(const char *s, float *out)
{
    unsigned int whole = 0, frac = 0;
    int digits = 0, places = 0;
    if(!s || !*s) return 0;
    while(*s >= '0' && *s <= '9')
    { if(++digits > 3) return 0; whole = whole*10U + (unsigned int)(*s++ - '0'); }
    if(!digits) return 0;
    if(*s == '.')
    {
        s++;
        while(*s >= '0' && *s <= '9')
        { if(++places > 2) return 0; frac = frac*10U + (unsigned int)(*s++ - '0'); }
        if(!places) return 0;
    }
    if(*s) return 0;
    if(places == 1) frac *= 10U;
    *out = (float)whole + (float)frac*0.01f; return 1;
}
static int ra_tokenize(char *s)
{
    int n = 0;
    while(1)
    {
        if(n >= 8 || !*s) return 0;
        rx_fields[n++] = s;
        while(*s && *s != ',') s++;
        if(!*s) return n;
        *s++ = 0;
    }
}
static void ra_seq_ack(const char *name)
{
    snprintf(ack_body, sizeof(ack_body), "P3,SEQACK,%s,%lu,%u,%s,%.2f",
             name, trial_id, seq_step, seq_key, seq_value);
    ra_send(ack_body);
}
static void ra_status(u32 query_id, u32 query_step)
{
    RouteAPid pid;
    ra_read(&pid);
    if(!ra_pid_safe(&pid)) { ra_error("PID_DATA"); return; }
    snprintf(status_body, sizeof(status_body), "P3,STATUS,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,P%c,S%u,L%u,B%u,Q%lu,I%lu",
             pid.ap,pid.ad,pid.vp,pid.vi,pid.tp,pid.td,ra_profile_char(active_profile),
             state,training_locked,balance_started,query_id,query_step);
    ra_send(status_body);
}

void RouteA_HandleFrame(char *frame)
{
    char *command, *crc_text;
    int fields, index;
    u32 id = 0, ttl = 0, step = 0;
    u8 profile;
    float value;
    RouteAPid pid;
    if(!frame || strlen(frame) >= 80 || !ra_verify(frame)) { ra_error("CRC"); return; }
    if(!strncmp(frame+1,"P3,MSET,",8) || !strncmp(frame+1,"P3,PREP,",8) || !strncmp(frame+1,"P3,APPLY,",9))
    { ra_error("SEQ_REQUIRED");return; }
    crc_text = strstr(frame, ",C"); *crc_text = 0;
    fields = ra_tokenize(frame+1);
    if(fields < 2 || strcmp(rx_fields[0],"P3")) { ra_error("FORMAT"); return; }
    command = rx_fields[1];
    if(!strcmp(command,"GET"))
    { if(fields != 2) ra_error("FORMAT"); else ra_status(0,0); return; }
    if(!strcmp(command,"SGET"))
    {
        if(fields != 4 || !ra_uint(rx_fields[2],&id) || !ra_uint(rx_fields[3],&step) || step > 7 ||
           id != seq_last_id || (state != RA_IDLE && id != trial_id)) ra_error("QUERY");
        else ra_status(id,step);
        return;
    }
    if(!strcmp(command,"PROBE"))
    { if(fields != 2) ra_error("FORMAT"); else ra_ack("PROBE"); return; }
    if(!strcmp(command,"DIAG"))
    {
        if(fields!=3) { ra_error("FORMAT");return; }
        if(balance_started || training_locked || state!=RA_IDLE) { ra_error("DIAG_STATE");return; }
        diag_left=diag_right=diag_ticks=0;
        if(!strcmp(rx_fields[2],"L+"))diag_left=1500;
        else if(!strcmp(rx_fields[2],"L-"))diag_left=-1500;
        else if(!strcmp(rx_fields[2],"R+"))diag_right=1500;
        else if(!strcmp(rx_fields[2],"R-"))diag_right=-1500;
        else if(strcmp(rx_fields[2],"STOP")) { ra_error("FORMAT");return; }
        if(diag_left || diag_right)diag_ticks=120;else Set_Pwm(0,0);
        ra_ack(rx_fields[2]);return;
    }
    if(!strcmp(command,"MSET") || !strcmp(command,"PREP") || !strcmp(command,"APPLY"))
    { ra_error("SEQ_REQUIRED"); return; }
    if(!strcmp(command,"TRAIN"))
    {
        if(fields != 3) { ra_error("FORMAT"); return; }
        if(!strcmp(rx_fields[2],"START"))
        {
            if(!balance_started) { ra_error("NOTREADY"); return; }
            if(state != RA_IDLE || seq_active || rollback_notice) { ra_error("BUSY"); return; }
            ra_read(&pid); if(!ra_pid_safe(&pid)) { ra_error("PID_DATA"); return; }
            champion=pid;champion_profile=active_profile;training_locked=1;ra_ack("TRAIN_START");return;
        }
        if(!strcmp(rx_fields[2],"STOP"))
        {
            if(state != RA_IDLE) { unlock_after_rollback=1;rollback_request=1;g_newcarstate=enSTOP; }
            else training_locked=0;
            ra_ack("TRAIN_STOP");return;
        }
        ra_error("FORMAT");return;
    }
    if(!strcmp(command,"BSEQ"))
    {
        if(fields != 6 || !ra_uint(rx_fields[2],&id) || id==0 || id>999999UL ||
           !ra_uint(rx_fields[3],&ttl) || ttl<1000UL || ttl>RA_MAX_TTL_MS ||
           strlen(rx_fields[4])!=1 || (rx_fields[4][0]!='M' && rx_fields[4][0]!='T') ||
           strlen(rx_fields[5])!=1 || !ra_parse_profile(rx_fields[5][0],&profile))
        { ra_error("FORMAT");return; }
        if(seq_active || state!=RA_IDLE || rollback_notice) { ra_error("BUSY");return; }
        if(rx_fields[4][0]=='M' && training_locked) { ra_error("LOCKED");return; }
        if(rx_fields[4][0]=='T' && (!training_locked || !balance_started)) { ra_error("NOTRAIN");return; }
        ra_read(&pid);if(!ra_pid_safe(&pid)) { ra_error("PID_DATA");return; }
        champion=pid;champion_profile=active_profile;
        seq_manual=rx_fields[4][0]=='M';seq_target_profile=profile;
        seq_step=seq_finishing=seq_one_notice=seq_done_notice=0;
        trial_id=seq_last_id=id;deadline_ms=now_ms+ttl;heartbeat_ms=now_ms;
        seq_active=1;state=RA_PREPARED;ra_ack("SEQ_BEGIN");return;
    }
    if(fields < 3 || !ra_uint(rx_fields[2],&id) || id!=trial_id)
    { ra_error("TRIAL");return; }
    if(!strcmp(command,"ONE"))
    {
        if(!seq_active || seq_finishing || rollback_request || seq_apply || seq_one_notice) { ra_error("BUSY");return; }
        if(fields!=6 || !ra_uint(rx_fields[3],&step) || step<1 || step>6 || strlen(rx_fields[4])!=2 ||
           !ra_decimal(rx_fields[5],&value)) { ra_error("FORMAT");return; }
        if(step==seq_step && !strcmp(seq_key,rx_fields[4]) && ra_abs(seq_value-value)<0.0001f)
        { ra_seq_ack("ONE_APPLIED");return; }
        if(step!=(u32)seq_step+1UL) { ra_error("SEQUENCE");return; }
        ra_read(&pid);
        if(!strcmp(rx_fields[4],"AP")) { pid.ap=value;index=0; }
        else if(!strcmp(rx_fields[4],"AD")) { pid.ad=value;index=1; }
        else if(!strcmp(rx_fields[4],"VP")) { pid.vp=value;index=2; }
        else if(!strcmp(rx_fields[4],"VI")) { pid.vi=value;index=3; }
        else if(!strcmp(rx_fields[4],"TP")) { pid.tp=value;index=4; }
        else if(!strcmp(rx_fields[4],"TD")) { pid.td=value;index=5; }
        else { ra_error("KEY");return; }
        if(!ra_pid_safe(&pid)) { ra_error("RANGE");return; }
        seq_step=(u8)step;seq_index=(u8)index;seq_value=value;
        seq_raw=(index==0 || index==2 || index==4) ? value*100.0f : value;
        seq_key[0]=rx_fields[4][0];seq_key[1]=rx_fields[4][1];seq_key[2]=0;
        heartbeat_ms=now_ms;seq_apply=1;ra_seq_ack("ONE_QUEUED");return;
    }
    if(!strcmp(command,"FSEQ"))
    {
        if(fields!=3 || !seq_active || seq_finishing || seq_apply || seq_one_notice || rollback_request)
        { ra_error("BUSY");return; }
        seq_finishing=1;heartbeat_ms=now_ms;deadline_ms=now_ms+RA_MAX_TTL_MS;
        ra_ack("SEQ_FINISH_QUEUED");return;
    }
    if(!strcmp(command,"HB") && (state==RA_PREPARED || state==RA_TRIAL))
    { heartbeat_ms=now_ms;if(!seq_active)ra_ack("HB");return; }
    if(!strcmp(command,"ACCEPT") && state==RA_TRIAL && !seq_active && !rollback_request)
    {
        ra_read(&champion);champion_profile=active_profile;state=RA_IDLE;
        ra_ack("ACCEPT");trial_id=0;return;
    }
    if(!strcmp(command,"ROLLBACK") && state!=RA_IDLE)
    { rollback_request=1;ra_ack("ROLLBACK");return; }
    if(!strcmp(command,"STOP")) { g_newcarstate=enSTOP;ra_ack("STOP");return; }
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
        ra_write(&champion); active_profile=champion_profile;
        rollback_request=seq_apply=seq_finishing=seq_active=seq_one_notice=seq_done_notice=0;
        state=RA_IDLE;rollback_notice=1;
        angle_danger_count=saturation_danger_count=0;
        if(unlock_after_rollback) { training_locked=0;unlock_after_rollback=0; }
    }
    else if(seq_apply && seq_active)
    {
        switch(seq_index)
        {
            case 0: Balance_Kp=seq_raw;break;
            case 1: Balance_Kd=seq_raw;break;
            case 2: Velocity_Kp=seq_raw;break;
            case 3: Velocity_Ki=seq_raw;break;
            case 4: Turn_Kp=seq_raw;break;
            case 5: Turn_Kd=seq_raw;break;
            default: break;
        }
        seq_apply=0;seq_one_notice=1;state=RA_TRIAL;
    }
    else if(seq_finishing && seq_active && !seq_done_notice)
    {
        active_profile=seq_target_profile;
        seq_done_notice=1;state=RA_TRIAL;
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
    RouteAPid pid;
    if(rollback_notice)
    {
        rollback_notice=0;ra_ack("ROLLED_BACK");trial_id=0;
    }
    if(seq_one_notice && !rollback_request)
    { seq_one_notice=0;ra_seq_ack("ONE_APPLIED"); }
    if(seq_done_notice && !rollback_request)
    {
        seq_done_notice=seq_finishing=seq_active=0;
        if(seq_manual) { ra_read(&champion);champion_profile=active_profile;state=RA_IDLE; }
        ra_ack("SEQ_DONE");if(seq_manual)trial_id=0;
    }
    if(fast_due)
    {
        fast_due=0;
        snprintf(task_body,sizeof(task_body),"T3,F,%lu,A%.2f,G%.1f,O%.3f,N%.3f,EL%d,ER%d,ML%d,MR%d,S%u,L%u,P%c",
                 now_ms,Angle_Balance,Gyro_Balance,feature_osc,feature_lean,snap_el,snap_er,snap_ml,snap_mr,
                 state,training_locked,ra_profile_char(active_profile));ra_send(task_body);
    }
    if(slow_due)
    {
        slow_due=0;ra_read(&pid);
        if(!ra_pid_safe(&pid)) { ra_error("PID_DATA");return; }
        snprintf(task_body,sizeof(task_body),"T3,S,%lu,V%.2f,A2%.3f,G2%.3f,PK%.2f,X%.3f,AP%.2f,AD%.2f,VP%.2f,VI%.2f,TP%.2f,TD%.2f,P%c,D%u,B%u,%s",
                 now_ms,battery,snap_angle_sq_mean,snap_gyro_sq_mean,snap_peak,snap_saturation,
                 pid.ap,pid.ad,pid.vp,pid.vi,pid.tp,pid.td,ra_profile_char(active_profile),tx_dropped,balance_started,RA_FW_VERSION);
        ra_send(task_body);
    }
}
