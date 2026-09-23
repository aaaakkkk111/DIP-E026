#include "route_a.h"
#define RA_IDLE 0
#define RA_PREPARED 1
#define RA_TRIAL 2
#define RA_FW_VERSION "RA1.5-TEST-MODE"
#define RA_HEARTBEAT_MS 2000UL
#define RA_MAX_TTL_MS 30000UL
extern float Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki,Turn_Kp,Turn_Kd;
extern enCarState g_newcarstate;
typedef struct{float ap,ad,vp,vi,tp,td;}RouteAPid;
static RouteAPid champion,candidate;
static volatile u8 state=RA_IDLE,apply_request=0,manual_apply_request=0,rollback_request=0,unlock_after_rollback=0;
static volatile u8 fast_due=0,slow_due=0,training_locked=0,balance_started=0;
static volatile u32 now_ms=0,deadline_ms=0,heartbeat_ms=0;
static volatile u16 tx_dropped=0;
static u32 trial_id=0;
static int snap_el,snap_er,snap_bp,snap_vp,snap_tp,snap_ml,snap_mr;
static volatile int diag_left=0,diag_right=0;
static volatile u16 diag_ticks=0;
static u16 snap_c1=0,snap_c2=0,snap_c3=0,snap_c4=0;
static u8 danger_count=0;
static float ra_abs(float v){return v<0.0f?-v:v;}
static int ra_numbers_ok(const RouteAPid*p){return p->ap==p->ap&&p->ad==p->ad&&p->vp==p->vp&&p->vi==p->vi&&p->tp==p->tp&&p->td==p->td;}
static u16 ra_crc16(const char*s,int len){u16 c=0xFFFF;int i,b;for(i=0;i<len;i++){c^=(u16)((u8)s[i])<<8;for(b=0;b<8;b++)c=(c&0x8000)?(u16)((c<<1)^0x1021):(u16)(c<<1);}return c;}
static int ra_hex(char c){if(c>='0'&&c<='9')return c-'0';if(c>='A'&&c<='F')return c-'A'+10;if(c>='a'&&c<='f')return c-'a'+10;return-1;}
static int ra_verify(char*f){char*p;int i,h,e=0;if(f[0]!='$')return 0;p=strstr(f,",C");if(!p||strlen(p)!=7||p[6]!='#')return 0;for(i=0;i<4;i++){h=ra_hex(p[2+i]);if(h<0)return 0;e=(e<<4)|h;}return ra_crc16(f+1,(int)(p-f-1))==(u16)e;}
static void ra_send(const char*b){char o[192];u16 c=ra_crc16(b,strlen(b));sprintf(o,"$%s,C%04X#",b,c);if(!UART5_TxEnqueue(o))tx_dropped++;}
static void ra_ack(const char*c){char b[72];sprintf(b,"P1,ACK,%s,%lu,%u,%u",c,trial_id,state,training_locked);ra_send(b);}
static void ra_err(const char*c){char b[72];sprintf(b,"P1,ERR,%s,%lu",c,trial_id);ra_send(b);}
static void ra_read(RouteAPid*p){p->ap=Balance_Kp/100.0f;p->ad=Balance_Kd;p->vp=Velocity_Kp/100.0f;p->vi=Velocity_Ki;p->tp=Turn_Kp/100.0f;p->td=Turn_Kd;}
static void ra_write(const RouteAPid*p){Balance_Kp=p->ap*100.0f;Balance_Kd=p->ad;Velocity_Kp=p->vp*100.0f;Velocity_Ki=p->vi;Turn_Kp=p->tp*100.0f;Turn_Kd=p->td;}
static int ra_eq(float a,float b){return ra_abs(a-b)<0.011f;}
static int ra_manual_safe(const RouteAPid*p){if(!ra_numbers_ok(p))return 0;return p->ap>=20&&p->ap<=288&&p->ad>=5&&p->ad<=200&&p->vp>=10&&p->vp<=72&&p->vi>=1&&p->vi<=100&&p->tp>=1&&p->tp<=100&&p->td>=1&&p->td<=100;}
static int ra_candidate_safe(const RouteAPid*p){float a,d;if(!ra_manual_safe(p))return 0;if(!ra_eq(p->vp,champion.vp)||!ra_eq(p->vi,champion.vi)||!ra_eq(p->tp,champion.tp)||!ra_eq(p->td,champion.td))return 0;a=champion.ap>.01f?ra_abs(p->ap-champion.ap)*100/champion.ap:100;d=champion.ad>.01f?ra_abs(p->ad-champion.ad)*100/champion.ad:100;return a<=5.01f&&d<=5.01f;}
void RouteA_Init(void){ra_read(&champion);candidate=champion;state=RA_IDLE;training_locked=0;balance_started=0;}
void RouteA_BalanceStarted(void){diag_ticks=0;diag_left=0;diag_right=0;Set_Pwm(0,0);balance_started=1;ra_send("P1,READY,1");}
int RouteA_ManualLocked(void){return training_locked?1:0;}
int RouteA_IsFrame(const u8*f){return f&&f[0]=='$'&&f[1]=='P'&&f[2]=='1'&&f[3]==',';}
void RouteA_HandleFrame(char*f){char*p,cmd[12],action[12];unsigned long id=0,ttl=0;RouteAPid x;int n;if(!ra_verify(f)){ra_err("CRC");return;}p=strstr(f,",C");*p=0;cmd[0]=0;if(sscanf(f,"$P1,%11[^,]",cmd)!=1){ra_err("FORMAT");return;}
if(!strcmp(cmd,"GET")){char b[128];ra_read(&x);sprintf(b,"P1,PID,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u,%u",x.ap,x.ad,x.vp,x.vi,x.tp,x.td,state,training_locked,balance_started);ra_send(b);return;}
if(!strcmp(cmd,"DIAG")){if(sscanf(f,"$P1,DIAG,%11[^,]",action)!=1){ra_err("FORMAT");return;}if(balance_started||training_locked||state!=RA_IDLE){ra_err("DIAG_STATE");return;}diag_left=0;diag_right=0;diag_ticks=0;if(!strcmp(action,"L+"))diag_left=1500;else if(!strcmp(action,"L-"))diag_left=-1500;else if(!strcmp(action,"R+"))diag_right=1500;else if(!strcmp(action,"R-"))diag_right=-1500;else if(strcmp(action,"STOP")){ra_err("FORMAT");return;}if(diag_left||diag_right)diag_ticks=120;else Set_Pwm(0,0);ra_ack(action);return;}
if(!strcmp(cmd,"MSET")){n=sscanf(f,"$P1,MSET,%f,%f,%f,%f,%f,%f",&x.ap,&x.ad,&x.vp,&x.vi,&x.tp,&x.td);if(n!=6){ra_err("FORMAT");return;}if(training_locked){ra_err("LOCKED");return;}if(state!=RA_IDLE){ra_err("BUSY");return;}if(!ra_manual_safe(&x)){ra_err("RANGE");return;}candidate=x;manual_apply_request=1;ra_ack("MSET");return;}
if(!strcmp(cmd,"TRAIN")){if(sscanf(f,"$P1,TRAIN,%11[^,]",action)!=1){ra_err("FORMAT");return;}if(!strcmp(action,"START")){if(!balance_started){ra_err("NOTREADY");return;}if(state!=RA_IDLE){ra_err("BUSY");return;}ra_read(&champion);training_locked=1;ra_ack("TRAIN_START");return;}if(!strcmp(action,"STOP")){if(state!=RA_IDLE){unlock_after_rollback=1;rollback_request=1;g_newcarstate=enSTOP;}else training_locked=0;ra_ack("TRAIN_STOP");return;}ra_err("FORMAT");return;}
if(!strcmp(cmd,"PREP")){if(!training_locked){ra_err("NOTRAIN");return;}n=sscanf(f,"$P1,PREP,%lu,%lu,%f,%f,%f,%f,%f,%f",&id,&ttl,&x.ap,&x.ad,&x.vp,&x.vi,&x.tp,&x.td);if(n!=8){ra_err("FORMAT");return;}if(state!=RA_IDLE){ra_err("BUSY");return;}if(ttl<1000||ttl>RA_MAX_TTL_MS){ra_err("TTL");return;}if(!ra_candidate_safe(&x)){ra_err("RANGE");return;}candidate=x;trial_id=id;deadline_ms=now_ms+ttl;heartbeat_ms=now_ms;state=RA_PREPARED;ra_ack("PREP");return;}
if(sscanf(f,"$P1,%*[^,],%lu",&id)!=1||id!=trial_id){ra_err("TRIAL");return;}if(!strcmp(cmd,"APPLY")&&state==RA_PREPARED){apply_request=1;heartbeat_ms=now_ms;ra_ack("APPLY");return;}if(!strcmp(cmd,"HB")&&(state==RA_PREPARED||state==RA_TRIAL)){heartbeat_ms=now_ms;ra_ack("HB");return;}if(!strcmp(cmd,"ACCEPT")&&state==RA_TRIAL){ra_read(&champion);state=RA_IDLE;ra_ack("ACCEPT");trial_id=0;return;}if(!strcmp(cmd,"ROLLBACK")&&state!=RA_IDLE){rollback_request=1;ra_ack("ROLLBACK");return;}if(!strcmp(cmd,"STOP")){g_newcarstate=enSTOP;ra_ack("STOP");return;}ra_err("STATE");}
void RouteA_Tick10ms(void){static u16 f=0,s=0;now_ms+=10;if(++f>=20){f=0;fast_due=1;}if(++s>=100){s=0;slow_due=1;}if(state!=RA_IDLE&&((s32)(now_ms-deadline_ms)>=0||(u32)(now_ms-heartbeat_ms)>RA_HEARTBEAT_MS)){rollback_request=1;g_newcarstate=enSTOP;}}
void RouteA_ControlBegin(void){if(rollback_request){ra_write(&champion);rollback_request=0;apply_request=0;state=RA_IDLE;trial_id=0;if(unlock_after_rollback){training_locked=0;unlock_after_rollback=0;}}else if(manual_apply_request&&!training_locked&&state==RA_IDLE){ra_write(&candidate);champion=candidate;manual_apply_request=0;}else if(apply_request&&state==RA_PREPARED){ra_write(&candidate);apply_request=0;state=RA_TRIAL;}}
void RouteA_ControlCapture(int el,int er,int bp,int vp,int tp,int ml,int mr){snap_el=el;snap_er=er;snap_bp=bp;snap_vp=vp;snap_tp=tp;snap_ml=ml;snap_mr=mr;if(state==RA_TRIAL){if(ra_abs(Angle_Balance)>25||battery<9.6f||ra_abs((float)ml)>=2595||ra_abs((float)mr)>=2595){if(++danger_count>=3){rollback_request=1;g_newcarstate=enSTOP;}}else danger_count=0;}}
void RouteA_ControlEnd(void){if(diag_ticks&&!balance_started&&Stop_Flag==1){Set_Pwm(diag_left,diag_right);diag_ticks--;if(diag_ticks==0){diag_left=0;diag_right=0;Set_Pwm(0,0);}}snap_c1=(u16)L_PWMA;snap_c2=(u16)L_PWMB;snap_c3=(u16)R_PWMA;snap_c4=(u16)R_PWMB;}
void RouteA_Task(void){char b[176];if(fast_due){fast_due=0;sprintf(b,"T1,F,%lu,A%.2f,G%.1f,EL%d,ER%d,ML%d,MR%d,S%u,L%u,R%u,C1%u,C2%u,C3%u,C4%u",now_ms,Angle_Balance,Gyro_Balance,snap_el,snap_er,snap_ml,snap_mr,state,training_locked,balance_started,snap_c1,snap_c2,snap_c3,snap_c4);ra_send(b);}if(slow_due){RouteAPid p;slow_due=0;ra_read(&p);sprintf(b,"T1,S,%lu,V%.2f,B%d,V%d,T%d,AP%.2f,AD%.2f,VP%.2f,VI%.2f,TP%.2f,TD%.2f,D%u,L%u,R%u,%s",now_ms,battery,snap_bp,snap_vp,snap_tp,p.ap,p.ad,p.vp,p.vi,p.tp,p.td,tx_dropped,training_locked,balance_started,RA_FW_VERSION);ra_send(b);}}
