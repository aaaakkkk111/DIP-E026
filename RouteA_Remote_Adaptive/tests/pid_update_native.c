#include "AllHeader.h"
#ifndef PID_SOURCE
#define PID_SOURCE "../firmware/APP/PID/pid_control.c"
#endif
#ifndef ROUTE_SOURCE
#define ROUTE_SOURCE "../firmware/APP/RouteA/route_a.c"
#endif
#include PID_SOURCE
#include ROUTE_SOURCE
int mode=Test_Mode, Mid_Angle=0;
float Angle_Balance=0,Gyro_Balance=0,Gyro_Turn=0,battery=12,Move_X=0,Move_Z=0;
float Velocity_Left=0,Velocity_Right=0,Acceleration_Z=0;
u32 g_distance=0,__stack_limit[1024];
u8 Stop_Flag=0;
enCarState g_newcarstate=enSTOP;
static char last_tx[1024];
static unsigned int primask=0;
static int last_left=0,last_right=0;
int UART5_TxEnqueue(const char *s) { strcpy(last_tx,s); return 1; }
void Set_Pwm(int left,int right) { last_left=left;last_right=right; }
u8 Turn_Off(float angle,float voltage) { return fabsf(angle)>40 || voltage<9.6f || Stop_Flag; }
u32 __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask=1; }
void __set_PRIMASK(u32 mask) { primask=(unsigned int)mask; }
#define CHECK(x) do { if(!(x)){fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);exit(1);} }while(0)
#define NEAR(x,y) CHECK(fabsf((x)-(y))<0.005f)
static void command_body(const char *body) {
 char f[192];sprintf(f,"$%s,C%04X#",body,ra_crc16(body,(int)strlen(body)));RouteA_HandleFrame(f);
}
#ifdef RA_ALL_MASK
static void legacy(const char *raw) { char f[192];strcpy(f,raw);RouteA_HandleLegacy(f); }
#endif
static void seed(void) {
 int i;for(i=0;i<1024;i++)__stack_limit[i]=0xA5A5A5A5UL;
 RouteA_Init();PID_ResetDynamicState();
 PID_Encoder_bias=20;PID_Encoder_Integral=1200;PID_Turn_Target=4;PID_Turn_PWM=37;
 Stop_Flag=0;balance_started=1;Angle_Balance=0;battery=12;g_newcarstate=enSTOP;
}
int main(void) {
 int expected,actual;float bias,integral;
 seed();expected=Velocity_PI(1,-2);bias=PID_Encoder_bias;integral=PID_Encoder_Integral;
 PID_Encoder_bias=20;PID_Encoder_Integral=1200;
 command_body("P3,MSET,N,96,48,62,31,17,20");RouteA_ControlBegin();actual=Velocity_PI(1,-2);
 printf("Identical PID: reference=%d, updated=%d PWM\n",expected,actual);
 CHECK(actual==expected);NEAR(PID_Encoder_bias,bias);NEAR(PID_Encoder_Integral,integral);CHECK(PID_Turn_PWM==37);
#ifdef RA_ALL_MASK
 seed();g_newcarstate=enRUN;Move_X=2;Mid_Angle=3;
 command_body("P3,MSET,N,97,49,0,0,0,0,M1");
 NEAR(Balance_Kp,9600);CHECK(manual_apply_request);
 command_body("P3,MSET,N,98,50,0,0,0,0,M1");CHECK(strstr(last_tx,"BUSY"));
 RouteA_ControlBegin();NEAR(Balance_Kp,9700);NEAR(Balance_Kd,49);
 NEAR(Velocity_Kp,6200);NEAR(Velocity_Ki,31);NEAR(Turn_Kp,1700);
 NEAR(PID_Encoder_Integral,1200);NEAR(PID_Encoder_bias,20);CHECK(PID_Turn_PWM==37);
 CHECK(g_newcarstate==enRUN && Move_X==2 && Mid_Angle==3 && Stop_Flag==0);
 CHECK(active_profile==0 && manual_applied_notice);RouteA_Task();CHECK(strstr(last_tx,"MSET_APPLIED"));
 Move_X=0;Mid_Angle=0;
 puts("PASS balance-only write: one-boundary commit, untouched loops/state/motion, BUSY before commit");

 command_body("P3,MSET,N,0,0,63,32,0,0,M2");RouteA_ControlBegin();
 NEAR(Balance_Kp,9700);NEAR(Velocity_Kp,6300);NEAR(Velocity_Ki,32);NEAR(PID_Encoder_Integral,1200);RouteA_Task();
 command_body("P3,MSET,N,0,0,0,0,18,21,M4");RouteA_ControlBegin();NEAR(Turn_Kp,1800);NEAR(Balance_Kd,49);RouteA_Task();
 puts("PASS velocity/turn group isolation and no Ki integral rebasing");

 seed();command_body("P3,MSET,L,96,75,70,35,14,20,M7");CHECK(strstr(last_tx,"PROFILE"));
 CHECK(!manual_apply_request);NEAR(RouteA_GetBalanceScale(),1);
 command_body("P3,PRESET,L");RouteA_ControlBegin();
 NEAR(Balance_Kp,9600);NEAR(Balance_Kd,75);NEAR(Velocity_Kp,7000);
 NEAR(RouteA_GetBalanceScale(),2);NEAR(RouteA_GetVelocityScale(),1.35f);NEAR(PID_Encoder_Integral,1200);
 mode=Weight_M;expected=Balance_PD(2.3f,114.0f);mode=Test_Mode;CHECK(Balance_PD(2.3f,114.0f)==expected);
 PID_Encoder_bias=20;PID_Encoder_Integral=1200;mode=Weight_M;expected=Velocity_PI(1,-2);
 PID_Encoder_bias=20;PID_Encoder_Integral=1200;mode=Test_Mode;CHECK(Velocity_PI(1,-2)==expected);
 puts("PASS only explicit PRESET changes profile; endpoint equals official Load");

 seed();command_body("P3,TRAIN,START");command_body("P3,PREP,42,30000,N,96,54,0,0,0,0,M1");
 NEAR(Balance_Kd,48);CHECK(state==RA_PREPARED);
 command_body("P3,ACCEPT,42");CHECK(strstr(last_tx,"STATE"));
 command_body("P3,APPLY,42");NEAR(Balance_Kd,48);RouteA_ControlBegin();
 CHECK(state==RA_TRIAL);NEAR(Balance_Kd,54);NEAR(PID_Encoder_Integral,1200);
 command_body("P3,ROLLBACK,42");RouteA_ControlBegin();CHECK(state==RA_IDLE);NEAR(Balance_Kd,48);
 NEAR(PID_Encoder_bias,20);NEAR(PID_Encoder_Integral,1200);
 puts("PASS LLM PREP/APPLY/ROLLBACK: direct values, no early application, no reset");

 seed();command_body("P3,TRAIN,START");command_body("P3,PPRESET,43,30000,L");command_body("P3,APPLY,43");RouteA_ControlBegin();
 CHECK(state==RA_TRIAL && active_profile==RA_PROFILE_LOAD);NEAR(RouteA_GetBalanceScale(),2);
 command_body("P3,ACCEPT,43");CHECK(state==RA_IDLE);NEAR(champion.ad,75);
 puts("PASS LLM explicit preset switches only on APPLY and saves confirmed profile");

 seed();command_body("P3,TRAIN,START");command_body("P3,PREP,44,1000,N,96,54,62,31,17,20");
 command_body("P3,APPLY,44");RouteA_ControlBegin();now_ms=2000;RouteA_Tick10ms();RouteA_ControlBegin();
 CHECK(state==RA_IDLE);NEAR(Balance_Kd,48);NEAR(PID_Encoder_Integral,1200);
 puts("PASS timeout rollback uses same direct setter and preserves PI state");

 seed();legacy("$0,0,0,0,1,0,0,AP97,AD49,VP0,VI0,TP0,TD0#");NEAR(Balance_Kp,9600);
 RouteA_ControlBegin();NEAR(Balance_Kp,9700);NEAR(Balance_Kd,49);NEAR(Velocity_Kp,6200);
 NEAR(PID_Encoder_Integral,1200);RouteA_Task();CHECK(!strcmp(last_tx,"$OK#"));
 legacy("$0,0,1,0,0,0,0,AP0,AD0,VP0,VI0,TP0,TD0#");CHECK(strstr(last_tx,"$AP97.00,AD49.00"));
 command_body("P3,TRAIN,START");legacy("$0,0,0,0,1,0,0,AP98,AD50,VP0,VI0,TP0,TD0#");CHECK(strstr(last_tx,"TrainingLocked"));
 NEAR(Balance_Kp,9700);
 puts("PASS official-compatible frame and P3 share direct writer; legacy query/lock preserved");

 seed();command_body("P3,PROBE");CHECK(strstr(last_tx,"ACK,PROBE"));CHECK(!manual_apply_request);
 command_body("P3,MSET,N,96,48,62,31,17,20,M0");CHECK(strstr(last_tx,"FORMAT"));
 command_body("P3,MSET,N,96,48,62,31,17,20,M7,tail");CHECK(strstr(last_tx,"FORMAT"));
 command_body("P3,MSET,N,NaN,48,62,31,17,20");CHECK(strstr(last_tx,"FORMAT"));
 command_body("P3,MSET,N,96e0,48,62,31,17,20");CHECK(strstr(last_tx,"FORMAT"));
 CHECK(ra_stack_free()==4096);__stack_limit[900]=0;CHECK(ra_stack_free()==3600);
 seed();Balance_Kp=-3.54142e22f;RouteA_ControlBegin();CHECK(memory_fault && Stop_Flag==1);
 Stop_Flag=0;RouteA_ControlEnd();CHECK(Stop_Flag==1 && last_left==0 && last_right==0);
 command_body("P3,MSET,N,96,48,62,31,17,20");CHECK(strstr(last_tx,"MEMORY_LATCH"));
 seed();command_body("P3,MSET,N,96,54,62,31,17,20");candidate.ad=-1;RouteA_ControlBegin();CHECK(memory_fault);NEAR(Balance_Kd,48);
 puts("PASS bounded frames, PROBE, stack watermark and corrupt live/pending PID locks");
#endif
 puts("PID direct-update native C regression tests: PASS");return 0;
}
