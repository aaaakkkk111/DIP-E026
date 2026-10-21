#include "AllHeader.h"
#include "../firmware/APP/PID/pid_control.c"
#include "../firmware/APP/RouteA/route_a.c"
Car_Mode mode=Test_Mode;
int Mid_Angle=0;
float Angle_Balance=0,Gyro_Balance=0,Gyro_Turn=0,battery=12,Move_X=0,Move_Z=0;
u8 Stop_Flag=0;
enCarState g_newcarstate=enSTOP;
static char tx[8192];static int pwm_calls=0;
int UART5_TxEnqueue(const char*s){strncat(tx,s,sizeof(tx)-strlen(tx)-1);return 1;}
void Set_Pwm(int l,int r){(void)l;(void)r;pwm_calls++;}
u8 Turn_Off(float a,float v){return fabsf(a)>40||v<9.6f||Stop_Flag;}
u32 __get_PRIMASK(void){return 0;}
void __disable_irq(void){}
void __set_PRIMASK(u32 n){(void)n;}
#define CHECK(x) do{if(!(x)){fprintf(stderr,"FAIL line%d: %s\n",__LINE__,#x);exit(1);}}while(0)
#define NEAR(a,b) CHECK(fabsf((a)-(b))<0.005f)
static void cmd(const char*b){char f[160];snprintf(f,sizeof(f),"$%s,C%04X#",b,ra_crc16(b,(int)strlen(b)));RouteA_HandleFrame(f);}
static void flush(void){tx[0]=0;}
static void reset(void){RouteA_Init();PID_ResetDynamicState();PID_Encoder_bias=20;PID_Encoder_Integral=1200;PID_Turn_PWM=37;balance_started=1;Stop_Flag=0;pwm_calls=0;g_newcarstate=enRUN;flush();}
int main(void){
 float integral,bias;int i;
 reset();for(i=0;i<20000;i++){RouteA_ControlEnter();RouteA_ControlBegin();}CHECK(pwm_calls==0&&Stop_Flag==0);NEAR(Balance_Kp,9600);NEAR(PID_Encoder_Integral,1200);
 cmd("P3,BSEQ,101,30000,M,N");CHECK(strstr(tx,"ACK,SEQ_BEGIN,101")&&state==RA_PREPARED);
 cmd("P3,ONE,101,1,AP,95.00");NEAR(Balance_Kp,9600);CHECK(seq_apply);CHECK(strstr(tx,"ONE_QUEUED"));
 integral=PID_Encoder_Integral;bias=PID_Encoder_bias;RouteA_ControlBegin();
 NEAR(Balance_Kp,9500);NEAR(Balance_Kd,48);NEAR(Velocity_Kp,6200);NEAR(Velocity_Ki,31);NEAR(Turn_Kp,1700);NEAR(Turn_Kd,20);
 NEAR(PID_Encoder_Integral,integral);NEAR(PID_Encoder_bias,bias);NEAR(PID_Turn_PWM,37);CHECK(g_newcarstate==enRUN&&pwm_calls==0);
 RouteA_Task();CHECK(strstr(tx,"ONE_APPLIED,101,1,AP,95.00"));
 cmd("P3,SGET,101,1");CHECK(strstr(tx,",Q101,I1,"));
 flush();cmd("P3,ONE,101,1,AP,95.00");CHECK(strstr(tx,"ONE_APPLIED")&&!seq_apply);
 cmd("P3,ONE,101,3,TP,18.00");CHECK(strstr(tx,"ERR,SEQUENCE"));NEAR(Turn_Kp,1700);
 cmd("P3,ONE,101,2,TP,18.00");RouteA_ControlBegin();RouteA_Task();NEAR(Turn_Kp,1800);NEAR(Balance_Kp,9500);
 cmd("P3,FSEQ,101");RouteA_ControlBegin();RouteA_Task();CHECK(state==RA_IDLE&&!seq_active);NEAR(champion.ap,95);NEAR(champion.tp,18);NEAR(PID_Encoder_Integral,integral);
 puts("PASS manual: one key per command, one boundary commit, exact scaling, independent fields, unchanged controller state and correlated readback");
 reset();cmd("P3,TRAIN,START");cmd("P3,BSEQ,102,30000,T,L");cmd("P3,ONE,102,1,AD,75.00");RouteA_ControlBegin();RouteA_Task();NEAR(Balance_Kd,75);CHECK(!RouteA_ProfileIsLoad());
 cmd("P3,ACCEPT,102");CHECK(state==RA_TRIAL&&seq_active);
 cmd("P3,FSEQ,102");RouteA_ControlBegin();RouteA_Task();CHECK(RouteA_ProfileIsLoad()&&state==RA_TRIAL&&!seq_active);
 cmd("P3,ROLLBACK,102");RouteA_ControlBegin();RouteA_Task();NEAR(Balance_Kd,48);CHECK(!RouteA_ProfileIsLoad()&&state==RA_IDLE);NEAR(PID_Encoder_Integral,1200);
 reset();cmd("P3,TRAIN,START");cmd("P3,BSEQ,103,30000,T,N");cmd("P3,ONE,103,1,AD,49.00");RouteA_ControlBegin();RouteA_Task();cmd("P3,FSEQ,103");RouteA_ControlBegin();RouteA_Task();cmd("P3,ACCEPT,103");NEAR(champion.ad,49);CHECK(state==RA_IDLE);
 puts("PASS training: profile changes only at FSEQ, no partial ACCEPT, full pre-sequence rollback, accepted champion");
 reset();cmd("P3,BSEQ,104,30000,M,N");cmd("P3,ONE,104,1,AP,95.00");RouteA_ControlBegin();RouteA_Task();for(i=0;i<202;i++)RouteA_Tick10ms();CHECK(rollback_request);RouteA_ControlBegin();RouteA_Task();NEAR(Balance_Kp,9600);CHECK(state==RA_IDLE&&!seq_active);NEAR(PID_Encoder_Integral,1200);
 reset();cmd("P3,BSEQ,105,30000,M,N");
 cmd("P3,ONE,105,1,AP,nan");CHECK(!seq_apply);cmd("P3,ONE,105,1,AP,1e38");CHECK(!seq_apply);cmd("P3,ONE,105,1,AP,999.00");CHECK(!seq_apply);cmd("P3,ONE,105,1,ZZ,95.00");CHECK(!seq_apply);
 flush();cmd("P3,MSET,N,96,48,62,31,17,20");CHECK(strstr(tx,"SEQ_REQUIRED"));
 reset();cmd("P3,BSEQ,106,30000,M,N");
 { const char*keys[]={"AP","AD","VP","VI","TP","TD"};const char*vals[]={"95.00","49.00","63.00","32.00","18.00","21.00"};char body[80];
 for(i=0;i<6;i++){snprintf(body,sizeof(body),"P3,ONE,106,%d,%s,%s",i+1,keys[i],vals[i]);cmd(body);RouteA_ControlBegin();RouteA_Task();CHECK(seq_step==i+1);}
 }
 cmd("P3,FSEQ,106");RouteA_ControlBegin();RouteA_Task();NEAR(Balance_Kp,9500);NEAR(Balance_Kd,49);NEAR(Velocity_Kp,6300);NEAR(Velocity_Ki,32);NEAR(Turn_Kp,1800);NEAR(Turn_Kd,21);CHECK(state==RA_IDLE);
 puts("PASS timeout/validation: partial sequence rolls back, invalid decimals/key/range rejected, legacy six-value writes refused");
 return 0;
}
