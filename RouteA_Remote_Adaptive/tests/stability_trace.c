#include "AllHeader.h"
#define enps2Fleft 100
#define enps2Fright 101
#define enps2Bleft 102
#define enps2Bright 103
#define enAvoid 104
#define enFollow 105
#ifndef TRACE_PID
#define TRACE_PID "../firmware/APP/PID/pid_control.c"
#endif
#include TRACE_PID
#ifndef TRACE_ROUTE
#define TRACE_ROUTE "../firmware/APP/RouteA/route_a.c"
#endif
#include TRACE_ROUTE
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

int main(void){
 int profile,c,i,b,v,t,ml,mr;
 const enCarState commands[]={enSTOP,enRUN,enBACK,enLEFT,enRIGHT,enTLEFT,enTRIGHT};
 const Car_Mode modes[]={Normal,Weight_M,Test_Mode};
 for(profile=0;profile<3;profile++){
  mode=modes[profile];RouteA_Init();
  if(mode==Weight_M){Balance_Kp=9600;Balance_Kd=75;Velocity_Kp=7000;Velocity_Ki=35;Turn_Kp=1400;Turn_Kd=20;}
  Car_Target_Velocity=30;Car_Turn_Amplitude_speed=36;
  for(c=0;c<7;c++){
   PID_ResetDynamicState();g_newcarstate=commands[c];
   for(i=0;i<200;i++){
    Angle_Balance=((i*13)%49-24)*0.13f;Gyro_Balance=((i*7)%151-75)*2.3f;
    battery=i==110?9.4f:12;Stop_Flag=i==130?1:0;
    if(mode==Test_Mode)RouteA_ControlBegin();
    b=Balance_PD(Angle_Balance,Gyro_Balance);v=Velocity_PI((i*3)%17-8,(i*5)%19-9);t=Turn_PD((i*11)%101-50);
    ml=b+v+t;mr=b+v-t;ml+=ml>0?1300:(ml<0?-1300:0);mr+=mr>0?1300:(mr<0?-1300:0);
    if(ml>2600)ml=2600;if(ml< -2600)ml=-2600;if(mr>2600)mr=2600;if(mr< -2600)mr=-2600;
    printf("%d,%d,%d,%d,%d,%d,%d,%d\n",profile,c,i,b,v,t,ml,mr);
   }
  }
 }
 return 0;
}
