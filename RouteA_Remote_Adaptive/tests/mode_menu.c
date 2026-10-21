#include "AllHeader.h"
#include <stdbool.h>
int Mid_Angle;
float Car_Target_Velocity,Car_Turn_Amplitude_speed,Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki,Turn_Kp,Turn_Kd;
static char display[40];
int myabs(int n){return n<0?-n:n;}
int Key1_State(int n){(void)n;return 1;}
int Read_Encoder(Motor_ID n){(void)n;return 0;}
void OLED_Draw_Line(const char*s,int row,bool a,bool b){(void)row;(void)a;(void)b;strcpy(display,s);}
void show_mode_oled(void);
void Set_Mid_Angle(void);
void Set_angle(void);
void Set_control_speed(void);
void Set_PID(void);
#include "../firmware/APP/mode/app_mode.c"
#include "../firmware/APP/OLED_Show/oled_show.c"
#define CHECK(x) do{if(!(x)){fprintf(stderr,"Menu FAIL %d\n",__LINE__);return 1;}}while(0)
int main(void){
 const Car_Mode order[]={Normal,Weight_M,Test_Mode};int i;
 CHECK(Normal==0&&Weight_M==3&&Test_Mode==20&&SUPPORTED_MODE_COUNT==3);
 for(i=1;i<=6;i++){car_mode((int16_t)(251*i));CHECK(mode==order[i%3]);}
 for(i=5;i>=0;i--){car_mode((int16_t)(251*i));CHECK(mode==order[i%3]);}
 car_mode(-251);CHECK(mode==Test_Mode);
 mode=Normal;show_mode_oled();CHECK(!strcmp(display,"1.Standard Mode"));Set_PID();CHECK(Balance_Kd==48&&Velocity_Kp==6200&&Turn_Kp==1700);
 mode=Weight_M;show_mode_oled();CHECK(!strcmp(display,"2.Load Mode"));Set_PID();CHECK(Balance_Kp==9600&&Balance_Kd==75&&Velocity_Kp==7000&&Velocity_Ki==35&&Turn_Kp==1400&&Turn_Kd==20);
 mode=Test_Mode;show_mode_oled();CHECK(!strcmp(display,"3.TEST MODE"));Set_PID();Set_Mid_Angle();Set_angle();Set_control_speed();CHECK(Balance_Kd==48&&Mid_Angle==0&&angle_max==40&&Car_Target_Velocity==30&&Car_Turn_Amplitude_speed==36);
 puts("PASS physical menu1/2/3 forward/reverse, original mode IDs0/3/20, Standard/Load/TEST startup PID");return 0;
}
