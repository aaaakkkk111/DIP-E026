#ifndef HOST_ALLHEADER_H
#define HOST_ALLHEADER_H
#define RA_HOST_TEST 1
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned long u32;
typedef long s32;
typedef int enCarState;
enum { Normal=1, Weight_M=4, Test_Mode=21 };
enum { enSTOP=0, enRUN, enBACK, enLEFT, enRIGHT, enAvoid, enFollow, enps2Fleft, enps2Fright, enps2Bleft, enps2Bright, enTLEFT, enTRIGHT };
extern int mode, Mid_Angle;
extern float Angle_Balance, Gyro_Balance, Gyro_Turn, battery, Move_X, Move_Z;
extern u8 Stop_Flag;
extern float Velocity_Left,Velocity_Right,Acceleration_Z;
extern u32 g_distance;
extern enCarState g_newcarstate;
int UART5_TxEnqueue(const char *s);
void Set_Pwm(int left, int right);
u8 Turn_Off(float angle, float voltage);
u32 __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(u32 mask);
void PID_ResetDynamicState(void);
int RouteA_ProfileIsLoad(void);
float RouteA_GetBalanceScale(void);
float RouteA_GetVelocityScale(void);
#endif
