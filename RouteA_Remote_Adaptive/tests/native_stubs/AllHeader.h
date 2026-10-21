#ifndef HOST_ALLHEADER_H
#define HOST_ALLHEADER_H
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "../../firmware/USER/myenum.h"
typedef unsigned char u8;
typedef unsigned short u16;
typedef uint32_t u32;
typedef int32_t s32;
extern Car_Mode mode;
extern int Mid_Angle;
extern float Angle_Balance,Gyro_Balance,Gyro_Turn,battery,Move_X,Move_Z;
extern u8 Stop_Flag;
extern enCarState g_newcarstate;
int UART5_TxEnqueue(const char*);
void Set_Pwm(int,int);
u8 Turn_Off(float,float);
u32 __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(u32);
void PID_ResetDynamicState(void);
int RouteA_ProfileIsLoad(void);
#endif
