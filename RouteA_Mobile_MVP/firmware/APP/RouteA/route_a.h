#ifndef __ROUTE_A_H
#define __ROUTE_A_H
#include "AllHeader.h"
void RouteA_Init(void);
void RouteA_BalanceStarted(void);
int RouteA_ManualLocked(void);
int RouteA_IsFrame(const u8 *frame);
void RouteA_HandleFrame(char *frame);
void RouteA_Tick10ms(void);
void RouteA_ControlBegin(void);
void RouteA_ControlCapture(int,int,int,int,int,int,int);
void RouteA_ControlEnd(void);
void RouteA_Task(void);
#endif
