#ifndef __APP_TRACKING_H_
#define __APP_TRACKING_H_


#include "AllHeader.h"

void Car_tracking(void);

//PID

void Set_IRtrack_speed(void);
int Turn_IRTrack_PD(float gyro);
void PID_track_get(void);

void Set_Difftrack_speed(void);
void PID_Difftrack_get(void);
int Turn_DiffTrack_PD(float gyro);

#endif


