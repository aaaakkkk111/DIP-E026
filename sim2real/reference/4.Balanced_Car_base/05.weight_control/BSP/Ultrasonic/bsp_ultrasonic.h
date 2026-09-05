#ifndef __BSP_ULTRASONIC_H_
#define __BSP_ULTRASONIC_H_

#include "AllHeader.h"

#define ULTRASONIC_RCC   RCC_APB2Periph_GPIOA

#define TRIG_PORT  GPIOA
#define TRIG_PIN   GPIO_Pin_0

#define ECHO_PORT  GPIOA
#define ECHO_PIN   GPIO_Pin_1

#define TRIG_SIG 	PAout(0)

void TIM2_Cap_Init(u16 arr,u16 psc);	
void Get_Distane(void);
void App_Change_Car(void);
void switch_wave_contorl(void);

void APP_avoid(void);

#endif

