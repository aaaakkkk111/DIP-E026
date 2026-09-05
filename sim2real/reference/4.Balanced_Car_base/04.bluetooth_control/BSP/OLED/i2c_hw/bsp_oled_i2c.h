#ifndef __BSP_OLED_I2C_H_
#define __BSP_OLED_I2C_H_

#include "AllHeader.h"


#define OLED_ADDRESS 		0x78 //(0x3c<<1)
#define OLEDI2C_RCC  		RCC_APB1Periph_I2C1
#define OLEDI2C 				I2C1

#define OLED_RCC 				RCC_APB2Periph_GPIOB
#define OLED_Port 			GPIOB
#define OLED_SCL_Pin 		GPIO_Pin_8
#define OLED_SDA_Pin 		GPIO_Pin_9


void OLED_I2C_Init(void);



#endif

