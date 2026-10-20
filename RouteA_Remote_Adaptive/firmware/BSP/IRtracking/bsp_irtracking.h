#ifndef _BSP_IRTRACKING_H_
#define _BSP_IRTRACKING_H_

#include "AllHeader.h"

//X1\2\3\4对应模块的旋钮开关   X1\2\3\4 corresponds to the knob switch of the module

#define IR_X1_Clk 	RCC_APB2Periph_GPIOC
#define IR_X1_Pin 	GPIO_Pin_4
#define IR_X1_Port 	GPIOC

#define IR_X2_Clk 	RCC_APB2Periph_GPIOC
#define IR_X2_Pin 	GPIO_Pin_5
#define IR_X2_Port 	GPIOC

#define IR_X3_Clk 	RCC_APB2Periph_GPIOB
#define IR_X3_Pin 	GPIO_Pin_0
#define IR_X3_Port 	GPIOB

#define IR_X4_Clk 	RCC_APB2Periph_GPIOB
#define IR_X4_Pin 	GPIO_Pin_1
#define IR_X4_Port 	GPIOB


//红外状态
#define IN_X1 GPIO_ReadInputDataBit(IR_X1_Port, IR_X1_Pin)
#define IN_X2 GPIO_ReadInputDataBit(IR_X2_Port, IR_X2_Pin)
#define IN_X3 GPIO_ReadInputDataBit(IR_X3_Port, IR_X3_Pin)
#define IN_X4 GPIO_ReadInputDataBit(IR_X4_Port, IR_X4_Pin)


void irtracking_init(void);

#endif

