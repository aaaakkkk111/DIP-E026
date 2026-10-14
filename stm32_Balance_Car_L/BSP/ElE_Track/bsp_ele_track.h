#ifndef _BSP_ELE_TRACK_H_
#define _BSP_ELE_TRACK_H_

#include "AllHeader.h"

#define ELE_L1_Clk 		RCC_APB2Periph_GPIOC
#define ELE_L1_Pin 		GPIO_Pin_0
#define ELE_L1_Port 	GPIOC

#define ELE_L2_Clk 		RCC_APB2Periph_GPIOC
#define ELE_L2_Pin 		GPIO_Pin_1
#define ELE_L2_Port 	GPIOC

#define ELE_L3_Clk 		RCC_APB2Periph_GPIOC
#define ELE_L3_Pin 		GPIO_Pin_2
#define ELE_L3_Port 	GPIOC

#define ELE_MID_Clk 	RCC_APB2Periph_GPIOC
#define ELE_MID_Pin 	GPIO_Pin_3
#define ELE_MID_Port 	GPIOC

#define ELE_R1_Clk 		RCC_APB2Periph_GPIOC
#define ELE_R1_Pin 		GPIO_Pin_4
#define ELE_R1_Port 	GPIOC

#define ELE_R2_Clk 		RCC_APB2Periph_GPIOC
#define ELE_R2_Pin 		GPIO_Pin_5
#define ELE_R2_Port 	GPIOC

#define ELE_R3_Clk 		RCC_APB2Periph_GPIOB
#define ELE_R3_Pin 		GPIO_Pin_0
#define ELE_R3_Port 	GPIOB

#define ELE_ADC				ADC2
#define ELE_ADC_CLK		RCC_APB2Periph_ADC2



#define ELE_L1_CH 		ADC_Channel_10  //PC0
#define ELE_L2_CH 		ADC_Channel_11  //PC1
#define ELE_L3_CH 		ADC_Channel_12  //PC2

#define ELE_M1_CH 		ADC_Channel_13  //PC3

#define ELE_R1_CH 		ADC_Channel_14  //PC4
#define ELE_R2_CH 		ADC_Channel_15  //PC5
#define ELE_R3_CH 		ADC_Channel_8  //PB0


extern int ele_seat ;//引出算法处理后的变量  Extract the variables processed by the algorithm


u16 Get_Adc_ele(u8 ch);
void  ele_Init(void);
void EleDataDeal(void);
void getEleData(void);
int guiyi_way(void);
int deal_getdata(int a);




#endif

