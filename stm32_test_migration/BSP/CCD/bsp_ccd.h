#ifndef _BSP_CCD_H
#define _BSP_CCD_H

#include "AllHeader.h"

#define TSL_SI    PBout(5)   //SI  
#define TSL_CLK   PBout(4)   //CLK 

#define CCD_SI_CLK		RCC_APB2Periph_GPIOB
#define CCD_SI_PIN   	GPIO_Pin_5
#define CCD_SI_PORT		GPIOB

#define CCD_CLK_CLK		RCC_APB2Periph_GPIOB
#define CCD_CLK_PIN   GPIO_Pin_4
#define CCD_CLK_PORT	GPIOB

#define CCD_AO_CLK		RCC_APB2Periph_GPIOA
#define CCD_AO_PIN   	GPIO_Pin_4
#define CCD_AO_PORT		GPIOA

#define CCD_ADC				ADC2
#define CCD_ADC_CLK		RCC_APB2Periph_ADC2
#define CCD_ADC_CH 		ADC_Channel_4


u16 Get_Adc_CCD(u8 ch);
void Dly_us(void);
void RD_TSL(void); 
void ccd_Init(void);
void deal_data_ccd(void);

void  Find_CCD_Zhongzhi(void);
uint8_t* CCD_Get_ADC_128X32(void);
void OLED_Show_CCD_Image(uint8_t* p_img);

char binToHex_low(u8 num);
char binToHex_high(u8 num);
void slove_data(void);
void sendToPc(void);

uint8_t* CCD_Get_ADC_128X64(void);


#endif


