/*4路循迹驱动文件 4-way tracking drive file */

#include "bsp_irtracking.h"

void irtracking_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(IR_X1_Clk|IR_X2_Clk|IR_X3_Clk|IR_X4_Clk, ENABLE); 
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin=IR_X1_Pin;
  GPIO_Init(IR_X1_Port,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin=IR_X2_Pin;
  GPIO_Init(IR_X2_Port,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin=IR_X3_Pin;
  GPIO_Init(IR_X3_Port,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin=IR_X4_Pin;
  GPIO_Init(IR_X4_Port,&GPIO_InitStructure);
	
	
	
}



