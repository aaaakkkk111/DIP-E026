#include "bsp_ele_track.h"

int Sensor_Left_1,Sensor_Left_2,Sensor_Left_3;
int Sensor_Right_1,Sensor_Right_2,Sensor_Right_3;
int Sensor_Middle;
int ele_seat = 0;

/**************************************************************************
Function function: Electromagnetic sensor sampling initialization
Entrance parameters: None
Return value: None
函数功能：电磁传感器采样初始化
入口参数：无
返回  值：无
**************************************************************************/

//PC0-3  pC4-5 pB0-1 做电磁传感器采集  pC4-5 pB0-1这几个接收右边,和红外巡线传感器引脚重定义
//PC0-3 pC4-5 pB0-1 for electromagnetic sensor collection pC4-5 pB0-1 these receive the right side and infrared line patrol sensor pin redefine
void  ele_Init(void)
{    
 	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB | ELE_ADC_CLK	, ENABLE );	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M //Set the ADC division factor to 6 72M/6=12, and the maximum ADC time cannot exceed 14M
	//设置模拟通道输入引脚    //Set analog channel input pins
	
	
	
	//左边 ADC 10,11,12   Left ADC 10,11,12
	GPIO_InitStructure.GPIO_Pin = ELE_L1_Pin | ELE_L2_Pin |ELE_L3_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		//模拟输入引脚  悬空不为0，可以改为下拉 //The analog input pin is not suspended at 0 and can be changed to pull-down
	GPIO_Init(ELE_L1_Port, &GPIO_InitStructure);			//都是GPIOC，选一个就好 //All are GPOC, just choose one
	
	
	//中间 ADC 13   Middle ADC 13
	GPIO_InitStructure.GPIO_Pin = ELE_MID_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		//模拟输入引脚  悬空不为0，可以改为下拉 //The analog input pin is not suspended at 0 and can be changed to pull-down
	GPIO_Init(ELE_MID_Port, &GPIO_InitStructure);		
	
	//右边 ADC 14,15   Right ADC 14,15
	GPIO_InitStructure.GPIO_Pin = ELE_R1_Pin |ELE_R2_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		//模拟输入引脚  悬空不为0，可以改为下拉 //The analog input pin is not suspended at 0 and can be changed to pull-down
	GPIO_Init(ELE_R1_Port, &GPIO_InitStructure);	 //都是GPIOC，选一个就好  //All are GPOC, just choose one
	
	//右边 ADC 8  Right ADC 8
	GPIO_InitStructure.GPIO_Pin = ELE_R3_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		//模拟输入引脚  悬空不为0，可以改为下拉  //The analog input pin is not suspended at 0 and can be changed to pull-down
	GPIO_Init(ELE_R3_Port, &GPIO_InitStructure);	 
	
	
	ADC_DeInit(ELE_ADC);  //复位ADC,将外设 ADC 的全部寄存器重设为缺省值  //Reset ADC, reset all registers of the peripheral ADC to default values
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式  //ADC working mode: ADC1 and ADC2 work in independent mode
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式  //Analog to digital conversion works in single channel mode
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式  //Analog to digital conversion works in single conversion mode
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动  //The conversion is initiated by software rather than external triggers
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐 //Right aligned ADC data
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目  //The number of ADC channels for sequential rule conversion
	ADC_Init(ELE_ADC, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器    //Initialize the registers of the peripheral ADCx based on the parameters specified in ADC_initStruct
	ADC_Cmd(ELE_ADC, ENABLE);	
	ADC_ResetCalibration(ELE_ADC);	//使能复位校准   //Enable reset calibration	 
	while(ADC_GetResetCalibrationStatus(ELE_ADC));	//等待复位校准结束	 //Waiting for reset calibration to end
	ADC_StartCalibration(ELE_ADC);	 //开启AD校准  //Enable AD calibration
	while(ADC_GetCalibrationStatus(ELE_ADC));	 //等待校准结束 //Waiting for calibration to end
}		

/**************************************************************************
Function Function: AD Sampling
Entry parameter: Channel of ADC1
Return value: AD conversion result
函数功能：AD采样
入口参数：ADC1 的通道
返回  值：AD转换结果
**************************************************************************/
u16 Get_Adc_ele(u8 ch)   
{
	  	//设置指定ADC的规则组通道，一个序列，采样时间 //Set the rule group channel for the specified ADC, a sequence, and sampling time
	ADC_RegularChannelConfig(ELE_ADC, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道,采样时间为239.5周期	 //ADC1, ADC channel, sampling time is 239.5 cycles  			     
	ADC_SoftwareStartConvCmd(ELE_ADC, ENABLE);		//使能指定的ADC1的软件转换启动功能	 //Enable the software conversion startup function of the specified ADC1	 
	while(!ADC_GetFlagStatus(ELE_ADC, ADC_FLAG_EOC ));//等待转换结束 //Waiting for the conversion to end
	return ADC_GetConversionValue(ELE_ADC);	//返回最近一次ADC1规则组的转换结果 //Return the latest conversion result of ADC1 rule group
}

//得到的数据做归一算法 //Use the obtained data as a normalization algorithm
int guiyi_way(void)
{
	int sum , Sensor;
	int Sensor_Left,Sensor_Right;
	
	//归一化处理  Normalization
//	sum=(Sensor_Left_1*1+Sensor_Left_3*100) 
//			+ Sensor_Middle *200 
//			+(Sensor_Right_1*300+Sensor_Right_3*399);  
//	Sensor_Left =  Sensor_Left_1+Sensor_Left_3;
//	Sensor_Right = Sensor_Right_1+Sensor_Right_3;
	
	sum=(Sensor_Left_3*1) 
			+ Sensor_Middle *100 
			+(Sensor_Right_1*199);  
	
		Sensor_Left =  Sensor_Left_3; //+ Sensor_Left_1;
  	Sensor_Right = Sensor_Right_1;// + Sensor_Right_3;
	
	Sensor=sum/(Sensor_Left+Sensor_Middle+Sensor_Right);   //求偏差 //Request deviation
	return Sensor;//返回目前的在磁场的位置  //Return to the current position in the magnetic field
}

//获取传感器数据  //Obtain sensor data
void getEleData(void)
{
	//小车屁股对着自己，从左到右数 //The car's butt is facing itself, counting from left to right
	
	
	Sensor_Left_1=Get_Adc_ele(ELE_L1_CH)>>4;                //采集左边电感的数据  Collect data from the left inductor
	Sensor_Left_3=Get_Adc_ele(ELE_L3_CH)>>4;                //采集左边电感的数据  Collect data from the left inductor
	
	Sensor_Middle=Get_Adc_ele(ELE_M1_CH)>>4;              //采集中间电感的数据  Collect data on the intermediate inductance
	
	Sensor_Right_1=Get_Adc_ele(ELE_R1_CH)>>4;               //采集右边电感的数据  Collect data from the right inductor
	Sensor_Right_3=Get_Adc_ele(ELE_R3_CH)>>4;               //采集右边电感的数据  Collect data from the right inductor

	//因为放大器不稳，滤波一下，正常是不会，巡线只用此3个
	//Because the amplifier is unstable, filtering it will not work normally. Only these 3 are used for line inspection
	Sensor_Left_3 = deal_getdata(Sensor_Left_3);
	Sensor_Right_1 = deal_getdata(Sensor_Right_1);
	Sensor_Middle = deal_getdata(Sensor_Middle);
	
	ele_seat = guiyi_way();
	
}


int deal_getdata(int a)
{
	if(a<=10)
	{
		return 0;
	}
	else
		return a;
}

//数据显示在屏幕上
//Data is displayed on the screen
void EleDataDeal(void)
{
	
	char ele_data[30]={'\0'};
	//getEleData();中断调用了  The interrupt call

//	sprintf(ele_data,"ele_seat:%d     ",ele_seat);
//	OLED_Draw_Line(ele_data, 1 , false, true);
	//	sprintf(ele_data,"MID:%d        ",Sensor_Middle);
//	OLED_Draw_Line(ele_data, 2 , false, true);
	
	sprintf(ele_data,"seat:%d  MID:%d   ",ele_seat,Sensor_Middle);
	OLED_Draw_Line(ele_data, 1 , false, true);
		
	sprintf(ele_data,"L1:%d  L3:%d     ",Sensor_Left_1,Sensor_Left_3);
	OLED_Draw_Line(ele_data, 2 , false, true);
		
	sprintf(ele_data,"R1:%d  R3:%d     ",Sensor_Right_1,Sensor_Right_3);
	OLED_Draw_Line(ele_data, 3 , false, true);
	

}



