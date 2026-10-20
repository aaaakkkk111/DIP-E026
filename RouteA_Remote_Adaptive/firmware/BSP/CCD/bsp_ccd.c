#include "bsp_ccd.h"

char buf_CCD[20] = {'\0'};
u16 ADV[128]={0};
u8 CCD_Zhongzhi,CCD_Yuzhi;

/**************************************************************************
Function function: Linear CCD initialization
Entrance parameters: None
Return value: None
函数功能：线性CCD初始化
入口参数：无
返回  值：无
**************************************************************************/
/*
*PF5 -> CLK
*PF4 -> CS
*PF6 ->AO
*/

void  ccd_Init(void)
{    
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd( CCD_AO_CLK | CCD_ADC_CLK	, ENABLE );	  //使能ADC3通道时钟  // Enable ADC3 channel clock
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M  //Set ADC division factor 6 72M/6=12, ADC maximum time cannot exceed 14M
	
	//设置模拟通道输入引脚       Set analog channel input pins                   
	GPIO_InitStructure.GPIO_Pin = CCD_AO_PIN;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚 Analog input pins
	GPIO_Init(CCD_AO_PORT, &GPIO_InitStructure);	
	
	//初始化SI接口 Initialize SI interface
	RCC_APB2PeriphClockCmd( CCD_SI_CLK	, ENABLE );
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出 Push pull output
	GPIO_InitStructure.GPIO_Pin = CCD_SI_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(CCD_SI_PORT,&GPIO_InitStructure);
	
	//初始化CLK接口 Initialize CLK interface
	RCC_APB2PeriphClockCmd( CCD_CLK_CLK	, ENABLE );
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出 Push pull output
	GPIO_InitStructure.GPIO_Pin = CCD_CLK_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(CCD_CLK_PORT,&GPIO_InitStructure);
	
	ADC_DeInit(CCD_ADC);  //复位ADC2,将外设 ADC2 的全部寄存器重设为缺省值   // Reset ADC2 and reset all registers of the peripheral ADC2 to default values
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式    // ADC working mode: ADC1 and ADC2 work in independent mode
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式    // Analog to digital conversion works in single channel mode
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式   // Analog to digital conversion works in single conversion mode
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动   // The conversion is initiated by software rather than external triggers
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐   // Right aligned ADC data
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目   // The number of ADC channels for sequential rule conversion
	ADC_Init(CCD_ADC, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   // Initialize the registers of the peripheral ADCx based on the parameters specified in ADC_initStruct
	ADC_Cmd(CCD_ADC, ENABLE);	//使能指定的ADC1 Enable specified ADC1
	
	ADC_ResetCalibration(CCD_ADC);	//使能复位校准   Enable reset calibration	 
	while(ADC_GetResetCalibrationStatus(CCD_ADC));	//等待复位校准结束	Waiting for reset calibration to end
	ADC_StartCalibration(CCD_ADC);	 //开启AD校准  Enable AD calibration
	while(ADC_GetCalibrationStatus(CCD_ADC));	 //等待校准结束 Waiting for calibration to end

}	
/**************************************************************************
Function: The AD sampling
Input   : The ADC channels
Output  : AD conversion results
函数功能：AD采样
入口参数：ADC的通道
返回  值：AD转换结果
**************************************************************************/
u16 Get_Adc_CCD(u8 ch)   
{
	//Sets the specified ADC rule group channel, one sequence, and sampling time
	//设置指定ADC的规则组通道，一个序列，采样时间
	
	//ADC1,ADC通道,采样时间为480周期 ADC1, ADC channel, sampling time is 480 cycles
	ADC_RegularChannelConfig(CCD_ADC, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道,采样时间为239.5周期	   ADC1, ADC channel, sampling time is 239.5 cycles   
  //Enable the specified ADC1 software transformation startup function	
  //使能指定的ADC1的软件转换启动功能	
	ADC_SoftwareStartConvCmd(CCD_ADC, ENABLE);			 
	//Wait for the conversion to finish
  //等待转换结束	
	while(!ADC_GetFlagStatus(CCD_ADC, ADC_FLAG_EOC ));
	//Returns the result of the last ADC1 rule group conversion
	//返回最近一次ADC1规则组的转换结果
	return ADC_GetConversionValue(CCD_ADC);	
}

/**************************************************************************
Function Function: Delay
Entrance parameters: None
Return value: None
函数功能：延时
入口参数：无
返回  值：无
**************************************************************************/
void Dly_us(void)
{
   int ii;    
   for(ii=0;ii<10;ii++); 
}

/**************************************************************************
Function function: CCD data acquisition
Entrance parameters: None
Return value: None
函数功能：CCD数据采集
入口参数：无
返回  值：无
**************************************************************************/
 void RD_TSL(void) 
{
  u8 i=0,tslp=0;
  TSL_CLK=1;
  TSL_SI=0; 
  Dly_us();
      
  TSL_SI=1; 
  TSL_CLK=0;
  Dly_us();
	
	
  TSL_CLK=1;
  TSL_SI=0;
  Dly_us(); 
  for(i=0;i<128;i++)					//读取128个像素点电压值 Read 128 pixel voltage values
  { 
    TSL_CLK=0; 
    Dly_us();  //调节曝光时间 Adjust exposure time
		Dly_us();
		
    ADV[tslp]=(Get_Adc_CCD(CCD_ADC_CH))>>4; 
    ++tslp;
    TSL_CLK=1;
    Dly_us();	
		


  }  
}

//开始CCD采集并处理输出数据
//Start CCD collection and processing of output data
void deal_data_ccd(void)
{
//		RD_TSL();  //获取图像时已经获取过了 Already acquired when acquiring the image
		Find_CCD_Zhongzhi();	 
}

/**************************************************************************
Function function: Linear CCD takes the median value
Entrance parameters: None
Return value: None
函数功能：线性CCD取中值
入口参数：无
返回  值：无
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{ 
	 static u16 i,j,Left,Right;
	 static u16 value1_max,value1_min;
	
	   value1_max=ADV[0];  //动态阈值算法，读取最大和最小值 Dynamic threshold algorithm, reading maximum and minimum values
     for(i=5;i<123;i++)   //两边各去掉5个点 Remove 5 points on each side
     {
        if(value1_max<=ADV[i])
        value1_max=ADV[i];
     }
	   value1_min=ADV[0];  //最小值 minimum value
     for(i=5;i<123;i++) 
     {
        if(value1_min>=ADV[i])
        value1_min=ADV[i];
     }
   CCD_Yuzhi=(value1_max+value1_min)/2;	  //计算出本次中线提取的阈值 Calculate the threshold for extracting the centerline in this round
	 for(i = 5;i<118; i++)   //寻找左边跳变沿 Find the left jump edge
	{
		if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		{	
			Left=i;
			break;	
		}
	}
	 for(j = 118;j>5; j--)//寻找右边跳变沿 Find the right jump edge
  {
		if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
		{	
		  Right=j;
		  break;	
		}
  }
	CCD_Zhongzhi=(Right+Left)/2;//计算中线位置 Calculate the centerline position
	
	
	
//	printf("zhong: %d\r\n",CCD_Yuzhi);
//	printf("middle : %d\r\n",CCD_Zhongzhi);
	
//	sprintf(buf_CCD,"Yuzhi:%d ",CCD_Yuzhi);
//	OLED_Draw_Line(buf_CCD, 2 , false, true);
//	sprintf(buf_CCD,"Zhongzhi:%d ",CCD_Zhongzhi);
//	OLED_Draw_Line(buf_CCD, 3 , false, true);
//	
//	memset(buf_CCD,0,sizeof(buf_CCD));
	
	//根据实际情况  According to the actual situation
//	if(math_abs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //计算中线的偏差，如果太大 Calculate the deviation from the midline, if it is too large
//	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //则取上一次的值 Then take the last value
//	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //保存上一次的偏差 Save the last deviation
	
}


uint8_t ADC_128X32[128] = {0};
// 返回128个像素点的ADV采集电压值，并将幅值压缩成128*32。
// Return the ADV collected voltage value of 128 pixels and compress the amplitude into 128*32.
uint8_t* CCD_Get_ADC_128X32(void)
{
		RD_TSL();
    // 将8位AD值转化成5位AD值
		// Convert 8-bit AD value to 5-bit AD value
    for (int i = 0; i < 128; i++)
    {
        ADC_128X32[i] = ADV[i] >> 3;
    }
    return ADC_128X32;
}

void OLED_Show_CCD_Image(uint8_t* p_img)
{
    OLED_Clear();
    for (int i = 0; i < 128; i++)
    {
        if (p_img[i] < 32)
        {
            SSD1306_DrawPixel(i, p_img[i], SSD1306_COLOR_WHITE);
        }
    }
    OLED_Refresh();
}
