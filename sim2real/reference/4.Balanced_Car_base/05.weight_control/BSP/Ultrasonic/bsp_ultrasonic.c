#include "bsp_ultrasonic.h"

u32 g_distance = 0;//超声波距离  Ultrasonic distance

static void ultrasonic_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(ULTRASONIC_RCC, ENABLE);  //使能GPIOA时钟  Enable GPIOA clock
	
	GPIO_InitStructure.GPIO_Pin  = ECHO_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA1 输入  enter
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(ECHO_PORT, &GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Pin  = TRIG_PIN;     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //PA3输出   PA3 Output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(TRIG_PORT, &GPIO_InitStructure);

}


/**************************************************************************
Function: Timer 2 channel 2 input capture initialization
Input parameters: arr: auto-reload value; psc: clock pre-division number
Return value: None
函数功能：定时器2通道2输入捕获初始化
入口参数: arr：自动重装值； psc：时钟预分频数 
返回  值：无
**************************************************************************/
void TIM2_Cap_Init(u16 arr,u16 psc)	
{
	TIM_ICInitTypeDef  TIM2_ICInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	ultrasonic_init();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	//使能TIM2时钟 Enable TIM2 clock
	
	//初始化定时器2 TIM2	  Initialize timer 2 TIM2
	TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值  Set the counter auto-reload value
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 	//预分频器   Prescaler
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim  Set clock division: TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式  TIM up counting mode
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位  Initializes the time base unit of TIMx according to the parameters specified in TIM_TimeBaseInitStruct
	
	//初始化TIM2输入捕获参数  Initialize TIM2 input capture parameters
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=02 	选择输入端 IC2映射到TI1上  Select input IC2 to map to TI1
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获 Rising edge capture
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频  Configure input frequency division, no frequency division
	TIM2_ICInitStructure.TIM_ICFilter = 0x00;//配置输入滤波器 不滤波  Configure input filter No filtering
	TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
	//中断分组初始化  Interrupt group initialization
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2中断  //TIM2 interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级1级  Preemption priority level 1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级1级 From priority level 1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能  IRQ channel is enabled
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 	 Initializes peripheral NVIC registers according to the parameters specified in NVIC_InitStruct
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC2,ENABLE);//允许更新中断 ,允许CC2IE捕获中断	 Allow update interrupt, allow CC2IE to capture interrupt
  TIM_Cmd(TIM2,ENABLE ); 	//使能定时器2 Enable timer 2

}


//获取超声波的距离
//Distance to obtain ultrasonic waves
u16 TIM2CH2_CAPTURE_STA,TIM2CH2_CAPTURE_VAL;
void Get_Distane(void)        
{   
	 TRIG_SIG = 1;         
	 delay_us(15);  
	 TRIG_SIG = 0;	
	 if(TIM2CH2_CAPTURE_STA&0X80)//成功捕获到了一次高电平 //Successfully captured a high level once
	 {
		 g_distance=TIM2CH2_CAPTURE_STA&0X3F; 
		 g_distance*=65536;					        //溢出时间总和 Overflow time sum
		 g_distance+=TIM2CH2_CAPTURE_VAL;		//得到总的高电平时间 Get the total high level time
		 g_distance=g_distance*170/1000;      //时间*声速/2（来回） 一个计数0.001ms  Time * speed of sound/2 (round trip), one count 0.001ms
		 TIM2CH2_CAPTURE_STA=0;			//开启下一次捕获 Start the next capture
	 }				
}

//超声波回波脉宽读取中断 Ultrasonic echo pulse width reading interruption
void TIM2_IRQHandler(void)
{ 		    		  			    
	u16 tsr;
	tsr=TIM2->SR;
	if((TIM2CH2_CAPTURE_STA&0X80)==0)//还未成功捕获	Not captured yet
	{
		if(tsr&0X01)//定时器溢出 Timer overflow
		{	    
			 if(TIM2CH2_CAPTURE_STA&0X40)//已经捕获到高电平了  The high level has been captured
			 {
				 if((TIM2CH2_CAPTURE_STA&0X3F)==0X3F)//高电平太长了 The high level is too long
				 {
					  TIM2CH2_CAPTURE_STA|=0X80;      //标记成功捕获了一次 Marks a successful capture
						TIM2CH2_CAPTURE_VAL=0XFFFF;
				 }else TIM2CH2_CAPTURE_STA++;
			 }	 
		}
		if(tsr&0x04)//捕获2发生捕获事件 Capture 2 Capture event occurs
		{	
			if(TIM2CH2_CAPTURE_STA&0X40)		  //捕获到一个下降沿 	Capture a falling edge	
			{	  	
         	    
				TIM2CH2_CAPTURE_STA|=0X80;		  //标记成功捕获到一次高电平脉宽  //The marker successfully captures a high level pulse width
				TIM2CH2_CAPTURE_VAL=TIM2->CCR2;	//获取当前的捕获值.  //Get the current capture value.
				TIM2->CCER&=~(1<<5);			      //CC2P=0 设置为上升沿捕获   //CC2P=0 Set to rising edge capture
			}
			else  								     //还未开始,第一次捕获上升沿  Not started yet, first capture rising edge
			{
				 TIM2CH2_CAPTURE_STA=0;	 //清空 Clear
				 TIM2CH2_CAPTURE_VAL=0;
				 TIM2CH2_CAPTURE_STA|=0X40;		//标记捕获到了上升沿  The marker captures the rising edge
				 TIM2->CNT=0;									//计数器清空  Counter clear
				 TIM2->CCER|=1<<5; 						//CC2P=1 设置为下降沿捕获  CC2P=1 is set to capture the falling edge
			}		    
		}			     	    					   
	}
	TIM2->SR=0;//清除中断标志位  Clear interrupt flag
}





