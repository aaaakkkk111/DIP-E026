#include "app_mode.h"

uint8_t angle_max = 40;
Car_Mode mode = Normal;//Normal; //模式为正常


//模式选择 用手拧轮子来进行模式切换
//Mode selection: Use the hand to twist the wheel to switch modes
void Mode_select(void)
{
	int16_t mode_cnt = 0;
	OLED_Draw_Line("1.Standard Mode", 1, true, true); 

	while(!Key1_State(1)) 
	{
		mode_cnt +=Read_Encoder(MOTOR_ID_ML);
		mode_cnt +=-Read_Encoder(MOTOR_ID_MR);
		car_mode(mode_cnt);//模式选择 Mode selection
		show_mode_oled();//oled显示模式 oled display mode
		TUNE_Poll();   /* keep the serial console alive while choosing a mode */
		
	}
	
	Set_Mid_Angle();//模式设置好后,设置机械中值 After setting the mode, set the mechanical median value
	Set_angle();//设置跌倒倾角 Set the inclination angle for falls
	Set_control_speed();//设置遥控的速度 Set the speed of the remote control


	Set_PID();//某些模式的需要特殊设置一下平衡pid Some modes require special settings for balancing PID

}


void car_mode(int16_t cnt)
{
	static int16_t cnt_old ;
	
	if(myabs(myabs(cnt)-myabs(cnt_old))>250)
	{
		if(cnt < cnt_old)
		{
			mode = (Car_Mode)((mode - 1) %Mode_Max); //大到小 枚举(u8) -1即为255  Large to small   enumeration (u8) -1 is 255
			if(mode > Mode_Max)
			{
				mode = (Car_Mode)(Mode_Max -1);
			}
		}
		else
		{
			mode = (Car_Mode)((mode + 1) %Mode_Max); //小到大  Small to Large
		}
		
		cnt_old = cnt; //赋值  Assignment
//		printf("%d\r\n",mode);
	}
	
}


//根据模式设置机械中值 Set the mechanical median according to the mode
void Set_Mid_Angle(void)
{
	switch ((uint8_t)mode)
	{
		case Normal:   	
		case U_Follow:
		case U_Avoid:    		 	
		case Weight_M:  		 	
		case Line_Track: 
		case Diff_Line_track:
		case PPO_Nav:
		case Load_Auto:
		case Load_DB650:
		case Load_DB900:
		case Load_DB1300:
		case Load_Adapt:
		case Adapt_V4:
		case PS2_Control:   	Mid_Angle = 0;  break;
		
		case CCD_Mode:   			Mid_Angle = 1;  break;
		case ElE_Mode:   			Mid_Angle = -4; break;
		
		case K210_QR:   			
		case K210_Line:  	 		
		case K210_Follow:
		case K210_SelfLearn:
		case K210_mnist:			Mid_Angle = -1; break;
		
		case LiDar_avoid:   	
		case LiDar_Follow:    
		case LiDar_aralm:   	
		case LiDar_Patrol:  	
		case LiDar_Line: 			
		case LiDar_wall_Line: 		Mid_Angle = 0;  break;
	}

}

//遥控的速度初始化
//Speed initialization of remote control
void Set_control_speed()
{
	if(mode == Normal || mode == Weight_M || IS_LOAD_MODE(mode) || mode == Load_Adapt || mode == Adapt_V4) //正常和负重模式都需要初始化该值  Both normal and load modes need to initialize this value
	{
		Car_Target_Velocity=30;
		Car_Turn_Amplitude_speed=36;
	}
	else if(mode == PPO_Nav)
	{
		//Turn_Kp is 95.2 here, 5.6x the stock 17. A Turn_Target of 36 would
		//ask for 3427 PWM and saturate the +-2600 limiter at once, so the
		//remote turn amplitude is forced to 0: nav steers through Move_Z only.
		Car_Target_Velocity = 0;
		Car_Turn_Amplitude_speed = 0;
	}
	else if(mode == PS2_Control)
	{
		Car_Target_Velocity = 30;
		Car_Turn_Amplitude_speed = 48;
	}

}

//设置跌倒的倾角
//Set the inclination angle for falls
void Set_angle(void)
{
	if((mode == Line_Track)||(mode == ElE_Mode)||(mode == Diff_Line_track))
	{
		angle_max = 16;
	}
	else if(mode == CCD_Mode)
	{
		angle_max = 25;
	}
	else if((mode == K210_QR)||(mode == K210_Line)||(mode == K210_Follow)||(mode == K210_SelfLearn)||(mode == K210_mnist))
	{
		angle_max = 30;
	}
	else
	{
		angle_max = 40;
	}

}


extern float Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki,Turn_Kp,Turn_Kd; //引入立直环、速度环、转向环 //Introduce vertical rings, speed rings, and steering rings
extern float myTurn_Kd; //Turn_PD() uses this one unless the bluetooth remote asks for forward/back
void Set_PID(void)
{
	if(mode == Weight_M || mode == K210_Follow) //负重 Load bearing
	{
		
		Balance_Kp =9600;
		Balance_Kd =75 ; 
		Velocity_Kp=7000; 
	  Velocity_Ki=35;  
	  Turn_Kp=1400; 
		Turn_Kd=20;

	}
//	else if(mode ==Diff_Line_track) //高难度巡线 High-difficulty line patrol
//	{
//		Balance_Kp =8400;
//		Balance_Kd =42; 
//		Velocity_Kp=6500;
//		Velocity_Ki=32.5; 
//		Turn_Kp=2500; 
//		Turn_Kd=20; 
//	}
	else if(mode == ElE_Mode)//电磁 electromagnetism
	{
		Balance_Kp =9900;
		Balance_Kd =72 ;

		Velocity_Kp=7000; 
		Velocity_Ki=35; 


		Turn_Kp=2500; 
		Turn_Kd=20;
	}
	else if(mode == K210_Line)
	{
		Balance_Kp =12000;
		Balance_Kd =72 ;

		Velocity_Kp=8000; 
		Velocity_Ki=40;  

		Turn_Kp=2500; 
		Turn_Kd=20;

	}

	else if(mode == LiDar_Line || mode == LiDar_wall_Line)
	{
		Balance_Kp =10200;
		Balance_Kd =75 ; 

		Velocity_Kp =9000; 
		Velocity_Ki =45;  

		Turn_Kp=2500; 
		Turn_Kd=20;

	}
	else if(mode == PPO_Nav) //PPO trained gains, policy_stm32_goto_v2.c  2026-09-12 23:26  24000 steps
	{
		Balance_Kp =22221.4f;	//twin balance_kp  = 222.21
		Balance_Kd =33.3f;		//twin balance_kd  = 0.3327

		Velocity_Kp=7291.8f;	//twin velocity_kp = 72.92
		Velocity_Ki=21.7f;		//twin velocity_ki = 0.2165

		Turn_Kp    =9520.1f;	//twin turn_kp     = 95.20
		Turn_Kd    =37.3f;		//twin turn_kd     = 0.3727

		//Turn_PD() only uses Turn_Kd while the remote asks for forward/back,
		//otherwise it uses myTurn_Kd. The twin trained with the yaw damping
		//always on, so mirror it here or Turn_Kd never takes effect.
		myTurn_Kd  =Turn_Kd;
	}
	else if(mode == Load_Adapt) /* mode 26 -- LA_Tick() writes all six gains
	                              every tick, interpolating between the stock
	                              Normal and Weight_M sets.  Nothing set here. */
	{
		LA_Reset();
	}
	else if(mode == Adapt_V4) /* mode 27 -- V4_Tick() writes all six gains every tick */
	{
		V4_Reset();
	}
	else if(IS_LOAD_MODE(mode)) /* modes 22-25 -- LD_Tick() rewrites all six gains
	                              every control tick, so nothing is set here.
	                              LD_Reset() boots the detector in HEAVY. */
	{
		LD_SetProfile( mode == Load_DB650  ? LD_PROF_DB650  :
		               mode == Load_DB900  ? LD_PROF_DB900  :
		               mode == Load_DB1300 ? LD_PROF_DB1300 :
		                                     LD_PROF_LEGACY );
		LD_Reset();
	}
	else if(mode == Normal || mode == PS2_Control || mode == LiDar_Patrol)
	{
		Balance_Kp =9600;
		Balance_Kd =48 ; 

		Velocity_Kp =6200; 
		Velocity_Ki =31;  

		Turn_Kp =1700; 
		Turn_Kd =20;
	}

}
