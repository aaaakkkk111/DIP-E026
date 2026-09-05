#include "app_control.h"


//相关状态数据 Related status data
float x_pose=0, x_speed, angle_x, gyro_x, angle_z=0, gyro_z, last_angle=0;
float L_accel, R_accel, velocity_L, velocity_R;

//LQR状态反馈系数 LQR state feedback coefficient
float K1= -62.0484, K2= -73.3232, K3=-361.4617, K4=-35.9024 , K5=15.8114, K6=15.8114;
float K5OLD=15.8114, K6OLD=15.8114;

//目标状态值 Target state value
float Target_x_speed=0, Target_angle_x=0.0349, Target_gyro_z=0;

//速度换算成PWM占空比的比例系数 Proportional coefficient for converting speed to PWM duty cycle
float Ratio_accel=2400;	


float battery = 12;//初始状态处于满电 12v 12v The initial state is fully charged 12v
u8 Flag_velocity=2;

void EXTI15_10_IRQHandler(void)
{
	int Encoder_Left,Encoder_Right;             					//左右编码器的脉冲计数  Pulse counting of left and right encoders
		
  // 检查是否发生中断事件  Check if any interruption events have occurred
  if(MPU6050_INT==0)		
	{   
		EXTI->PR=1<<12;                           					//清除中断标志位 Clear interrupt flag bit
		
		Get_Angle(GET_Angle_Way);                     			//更新姿态，5ms一次，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果 Updating the posture once every 5ms, a higher sampling frequency can improve the effectiveness of Kalman filtering and complementary filtering
		Encoder_Left=Read_Encoder(MOTOR_ID_ML);            					//读取左轮编码器的值，前进为正，后退为负 Read the value of the left wheel encoder, forward is positive, backward is negative
		Encoder_Right=-Read_Encoder(MOTOR_ID_MR);           					//读取右轮编码器的值，前进为正，后退为负 Read the value of the right wheel encoder, forward is positive, backward is negative
		Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right); //获取速度 Obtain speed
				
		x_speed=(Encoder_Left+Encoder_Right)/2*PI*Diameter_67/1000/1560*Control_Frequency; // 除1000是mm转m  Except for 1000 mm to m
		x_pose+=x_speed/Control_Frequency;
		//获取倾角(rad)、角速度(rad/s)  Obtain inclination angle (rad) and angular velocity (rad/s)
		angle_x=Angle_Balance/180*PI;
		gyro_x=(angle_x-last_angle)*Control_Frequency;
		last_angle=angle_x;
		//获取转向速度(rad/s)、转向角(rad)  Obtain steering speed (rad/s) and steering angle (rad)
		gyro_z=(Encoder_Right-Encoder_Left)/Wheel_spacing/1000*PI*Diameter_67/1000/1560*Control_Frequency;
		angle_z+=gyro_z/Control_Frequency;
				
		if(g_newcarstate==enRUN)  
		{
//			Target_x_speed = 0.6f;
//			Target_x_speed = Target_x_speed/Flag_velocity;//前进速度(m/s)  Forward speed (m/s)
			Target_x_speed = 0.5f;
			x_pose = 0;
		}			
		else if(g_newcarstate==enBACK)
		{
			Target_x_speed = -0.6f;
			Target_x_speed = Target_x_speed/Flag_velocity;//后退速度(m/s)   Backward speed (m/s)
			x_pose = 0;
		}	
		//控制小车左转和右转  Control the left and right turns of the car
		else if(g_newcarstate==enLEFT)
		{
			Target_gyro_z = -4;					//左转转向速度(rad/s)  Left turn speed (rad/s)
			Target_gyro_z = Target_gyro_z/Flag_velocity;
			angle_z = 0;
		}
		else if(g_newcarstate==enRIGHT)
		{
			Target_gyro_z = 4;					//右转转向速度(rad/s)  Right turn speed (rad/s)
			Target_gyro_z = Target_gyro_z/Flag_velocity;
			angle_z = 0;
		}
		else if(g_newcarstate == enTLEFT)//左旋 Left turn
		{
			Target_gyro_z = -4;					//左转转向速度(rad/s)  Left turn speed (rad/s)
			Target_gyro_z = Target_gyro_z/Flag_velocity;
			angle_z = 0;
			K5 =  22.3607;
			K6 =  22.3607;
		}
		else if(g_newcarstate == enTRIGHT)//右旋  Right rotation
		{
			Target_gyro_z = 4;					//右转转向速度(rad/s)  Right turn speed (rad/s)
			Target_gyro_z = Target_gyro_z/Flag_velocity;
			angle_z = 0;
			K5 =  22.3607;
			K6 =  22.3607;
			
		}
		//超声波功能（跟随）  Ultrasonic function (follow)
	  else if(g_newcarstate==enFollow) //跟随 follow
		{
			Target_x_speed = 0.5;				//前进速度(m/s)  Forward speed (m/s)
			x_pose = 0;
			K5 = 0;
			K6 = 0;
		}
		else if(g_newcarstate==enAvoid) //躲避  avoid
		{
			Target_x_speed = -0.3;				//后退速度(m/s) Backward speed (m/s)
			x_pose = 0;
			K5 = 0;
			K6 = 0;
		}
		else
		{
			
			Target_x_speed = 0;				//平衡速度(m/s) Balance speed (m/s)
			Target_gyro_z = 0;						//平衡转向速度(rad/s) Balanced steering speed (rad/s)
			K5=K5OLD;
			K6=K6OLD;
		}
		
		//计算输入变量(LQR控制器) Calculate input variables (LQR controller)
		L_accel=-(K1*x_pose+K2*(x_speed-Target_x_speed)+K3*(angle_x-Target_angle_x)+K4*gyro_x+K5*angle_z+K6*(gyro_z-Target_gyro_z));
		R_accel=-(K1*x_pose+K2*(x_speed-Target_x_speed)+K3*(angle_x-Target_angle_x)+K4*gyro_x-K5*angle_z-K6*(gyro_z-Target_gyro_z));
		//速度换算成PWM占空比  Convert speed to PWM duty cycle
		velocity_L=(int)(Ratio_accel*(x_speed+L_accel/Control_Frequency));
		velocity_R=(int)(Ratio_accel*(x_speed+R_accel/Control_Frequency));
		
		
		//PWM限幅  PWM limiting
		Motor_Left=PWM_Limit(velocity_L,2600,-2600); //25khz->2592 
		Motor_Right=PWM_Limit(velocity_R,2600,-2600);		
		
		//滤掉死区  Filter out dead zones
		Motor_Left = PWM_Ignore(Motor_Left);
		Motor_Right = PWM_Ignore(Motor_Right);

		
		if(Turn_Off(Angle_Balance,battery)==0)     					//如果不存在异常 		If there are no abnormalities
			Set_Pwm(Motor_Left,Motor_Right);         					//赋值给PWM寄存器 	Assign to PWM register

   }
	
}

/**************************************************************************
Function: Get angle
Input   : way：The algorithm of getting angle 1：DMP  2：kalman  3：Complementary filtering
Output  : none
函数功能：获取角度	
入口参数：way：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/	
void Get_Angle(u8 way)
{ 
	float gyro_x,gyro_y,accel_x,accel_y,accel_z;
	float Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
	Temperature=Read_Temperature();      //读取MPU6050内置温度传感器数据，近似表示主板温度。 //Read the data from the MPU6050 built-in temperature sensor, which approximately represents the motherboard temperature.
	if(way==1)                           //DMP的读取在数据采集中断读取，严格遵循时序要求  //The reading of DMP is interrupted during data collection, strictly following the timing requirements
	{	
		Read_DMP();                      	 //读取加速度、角速度、倾角  //Read acceleration, angular velocity, and tilt angle
		Angle_Balance=Pitch;             	 //更新平衡倾角,前倾为正，后倾为负 //Update the balance tilt angle, with positive forward tilt and negative backward tilt
		Gyro_Balance=gyro[0];              //更新平衡角速度,前倾为正，后倾为负  //Update the balance angular velocity, with positive forward tilt and negative backward tilt
		Gyro_Turn=gyro[2];                 //更新转向角速度 //Update steering angular velocity
		Acceleration_Z=accel[2];           //更新Z轴加速度计 //Update Z-axis accelerometer
	}			
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取X轴陀螺仪 //Read X-axis gyroscope
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪 //Read Y-axis gyroscope
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪 //Read Z-axis gyroscope
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度计 //Read X-axis accelerometer
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计 //Read Y-axis accelerometer
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计 //Read Z-axis accelerometer
		if(Gyro_X>32768)  Gyro_X-=65536;                 //数据类型转换  也可通过short强制类型转换 Data type conversion can also be enforced through short type conversion
		if(Gyro_Y>32768)  Gyro_Y-=65536;                 //数据类型转换  也可通过short强制类型转换 Data type conversion can also be enforced through short type conversion
		if(Gyro_Z>32768)  Gyro_Z-=65536;                 //数据类型转换 Data type conversion
		if(Accel_X>32768) Accel_X-=65536;                //数据类型转换 Data type conversion
		if(Accel_Y>32768) Accel_Y-=65536;                //数据类型转换 Data type conversion
		if(Accel_Z>32768) Accel_Z-=65536;                //数据类型转换 Data type conversion
		Gyro_Balance=-Gyro_X;                            //更新平衡角速度 Update balance angular velocity
		accel_x=Accel_X/1671.84;
		accel_y=Accel_Y/1671.84;
		accel_z=Accel_Z/1671.84;
		gyro_x=Gyro_X/939.8;                              //陀螺仪量程转换 Gyroscope range conversion
		gyro_y=Gyro_Y/939.8;                              //陀螺仪量程转换 Gyroscope range conversion
		if(GET_Angle_Way==2)		  	
		{
			 Pitch= KF_X(accel_y,accel_z,-gyro_x)/PI*180;//卡尔曼滤波 Kalman filtering 
			 Roll = KF_Y(accel_x,accel_z,gyro_y)/PI*180;
		}
		else if(GET_Angle_Way==3) 
		{  
				Accel_Angle_x = atan2(Accel_Y,Accel_Z)*180/PI; //用Accel_Y和accel_y的参数得出的角度是一样的，只是边长不同 The angle obtained using Accel_Y and its parameters is the same, only the side length is different
				Accel_Angle_y = atan2(Accel_X,Accel_Z)*180/PI;
			
			 Pitch = -Complementary_Filter_x(Accel_Angle_x,Gyro_X/16.4);//互补滤波 Complementary filtering
			 Roll = -Complementary_Filter_y(Accel_Angle_y,Gyro_Y/16.4);
		}
		Angle_Balance=Pitch;                              //更新平衡倾角    Update the balance tilt angle
		Gyro_Turn=Gyro_Z;                                 //更新转向角速度  Update steering angular velocity
		Acceleration_Z=Accel_Z;                           //更新Z轴加速度计 Update Z-axis accelerometer
	}

}
