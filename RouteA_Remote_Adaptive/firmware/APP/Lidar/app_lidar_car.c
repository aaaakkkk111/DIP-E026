#include "app_lidar_car.h"

//Print radar data
//打印雷达数据
void App_Print_Data(void)
{
	for(uint16_t i=0;i<360;i++)
	{
		if(Tminidis[i]!=0)
		printf("%d\t%d\r\n",i,Tminidis[i]);
	}
	printf("######################\r\n");
}

//Take the average value based on a certain area of the radar
//Pass in parameters: start: starting angle end: ending angle
//根据雷达的一定的区域,取平均值
//传入参数：start：起始的角度  end:结束的角度
float App_Data_Avg(uint16_t start,uint16_t end)
{
	uint16_t sum = 0;
	uint16_t new_sum = 0;
	float average = 0;
	if(start > 360 || end > 360)//角度不合法 Illegal angle
	{
		return -1;
	}

	if(start-end > 0)
	{
		sum = 360 - start + end;
	}
	else
	{
		sum = end - start;
	}

	new_sum = sum;

	for(uint16_t i=0;i<sum;i++)
	{
		if(Tminidis[(start+i)%360]==0)
		{
			new_sum -- ;
		}
		else
		{
			average +=Tminidis[(start+i)%360]; //刚好满足数组的下标0-359 Exactly satisfies the index 0-359 of the array
		}

	}

	if(new_sum == 0)
	{
		return -1;//没接收到对应范围内角度的距离 Not receiving the distance of the corresponding angle within the range
	}

	return average/sum;
}

//According to a certain area of ??the radar, take the minimum value
//Input parameters: start: starting angle end: ending angle
//根据雷达的一定的区域,取最小值
//传入参数：start：起始的角度  end:结束的角度
float App_Data_Min(uint16_t start,uint16_t end)
{
	uint16_t sum = 0;
	int min = 20000;//该值大于雷达协议检测的最大值 This value is greater than the maximum value detected by the radar protocol
	if(start >360 || end > 360)//角度不合法 Illegal angle
	{
		return -1;
	}

	if(start-end > 0)
	{
		sum = 360 - start + end;
	}
	else
	{
		sum = end - start;
	}

	for(uint16_t i=0;i<sum;i++)
	{
		if(Tminidis[(start+i)%360]!=0)//抛弃为0的数 Discard numbers with 0
		{
			if(min > Tminidis[(start+i)%360])
			{
				min = Tminidis[(start+i)%360];
			}
		}
	}

	return min;
}

//----------------------------------------------避障 Obstacle--------------------------------//
#define avoid_dis 350
#define GO_avoid_speed 20
//避障功能 Obstacle avoidance function
void Car_Avoid(void)
{
	static u8 avoid_step = 0;//避障步骤 Obstacle avoidance steps
	
	//每次调用此函数都动态刷新一下值  // Dynamically refresh the value each time this function is called
	float get_data_mid = Tminidis[0];	
	float get_data_Lmid = Tminidis[270];//Left 左边
	float get_data_Rmid = Tminidis[90];////Right 右边
	
	
	switch(avoid_step)
	{
		case 0:
		if(get_data_mid <avoid_dis && get_data_mid >0 )//避障   Obstacle
		{ 
			//小车停止   Car stop
			Move_X = 0,Move_Z = 0;
			delay_time_int(10);
			avoid_step = 1;
		}
		else	
		{
			//小车前进   Car moving forward
			Move_X = GO_avoid_speed,Move_Z = 0;
		}
		break;
		
		case 1: if(get_time_int()==0) avoid_step ++;break; //时间到达去下一步 Time to go to next step
		case 2: 
		{
			//小车后退 Trolley backward
			Move_X = -15,Move_Z = 0;
			delay_time_int(100); //1s
			avoid_step = 3;
		}
		case 3: if(get_time_int()==0) avoid_step ++;break; //时间到达去下一步  Time to go to next step
		
		case 4: 
		{
			if(get_data_Lmid >= get_data_Rmid )
			{
				//左边 left
				Move_X = 0,Move_Z = -450;
				delay_time_int(100);
			}
			else
			{
				//右边 right
				Move_X = 0,Move_Z = 450;
				delay_time_int(100);
			}
			avoid_step = 5;
		}
		case 5: if(get_time_int()==0) avoid_step=0;break; //时间到达去下一步  Time to go to next step
		
	}

		
}



//--------------------------------------警卫 Security function---------------------------------------//
#define LEN_MUN 7
#define Alarm_Dis 380
#define BEEP_Alarm_Dis 200 //200

#define decect_Max_dis 450 //检测的距离范围不能超过这么远的距离 mm The detection distance range cannot exceed this far distance in millimeters

float get_data[LEN_MUN];
float data_min = 0;
int data_min_index = 4;

void Get_DIS_Group(void)
{
	data_min = 9999;//每次重新赋值  Each time you reassign
	data_min_index = 3; //中间 middle
	
//	App_Data_Min(150,210); //0
//	App_Data_Min(210,270); //1
//	App_Data_Min(270,330); //2
//	App_Data_Min(330,30); //3 正前方  Front
//	App_Data_Min(30,90);  //4
//	App_Data_Min(90,150); //5
	
	//尽量减少数据处理  Minimize data processing
	get_data[1] = Tminidis[270];
	get_data[2] = Tminidis[300];
	get_data[3] = Tminidis[0]; //3 正前方 Front
	get_data[4] = Tminidis[60]; 
	get_data[5] = Tminidis[90]; 
	
	
	for(uint8_t i = 0;i<LEN_MUN;i++) //存数据  Storing data
	{
		if(get_data[i] == 0) continue;//去掉0的干扰数  Remove the interference number of 0
		
		if(data_min > get_data[i])
		{
			data_min = get_data[i];//保存最小的数据  Save minimal data
			data_min_index = i;
		}

//		printf("%d = %.2f\r\n",i,get_data[i]);
	}

//	printf("min:%d = %.2f\r\n",data_min_index,data_min);


}

//Security function
//警卫功能
void Car_Alarm(void)
{
	
//	Get_DIS_Group(); //放到平衡中断服务处理，增加响应 Put it into balanced interrupt service processing to increase response
	
	if(data_min >= decect_Max_dis) //超过最大距离 Exceeding the maximum distance
	{
		Move_Z = 0; //小车停止 car stop
		BEEP_OFF;
		return;
	}

	if( data_min <= BEEP_Alarm_Dis)
	{
		BEEP_ON;
	}
	else if( data_min <= Alarm_Dis)
	{
		BEEP_OFF;
	}
	else //都不在范围 Not in range
	{
		BEEP_OFF;
		Move_Z = 0;//小车停止 car stop
	}

	//控制小车 control car
	if(data_min_index<3) //3就是中间值  3 is the middle value
	{
		Move_Z = -450; //小车左旋 Left turn of the car
	}
	else if(data_min_index>3)
	{
		Move_Z = 450;//小车右旋   Turn the car right
	}
	else
	{
		Move_Z = 0;
	}


}


//-----------------------------------跟随功能 Follow function------------------------//
//跟随功能 Follow function
void Car_Follow(void)
{
	float get_data_mid = Tminidis[0];

	float get_data_LL = Tminidis[315];

	float get_data_RR = Tminidis[45];

//	printf("data = %.2f\t %.2f\t %.2f\r\n",get_data_mid,get_data_LL,get_data_RR);

	// positive direction 正方向
	if(get_data_mid < 130 && get_data_mid>0)
	{
		//小车后退  The car moves backward
		Move_X = -15;
		Move_Z = 0;
	}
	else if (get_data_mid > 320 && get_data_mid < 450)
	{
		//小车前进 The car moves forward
		Move_X = 15;
		Move_Z = 0;
	}          


	// Left direction 左方向
	else if (get_data_LL < 340 && get_data_mid>250)
	{
		// Left rotation of the car 小车左旋
		Move_X = 0;
		Move_Z = -450;
	}


	// Right direction 右方向
	else if (get_data_RR < 340 && get_data_mid>250)
	{
		//小车右旋  The car turns right
		Move_X = 0;
		Move_Z = 450;
	}

	else//Not following 不跟随
	{
		Move_X = 0;
		Move_Z = 0;
	}

}


//------------------------------------巡逻 patrol--------------------------//
#define Go_time 200 //2s 
#define Trun_time 63 //650ms 

#define patrol_times 2 //巡逻圈数  Patrol laps
#define string_pd (70)
#define beep_Patroldis 300  //障碍物距离  Obstacle distance

extern float myTurn_Kd;
float Move_X_old,Move_Z_old;
u8 start_patrol = 1;
u16 lasttime = 0;


//巡逻功能 patrol
void Car_Patrol(void) 
{
	
	static u8 patrol_step = 0;
	static u8 runtimes = 1;
	
	float Patroldis = Tminidis[0];//只处理0°角  Only handles 0° angles
	
	if(patrol_step >= 7)
	{
		//巡逻功能已停止 Patrol function has been stopped
		OLED_Draw_Line("patrol put key1    ", 2, false, true); 
		while(!Key1_State(1));//死等,等待按键按下，重新巡逻  Wait for the button to be pressed and resume patrolling
		patrol_step = 0;
		runtimes = 1;
		return;
	}
	
	
	//当巡逻的路径有障碍物 When there are obstacles in the patrol path
	if(Patroldis < beep_Patroldis && Patroldis>0)
	{
		//以下操作只做一次  The following operations are performed only once
		if(start_patrol == 1)
		{
			BEEP_ON;
			Move_X_old = Move_X; //备份  Backup
			Move_Z_old = Move_Z; //备份  Backup
			Move_X = 0,Move_Z = 0;//停车 stop
			start_patrol = 0;//暂停计时 Pause timer
			lasttime = get_time_int();//把剩下的计时保留下来  Keep the remaining time
		}
		return;
	}
	
	if(start_patrol == 0)
	{
		Move_X = Move_X_old; //恢复  recover
		Move_Z = Move_Z_old; //恢复 recover
		BEEP_OFF;
		start_patrol = 1;
		
		set_time_int(lasttime);// 把上次停止的时间算完 Complete the time when you last stopped
		return;
	}
	
	switch(patrol_step)
	{
		case 0:myTurn_Kd = string_pd,Move_X = 15;Move_Z = 0; delay_time_int(Go_time);patrol_step++;break; //前进 go ahead
		case 1:if(get_time_int() == 0)patrol_step++;break; //等待时间到达 Waiting time to arrive
			
		
		case 2:myTurn_Kd = 35,Move_X = 0;Move_Z = 1300; delay_time_int(Trun_time);patrol_step++;break; //转弯  Turning
		case 3:if(get_time_int() == 0)patrol_step++;break; //等待时间到达 Waiting time to arrive
		
		
		//防止抖动 Preventing jitter
		case 4:Move_X = 0;Move_Z = 0;delay_time_int(30);patrol_step++;break;//停止 stop
		case 5:if(get_time_int() == 0)patrol_step++;break; //等待时间到达 Waiting time to arrive
		
		case 6:
			if(runtimes < (patrol_times*4) )
			{
				runtimes++;
				patrol_step = 0;
			}else patrol_step =7;//复位键清除	Reset button clear	 
			break; 
			
		case 7:Move_X = 0;Move_Z = 0;//停止 stop
		
	}

}

//------------------------------------巡墙边做直线 Make a straight line by patrolling the wall--------------------------//

u8 g_lidar_go_flag = 0;
//动态获取目标距离的时间 Dynamic acquisition of target distance time
#define GET_LidarDIS_Time 200 //5ms*200 启动后获取1s的距离  Get the distance in 1s after startup

//给一个前进的速度 Give a forward speed
#define Track_Lidar_Speed 15 
#define Track_Lidar_KP (2) //2 
#define Track_Lidar_KD (12) //12 8

#define Lidar_Angle 72

void Set_lidartrack_speed(void)
{
	Move_X = Track_Lidar_Speed;       
}


void LiDar_Straight(void) 
{
	static u16 target_distance=0;
	
	u16 current_distance=target_distance;
	
	static u16 Limit_distance=0;   //雷达最大的探测距离 Maximum detection range of radar
	static u16 get_timedis = 0; 
	
	for (u8 i = 0;i<5;i++) //探测的距离为72度 The detection distance is 72 degrees
	{
		if(get_timedis<GET_LidarDIS_Time)
		{
			if(Tminidis[Lidar_Angle]==0 || Tminidis[Lidar_Angle]>400) continue; //抛弃为0和大于400mm的数据 Discard data with values of 0 and greater than 400mm
			
			get_timedis ++;
			target_distance = Tminidis[Lidar_Angle];//动态获取到目标角度 Dynamically obtain the target angle
			Limit_distance=target_distance+200;//比目标距离大200mm,主要避免参照物的消失导致小车快速转向 200mm greater than the target distance, mainly to avoid the disappearance of the reference object causing the car to quickly turn
			
			if(get_timedis == (GET_LidarDIS_Time-1)) 
			{
				g_lidar_go_flag = 1;
				get_timedis = GET_LidarDIS_Time;
			}
		}
		
		 if(Tminidis[Lidar_Angle]<(Limit_distance))//限制一下雷达的探测距离 Limit the detection range of the radar
		{
			 current_distance=Tminidis[Lidar_Angle];//确定距离 Determine distance
		}
				
	}
	
	if(get_timedis<GET_LidarDIS_Time) //当距离没确定完，不做任何操作 Do not take any action until the distance is determined
	{
		return;
	}
	
	
	if(Tminidis[0]>0 && Tminidis[0]<400)//前方有障碍物停止 Stop with obstacles ahead
	{
		Move_X = 0;
		Move_Z = 0;
			return;
	}
	
	Set_lidartrack_speed();
	
	Move_Z = Distance_Adjust_PID(current_distance,target_distance);
}


float Distance_Adjust_PID(float Current_Distance,float Target_Distance)//距离调整PID Distance adjustment PID
{
	static float error,OutPut,Last_error;
	error=Target_Distance-Current_Distance;                          	//计算偏差 Calculate deviation
	OutPut=-Track_Lidar_KP*error-Track_Lidar_KD*(error-Last_error);								//位置式PID控制器  Position based PID controller
	Last_error=error;                                       		 			//保存上一次偏差 Save last deviation
	return OutPut;                                          	 
}



//---------------------雷达沿着墙壁走 The radar moves along the wall------------------------------------\\

u8 g_lidar_go_flag_wall = 0;
//动态获取目标距离的时间 Dynamic acquisition of target distance time
#define GET_LidarDIS_Time_wall 200 //5ms*200 启动后获取1s的距离  Get the distance in 1s after startup

//给一个前进的速度 Give a forward speed
#define Track_Lidar_Speed_wall 15 
#define Track_Lidar_KP_wall (2) //2 
#define Track_Lidar_KD_wall (12) //12 8
static float error_wall,OutPut_wall,Last_error_wall;
	
#define Lidar_Angle_wall 72

u8 lidar_wall_flag;//1:有障碍物在转弯 There are obstacles turning
extern float myTurn_Kd; //引入变量  Introduce variables
void Set_lidartrack_speed_wall(void)
{
	Move_X = Track_Lidar_Speed_wall;       
}


void LiDar_Straight_wall(void) 
{
	static u16 target_distance=0;
	
	u16 current_distance=target_distance;
	
	static u16 Limit_distance_wall=0;   //雷达最大的探测距离 Maximum detection range of radar
	static u16 get_timedis_wall = 0; 
	
	for (u8 i = 0;i<5;i++) //探测的距离为72度 The detection distance is 72 degrees
	{
		if(get_timedis_wall<GET_LidarDIS_Time_wall)
		{
			if(Tminidis[Lidar_Angle_wall]==0 || Tminidis[Lidar_Angle_wall]>400) continue; //抛弃为0和大于400mm的数据 Discard data with values of 0 and greater than 400mm
			
			get_timedis_wall ++;
			target_distance = Tminidis[Lidar_Angle_wall];//动态获取到目标角度 Dynamically obtain the target angle
			Limit_distance_wall=target_distance+200;//比目标距离大200mm,主要避免参照物的消失导致小车快速转向 200mm greater than the target distance, mainly to avoid the disappearance of the reference object causing the car to quickly turn
			
			if(get_timedis_wall == (GET_LidarDIS_Time_wall-1)) 
			{
				g_lidar_go_flag_wall = 1;
				get_timedis_wall = GET_LidarDIS_Time_wall;
			}
		}
		
		 if(Tminidis[Lidar_Angle_wall]<(Limit_distance_wall))//限制一下雷达的探测距离 Limit the detection range of the radar
		{
			 current_distance=Tminidis[Lidar_Angle_wall];//确定距离 Determine distance
		}
				
	}
	
	if(get_timedis_wall<GET_LidarDIS_Time_wall) //当距离没确定完，不做任何操作 Do not take any action until the distance is determined
	{
		return;
	}
	
	
	if(Tminidis[0]>0 && Tminidis[0]<Limit_distance_wall)//根据初始值,确定避障的距离；因为跟随的距离是72°，所以只需要左转 Determine the obstacle avoidance distance based on the initial value; Because the following distance is 72 degrees, only a left turn is needed
	{
		if(lidar_wall_flag == 0)
		{
			lidar_wall_flag = 1;
			myTurn_Kd = 30;
			Move_Z = -600; //左转 turnleft
			delay_time_int(1);//10ms 
		}
			Move_X = 0; 
			Last_error_wall = 0;//清空pid的错误,避免转向太大 Clear PID errors to avoid excessive steering
			return;
	}
	
	if(get_time_int()!=0)
	{
		return;
	} 
	else //转弯 Turning
	{
		if(lidar_wall_flag ==1)
		{
			lidar_wall_flag = 0;
			Move_X = 0;
			Move_Z = 0;
			myTurn_Kd = 0;
			return;
		} 
		
	}
	
	Set_lidartrack_speed();
	
	Move_Z = Distance_Adjust_PID_wall(current_distance,target_distance);
}



float Distance_Adjust_PID_wall(float Current_Distance,float Target_Distance)//距离调整PID Distance adjustment PID
{
	error_wall=Target_Distance-Current_Distance;                          	//计算偏差 Calculate deviation
	OutPut_wall=-Track_Lidar_KP_wall*error_wall-Track_Lidar_KD_wall*(error_wall-Last_error_wall);								//位置式PID控制器  Position based PID controller
	Last_error_wall=error_wall;                                       		 			//保存上一次偏差 Save last deviation
	return OutPut_wall;                                          	
}



