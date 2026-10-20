#include "app_lidar.h"


#define Lidar_send USART3_Send_U8 //USART2_Send_U8 

 
uint8_t g_recvbuf[TempLen_Max]={'\0'};//一包数据的缓存 Cache of a packet of data

uint8_t g_recvbuf_backup[TempLen_Max]={'\0'};//一包数据的缓存  Cache of a packet of data

uint32_t Tminidis[TminiCYCLE]={'\0'}; //数据下标:代表角度,数据即距离 0-359°  Data index: represents angle, data refers to distance 0-359 °

TminiPlus_Radar_t lidar_data[TminiCYCLE];
TminiPlus_MsgData_t timiplus_msg;

uint8_t lidar_new_pack = 0;//1：代表接收到数据了 1: The representative has received the data
uint8_t lidar_new_deal = 0;//1：代表接收到正确数据可以处理了 1: The representative has received the correct data and can now process it

//雷达开始扫描 Radar begins scanning
void Start_Lidar(void)
{
	Lidar_send(0xA5);
	Lidar_send(0X60);
}

//雷达停止扫描 Radar stops scanning
void Stop_Lidar(void)
{	
	Lidar_send(0xA5);
	Lidar_send(0X65);
}

//接收雷达数据 Receive radar data
void recv_lidar_data(u8 rxtemp)
{
	static uint8_t step = 0;
	static uint16_t si_len = 0;
	static uint8_t si_index = 0;
	switch(step)
	{
		
		case 0:
			if(rxtemp == Lidar_HeaderLSB)
			{
				step = 1;
				g_recvbuf[0] = Lidar_HeaderLSB; 
			}	
		break;
		
		case 1:
			if(rxtemp == Lidar_HeaderMSB) 
			{
				step = 2;
				g_recvbuf[1] = Lidar_HeaderMSB; 
			}
		break;
			
		case 2: g_recvbuf[step] = rxtemp; step++; break; //CT信息  CT Information
		case 3: g_recvbuf[step] = rxtemp; step++; si_len = rxtemp * 3; //S的数量  Number of S
			
			//在此判断数据长度时候是否会大于缓存 When determining whether the data length will be greater than the cache here
			if(si_len+10>=TempLen_Max)
			{
					si_len = 0;
					step = 0;
					memset(g_recvbuf,0,sizeof(g_recvbuf));
			}				
			break; 
			
		case 4: g_recvbuf[step] = rxtemp; step++;  break; //起始角低8位 Starting angle is 8 digits lower
		case 5: g_recvbuf[step] = rxtemp; step++;  break; //起始角高8位 Starting angle height of 8 digits
			
		case 6: g_recvbuf[step] = rxtemp; step++;  break; //结束角低8位 end angle is 8 digits lower
		case 7: g_recvbuf[step] = rxtemp; step++;  break; //结束角高8位 End angle height of 8 digits
			
		case 8: g_recvbuf[step] = rxtemp; step++;  break; //校验码低8位  Low 8 digits of verification code
		case 9: g_recvbuf[step] = rxtemp; step++;  break; //校验码高8位  High 8 digits of verification code
				
		case 10:
		{
				g_recvbuf[step + si_index] = rxtemp;  
				si_index++;
				
				if(si_index >= si_len )
				{
					//Deal_Radar();

					lidar_new_pack = 1;//接收完一包数据 After receiving a packet of data

					si_index = 0;
					si_len = 0;
					step = 0;//接收完一包数据  After receiving a packet of data
//					memset(g_recvbuf,0,sizeof(g_recvbuf));//清掉  Clear
				}
				break;
		
		}
	}
}


void Deal_Radar(void)
{
	TminiPlus_MsgData_t *timiplus_msg_p = &timiplus_msg;
	uint16_t llen = g_recvbuf_backup[3]*3;//长度 length
	uint8_t si_step = 0 ;
	
	
	//小端转大端 Small end to large end
	timiplus_msg_p->PH 	= g_recvbuf_backup[1]<<8 | g_recvbuf_backup[0];
	timiplus_msg_p->CT 	= g_recvbuf_backup[2];
	timiplus_msg_p->LSN = g_recvbuf_backup[3];
	timiplus_msg_p->FSA = g_recvbuf_backup[5]<<8 | g_recvbuf_backup[4];
	timiplus_msg_p->LSA = g_recvbuf_backup[7]<<8 | g_recvbuf_backup[6];
	timiplus_msg_p->CS  = g_recvbuf_backup[9]<<8 | g_recvbuf_backup[8];
	
	for(uint16_t i = 0;i<llen;i+=3)
	{
		timiplus_msg_p->SI[si_step].Intensity = g_recvbuf_backup[10+i]&0x00FF;//光照强度 Light intensity
		timiplus_msg_p->SI[si_step++].SI_dis = g_recvbuf_backup[10+i+2]<<8 | g_recvbuf_backup[10+i+1]; //距离的有效值 Effective value of distance
	}
	
	//异或处理 XOR processing
	if(Tmini_checkout(timiplus_msg_p)!=0)//校验码错误 Verification code error
	{
		memset(timiplus_msg_p,0,sizeof(timiplus_msg));
		return;
	}
	
	lidar_new_deal = 1;//一包数据正确 One packet of data is correct

	memset(g_recvbuf_backup,0,sizeof(g_recvbuf_backup));
//	Get_Tmini_Data(timiplus_msg_p);//数据处理 要放到其它地方,这里会影响平衡效果 Data processing needs to be placed elsewhere, which will affect the balance effect
	
	
	
}

void Get_Tmini_Data(TminiPlus_MsgData_t *msg)
{

	double start_angle,end_angle,abs_angle;
	
	for(uint8_t len = 0;len < msg->LSN;len++)
	{		//距离解算 Distance calculation
			lidar_data[len].dis  = Get_Tminidis(msg->SI[len].SI_dis); 		//获取距离数据 Obtain distance data
	}
	
	
	//角度解算 Angle calculation
	start_angle = Get_Start_Stop_Tminiangle(msg->FSA); //起始角 Starting angle
	end_angle = Get_Start_Stop_Tminiangle(msg->LSA);  //结束角 End Corner
	
	//开始角数据存储 Start corner data storage
		lidar_data[0].angle = start_angle; 
	
	//结束角数据存储 End corner data storage
		lidar_data[msg->LSN -1].angle = end_angle; 
	
	//角度限制 Angle limitation
		if(start_angle>end_angle) //350->10这种情况  350->10 This situation
		{
			abs_angle = 360 - start_angle + end_angle;
		}
		else//正常情况 Under normal circumstances
		{
			abs_angle = end_angle - start_angle;
		}
		
		//中间角解算 Middle angle calculation
		if (abs_angle != 0) 
		{
			for(uint8_t len=1;len < msg->LSN-1;len++)//0是起始角;  msg->LSN-1:最终角  0 is the starting angle; msg->LSN-1: the final angle
			{
					lidar_data[len].angle = Get_Mid_Tminiangle(abs_angle,len,msg->LSN-1,start_angle);
			}
		}
	
	
	//赋值到有效数据的数组 Assign an array to valid data
		Store_360angle_data(msg->LSN);
		
}



uint8_t Tmini_checkout(TminiPlus_MsgData_t *msg)
{
	uint16_t result = 0;
	uint16_t second = (msg->LSN <<8) | msg->CT; //CT和LSN的合体，LSN在前 Combination of CT and LSN, with LSN before
	uint8_t len_temp = msg->LSN;
	result = msg->PH ^ second ^ msg->FSA ^ msg->LSA;
	
	do
	{
		result ^= msg->SI[len_temp - 1].Intensity;//光照强度 Light intensity
		result ^= msg->SI[len_temp - 1].SI_dis; //距离值 Distance value
		len_temp --;
	}while(len_temp);
	
	if (result != msg->CS)
	{
		return 1; //校验码错误 Verification code error
	}
	
	return 0;
}

//获取距离 Get distance
int Get_Tminidis(uint16_t dis_temp)
{
   return ((dis_temp>> 8) << 6) | ((dis_temp &0x00FF)>>2); //高8位*64 + 低8位去掉最后两位即为距离 High 8 bits * 64+Low 8 bits. Removing the last two bits is the distance
}


//中间角度解算 Intermediate angle calculation
double Get_Mid_Tminiangle(double abs_angle,uint8_t index,uint8_t data_len,double s_angle)
{
	double angle_temp = abs_angle/data_len * index; 
	angle_temp += s_angle;
	return  measure_if_360(angle_temp);
}


//起始角、结束角解算 Starting angle and ending angle calculation
double Get_Start_Stop_Tminiangle(uint16_t S_angle)
{
	//1级解算 Level 1 solution
	double angle_temp = (double)((S_angle >> 1)/64.0);
	return angle_temp ;
}


//判断角是否符合360°的范围 Determine whether the angle falls within the range of 360 °
double measure_if_360(double angle)
{
	if(angle > 360)
	{
		return angle - 360;
	}
	
	if(angle < 0)
	{
		return 360 + angle;
	}
	
	return angle;
}


//保存360度的有效数据 Save 360 degree valid data
void Store_360angle_data(uint8_t mylength)
{
	static u16 dis_max = 16383;
	u16 Tminidis_index = 0;	
	for(uint8_t index = 0;index<mylength;index++)
	{
		if(lidar_data[index].angle>360 || lidar_data[index].angle <0) //角度不合法 Illegal angle
			continue;
		Tminidis_index = lidar_data[index].angle;//角度对应数组下标 Angle corresponding array index
		
		Tminidis[Tminidis_index] = lidar_data[index].dis; //距离的数据和角度一一对应 Corresponding distance data and angles one by one
		if(Tminidis[Tminidis_index]>dis_max)
		{
			Tminidis[Tminidis_index] = dis_max;//不超过协议距离最大值 Not exceeding the maximum protocol distance
		}
		
	}
	
}

//给外面使用 解算雷达数据 
//Provide radar data for external use
void Deal_lidardata(void)
{
	memcpy(g_recvbuf_backup,g_recvbuf,256);//复制 copy 
	memset(g_recvbuf,0,sizeof(g_recvbuf));//清掉  clean

	Deal_Radar();

	if(lidar_new_deal == 1)
	{
		lidar_new_deal = 0;
		Get_Tmini_Data(&timiplus_msg);
	}


}


