#ifndef __APP_LIDAR_H_
#define __APP_LIDAR_H_


#include "ALLHeader.h"

//此型号 stm32的内存 512k  This model of stm32 has 512k memory
#define TempLen_Max				256

#define SILEN					  	360   //一包协议的SI长度  SI length of a packet protocol
#define TminiLEN_MAX  		360  	//有效数据储存  Effective data storage
#define TminiCYCLE 	  	  360   

#define Lidar_HeaderLSB 0xAA
#define Lidar_HeaderMSB 0x55

//雷达有效数据  Radar valid data
extern uint32_t Tminidis[TminiCYCLE];
extern uint8_t lidar_new_pack;

//雷达的最终信息结构体  The final information structure of the radar
typedef struct _TminiPlus_radar
{
	int 		dis; //距离 distance
	double  angle;//角度  angle
}TminiPlus_Radar_t;


//雷达SI采样有效数据结构体  Radar SI sampling valid data structure
typedef struct _DataSI_
{
	uint16_t 		Intensity; //光强 协议是8位,因为后续要异或，直接补成16位  The light intensity protocol is 8 bits, but because it needs to be XORed later, it is directly filled to 16 bits
	uint16_t  	SI_dis;//没解算前的距离  Distance before solution
}DataSI_t;


typedef struct TminiPlus_Data{
	uint16_t PH;		 		//包头  Baotou
	uint8_t	 CT;		 		//CT信息  CT Information
	uint8_t	 LSN; 	 		//SI的长度  SI length
	uint16_t FSA; 	 		//起始角  Starting angle
	uint16_t LSA; 	 		//结束角  End Angle
	uint16_t CS;  	 		//校验码  Check code
	DataSI_t SI[SILEN]; //数据有效数组  Data valid array
}TminiPlus_MsgData_t;


void Start_Lidar(void);
void Stop_Lidar(void);

void recv_lidar_data(u8 rxtemp);
void Deal_lidardata(void);


//以下函数尽量不要在其它文件使用  Try not to use the following functions in other files
void Deal_Radar(void);
void Get_Tmini_Data(TminiPlus_MsgData_t *msg);
uint8_t Tmini_checkout(TminiPlus_MsgData_t *msg);
int Get_Tminidis(uint16_t dis_temp);
double Get_Mid_Tminiangle(double abs_angle,uint8_t index,uint8_t data_len,double s_angle);
double Get_Start_Stop_Tminiangle(uint16_t S_angle);
double measure_if_360(double angle);
void Store_360angle_data(uint8_t mylength);





#endif

