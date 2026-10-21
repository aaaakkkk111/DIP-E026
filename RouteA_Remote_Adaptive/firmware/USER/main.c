/**
* @par Copyright (C): 2018-2028, Shenzhen Yahboom Tech
* @file         // main.c
* @author       // lly
* @version      // V1.0
* @date         // 240628
* @brief        // 程序入口 Program entry
* @details      
* @par History  // 修改历史记录列表，每条修改记录应包括修改日期、修改者及
*               // 修改内容简述  Modification history list, each modification record should include the modification date, modifier and a brief description of the modification content
*/ 

#include "AllHeader.h"
#include "intsever.h"
//注意:操作蜂鸣器的时候，要判断是否处于正常电压
//Attention: When operating the buzzer, check if it is at normal voltage

uint8_t GET_Angle_Way=2;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波  //Algorithm for obtaining angles, 1: Quaternion 2: Kalman 3: Complementary filtering
float Angle_Balance,Gyro_Balance,Gyro_Turn;     		//平衡倾角 平衡陀螺仪 转向陀螺仪 //Balance tilt angle balance gyroscope steering gyroscope
int Motor_Left,Motor_Right;                 	  		//电机PWM变量 //Motor PWM variable
int Temperature;                                		//温度变量 		//Temperature variable
float Acceleration_Z;                           		//Z轴加速度计  //Z-axis accelerometer
int Mid_Angle;                          						//机械中值  //Mechanical median
float Move_X,Move_Z; //Move_X:前进速度  Move_Z：转向速度  //Move_X: Forward speed Move_Z: Steering speed
u8 Stop_Flag = 1; //0:开始 1:停止  //0: Start 1: Stop


char showbuf[20]={'\0'};


extern u8 newLineReceived;//蓝牙接收 //Bluetooth reception

extern u8 bulettohflag;
extern u8 ProtocolString[80];


static void RouteA_ServiceBluetooth(void)
{
    static char frame[80];
    static u8 length = 0, receiving = 0;
    u8 value;
    while(UART5_RxRead(&value))
    {
        if(value == 0) { receiving = 0; length = 0; continue; }
        if(value == '$') { receiving = 1; length = 0; }
        if(!receiving) continue;
        if(length >= sizeof(frame) - 1) { receiving = 0; length = 0; continue; }
        frame[length++] = (char)value;
        if(value != '#') continue;
        frame[length] = 0;
        receiving = 0;
        memcpy(ProtocolString, frame, length + 1);
        if(RouteA_IsFrame(ProtocolString))
        {
            RouteA_HandleFrame((char *)ProtocolString);
            newLineReceived = 0;
            memset(ProtocolString, 0, sizeof(ProtocolString));
        }
        else if(frame[1] >= '0' && frame[1] <= '4' && frame[2] == ',' &&
                frame[3] >= '0' && frame[3] <= '2' && frame[4] == ',' &&
                !strncmp(frame + 5, "0,0,0,0,0,", 10))
        {
            Protocol();
        }
        newLineReceived = 0;
        memset(ProtocolString, 0, sizeof(ProtocolString));
    }
}
int main(void)
{
    bsp_init();
    Mode_select();
    if(mode == Test_Mode) RouteA_Init();
    bsp_mode_init();
    MPU6050_EXTI_Init();
    OLED_Draw_Line("put down key start!", 2, false, true);
    if(mode == Test_Mode)
    {
        while(!Key1_State(1) && Stop_Flag == 1)
        {
            RouteA_ServiceBluetooth();
            if(bulettohflag == 1)
            {
                bulettohflag = 0;
                RouteA_Task();
            }
        }
    }
    else
    {
        while(!Key1_State(1) && Stop_Flag == 1);
    }
    Stop_Flag = 0;
    if(mode == Test_Mode) RouteA_BalanceStarted();
    OLED_Draw_Line("start control!        ", 2, false, true);
    while(1)
    {
        if(mode == Test_Mode)
        {
            RouteA_ServiceBluetooth();
            if(bulettohflag == 1)
            {
                bulettohflag = 0;
                RouteA_Task();
            }
            sprintf(showbuf,"TEST A=%d       ",(int)Angle_Balance);
        }
        else
        {
            if(newLineReceived)
            {
                ProtocolCpyData();
                Protocol();
            }
            if(bulettohflag == 1)
            {
                bulettohflag = 0;
                SendAutoUp();
            }
            sprintf(showbuf,"dis =%d mm   ",g_distance);
        }
        OLED_Draw_Line(showbuf, 3, false, true);
    }
}
