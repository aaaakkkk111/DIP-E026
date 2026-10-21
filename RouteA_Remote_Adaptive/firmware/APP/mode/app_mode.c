#include "app_mode.h"

uint8_t angle_max = 40;
Car_Mode mode = Normal;

void Mode_select(void)
{
    int16_t mode_cnt = 0;
    OLED_Draw_Line("1.Standard Mode", 1, true, true);
    while(!Key1_State(1))
    {
        mode_cnt += Read_Encoder(MOTOR_ID_ML);
        mode_cnt += -Read_Encoder(MOTOR_ID_MR);
        car_mode(mode_cnt);
        show_mode_oled();
    }
    Set_Mid_Angle();
    Set_angle();
    Set_control_speed();
    Set_PID();
}

void car_mode(int16_t cnt)
{
    static int16_t cnt_old = 0;
    static u8 index = 0;
    static const Car_Mode supported[SUPPORTED_MODE_COUNT] = {Normal, Weight_M, Test_Mode};
    int delta = (int)cnt - (int)cnt_old;
    if(myabs(delta) > 250)
    {
        if(delta < 0) index = index == 0 ? SUPPORTED_MODE_COUNT-1 : index-1;
        else index = (u8)((index+1) % SUPPORTED_MODE_COUNT);
        mode = supported[index];
        cnt_old = cnt;
    }
}

void Set_Mid_Angle(void) { Mid_Angle = 0; }
void Set_angle(void) { angle_max = 40; }
void Set_control_speed(void)
{
    Car_Target_Velocity = 30;
    Car_Turn_Amplitude_speed = 36;
}

extern float Balance_Kp, Balance_Kd, Velocity_Kp, Velocity_Ki, Turn_Kp, Turn_Kd;
void Set_PID(void)
{
    Balance_Kp = 9600;
    Turn_Kd = 20;
    if(mode == Weight_M)
    {
        Balance_Kd = 75;
        Velocity_Kp = 7000;
        Velocity_Ki = 35;
        Turn_Kp = 1400;
    }
    else
    {
        Balance_Kd = 48;
        Velocity_Kp = 6200;
        Velocity_Ki = 31;
        Turn_Kp = 1700;
    }
}
