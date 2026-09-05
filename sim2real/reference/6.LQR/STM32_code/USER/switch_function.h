//此文件是一个功能编译文件 0:不参与编译  1:参与编译

#ifndef __SWITCH_FUNCTION_H_
#define __SWITCH_FUNCTION_H_


#define DEBUG_USARTx USART1

//以下两者不能同时使用
#define IR_TRACK_SWITCH 0  //4路循迹开关
#define ELE_SWITCH      0  //电磁巡线开关 
#define CCD_SWITCH      0 //CCD开关

#define K210_SWITCH     0 //k210初始化.

//使用雷达注意事项
//1.把不用的外设都给释放掉 特别是超声波和定时器 减小cpu负担
//2.雷达数据处理需要在main函数里面处理吧，减少中断占用时间
#define Lidar_SWITCH    0 //雷达开关

#endif

