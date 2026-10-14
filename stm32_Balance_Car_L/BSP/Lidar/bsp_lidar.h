#ifndef __BSP_LIDAR_H
#define __BSP_LIDAR_H


#include "AllHeader.h"


void USART3_init(u32 baudrate);
void USART3_Send_U8(uint8_t ch);
void USART3_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);

#endif

