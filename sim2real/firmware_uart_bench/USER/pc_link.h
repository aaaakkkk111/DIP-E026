#ifndef PC_LINK_H
#define PC_LINK_H
#include <stdint.h>
extern volatile uint32_t pc_link_ms;
void pc_link_init(uint32_t baud);
void pc_link_poll(void);
#endif
