#include "oled_show.h"

void show_mode_oled(void)
{
    static uint8_t mode_old = 0xFF;
    if((uint8_t)mode == mode_old) return;
    mode_old = (uint8_t)mode;
    switch(mode)
    {
        case Normal: OLED_Draw_Line("1.Standard Mode", 1, true, true); break;
        case Weight_M: OLED_Draw_Line("2.Load Mode", 1, true, true); break;
        case Test_Mode: OLED_Draw_Line("3.TEST MODE", 1, true, true); break;
        default: break;
    }
}
