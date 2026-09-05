#include "bsp_oled.h"

/* Write command */
#define SSD1306_WRITECOMMAND(command) I2C_WriteByte(0x00, (command))
/* Write data */
#define SSD1306_WRITEDATA(data) I2C_WriteByte(0x40, (data))
/* Absolute value */

/* SSD1306 data buffer */
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

typedef struct
{
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Inverted;
    uint8_t Initialized;
} SSD1306_t;

static SSD1306_t SSD1306;

//延时函数 Delay function
void IIC_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 1000;
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)
	{
		Timeout --;
		if (Timeout == 0)
		{
			break;
		}
	}
}

//向OLED寄存器地址写一个byte的数据 -死等的方案，不要 Write a byte of data to the OLED register address - wait for the solution, don't
//void I2C_WriteByte(uint8_t addr,uint8_t data)
//{
//		while( I2C_GetFlagStatus(OLED_I2C, I2C_FLAG_BUSY) );
//		I2C_GenerateSTART(OLED_I2C, ENABLE);
//		
//		while( !I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_MODE_SELECT) );
//		I2C_Send7bitAddress(OLED_I2C, OLED_ADDRESS, I2C_Direction_Transmitter);
//		
//		while( !I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) );
//		I2C_SendData(OLED_I2C, addr);
//		
//		while( !I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING) );
//		I2C_SendData(OLED_I2C, data);
//		
//		while( !I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//		I2C_GenerateSTOP(OLED_I2C, ENABLE);
//}

void I2C_WriteByte(uint8_t RegAddress, uint8_t Data)
{
	I2C_GenerateSTART(OLED_I2C, ENABLE);
	IIC_WaitEvent(OLED_I2C, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(OLED_I2C, OLED_ADDRESS, I2C_Direction_Transmitter);
	IIC_WaitEvent(OLED_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(OLED_I2C, RegAddress);
	IIC_WaitEvent(OLED_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	
	I2C_SendData(OLED_I2C, Data);
	IIC_WaitEvent(OLED_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTOP(OLED_I2C, ENABLE);
}


void OLED_Init(void)
{

    delay_ms(100);

    SSD1306_WRITECOMMAND(0xae); // display off
    SSD1306_WRITECOMMAND(0xa6); // Set Normal Display (default)
    SSD1306_WRITECOMMAND(0xAE); // DISPLAYOFF
    SSD1306_WRITECOMMAND(0xD5); // SETDISPLAYCLOCKDIV
    SSD1306_WRITECOMMAND(0x80); // the suggested ratio 0x80
    SSD1306_WRITECOMMAND(0xA8); // SSD1306_SETMULTIPLEX
    SSD1306_WRITECOMMAND(0x1F);
    SSD1306_WRITECOMMAND(0xD3);       // SETDISPLAYOFFSET
    SSD1306_WRITECOMMAND(0x00);       // no offset
    SSD1306_WRITECOMMAND(0x40 | 0x0); // SETSTARTLINE
    SSD1306_WRITECOMMAND(0x8D);       // CHARGEPUMP
    SSD1306_WRITECOMMAND(0x14);       // 0x014 enable, 0x010 disable
    SSD1306_WRITECOMMAND(0x20);       // com pin HW config, sequential com pin config (bit 4), disable left/right remap (bit 5),
    SSD1306_WRITECOMMAND(0x02);       // 0x12 //128x32 OLED: 0x002,  128x32 OLED 0x012
    SSD1306_WRITECOMMAND(0xa1);       // segment remap a0/a1
    SSD1306_WRITECOMMAND(0xc8);       // c0: scan dir normal, c8: reverse
    SSD1306_WRITECOMMAND(0xda);
    SSD1306_WRITECOMMAND(0x02); // com pin HW config, sequential com pin config (bit 4), disable left/right remap (bit 5)
    SSD1306_WRITECOMMAND(0x81);
    SSD1306_WRITECOMMAND(0xcf); // [2] set contrast control
    SSD1306_WRITECOMMAND(0xd9);
    SSD1306_WRITECOMMAND(0xf1); // [2] pre-charge period 0x022/f1
    SSD1306_WRITECOMMAND(0xdb);
    SSD1306_WRITECOMMAND(0x40); // vcomh deselect level
    SSD1306_WRITECOMMAND(0x2e); // Disable scroll
    SSD1306_WRITECOMMAND(0xa4); // output ram to display
    SSD1306_WRITECOMMAND(0xa6); // none inverted normal display mode
    SSD1306_WRITECOMMAND(0xaf); // display on

    /* Clear screen */
    SSD1306_Fill(SSD1306_COLOR_BLACK);

    /* Update screen */
    SSD1306_UpdateScreen();

    /* Set default values */
    SSD1306.CurrentX = 0;
    SSD1306.CurrentY = 0;

    /* Initialized OK */
    SSD1306.Initialized = 1;
}

void SSD1306_UpdateScreen(void)
{
    uint8_t m, n;

    for (m = 0; m < 8; m++)
    {
        SSD1306_WRITECOMMAND(0xB0 + m);
        SSD1306_WRITECOMMAND(0x00);
        SSD1306_WRITECOMMAND(0x10);

        for (n = 0; n < SSD1306_WIDTH; n++)
        {
            SSD1306_WRITEDATA(SSD1306_Buffer[n + SSD1306_WIDTH * m]);
        }
    }
}

void SSD1306_Fill(SSD1306_COLOR_t color)
{
    /* Set memory */
    memset(SSD1306_Buffer, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer));
}

void SSD1306_DrawPixel(uint16_t x, uint16_t y, SSD1306_COLOR_t color)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
    {
        return; // 出错，超出范围 Error, out of range
    }

    /* 检查像素是否倒置
     * Check if pixel is inverted
     * */

    if (SSD1306.Inverted)
    {
        color = (SSD1306_COLOR_t)!color;
    }

    /* 设置颜色
     *
     * Set color
     *  */

    if (color == SSD1306_COLOR_WHITE)
    {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
    }
    else
    {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

void SSD1306_GotoXY(uint16_t x, uint16_t y)
{
    SSD1306.CurrentX = x;
    SSD1306.CurrentY = y;
}

char SSD1306_Putc(char ch, FontDef_t *Font, SSD1306_COLOR_t color)
{
    uint32_t i, b, j;

    if (
        SSD1306_WIDTH <= (SSD1306.CurrentX + Font->FontWidth) ||
        SSD1306_HEIGHT <= (SSD1306.CurrentY + Font->FontHeight))
    {
        return 0; // 出错，超出范围 Error, out of range
    }

    for (i = 0; i < Font->FontHeight; i++)
    {
        b = Font->data[(ch - 32) * Font->FontHeight + i];
        for (j = 0; j < Font->FontWidth; j++)
        {
            if ((b << j) & 0x8000)
            {
                SSD1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR_t)color);
            }
            else
            {
                SSD1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR_t)!color);
            }
        }
    }

    /* Increase pointer */
    SSD1306.CurrentX += Font->FontWidth;

    /* Return character written */
    return ch;
}

char SSD1306_Puts(char *str, FontDef_t *Font, SSD1306_COLOR_t color)
{
    /* Write characters */
    while (*str)
    {
        /* Write character by character */
        if (SSD1306_Putc(*str, Font, color) != *str)
        {
            /* Return error */
            return *str;
        }

        /* Increase string pointer */
        str++;
    }

    /* Everything OK, zero should be returned */
    return *str;
}

// 将OLED从休眠中唤醒 Wake OLED from sleep
void OLED_ON(void)
{
    SSD1306_WRITECOMMAND(0x8D);
    SSD1306_WRITECOMMAND(0x14);
    SSD1306_WRITECOMMAND(0xAF);
}

// 让OLED休眠 -- 休眠模式下,OLED功耗不到10uA Let OLED sleep -- In sleep mode, OLED power consumption is less than 10uA
void OLED_OFF(void)
{
    SSD1306_WRITECOMMAND(0x8D);
    SSD1306_WRITECOMMAND(0x10);
    SSD1306_WRITECOMMAND(0xAE);
}

void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1306_COLOR_t c)
{
    int16_t dx, dy, sx, sy, err, e2, i, tmp;

    /* Check for overflow */
    if (x0 >= SSD1306_WIDTH)
    {
        x0 = SSD1306_WIDTH - 1;
    }
    if (x1 >= SSD1306_WIDTH)
    {
        x1 = SSD1306_WIDTH - 1;
    }
    if (y0 >= SSD1306_HEIGHT)
    {
        y0 = SSD1306_HEIGHT - 1;
    }
    if (y1 >= SSD1306_HEIGHT)
    {
        y1 = SSD1306_HEIGHT - 1;
    }

    dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
    dy = (y0 < y1) ? (y1 - y0) : (y0 - y1);
    sx = (x0 < x1) ? 1 : -1;
    sy = (y0 < y1) ? 1 : -1;
    err = ((dx > dy) ? dx : -dy) / 2;

    if (dx == 0)
    {
        if (y1 < y0)
        {
            tmp = y1;
            y1 = y0;
            y0 = tmp;
        }

        if (x1 < x0)
        {
            tmp = x1;
            x1 = x0;
            x0 = tmp;
        }

        /* Vertical line */
        for (i = y0; i <= y1; i++)
        {
            SSD1306_DrawPixel(x0, i, c);
        }

        /* Return from function */
        return;
    }

    if (dy == 0)
    {
        if (y1 < y0)
        {
            tmp = y1;
            y1 = y0;
            y0 = tmp;
        }

        if (x1 < x0)
        {
            tmp = x1;
            x1 = x0;
            x0 = tmp;
        }

        /* Horizontal line */
        for (i = x0; i <= x1; i++)
        {
            SSD1306_DrawPixel(i, y0, c);
        }

        /* Return from function */
        return;
    }

    while (1)
    {
        SSD1306_DrawPixel(x0, y0, c);
        if (x0 == x1 && y0 == y1)
        {
            break;
        }
        e2 = err;
        if (e2 > -dx)
        {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dy)
        {
            err += dx;
            y0 += sy;
        }
    }
}

/* OLED清除屏幕
 *
 * OLED clear screen
 *  */

void OLED_Clear(void)
{
    SSD1306_Fill(SSD1306_COLOR_BLACK);
}

/* 刷新OLED屏幕
 *
 * Refresh OLED screen
 * */

void OLED_Refresh(void)
{
    SSD1306_UpdateScreen();
}

/* 写入字符
 *
 * Write characters
 *  */

void OLED_Draw_String(char *data, uint8_t x, uint8_t y, bool clear, bool refresh)
{
    if (clear)
        OLED_Clear();
    SSD1306_GotoXY(x, y);
    SSD1306_Puts(data, &Font_7x10, SSD1306_COLOR_WHITE);
    if (refresh)
        OLED_Refresh();
}

/* 写入一行字符
 *
 * Write a line of characters
 *  */

void OLED_Draw_Line(char *data, uint8_t line, bool clear, bool refresh)
{
    if (line > 0 && line <= 3)
    {
        OLED_Draw_String(data, 0, 10 * (line - 1), clear, refresh);
    }
}
