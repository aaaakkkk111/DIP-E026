/* Bench only: RX interrupt ring, TX DMA, framed PING/INFO. No motor commands. */
#include "stm32f10x.h"
#include "pc_link.h"
#include <string.h>

#define RX_SIZE 512U
#define FRAME_SIZE 78U
volatile uint32_t pc_link_ms;
static volatile uint8_t rx[RX_SIZE];
static volatile uint16_t head, tail;
static volatile uint32_t rx_overflow, uart_errors;
static uint32_t crc_errors, last_byte_ms;
static uint8_t frame[FRAME_SIZE], tx[FRAME_SIZE], tx_active;
static uint16_t used;

static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint16_t crc16(const uint8_t *p, uint16_t n) {
    uint16_t crc = 0xffffU;
    uint8_t bit;
    while (n--) {
        crc ^= (uint16_t)*p++ << 8;
        for (bit = 0; bit < 8; bit++)
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000U) ? 0x1021U : 0U));
    }
    return crc;
}

void pc_link_init(uint32_t baud)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef irq;
    DMA_InitTypeDef dma;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);
    USART_StructInit(&uart);
    uart.USART_BaudRate = baud;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &uart);
    irq.NVIC_IRQChannel = USART1_IRQn;
    irq.NVIC_IRQChannelPreemptionPriority = 2;
    irq.NVIC_IRQChannelSubPriority = 0;
    irq.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&irq);
    DMA_DeInit(DMA1_Channel4);
    DMA_StructInit(&dma);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    dma.DMA_MemoryBaseAddr = (uint32_t)tx;
    dma.DMA_DIR = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize = FRAME_SIZE;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_Medium;
    DMA_Init(DMA1_Channel4, &dma);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void)
{
    uint32_t status = USART1->SR;
    uint8_t byte;
    uint16_t next;
    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        byte = (uint8_t)USART1->DR; /* SR then DR clears the error condition. */
        if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
            uart_errors++;
            return;
        }
        if (!(status & USART_SR_RXNE)) return;
        next = (uint16_t)((head + 1U) & (RX_SIZE - 1U));
        if (next == tail) { rx_overflow++; return; }
        rx[head] = byte;
        head = next;
    }
}

static void shift(void) {
    used--;
    memmove(frame, frame + 1, used);
}

static void reply(uint8_t kind, uint16_t seq, const uint8_t *payload, uint16_t length)
{
    uint16_t total = (uint16_t)(14U + length);
    tx[0] = 0xaa; tx[1] = 0x55; tx[2] = 1; tx[3] = kind;
    put16(tx + 4, seq); put16(tx + 6, length);
    put32(tx + 8, pc_link_ms * 1000U); /* Bench timestamp has 1 ms resolution. */
    if (length) memcpy(tx + 12, payload, length);
    put16(tx + 12 + length, crc16(tx + 2, (uint16_t)(10U + length)));
    DMA_ClearFlag(DMA1_FLAG_GL4);
    DMA_SetCurrDataCounter(DMA1_Channel4, total);
    tx_active = 1;
    DMA_Cmd(DMA1_Channel4, ENABLE);
}

void pc_link_poll(void)
{
    uint16_t length, total, seq;
    uint8_t kind, info[20];
    if (tx_active) {
        if (DMA_GetCurrDataCounter(DMA1_Channel4) != 0) return;
        DMA_Cmd(DMA1_Channel4, DISABLE);
        tx_active = 0;
    }
    if (used && (uint32_t)(pc_link_ms - last_byte_ms) > 50U) used = 0;
    while (tail != head) {
        frame[used++] = rx[tail];
        tail = (uint16_t)((tail + 1U) & (RX_SIZE - 1U));
        last_byte_ms = pc_link_ms;
        while (used && frame[0] != 0xaa) shift();
        while (used >= 2U && (frame[0] != 0xaa || frame[1] != 0x55)) shift();
        if (used < 12U) continue;
        length = get16(frame + 6);
        if (frame[2] != 1 || length > 64U) { shift(); continue; }
        total = (uint16_t)(14U + length);
        if (used < total) continue;
        if (crc16(frame + 2, (uint16_t)(10U + length)) != get16(frame + 12 + length)) {
            crc_errors++; shift(); continue;
        }
        seq = get16(frame + 4); kind = frame[3];
        if (kind == 1) {
            reply(0x81, seq, frame + 12, length);
        } else if (kind == 2 && length == 0) {
            memcpy(info, "S2RB", 4);
            put32(info + 4, 1); put32(info + 8, rx_overflow);
            put32(info + 12, uart_errors); put32(info + 16, crc_errors);
            reply(0x82, seq, info, 20);
        }
        /* Normally used == total; retain any bytes after a recovered frame. */
        used = (uint16_t)(used - total);
        memmove(frame, frame + total, used);
        if (tx_active) return;
    }
}
