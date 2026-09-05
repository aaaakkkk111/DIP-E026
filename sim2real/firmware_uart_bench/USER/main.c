/* UART acceptance firmware. Motor inputs stay low. No balance task runs. */
#include "stm32f10x.h"
#include "pc_link.h"

int main(void)
{
    GPIO_InitTypeDef gpio;
    SystemCoreClockUpdate();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_ResetBits(GPIOC, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &gpio);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    pc_link_init(115200);
    SysTick_Config(SystemCoreClock / 1000U);
    for (;;) {
        pc_link_poll();
    }
}
