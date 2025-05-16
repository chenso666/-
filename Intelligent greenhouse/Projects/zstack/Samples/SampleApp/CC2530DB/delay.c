#include "delay.h"
#include "hal_timer.h"  // 如果你使用 HAL 的定时器，可以包含这个，否则不需要

// 简单的us级延时，基于for循环，CC2530大概1个循环1us（需根据编译优化调整）
void Delay_us(uint16 us)
{
    uint16 i;
    while (us--)
    {
        for (i = 0; i < 4; i++)  // 这里的值可以根据实际速度调节
        {
            asm("NOP");  // 空操作指令，确保延时更精确
        }
    }
}

// ms级延时
void Delay_ms(uint16 ms)
{
    while (ms--)
    {
        Delay_us(1000);
    }
}
