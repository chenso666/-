#include "hal_types.h"
#include "adc.h"
#include "ioCC2530.h"
#include "delay.h"
#include "hal_uart.h" // 假设使用 UART 打印数据，包含此头文件

void ADC_Init(void)
{
    // P0_7 设为模拟功能，输入
    P0SEL |= 0x80;   // P0_7 设为模拟功能
    P0DIR &= ~0x80;  // P0_7 设为输入

    // 使能ADC并配置为单端模式，通道7（P0_7）
    ADCCFG |= 0x80;      // 配置 P0_7 为模拟输入
    APCFG |= 0x80;       // 配置端口为模拟功能

    // 配置ADC，确保使用单端模式
    ADCCON1 = 0x30;      // 控制方式：CPU控制（逐次启动）
    ADCCON3 = 0x87;      // 配置通道7为单端模式（通道7）

    // 确保ADC转换初始化
    ADCCON1 |= 0x40;     // 启动一次转换
    while (!(ADCCON1 & 0x80));  // 等待EOC位（转换结束标志）
}

uint16 ADC_Read(void)
{
    uint16 value = 0;

    // 启动ADC转换
    ADCCON1 |= 0x40;            // 启动转换（bit6 = 1）
    while (!(ADCCON1 & 0x80));  // 等待转换完成（bit7为EOC）

    // 读取ADC值，ADCL是低8位，ADCH是高4位
    value = ADCL;               // 先读取ADCL（低字节）
    value |= ((uint16)ADCH) << 8;  // 再读取ADCH（高字节）

    // 清除ADC状态位
    ADCCON1 &= ~0x80;            // 清除EOC位

    return value;               // 返回ADC数值（0~4095）
}

uint16 GetHumidity(uint8 times)
{
    uint32 total = 0;
    uint8 t;

    for (t = 0; t < times; t++)
    {
        total += ADC_Read();  // 读取 ADC 值
        Delay_ms(10);         // 10ms延时，确保稳定性
    }

    uint16 avg = total / times;  // 计算平均值
    char buf[50];
    sprintf(buf, "Avg ADC: %u\n", avg);
    HalUARTWrite(0, (uint8*)buf, strlen(buf));  // 打印平均 ADC 值到 UART

    return avg;
}
