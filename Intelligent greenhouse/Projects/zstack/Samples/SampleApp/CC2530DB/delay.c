#include "delay.h"
#include "hal_timer.h"  // �����ʹ�� HAL �Ķ�ʱ�������԰��������������Ҫ

// �򵥵�us����ʱ������forѭ����CC2530���1��ѭ��1us������ݱ����Ż�������
void Delay_us(uint16 us)
{
    uint16 i;
    while (us--)
    {
        for (i = 0; i < 4; i++)  // �����ֵ���Ը���ʵ���ٶȵ���
        {
            asm("NOP");  // �ղ���ָ�ȷ����ʱ����ȷ
        }
    }
}

// ms����ʱ
void Delay_ms(uint16 ms)
{
    while (ms--)
    {
        Delay_us(1000);
    }
}
