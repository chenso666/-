#include "hal_types.h"
#include "adc.h"
#include "ioCC2530.h"
#include "delay.h"
#include "hal_uart.h" // ����ʹ�� UART ��ӡ���ݣ�������ͷ�ļ�

void ADC_Init(void)
{
    // P0_7 ��Ϊģ�⹦�ܣ�����
    P0SEL |= 0x80;   // P0_7 ��Ϊģ�⹦��
    P0DIR &= ~0x80;  // P0_7 ��Ϊ����

    // ʹ��ADC������Ϊ����ģʽ��ͨ��7��P0_7��
    ADCCFG |= 0x80;      // ���� P0_7 Ϊģ������
    APCFG |= 0x80;       // ���ö˿�Ϊģ�⹦��

    // ����ADC��ȷ��ʹ�õ���ģʽ
    ADCCON1 = 0x30;      // ���Ʒ�ʽ��CPU���ƣ����������
    ADCCON3 = 0x87;      // ����ͨ��7Ϊ����ģʽ��ͨ��7��

    // ȷ��ADCת����ʼ��
    ADCCON1 |= 0x40;     // ����һ��ת��
    while (!(ADCCON1 & 0x80));  // �ȴ�EOCλ��ת��������־��
}

uint16 ADC_Read(void)
{
    uint16 value = 0;

    // ����ADCת��
    ADCCON1 |= 0x40;            // ����ת����bit6 = 1��
    while (!(ADCCON1 & 0x80));  // �ȴ�ת����ɣ�bit7ΪEOC��

    // ��ȡADCֵ��ADCL�ǵ�8λ��ADCH�Ǹ�4λ
    value = ADCL;               // �ȶ�ȡADCL�����ֽڣ�
    value |= ((uint16)ADCH) << 8;  // �ٶ�ȡADCH�����ֽڣ�

    // ���ADC״̬λ
    ADCCON1 &= ~0x80;            // ���EOCλ

    return value;               // ����ADC��ֵ��0~4095��
}

uint16 GetHumidity(uint8 times)
{
    uint32 total = 0;
    uint8 t;

    for (t = 0; t < times; t++)
    {
        total += ADC_Read();  // ��ȡ ADC ֵ
        Delay_ms(10);         // 10ms��ʱ��ȷ���ȶ���
    }

    uint16 avg = total / times;  // ����ƽ��ֵ
    char buf[50];
    sprintf(buf, "Avg ADC: %u\n", avg);
    HalUARTWrite(0, (uint8*)buf, strlen(buf));  // ��ӡƽ�� ADC ֵ�� UART

    return avg;
}
