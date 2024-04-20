#include "V8.h"
#include "stdio.h"
#include "usart.h"
#include "stdio.h"
#include "tim.h"

// cpuΪС��ģʽ�洢��Ҳ�����ڴ洢��ʱ�򣬵�λ������0�ֽڣ���λ��1�ֽ�
#define BYTE0(dwTemp) (*(char *)(&dwTemp))		 // ȡ��int�ͱ����ĵ��ֽ�
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1)) //	ȡ�洢�ڴ˱�����һ�ڴ��ֽڵ����ݣ����ֽ�
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

void Sent_data_v8(float buff)
{
	int i;
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
	uint8_t _cnt = 0;
	uint16_t flen;
	uint8_t Buff[12];

	Buff[_cnt++] = 0xAB; // ֡ͷ
	Buff[_cnt++] = 0xFD; // Դ��ַ �����⣩
	Buff[_cnt++] = 0xFE; // Ŀ���ַ ����λ����
	Buff[_cnt++] = 0XF2; // ������
	Buff[_cnt++] = 0x04; // ���ݳ��� ��λ
	Buff[_cnt++] = 0x00; // ���ݳ��� ��λ
	flen = Buff[4] + (Buff[5] << 8);

	Buff[_cnt++] = BYTE0(buff); // ��������,С��ģʽ����λ��ǰ
	Buff[_cnt++] = BYTE1(buff); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	Buff[_cnt++] = BYTE2(buff);
	Buff[_cnt++] = BYTE3(buff);

	// SC��AC��У��
	for (i = 0; i < (flen + 6); i++)
	{
		sumcheck += Buff[i];  // ��֡ͷ��ʼ����ÿһ�ֽڽ�����ͣ�ֱ��DATA������
		addcheck += sumcheck; // ÿһ�ֽڵ���Ͳ���������һ��sumcheck���ۼ�
	}
	Buff[_cnt++] = sumcheck;
	Buff[_cnt++] = addcheck;

    HAL_UART_Transmit_DMA(&huart1, Buff, 12);   // DMA ��ʽ��������
    HAL_Delay(10);
}

// QT
void Sent_Data_DMA_Qt(float data1, float data2, uint8_t len)
{
	int i;
	uint8_t sumcheck = 0;
	uint8_t _cnt = 0;
	uint8_t Buff[20];

	Buff[_cnt++] = 0x3A; // ֡ͷ 3A
	Buff[_cnt++] = 0x3B; // ֡ͷ 3B
	Buff[_cnt++] = 0x01; // ������
	Buff[_cnt++] = len; // ��Ч�ֳ���

	Buff[_cnt++] = BYTE0(data1); // ��������,С��ģʽ����λ��ǰ
	Buff[_cnt++] = BYTE1(data1); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	Buff[_cnt++] = BYTE2(data1);
	Buff[_cnt++] = BYTE3(data1);

	Buff[_cnt++] = BYTE0(data2); // ��������,С��ģʽ����λ��ǰ
	Buff[_cnt++] = BYTE1(data2); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	Buff[_cnt++] = BYTE2(data2);
	Buff[_cnt++] = BYTE3(data2);

	// SC��AC��У��
	for (i = 0; i < (len+4); i++)
	{
		sumcheck += Buff[i];  // ��֡ͷ��ʼ����ÿһ�ֽڽ�����ͣ�ֱ��DATA������
	}
	Buff[_cnt++] = sumcheck;

    HAL_UART_Transmit_DMA(&huart1, Buff, _cnt);   // DMA ��ʽ��������
    HAL_Delay(20);

}


void max3010x_plot_ppg(
		float buff,
		float buff1,
		float buff2,
		float buff3,
		float buff4,
		float buff5,
		float buff6,
		float buff7,
		float buff8,
		float buff9,
		float buff10,
		float buff11)
{
	int i;
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
	uint8_t _cnt = 0;
	uint16_t flen = 0;
	uint8_t Buff[56] = {0};

	Buff[_cnt++] = 0xAB; // ֡ͷ
	Buff[_cnt++] = 0xFD; // Դ��ַ �����⣩
	Buff[_cnt++] = 0xFE; // Ŀ���ַ ����λ����
	Buff[_cnt++] = 0XF1; // ������
	Buff[_cnt++] = 0x30; // ���ݳ��� ��λ
	Buff[_cnt++] = 0x00; // ���ݳ��� ��λ
	flen = Buff[4] + (Buff[5] << 8);

	Buff[_cnt++] = BYTE0(buff); // ��������,С��ģʽ����λ��ǰ
	Buff[_cnt++] = BYTE1(buff); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	Buff[_cnt++] = BYTE2(buff);
	Buff[_cnt++] = BYTE3(buff);

	Buff[_cnt++] = BYTE0(buff1); // ��������,С��ģʽ����λ��ǰ
	Buff[_cnt++] = BYTE1(buff1); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	Buff[_cnt++] = BYTE2(buff1);
	Buff[_cnt++] = BYTE3(buff1);

	Buff[_cnt++] = BYTE0(buff2); // ��������,С��ģʽ����λ��ǰ
	Buff[_cnt++] = BYTE1(buff2); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	Buff[_cnt++] = BYTE2(buff2);
	Buff[_cnt++] = BYTE3(buff2);

	Buff[_cnt++] = BYTE0(buff3); // ��������,С��ģʽ����λ��ǰ
	Buff[_cnt++] = BYTE1(buff3); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	Buff[_cnt++] = BYTE2(buff3);
	Buff[_cnt++] = BYTE3(buff3);

	Buff[_cnt++] = BYTE0(buff4); // ��������,С��ģʽ����λ��ǰ
	Buff[_cnt++] = BYTE1(buff4); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	Buff[_cnt++] = BYTE2(buff4);
	Buff[_cnt++] = BYTE3(buff4);

	Buff[_cnt++] = BYTE0(buff5); // ��������,С��ģʽ����λ��ǰ
	Buff[_cnt++] = BYTE1(buff5); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	Buff[_cnt++] = BYTE2(buff5);
	Buff[_cnt++] = BYTE3(buff5);

	Buff[_cnt++] = BYTE0(buff6); // ��������,С��ģʽ����λ��ǰ
	Buff[_cnt++] = BYTE1(buff6); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	Buff[_cnt++] = BYTE2(buff6);
	Buff[_cnt++] = BYTE3(buff6);

	Buff[_cnt++] = BYTE0(buff7); // ��������,С��ģʽ����λ��ǰ
	Buff[_cnt++] = BYTE1(buff7); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	Buff[_cnt++] = BYTE2(buff7);
	Buff[_cnt++] = BYTE3(buff7);

	Buff[_cnt++] = BYTE0(buff8); // ��������,С��ģʽ����λ��ǰ
	Buff[_cnt++] = BYTE1(buff8); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	Buff[_cnt++] = BYTE2(buff8);
	Buff[_cnt++] = BYTE3(buff8);

	Buff[_cnt++] = BYTE0(buff9); // ��������,С��ģʽ����λ��ǰ
	Buff[_cnt++] = BYTE1(buff9); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	Buff[_cnt++] = BYTE2(buff9);
	Buff[_cnt++] = BYTE3(buff9);

	Buff[_cnt++] = BYTE0(buff10); // ��������,С��ģʽ����λ��ǰ
	Buff[_cnt++] = BYTE1(buff10); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	Buff[_cnt++] = BYTE2(buff10);
	Buff[_cnt++] = BYTE3(buff10);

	Buff[_cnt++] = BYTE0(buff11); // ��������,С��ģʽ����λ��ǰ
	Buff[_cnt++] = BYTE1(buff11); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	Buff[_cnt++] = BYTE2(buff11);
	Buff[_cnt++] = BYTE3(buff11);

	// SC��AC��У��
	for (i = 0; i < (flen + 6); i++)
	{
		sumcheck += Buff[i];  // ��֡ͷ��ʼ����ÿһ�ֽڽ�����ͣ�ֱ��DATA������
		addcheck += sumcheck; // ÿһ�ֽڵ���Ͳ���������һ��sumcheck���ۼ�
	}
	Buff[_cnt++] = sumcheck;
	Buff[_cnt++] = addcheck;

    HAL_UART_Transmit_DMA(&huart1, Buff, 56);   // DMA ��ʽ��������
    HAL_Delay(10);

}


void max3010x_plot(uint32_t buff,uint32_t buff1) {

	int i;
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
	uint8_t _cnt = 0;
	uint16_t flen = 0;
	uint8_t Buff[16] = {0};

	Buff[_cnt++] = 0xAB; // ֡ͷ
	Buff[_cnt++] = 0xFD; // Դ��ַ �����⣩
	Buff[_cnt++] = 0xFE; // Ŀ���ַ ����λ����
	Buff[_cnt++] = 0XF1; // ������
	Buff[_cnt++] = 0x08; // ���ݳ��� ��λ
	Buff[_cnt++] = 0x00; // ���ݳ��� ��λ
	flen = Buff[4] + (Buff[5] << 8);

	Buff[_cnt++] = BYTE0(buff); // ��������,С��ģʽ����λ��ǰ
	Buff[_cnt++] = BYTE1(buff); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	Buff[_cnt++] = BYTE2(buff);
	Buff[_cnt++] = BYTE3(buff);

	Buff[_cnt++] = BYTE0(buff1); // ��������,С��ģʽ����λ��ǰ
	Buff[_cnt++] = BYTE1(buff1); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	Buff[_cnt++] = BYTE2(buff1);
	Buff[_cnt++] = BYTE3(buff1);

	// SC��AC��У��
	for (i = 0; i < (flen + 6); i++)
	{
		sumcheck += Buff[i];  // ��֡ͷ��ʼ����ÿһ�ֽڽ�����ͣ�ֱ��DATA������
		addcheck += sumcheck; // ÿһ�ֽڵ���Ͳ���������һ��sumcheck���ۼ�
	}
	Buff[_cnt++] = sumcheck;
	Buff[_cnt++] = addcheck;

    HAL_UART_Transmit_DMA(&huart1, Buff, 16);   // DMA ��ʽ��������
	delay_ms(2);

}


void max3010x_plot1(uint32_t buff,uint32_t buff1) {

	int i;
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
	uint8_t _cnt = 0;
	uint16_t flen = 0;
	uint8_t Buff[16] = {0};

	Buff[_cnt++] = 0xAB; // ֡ͷ
	Buff[_cnt++] = 0xFD; // Դ��ַ �����⣩
	Buff[_cnt++] = 0xFE; // Ŀ���ַ ����λ����
	Buff[_cnt++] = 0XF2; // ������
	Buff[_cnt++] = 0x08; // ���ݳ��� ��λ
	Buff[_cnt++] = 0x00; // ���ݳ��� ��λ
	flen = Buff[4] + (Buff[5] << 8);

	Buff[_cnt++] = BYTE0(buff); // ��������,С��ģʽ����λ��ǰ
	Buff[_cnt++] = BYTE1(buff); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	Buff[_cnt++] = BYTE2(buff);
	Buff[_cnt++] = BYTE3(buff);

	Buff[_cnt++] = BYTE0(buff1); // ��������,С��ģʽ����λ��ǰ
	Buff[_cnt++] = BYTE1(buff1); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	Buff[_cnt++] = BYTE2(buff1);
	Buff[_cnt++] = BYTE3(buff1);

	// SC��AC��У��
	for (i = 0; i < (flen + 6); i++)
	{
		sumcheck += Buff[i];  // ��֡ͷ��ʼ����ÿһ�ֽڽ�����ͣ�ֱ��DATA������
		addcheck += sumcheck; // ÿһ�ֽڵ���Ͳ���������һ��sumcheck���ۼ�
	}
	Buff[_cnt++] = sumcheck;
	Buff[_cnt++] = addcheck;

    HAL_UART_Transmit_DMA(&huart1, Buff, 16);   // DMA ��ʽ��������
    HAL_Delay(10);

}



