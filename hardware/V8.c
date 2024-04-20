#include "V8.h"
#include "stdio.h"
#include "usart.h"
#include "stdio.h"
#include "tim.h"

// cpu为小端模式存储，也就是在存储的时候，低位被存在0字节，高位在1字节
#define BYTE0(dwTemp) (*(char *)(&dwTemp))		 // 取出int型变量的低字节
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1)) //	取存储在此变量下一内存字节的内容，高字节
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

	Buff[_cnt++] = 0xAB; // 帧头
	Buff[_cnt++] = 0xFD; // 源地址 （随意）
	Buff[_cnt++] = 0xFE; // 目标地址 （上位机）
	Buff[_cnt++] = 0XF2; // 功能码
	Buff[_cnt++] = 0x04; // 数据长度 低位
	Buff[_cnt++] = 0x00; // 数据长度 高位
	flen = Buff[4] + (Buff[5] << 8);

	Buff[_cnt++] = BYTE0(buff); // 数据内容,小段模式，低位在前
	Buff[_cnt++] = BYTE1(buff); // 需要将字节进行拆分，调用上面的宏定义即可。
	Buff[_cnt++] = BYTE2(buff);
	Buff[_cnt++] = BYTE3(buff);

	// SC和AC的校验
	for (i = 0; i < (flen + 6); i++)
	{
		sumcheck += Buff[i];  // 从帧头开始，对每一字节进行求和，直到DATA区结束
		addcheck += sumcheck; // 每一字节的求和操作，进行一次sumcheck的累加
	}
	Buff[_cnt++] = sumcheck;
	Buff[_cnt++] = addcheck;

    HAL_UART_Transmit_DMA(&huart1, Buff, 12);   // DMA 方式发送数据
    HAL_Delay(10);
}

// QT
void Sent_Data_DMA_Qt(float data1, float data2, uint8_t len)
{
	int i;
	uint8_t sumcheck = 0;
	uint8_t _cnt = 0;
	uint8_t Buff[20];

	Buff[_cnt++] = 0x3A; // 帧头 3A
	Buff[_cnt++] = 0x3B; // 帧头 3B
	Buff[_cnt++] = 0x01; // 功能码
	Buff[_cnt++] = len; // 有效字长度

	Buff[_cnt++] = BYTE0(data1); // 数据内容,小段模式，低位在前
	Buff[_cnt++] = BYTE1(data1); // 需要将字节进行拆分，调用上面的宏定义即可。
	Buff[_cnt++] = BYTE2(data1);
	Buff[_cnt++] = BYTE3(data1);

	Buff[_cnt++] = BYTE0(data2); // 数据内容,小段模式，低位在前
	Buff[_cnt++] = BYTE1(data2); // 需要将字节进行拆分，调用上面的宏定义即可。
	Buff[_cnt++] = BYTE2(data2);
	Buff[_cnt++] = BYTE3(data2);

	// SC和AC的校验
	for (i = 0; i < (len+4); i++)
	{
		sumcheck += Buff[i];  // 从帧头开始，对每一字节进行求和，直到DATA区结束
	}
	Buff[_cnt++] = sumcheck;

    HAL_UART_Transmit_DMA(&huart1, Buff, _cnt);   // DMA 方式发送数据
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

	Buff[_cnt++] = 0xAB; // 帧头
	Buff[_cnt++] = 0xFD; // 源地址 （随意）
	Buff[_cnt++] = 0xFE; // 目标地址 （上位机）
	Buff[_cnt++] = 0XF1; // 功能码
	Buff[_cnt++] = 0x30; // 数据长度 低位
	Buff[_cnt++] = 0x00; // 数据长度 高位
	flen = Buff[4] + (Buff[5] << 8);

	Buff[_cnt++] = BYTE0(buff); // 数据内容,小段模式，低位在前
	Buff[_cnt++] = BYTE1(buff); // 需要将字节进行拆分，调用上面的宏定义即可。
	Buff[_cnt++] = BYTE2(buff);
	Buff[_cnt++] = BYTE3(buff);

	Buff[_cnt++] = BYTE0(buff1); // 数据内容,小段模式，低位在前
	Buff[_cnt++] = BYTE1(buff1); // 需要将字节进行拆分，调用上面的宏定义即可。
	Buff[_cnt++] = BYTE2(buff1);
	Buff[_cnt++] = BYTE3(buff1);

	Buff[_cnt++] = BYTE0(buff2); // 数据内容,小段模式，低位在前
	Buff[_cnt++] = BYTE1(buff2); // 需要将字节进行拆分，调用上面的宏定义即可。
	Buff[_cnt++] = BYTE2(buff2);
	Buff[_cnt++] = BYTE3(buff2);

	Buff[_cnt++] = BYTE0(buff3); // 数据内容,小段模式，低位在前
	Buff[_cnt++] = BYTE1(buff3); // 需要将字节进行拆分，调用上面的宏定义即可。
	Buff[_cnt++] = BYTE2(buff3);
	Buff[_cnt++] = BYTE3(buff3);

	Buff[_cnt++] = BYTE0(buff4); // 数据内容,小段模式，低位在前
	Buff[_cnt++] = BYTE1(buff4); // 需要将字节进行拆分，调用上面的宏定义即可。
	Buff[_cnt++] = BYTE2(buff4);
	Buff[_cnt++] = BYTE3(buff4);

	Buff[_cnt++] = BYTE0(buff5); // 数据内容,小段模式，低位在前
	Buff[_cnt++] = BYTE1(buff5); // 需要将字节进行拆分，调用上面的宏定义即可。
	Buff[_cnt++] = BYTE2(buff5);
	Buff[_cnt++] = BYTE3(buff5);

	Buff[_cnt++] = BYTE0(buff6); // 数据内容,小段模式，低位在前
	Buff[_cnt++] = BYTE1(buff6); // 需要将字节进行拆分，调用上面的宏定义即可。
	Buff[_cnt++] = BYTE2(buff6);
	Buff[_cnt++] = BYTE3(buff6);

	Buff[_cnt++] = BYTE0(buff7); // 数据内容,小段模式，低位在前
	Buff[_cnt++] = BYTE1(buff7); // 需要将字节进行拆分，调用上面的宏定义即可。
	Buff[_cnt++] = BYTE2(buff7);
	Buff[_cnt++] = BYTE3(buff7);

	Buff[_cnt++] = BYTE0(buff8); // 数据内容,小段模式，低位在前
	Buff[_cnt++] = BYTE1(buff8); // 需要将字节进行拆分，调用上面的宏定义即可。
	Buff[_cnt++] = BYTE2(buff8);
	Buff[_cnt++] = BYTE3(buff8);

	Buff[_cnt++] = BYTE0(buff9); // 数据内容,小段模式，低位在前
	Buff[_cnt++] = BYTE1(buff9); // 需要将字节进行拆分，调用上面的宏定义即可。
	Buff[_cnt++] = BYTE2(buff9);
	Buff[_cnt++] = BYTE3(buff9);

	Buff[_cnt++] = BYTE0(buff10); // 数据内容,小段模式，低位在前
	Buff[_cnt++] = BYTE1(buff10); // 需要将字节进行拆分，调用上面的宏定义即可。
	Buff[_cnt++] = BYTE2(buff10);
	Buff[_cnt++] = BYTE3(buff10);

	Buff[_cnt++] = BYTE0(buff11); // 数据内容,小段模式，低位在前
	Buff[_cnt++] = BYTE1(buff11); // 需要将字节进行拆分，调用上面的宏定义即可。
	Buff[_cnt++] = BYTE2(buff11);
	Buff[_cnt++] = BYTE3(buff11);

	// SC和AC的校验
	for (i = 0; i < (flen + 6); i++)
	{
		sumcheck += Buff[i];  // 从帧头开始，对每一字节进行求和，直到DATA区结束
		addcheck += sumcheck; // 每一字节的求和操作，进行一次sumcheck的累加
	}
	Buff[_cnt++] = sumcheck;
	Buff[_cnt++] = addcheck;

    HAL_UART_Transmit_DMA(&huart1, Buff, 56);   // DMA 方式发送数据
    HAL_Delay(10);

}


void max3010x_plot(uint32_t buff,uint32_t buff1) {

	int i;
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
	uint8_t _cnt = 0;
	uint16_t flen = 0;
	uint8_t Buff[16] = {0};

	Buff[_cnt++] = 0xAB; // 帧头
	Buff[_cnt++] = 0xFD; // 源地址 （随意）
	Buff[_cnt++] = 0xFE; // 目标地址 （上位机）
	Buff[_cnt++] = 0XF1; // 功能码
	Buff[_cnt++] = 0x08; // 数据长度 低位
	Buff[_cnt++] = 0x00; // 数据长度 高位
	flen = Buff[4] + (Buff[5] << 8);

	Buff[_cnt++] = BYTE0(buff); // 数据内容,小段模式，低位在前
	Buff[_cnt++] = BYTE1(buff); // 需要将字节进行拆分，调用上面的宏定义即可。
	Buff[_cnt++] = BYTE2(buff);
	Buff[_cnt++] = BYTE3(buff);

	Buff[_cnt++] = BYTE0(buff1); // 数据内容,小段模式，低位在前
	Buff[_cnt++] = BYTE1(buff1); // 需要将字节进行拆分，调用上面的宏定义即可。
	Buff[_cnt++] = BYTE2(buff1);
	Buff[_cnt++] = BYTE3(buff1);

	// SC和AC的校验
	for (i = 0; i < (flen + 6); i++)
	{
		sumcheck += Buff[i];  // 从帧头开始，对每一字节进行求和，直到DATA区结束
		addcheck += sumcheck; // 每一字节的求和操作，进行一次sumcheck的累加
	}
	Buff[_cnt++] = sumcheck;
	Buff[_cnt++] = addcheck;

    HAL_UART_Transmit_DMA(&huart1, Buff, 16);   // DMA 方式发送数据
	delay_ms(2);

}


void max3010x_plot1(uint32_t buff,uint32_t buff1) {

	int i;
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
	uint8_t _cnt = 0;
	uint16_t flen = 0;
	uint8_t Buff[16] = {0};

	Buff[_cnt++] = 0xAB; // 帧头
	Buff[_cnt++] = 0xFD; // 源地址 （随意）
	Buff[_cnt++] = 0xFE; // 目标地址 （上位机）
	Buff[_cnt++] = 0XF2; // 功能码
	Buff[_cnt++] = 0x08; // 数据长度 低位
	Buff[_cnt++] = 0x00; // 数据长度 高位
	flen = Buff[4] + (Buff[5] << 8);

	Buff[_cnt++] = BYTE0(buff); // 数据内容,小段模式，低位在前
	Buff[_cnt++] = BYTE1(buff); // 需要将字节进行拆分，调用上面的宏定义即可。
	Buff[_cnt++] = BYTE2(buff);
	Buff[_cnt++] = BYTE3(buff);

	Buff[_cnt++] = BYTE0(buff1); // 数据内容,小段模式，低位在前
	Buff[_cnt++] = BYTE1(buff1); // 需要将字节进行拆分，调用上面的宏定义即可。
	Buff[_cnt++] = BYTE2(buff1);
	Buff[_cnt++] = BYTE3(buff1);

	// SC和AC的校验
	for (i = 0; i < (flen + 6); i++)
	{
		sumcheck += Buff[i];  // 从帧头开始，对每一字节进行求和，直到DATA区结束
		addcheck += sumcheck; // 每一字节的求和操作，进行一次sumcheck的累加
	}
	Buff[_cnt++] = sumcheck;
	Buff[_cnt++] = addcheck;

    HAL_UART_Transmit_DMA(&huart1, Buff, 16);   // DMA 方式发送数据
    HAL_Delay(10);

}



