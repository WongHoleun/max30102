#include "ALGORITHM_HRANDSPO2.h"
#include "math.h"
#include "V8.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

// 数据缓存区
uint32_t red_Buff[FS];
uint32_t ir_Buff1[FS];
uint32_t ir_Buff2[FS];

// 信号提取和绘图
void max30102_signal_analysis(
		volatile uint32_t *pun_led1_buffer,
		uint16_t buffer_length,
		volatile uint32_t *pun_led2_buffer,
		uint16_t un_offset)
{
	int k;
	int un_offset_tmp = un_offset;

	// 原始数据提取
	for (k = 0; k < FS; k++)
	{
	  red_Buff[k] = pun_led1_buffer[un_offset_tmp];
	  ir_Buff1[k] = pun_led2_buffer[un_offset_tmp];
	  un_offset_tmp = (un_offset_tmp + 1) % BUFFER_SIZE_MAX3010x;			//偏离量：指向下一个数据，防止超出范围
//	  max3010x_plot(red_Buff[k],ir_Buff[k]);
	}

	memset(red_Buff, 0, sizeof(red_Buff));
}

void max30102s_signal_analysis(
		volatile uint32_t *pun_led1_buffer,
		uint16_t buffer_length,
		volatile uint32_t *pun_led2_buffer,
		uint16_t un_offset)
{
	int k;
	int un_offset_tmp = un_offset;

	// 原始数据提取
	for (k = 0; k < FS; k++)
	{
	  red_Buff[k] = pun_led1_buffer[un_offset_tmp];
	  ir_Buff2[k] = pun_led2_buffer[un_offset_tmp];
	  un_offset_tmp = (un_offset_tmp + 1) % BUFFER_SIZE_MAX3010x;			//偏离量：指向下一个数据，防止超出范围
	  max3010x_plot(ir_Buff1[k],ir_Buff2[k]);
	}

	memset(red_Buff, 0, sizeof(red_Buff));
	memset(ir_Buff2, 0, sizeof(ir_Buff2));
	memset(ir_Buff1, 0, sizeof(ir_Buff1));
}



