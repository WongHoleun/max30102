#ifndef __V8_H
#define __V8_H

#include "stm32f4xx_hal.h"

void Sent_data_v8(float buff);
void Sent_Data_DMA_Qt(float data1, float data2, uint8_t len);
void max3010x_plot(uint32_t buff,uint32_t buff1);
void max3010x_plot1(uint32_t buff,uint32_t buff1);
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
		float buff11);

#endif /* V8_H_ */
