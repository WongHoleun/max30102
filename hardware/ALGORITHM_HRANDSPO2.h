/*
 * @Description: 心率血氧计算头文件
 * @Author: Haolin Wang
 * @Date: 2023-05-09 17:02:48
 * @LastEditTime: 2023-05-09 20:42:52
 * @Note: 
 */

#ifndef __ALGORITHM_HRANDSPO2
#define __ALGORITHM_HRANDSPO2

#include "stm32f4xx_hal.h"

#define ST 3  // 采样时间，警告：如果你改变ST，那么你必须重新计算下面的sum_X2参数！
#define FS 1000 // 采样频率，单位为Hz。警告：如果你改变了FS，那么你必须重新计算下面的sum_X2参数!

// 有效信号范围限制
#define MAX_HR 180 // 最大的心率
#define MIN_HR 40  // 最小的心率

#define BUFFER_SIZE_MAX3010x ((FS * ST)*2)	// 传感器缓存的样本数
#define BUFFER_SIZE (FS * ST)          // 单批次处理的样本数量
#define FS60 (FS * 60)                 // 心率从pps（每秒传输多少个数据包）到bpm（每分钟跳动的次数）的转换系数
#define LOWEST_PERIOD (FS60 / MAX_HR)  // 峰峰之间的最小距离
#define HIGHEST_PERIOD (FS60 / MIN_HR) // 峰峰之间的最大距离

// 数据处理 API
void max30102_signal_analysis(
		volatile uint32_t *pun_led1_buffer,
		uint16_t n_ir_buffer_length,
		volatile uint32_t *pun_led2_buffer,
		uint16_t un_offset);

void max30102s_signal_analysis(
		volatile uint32_t *pun_led1_buffer,
		uint16_t n_ir_buffer_length,
		volatile uint32_t *pun_led2_buffer,
		uint16_t un_offset);
void plot();
#endif
