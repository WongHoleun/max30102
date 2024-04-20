/*
 * @Description: ����Ѫ������ͷ�ļ�
 * @Author: Haolin Wang
 * @Date: 2023-05-09 17:02:48
 * @LastEditTime: 2023-05-09 20:42:52
 * @Note: 
 */

#ifndef __ALGORITHM_HRANDSPO2
#define __ALGORITHM_HRANDSPO2

#include "stm32f4xx_hal.h"

#define ST 3  // ����ʱ�䣬���棺�����ı�ST����ô��������¼��������sum_X2������
#define FS 1000 // ����Ƶ�ʣ���λΪHz�����棺�����ı���FS����ô��������¼��������sum_X2����!

// ��Ч�źŷ�Χ����
#define MAX_HR 180 // ��������
#define MIN_HR 40  // ��С������

#define BUFFER_SIZE_MAX3010x ((FS * ST)*2)	// �����������������
#define BUFFER_SIZE (FS * ST)          // �����δ������������
#define FS60 (FS * 60)                 // ���ʴ�pps��ÿ�봫����ٸ����ݰ�����bpm��ÿ���������Ĵ�������ת��ϵ��
#define LOWEST_PERIOD (FS60 / MAX_HR)  // ���֮�����С����
#define HIGHEST_PERIOD (FS60 / MIN_HR) // ���֮���������

// ���ݴ��� API
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
