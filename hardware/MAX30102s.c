/**
****************************************************************************************
* @Description: ����Ѫ���Ǻ����ʼ��ģ�� ��������
* @Author: Haolin Wang
* @Date: 2023-03-20 15:55:29
* @LastEditTime: 2023-03-25 20:48:26
* @Note: ��Դ�ο���https://msalamon.pl/palec-mi-pulsuje-pulsometr-max30102-pod-kontrola-stm32/
* LEDλ�ý������⣺https://github.com/aromring/MAX30102_by_RF/issues/13#issue-601473302
* �Ա�ģ��������⣺http://bbs.eeworld.com.cn/thread-548367-1-1.html
****************************************************************************************
*/

#include "main.h"
#include "i2c.h"
#include "MAX3010x.h"
#include "ALGORITHM_HRANDSPO2.h"
#include "stdio.h"

// Ӳ�� IIC ��ʼ��
I2C_HandleTypeDef *i2c_max30102s;

// ȫ�ֱ���
volatile uint32_t max30102s_led1_buffer[BUFFER_SIZE_MAX3010x] = {0};		// LED1 ����������
volatile uint32_t max30102s_led2_buffer[BUFFER_SIZE_MAX3010x] = {0};	// LED2 ����������
volatile uint16_t max30102s_buffer_head = 0;		// ������ͷ
volatile uint16_t max30102s_buffer_tail = 0;		// ������β
volatile uint16_t max30102s_collected_samples; // �ɼ�������
uint8_t heart_rate_max30102s = 0;					// ����ֵ
int8_t hr_valid_max30102s = 0;					// ������Ч
float spo2_value_max30102s = 0;						// Ѫ��ֵ
int8_t spo2_valid_max30102s = 0;					// Ѫ��ֵ��Ч

// ״̬����ʼ��
Max3010xState max30102s_state;

/**
****************************************************************************************
* @Funticon name: IICд�Ĵ���
* @Berif:
* @Note:
* @param {uint8_t} uch_addr	�Ĵ�����ַ
* @param {uint8_t} uch_data	д�������ͷ��ַ
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_WriteReg(uint8_t uch_addr, uint8_t uch_data)
{
	if (HAL_I2C_Mem_Write(i2c_max30102s, MAX3010x_ADDRESS, uch_addr, 1, &uch_data, 1, I2C_TIMEOUT) == HAL_OK)
	{
		return MAX30102_OK;
	}

	return MAX30102_ERROR;
}

/**
****************************************************************************************
* @Funticon name: IIC���Ĵ���
* @Berif:
* @Note:
* @param {uint8_t} uch_addr	IIC��ַ
* @param {uint8_t} *puch_data	��ȡ������
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_ReadReg(uint8_t uch_addr, uint8_t *puch_data)
{
	if (HAL_I2C_Mem_Read(i2c_max30102s, MAX3010x_ADDRESS, uch_addr, 1, puch_data, 1, I2C_TIMEOUT) == HAL_OK)
	{
		return MAX30102_OK;
	}

	return MAX30102_ERROR;
}

/**
****************************************************************************************
* @Funticon name: д�Ĵ���ָ��λ
* @Berif:
* @Note:
* @param {uint8_t} Register	�Ĵ�����ַ
* @param {uint8_t} Bit			�Ĵ�����Ӧλ
* @param {uint8_t} Value		�޸�ֵ
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_WriteRegisterBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
	uint8_t tmp;
	if (MAX30102_OK != max30102s_ReadReg(Register, &tmp)) // �Ȼ�ȡ��Ӧ�Ĵ�����ֵ�����ݴ浽tmp�����ж�IIC�Ƿ�������ȡ
	{
		return MAX30102_ERROR;
	}

	/*����Ҫ�������õ�λ�������ٸ�ֵ*/
	tmp &= ~(1 << Bit);			  // 1����bitλ ��ȡ�����޸�λλ0������Ϊ1��������Ҫ�޸ĵ�λ��0�� ���롱 ����ﵽĳһλ�����Ч��
	tmp |= (Value & 0x01) << Bit; // ���޸�ֵ��1���������㣬ȷ������λΪ��֪״̬���ٽ����߼����Ƶ���Ӧλ����ͨ�� ��������ʵ�ָ�ֵ

	if (MAX30102_OK != max30102s_WriteReg(Register, tmp)) // ��tmp��ֵд��Ĵ��������ж�IIC�Ƿ�����д��
	{
		return MAX30102_ERROR;
	}

	return MAX30102_OK;
}

/**
****************************************************************************************
* @Funticon name: IIC��FIFO����
* @Berif:
* @Note:
* @param {volatile uint32_t} *pun_red_led	��ɫLED���������ݱ�����ַ
* @param {volatile uint32_t} *pun_ir_led	����LED���������ݱ�����ַ
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_ReadFifo(volatile uint32_t *pun_led_1, volatile uint32_t *pun_led_2)
{
	uint8_t ach_i2c_data[6];

	/*FIFO���ݶ�ȡ һ���������ȡ��ȡ6��IIC�ֽ�*/
	if (HAL_I2C_Mem_Read(i2c_max30102s, MAX3010x_ADDRESS, REG_FIFO_DATA, 1, ach_i2c_data, 6, I2C_TIMEOUT) != HAL_OK)
	{
		return MAX30102_ERROR;
	}

	/* �ڴ����� */
	*pun_led_1 = 0;
	*pun_led_2 = 0;

	/* ��ɫLED���ݴ��� */
	*pun_led_1 += (unsigned char) ach_i2c_data[0] << 16;	// ���ݵ��� ����16λ����ֵ
	*pun_led_1 += (unsigned char) ach_i2c_data[1] << 8;		// ���ݵ��� ����8λ����ֵ
	*pun_led_1 += (unsigned char) ach_i2c_data[2];					// ���ݵ���

	/* ����LED���ݴ��� */
	*pun_led_2 += (unsigned char) ach_i2c_data[3] << 16;		// ���ݵ��� ����16λ����ֵ
	*pun_led_2 += (unsigned char) ach_i2c_data[4] << 8;		// ���ݵ��� ����8λ����ֵ
	*pun_led_2 += (unsigned char) ach_i2c_data[5];					// ���ݵ���

	/* FIFO���ݴ��� FIFO DATA[23:18]δʹ�� �������� */
	*pun_led_1 &= 0x03FFFF;
	*pun_led_2 &= 0x03FFFF;

	return MAX30102_OK;
}

/**
****************************************************************************************
* @Funticon name: �����ж�ʹ�ܼĴ���
* @Berif: Enable����1
* @Note:
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_InterruptEnable(uint8_t Enable)
{
	// ����ȫ�� �ж�ʹ�ܣ�FIFO дָ��ʣ��һ�������Ŀ��ÿռ�ʱ�������ж�ʹ��
	if (MAX30102_OK != max30102s_WriteRegisterBit(REG_INTR_ENABLE_1, INT_A_FULL_BIT, Enable))
	{
		return MAX30102_ERROR;
	}

	// �µ� FIFO ���ݾ��� �ж�ʹ�ܣ����� FIFO ����������ʱ,�����ж�ʹ��
	if (MAX30102_OK != max30102s_WriteRegisterBit(REG_INTR_ENABLE_1, INT_PPG_RDY_BIT, Enable))
	{
		return MAX30102_ERROR;
	}

	return MAX30102_OK;
}

/**
****************************************************************************************
* @Funticon name: ��ȡ�ж�״̬�Ĵ���
* @Berif:
* @Note:
* @param {uint8_t} *Status	�жϱ�־λ״̬
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_ReadInterruptStatus(uint8_t *Status)
{
	uint8_t tmp;
	*Status = 0;

	/*��ȡ�ж�״̬�Ĵ���1*/
	if (MAX30102_OK != max30102s_ReadReg(REG_INTR_STATUS_1, &tmp))
	{
		return MAX30102_ERROR;
	}
	*Status |= tmp & 0xE1; // ״̬�Ĵ���1 4���ж�״̬ ���� 1110 0001��0xE1������Чλ���㣬��ͨ����������ȡ�ж�״̬�� status

	return MAX30102_OK;
}

/**
****************************************************************************************
* @Funticon name: �жϻص�����
* @Berif: �ⲿ�жϴ���ʱ������˺�������ȡ����������
* @Note:
* @return {*}
****************************************************************************************
*/
void max30102s_InterruptCallback(void)
{
	uint8_t Status;
	while (MAX30102_OK != max30102s_ReadInterruptStatus(&Status))
		; // ��ȡ�ж�״̬�Ĵ���

	/*�� FIFO������������ȫ���ж� �Ĵ��� */
	if (Status & (1 << INT_A_FULL_BIT)) // �ж��Ƿ�������ݽ����ж�
	{
		for (uint8_t i = 0; i < MAX3010x_FIFO_ALMOST_FULL_SAMPLES; i++)
		{
			while (MAX30102_OK != max30102s_ReadFifo(&max30102s_led1_buffer[max30102s_buffer_head], &max30102s_led2_buffer[max30102s_buffer_head])) {
				;
			}	// a+i�ȼ�&a[i], *(a+i)�ȼ���a[i]
			max30102s_buffer_head = (max30102s_buffer_head + 1) % BUFFER_SIZE_MAX3010x; // ������ͷָ���һ,ͨ���������������ѭ��������%�����a<b�Ļ���a%b����Ϊ0,��������a��
			max30102s_collected_samples++;							 // �����ɼ�����
		}
	}

	/*�� FIFO�����µ����ݾ����ж� �Ĵ���*/
	if (Status & (1 << INT_PPG_RDY_BIT)) // �ж��Ƿ���� �����ݾ����ж�
	{
		while (MAX30102_OK != max30102s_ReadFifo(&max30102s_led1_buffer[max30102s_buffer_head], &max30102s_led2_buffer[max30102s_buffer_head]))
		{
			;
		}
		max30102s_buffer_head = (max30102s_buffer_head + 1) % BUFFER_SIZE_MAX3010x;
		max30102s_collected_samples++;
	}
}

/**
****************************************************************************************
* @Funticon name: FIFO����
* @Berif:
* @Note:
* @param {uint8_t} Address	FIFO�Ĵ����ڲ��洢��ַ��bits��
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_FifoConfiguration(uint8_t Address)
{
	// FIFOдָ�����ã�FIFO_WR_PTR[4:0],����λ����
	if (MAX30102_OK != max30102s_WriteReg(REG_FIFO_WR_PTR, (Address & 0x1F)))
	{
		return MAX30102_ERROR;
	}

	// FIFO������������ã���OVF_COUNTER[4:0]
	if (MAX30102_OK != max30102s_WriteReg(REG_OVF_COUNTER, (Address & 0x1F)))
	{
		return MAX30102_ERROR;
	}

	// FIFO��ָ�����ã�FIFO_RD_PTR[4:0]
	if (MAX30102_OK != max30102s_WriteReg(REG_FIFO_RD_PTR, (Address & 0x1F)))
	{
		return MAX30102_ERROR;
	}
	return MAX30102_OK;
}

/**
****************************************************************************************
* @Funticon name: FIFO����ƽ������
* @Berif:
* @Note:
* @param {uint8_t} Value	ÿ�� FIFO ������ƽ��������
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_FifoSampleAveraging(uint8_t Value)
{
	uint8_t tmp;

	/*��ȡ��ǰFIFO���üĴ���״̬����*/
	if (MAX30102_OK != max30102s_ReadReg(REG_FIFO_CONFIG, &tmp))
	{
		return MAX30102_ERROR;
	}
	tmp &= ~(0x07);				// ����Ϊ��֪״̬
	tmp |= (Value & 0x07) << 5; // ��bit[7:5]���¸�ֵ

	/*�޸�FIFO���üĴ��� ����ƽ������λ*/
	if (MAX30102_OK != max30102s_WriteReg(REG_FIFO_CONFIG, tmp))
	{
		return MAX30102_ERROR;
	}
	return MAX30102_OK;
}

/**
****************************************************************************************
* @Funticon name: FIFOѭ��ʹ��
* @Berif:
* @Note:
* @param {uint8_t} Enable	ʹ��
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_FifoRolloverEnable(uint8_t Enable)
{
	return max30102s_WriteRegisterBit(REG_FIFO_CONFIG, FIFO_CONF_FIFO_ROLLOVER_EN_BIT, (Enable & 0x01));
}

/**
****************************************************************************************
* @Funticon name: FIFO����ȫ������
* @Berif:
* @Note: �üĴ������÷����ж�ʱ FIFO ��ʣ���������������3 �ֽ�/��������
* ���磬������ֶ�����Ϊ 0x0h���� FIFO ��ʣ�� 0 ����������ʱ������ 32 �� FIFO �ֶ���δ�����ݣ����������жϡ�
* ���⣬������ֶ�����Ϊ 0xFh ���� FIFO ��ʣ�� 15 ������������17 �� FIFO ����������δ�����ݣ�ʱ�����жϡ�
* @param {uint8_t} Value	ʹ��
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_FifoAlmostFullValue(uint8_t Value)
{
	/*�ж�Ԥ��δ�����������Ƿ񳬳���Χ*/
	if (Value < 17)
	{
		Value = 17;
	}
	if (Value > 32)
	{
		Value = 32;
	}

	Value = 32 - Value; // ��ȥδ�������ֽ��� �õ�����������
	uint8_t tmp;

	/* ��ȡfifo�Ĵ��� */
	if (MAX30102_OK != max30102s_ReadReg(REG_FIFO_CONFIG, &tmp))
	{
		return MAX30102_ERROR;
	}
	tmp &= ~(0x0F); // ��FIFO_A_FULL[3:0]�������㣬ȷ��״̬��֪

	/* д��fifo�Ĵ��� */
	tmp |= (Value & 0x0F); // ���趨ֵ������ʱ������
	if (MAX30102_OK != max30102s_WriteReg(REG_FIFO_CONFIG, tmp))
	{
		return MAX30102_ERROR;
	}
	return MAX30102_OK;
}

/**
****************************************************************************************
* @Funticon name: ģʽ���á�����������λ
* @Berif:
* @Note:
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_Reset(void)
{
	uint8_t tmp = 0xFF;
	/*ִ�и�λ*/
	if (MAX30102_OK != max30102s_WriteReg(REG_MODE_CONFIG, 0x40))
	{
		return MAX30102_ERROR;
	}

	/*�ȴ���λ���*/
	do
	{
		if (MAX30102_OK != max30102s_ReadReg(REG_MODE_CONFIG, &tmp))
		{
			return MAX30102_ERROR;
		}
	} while (tmp & (1 << 6));

	return MAX30102_OK;
}

/**
****************************************************************************************
* @Funticon name: ģʽ���á���led����ģʽ����
* @Berif:
* @Note:
* @param {uint8_t} Mode	����ģʽ
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_SetMode(uint8_t Mode)
{
	uint8_t tmp;
	/*��ȡ����������ģʽ*/
	if (MAX30102_OK != max30102s_ReadReg(REG_MODE_CONFIG, &tmp))
	{
		return MAX30102_ERROR;
	}
	tmp &= ~(0x07); // ָ��λ����

	/*����ģʽ����*/
	tmp |= (Mode & 0x07); // �ݴ�����λ
	if (MAX30102_OK != max30102s_WriteReg(REG_MODE_CONFIG, tmp))
	{
		return MAX30102_ERROR;
	}
	return MAX30102_OK;
}

/**
****************************************************************************************
* @Funticon name: SpO2 ����
* @Berif:
* @Note:
* @param {uint8_t} Value	����ֵ
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_SpO2AdcRange(uint8_t Value)
{
	uint8_t tmp;

	/* ��ȡ���ڷֱ��� */
	if (MAX30102_OK != max30102s_ReadReg(REG_SPO2_CONFIG, &tmp))
	{
		return MAX30102_ERROR;
	}
	tmp &= ~(0x03);
	tmp |= ((Value & 0x03) << 5); // SpO2_ADC_RGE bit[6:5]

	/* �������÷ֱ��� */
	if (MAX30102_OK != max30102s_WriteReg(REG_SPO2_CONFIG, tmp))
	{
		return MAX30102_ERROR;
	}
	return MAX30102_OK;
}

/* ���������� */
MAX30102_STATUS max30102s_SpO2SampleRate(uint8_t Value)
{
	uint8_t tmp;

	/* ��ȡ���ڲ����� */
	if (MAX30102_OK != max30102s_ReadReg(REG_SPO2_CONFIG, &tmp))
	{
		return MAX30102_ERROR;
	}
	tmp &= ~(0x07);
	tmp |= ((Value & 0x07) << 2); // SpO2_SR bit[4:2]

	/* �������÷ֱ��� */
	if (MAX30102_OK != max30102s_WriteReg(REG_SPO2_CONFIG, tmp))
	{
		return MAX30102_ERROR;
	}
	return MAX30102_OK;
}

/*LED����������*/
MAX30102_STATUS max30102s_SpO2LedPulseWidth(uint8_t Value)
{
	uint8_t tmp;

	/*��ȡ����������*/
	if (MAX30102_OK != max30102s_ReadReg(REG_SPO2_CONFIG, &tmp))
	{
		return MAX30102_ERROR;
	}
	tmp &= ~(0x03);
	tmp |= (Value & 0x03); // LED_PW bit[1:0]

	/* ��������LED������ */
	if (MAX30102_OK != max30102s_WriteReg(REG_SPO2_CONFIG, tmp))
	{
		return MAX30102_ERROR;
	}

	return MAX30102_OK;
}

/**
****************************************************************************************
* @Funticon name: LED ����Ŵ�������
* @Berif:
* @Note: LED���� == �Ĵ�������ֵ * 0.2 ��mA��
* @param {uint8_t} Value	����ֵ
* @return {*}
****************************************************************************************
*/
// LED1����Ŵ�������
MAX30102_STATUS max30102s_Led1PulseAmplitude(uint8_t Value)
{
	if (MAX30102_OK != max30102s_WriteReg(REG_LED1_PA, Value))
	{
		return MAX30102_ERROR;
	}
	return MAX30102_OK;
}

/* LED2����Ŵ������� */
MAX30102_STATUS max30102s_Led2PulseAmplitude(uint8_t Value)
{
	if (MAX30102_OK != max30102s_WriteReg(REG_LED2_PA, Value))
	{
		return MAX30102_ERROR;
	}
	return MAX30102_OK;
}

/**
****************************************************************************************
* @Funticon name: �˻�����
* @Berif: �������������ö�ȡ����Ѫ��
* @Note:
* @return {*}
****************************************************************************************
*/
/*���ʻ�ȡ*/
uint8_t max30102s_GetHeartRate(void)
{
	return heart_rate_max30102s;
}
/*Ѫ����ȡ*/
float max30102s_GetSpO2Value(void)
{
	return spo2_value_max30102s;
}

/**
****************************************************************************************
* @Funticon name: ״̬�������
* @Berif:
* @Note: ����4��״̬��״̬����ͬʱ���ж��ڼ�һֱ�ռ�������
* @return {*}
****************************************************************************************
*/
void max30102s_Task(void)
{
	static bool flag = false;    //�������;
	switch (max30102s_state)
	{
		case MAX3010x_STATE_BEGIN: // ��̬��׼������״̬
		{

			/*����*/
			max30102s_collected_samples = 0; // �ɼ�����Ϊ0
			max30102s_buffer_tail = max30102s_buffer_head;		  // ������ͷβ����
			max30102s_Led1PulseAmplitude(MAX3010x_RED_LED_CURRENT_HIGH); // ��LED~7mA
			max30102s_Led2PulseAmplitude(MAX3010x_IR_LED_CURRENT_HIGH);	// ����~7mA

			/*��̬*/
			max30102s_state = MAX3010x_STATE_CALIBRATE; // ��⵽��ָ�ӽ� ת��ΪУ׼̬
			break;
		}	// MAX3010x_STATE_BEGIN

		case MAX3010x_STATE_CALIBRATE: // ��̬������Ӵ������������ɼ���ȥ5����������Ч������仺����
		{

			/*����*/
			if (max30102s_collected_samples > BUFFER_SIZE) // �ɼ��������������������ռ�
			{
				/*��̬*/
				max30102s_state = MAX3010x_STATE_CALCULATE; // �ɼ���һ������������ ת��Ϊ����̬
			}
			break;
		}	// MAX3010x_STATE_CALIBRATE

		case MAX3010x_STATE_CALCULATE: // ��̬���ռ���һ������������ͨ���㷨����
		{

			/*����*/
			if (flag) {
				max30102s_signal_analysis(
						max30102s_led1_buffer,
						BUFFER_SIZE,
						max30102s_led2_buffer,
						max30102s_buffer_tail);
				flag = false;
			}
//			max30101_signal_analysis(
//					max30102s_led1_buffer,
//					BUFFER_SIZE,
//					max30102s_led2_buffer,
//					&heart_rate_max30102s,
//					&hr_valid_max30102s,
//					max30102s_buffer_tail);

			max30102s_buffer_tail = (max30102s_buffer_tail + FS) % BUFFER_SIZE_MAX3010x; // ������β
			max30102s_collected_samples = 0;						  // �ռ���������������

			/*��̬*/
			max30102s_state = MAX3010x_STATE_COLLECT_NEXT_PORTION; // �ɼ�̬�������ɼ��µ�����
			break;
		}	// MAX3010x_STATE_CALCULATE

		case MAX3010x_STATE_COLLECT_NEXT_PORTION: // ��̬��������һֱ�������ռ���һ������Ʒ����һ��֮��������״̬ 3 ���ظ�ѭ����ֱ�������뿪������
		{
			/*���� 2*/
			if (max30102s_collected_samples > FS) // ������������ÿ��ƽ��������
			{
				/*��̬*/
				max30102s_state = MAX3010x_STATE_CALCULATE; // �Բɼ������������㷨����
			}
			flag = true;
			break;
		}	// MAX3010x_STATE_COLLECT_NEXT_PORTION

	    default:
	    {
	        ;
	        break;
	    }	// Ĭ���������
	}	// switch
}

/**
****************************************************************************************
* @Funticon name: ��������ʼ��
* @Berif:
* @Note:
* @param {I2C_HandleTypeDef} *i2c
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_Init(I2C_HandleTypeDef *i2c)
{
	uint8_t uch_dummy;
	i2c_max30102s = i2c;

	/*�����λ*/
	if (MAX30102_OK != max30102s_Reset()) // ��λ MAX30102
	{
		return MAX30102_ERROR;
	}
	/*���Ĵ���*/
	if (MAX30102_OK != max30102s_ReadReg(0, &uch_dummy))
	{
		return MAX30102_ERROR;
	}
	/*FIFO���ã�дָ��,���������,��ָ��*/
	if (MAX30102_OK != max30102s_FifoConfiguration(0x00))
	{
		return MAX30102_ERROR;
	}
	/*FIFO����ƽ��*/
	if (MAX30102_OK != max30102s_FifoSampleAveraging(FIFO_SMP_AVE_1)) // ÿ�� FIFO ������ƽ��������Ϊ4
	{
		return MAX30102_ERROR;
	}
	/*FIFOѭ��ʹ��*/
	if (MAX30102_OK != max30102s_FifoRolloverEnable(0)) // ����Ϊ0��FIFO������£�ֱ����ȡ FIFO_DATA ����� WRITE/READ ָ��λ�á�
	{
		return MAX30102_ERROR;
	}
	/*FIFO��ֵ����*/
	if (MAX30102_OK != max30102s_FifoAlmostFullValue(MAX3010x_FIFO_ALMOST_FULL_SAMPLES)) //  δ������X��ʱ�����жϡ�
	{
		return MAX30102_ERROR;
	}
	/*ģʽ����*/
	if (MAX30102_OK != max30102s_SetMode(MODE_SPO2_MODE)) // Ѫ��ģʽ
	{
		return MAX30102_ERROR;
	}
	/*ADC��Χ*/
	if (MAX30102_OK != max30102s_SpO2AdcRange(SPO2_ADC_RGE_8192)) // ADC������4096nA
	{
		return MAX30102_ERROR;
	}
	/*������*/
	if (MAX30102_OK != max30102s_SpO2SampleRate(SPO2_SAMPLE_RATE_1000)) // ������
	{
		return MAX30102_ERROR;
	}
	/*LED������*/
	if (MAX30102_OK != max30102s_SpO2LedPulseWidth(SPO2_PULSE_WIDTH_411)) // ������411us �ֱ���18bits
	{
		return MAX30102_ERROR;
	}
	/*��ɫLED�������*/
	if (MAX30102_OK != max30102s_Led1PulseAmplitude(MAX3010x_RED_LED_CURRENT_LOW)) // ��ɫLED�����ֵ0mA
//	if (MAX30102_OK != max30102s_Led1PulseAmplitude(MAX3010x_RED_LED_CURRENT_HIGH)) // ��ɫLED�����ֵ0mA
	{
		return MAX30102_ERROR;
	}
	/*����LED�������*/
	if (MAX30102_OK != max30102s_Led2PulseAmplitude(MAX3010x_IR_LED_CURRENT_LOW)) // ����LED�����ֵ0.2mA
//	if (MAX30102_OK != max30102s_Led2PulseAmplitude(MAX3010x_IR_LED_CURRENT_HIGH)) // ����LED�����ֵ0.2mA
	{
		return MAX30102_ERROR;
	}
	/*FIFO �ж�ʹ�� */
	if (MAX30102_OK != max30102s_InterruptEnable(1))
	{
		return MAX30102_ERROR;
	}
	max30102s_state = MAX3010x_STATE_BEGIN; // ״̬����ʼ״̬

	return MAX30102_OK;
}
