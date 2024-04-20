/**
 ******************************************************************************
 * ģ�����ƣ�MAX3010x ͷ�ļ�
 * �ļ���	��MAX3010x.h
 * ˵��	��������MAX30101��MAX30102���ṩ�û��Զ������������Լ����ⲿ���õĽӿں���������
 * �汾��	�����˰�
 * �޸ļ�¼����Դ�ο���https://msalamon.pl/palec-mi-pulsuje-pulsometr-Max30101-pod-kontrola-stm32/
 LEDλ�ý�����https://github.com/aromring/Max30101_by_RF/issues/13#issue-601473302
 �Ա�ģ��������⣺http://bbs.eeworld.com.cn/thread-548367-1-1.html
 ******************************************************************************
 */

#ifndef MAX3010X_H_
#define MAX3010X_H_

#include "stm32f4xx_hal.h"
#include "ALGORITHM_HRANDSPO2.h"
#include "stdbool.h"

/***********************************************	��������  ***********************************************/
#define I2C_TIMEOUT 1	// ��ʱʱ��

#define MAX3010x_ADDRESS 0xAE          			//(0x57<<1)	������ IIC д��ַ ����ַ 0xAF
#define MAX3010x_FIFO_ALMOST_FULL_SAMPLES 17	//��������ȫ���ж�ʱ δ��������17
#define MAX3010x_IR_LED_CURRENT_LOW 0x00  		// ����LED�����ֵ��С 0mA
#define MAX3010x_RED_LED_CURRENT_LOW 0x00 		// ��ɫLED�����ֵ��С 0mA
#define MAX3010x_GREEN_LED_CURRENT_LOW 0x00  	// ��ɫ LED�����ֵ��С 0mA ��ע����� max30101 ���� led��

#define MAX3010x_IR_LED_CURRENT_HIGH	0x32			//����LED�����ֵ���10mA
#define MAX3010x_RED_LED_CURRENT_HIGH	0x32			//��ɫLED�����ֵ���10mA
#define MAX3010x_GREEN_LED_CURRENT_HIGH	0x32			//��ɫ LED�����ֵ 15mA4c

// ���ݽṹ��
typedef struct {
    volatile uint32_t led1_buffer[BUFFER_SIZE]; // ����LED����������
    volatile uint32_t led2_buffer[BUFFER_SIZE]; // ��ɫLED����������
    volatile uint16_t buffer_head;                      	// ������ͷ
    volatile uint16_t buffer_tail;                      	// ������β
    volatile uint16_t collected_samples;                	// �ɼ���������Ŀ
} Max3010xDataStruct;

// ״̬��
typedef enum
{
	MAX3010x_STATE_BEGIN,// ״̬1����ָû��ʩ���ڴ������ϡ���ɫLED�ƹرգ�����ƴ�����Сֵ������LED��Ϊ�˼����ڴ������ϵ���ָ��ÿ����ָ�Ӵ��������ƿ�ʱ������ͻ�ͨ��Ϩ��LED�ƶ��ָ������״̬��
	MAX3010x_STATE_CALIBRATE,// ״̬2��һ����ָ��Ӧ�ã�����������Ҫ�� "��Ч���� "������Ĭ������£��������ڹ�ȥ5��������Ͻ��еġ���ˣ�У׼������仺����������Ҫ�ܳ�ʱ�䡣
	MAX3010x_STATE_CALCULATE,// ״̬3��һ���ռ����㹻�������������ͻ����Maxim�㷨���м��㡣����һ�����У�ѭ������������λ����һ���Ӷ�Ӧ������������
	MAX3010x_STATE_COLLECT_NEXT_PORTION // ״̬4��������״̬�£����������������ֱ���ռ���һ����������������һ�뿪ʼ��֮��������3��״̬��ѭ��������ֱ����ָ�Ӵ��������ƿ���
} Max3010xState;


/***********************************************	�Ĵ�����ַ  ***********************************************/
// ״̬
#define REG_INTR_STATUS_1 0x00 // �ж�״̬1 ֻ��
#define REG_INTR_STATUS_2 0x01 // �ж�״̬2 ֻ��
#define REG_INTR_ENABLE_1 0x02 // �ж�ʹ��1 ��д
#define REG_INTR_ENABLE_2 0x03 // �ж�ʹ��2 ��д
#define REG_FIFO_WR_PTR 0x04   // FIFOдָ�룺FIFO дָ��ָ�� Max30101 д����һ��������λ�á��������� FIFO ��ÿ����������ָ�붼��ǰ����
#define REG_OVF_COUNTER 0x05   // �������������FIFO����ʱ���������ᱻ�Ƶ�FIFO�ϣ������ᶪʧ��OVF_COUNTER���㶪ʧ��������������0x1F���ﵽ���͡�
#define REG_FIFO_RD_PTR 0x06   // FIFO��ָ�룺FIFO��ȡָ��ָ������ͨ��I2C�ӿڴ�FIFO��ȡ��һ��������λ�á�
#define REG_FIFO_DATA 0x07     // FIFO���ݼĴ�����FIFO��������룬MSBʼ��λ�� bit17

// ����
#define REG_FIFO_CONFIG 0x08 // FIFO����
#define REG_MODE_CONFIG 0x09 // ģʽ����
#define REG_SPO2_CONFIG 0x0A // SPO2����
#define REG_LED1_PA 0x0C     // LED1 ������� RED
#define REG_LED2_PA 0x0D     // LED2 ������� IR
#define REG_LED3_PA 0x0E     // LED3 ������� GREEN
#define REG_PILOT_PA 0x10
#define REG_MULTI_LED_CTRL1 0x11 // ��LEDģʽ���ƼĴ���1
#define REG_MULTI_LED_CTRL2 0x12 // ��LEDģʽ���ƼĴ���2

// ģ���¶�
#define REG_TEMP_INTR 0x1F   // ģ���¶�����
#define REG_TEMP_FRAC 0x20   // ģ���¶ȷ���
#define REG_TEMP_CONFIG 0x21 // ģ���¶�����
#define REG_PROX_INT_THRESH 0x30

// ������
#define REG_REV_ID 0xFE  // �޶���
#define REG_PART_ID 0xFF // ������

/*	�ж�״̬1 (0x00) ���ж�״̬2 (0x01) ���ж�ʹ��1 (0x02)���ж�ʹ��2 (0x03)*/
// ֻҪ��ȡ�ж�״̬�Ĵ��������ȡ�����жϵļĴ������жϾͻᱻ�����
#define INT_A_FULL_BIT 7       // FIFO�������ı�־���� FIFO дָ��ʣ��һ�������Ŀ��ÿռ�ʱ���ᴥ�����жϡ����ж�״̬�Ĵ���1 bit7
#define INT_PPG_RDY_BIT 6      // �µ� FIFO ���ݾ����������� FIFO ����������ʱ�������жϡ����ж�״̬�Ĵ���1  bit6
#define INT_ALC_OVF_BIT 5      // ����������������� SpO2/HR �������ܵĻ������������ܴﵽ�������ʱ�����жϽ����������ж�״̬�Ĵ���1  bit5
#define INT_DIE_TEMP_RDY_BIT 1 // �ڲ��¶Ⱦ�����־�����������Զ�ȡ�¶����ݼĴ��������ж�״̬�Ĵ���2 bit1
#define INT_PWR_RDY_BIT 0      // ��Դ������־��ģ����ͨ�粢׼���ռ����ݣ�ֻ��״̬�Ĵ����У������ж�״̬�Ĵ���1 bit0
// ÿ��Ӳ���ж�Դ��������IC������Ĵ����н��á���Դ�����ж��޷����ã���Ϊģ�������״̬���ڵ�������£��͵�Դ��ѹ�����ã�Ĭ������������ж϶������á�

/*	FIFO ���� (0x08) */
#define FIFO_CONF_SMP_AVE_BIT 7          // bit7 ����ƽ����Ϊ�˼���������������ͨ�����øüĴ�����������оƬ�϶���������(ÿ������ͨ����)����ƽ���ͳ�ȡ��
#define FIFO_CONF_SMP_AVE_LENGHT 3       // ���ݳ���3  SMP_AVE[2:0]
#define FIFO_CONF_FIFO_ROLLOVER_EN_BIT 4 // bit4 FIFO��ѭ��������Ϊ (1)���� FIFO����ʱ��ַ��תΪ�㲢�� FIFO ������������ݡ����� (0)���� FIFO ������£�ֱ����ȡ FIFO_DATA ����� WRITE/READ ָ��λ�á�
#define FIFO_CONF_FIFO_A_FULL_BIT 3      // bit3 FIFO�����������жϷ���ʱFIFO��ʣ�������������3�ֽ�/������������
#define FIFO_CONF_FIFO_A_FULL_LENGHT 4   // ���ݳ���4 FIFO_A_FULL[3:0]

// SMP_AVE[2:0] ÿ�� FIFO ������ƽ��������
#define FIFO_SMP_AVE_1 0
#define FIFO_SMP_AVE_2 1
#define FIFO_SMP_AVE_4 2
#define FIFO_SMP_AVE_8 3
#define FIFO_SMP_AVE_16 4
#define FIFO_SMP_AVE_32 5 //[2:0]=101

/*	ģʽ ���� (0x09) */
#define MODE_SHDN_BIT 7    // bit7 �ػ����ƣ�����Ϊ1������ʹ�ò�������ʡ��ģʽ�����еļĴ������������ǵ�ֵ�����е��ж϶������Ϊ�㡣
#define MODE_RESET_BIT 6   // bit6 ��λ���ƣ�����Ϊ1�����е����á���ֵ�����ݼĴ�����ͨ���ϵ縴λ������Ϊ�ϵ�״̬
#define MODE_MODE_BIT 2    // bit2 ģʽ����
#define MODE_MODE_LENGTH 3 // MODE[2:0] ģʽ����λ����3bit

// ģʽѡ��
#define MODE_HEART_RATE_MODE 2 // ����ģʽ��ֻʹ�ú��
#define MODE_SPO2_MODE 3       // Ѫ��ģʽ��ʹ�ú�ƺͺ���
#define MODE_MULTI_LED_MODE 7  // ���ģʽ��ʹ�ú�ƺͺ�����̹�

/*	SpO2 ���� (0x0A) */
#define SPO2_CONF_ADC_RGE_BIT 6    // bit6 ADC��Χ����
#define SPO2_CONF_ADC_RGE_LENGTH 2 // SPO2_ADC_RGE[1:0] ����2bit
#define SPO2_CONF_SR_BIT 4         // bit4 �����ʿ��ƣ���Ч�����ʣ�һ��������һ�� ���� ����/ת����һ�� ��ɫ ����/ת����ɡ�
#define SPO2_CONF_SR_LENGTH 3      // SPO2_SR[2:0] ����3bit
#define SPO2_CONF_LED_PW_BIT 1     // bit1 LED �����ȿ��ƺ� ADC �ֱ��ʣ����� LED �����ȣ��������ÿ�������� ADC �Ļ���ʱ�䡣 ADC �ֱ��������ʱ��ֱ����ء�
#define SPO2_CONF_LED_PW_LENGTH 2  // SPO2_SR[2:0] ����2bit

// ADC��Χ ������ 2048nA 4096nA 8192nA 16384nA
#define SPO2_ADC_RGE_2048 0
#define SPO2_ADC_RGE_4096 1
#define SPO2_ADC_RGE_8192 2
#define SPO2_ADC_RGE_16384 3

// �����ʣ�ÿ��������
#define SPO2_SAMPLE_RATE_50 0
#define SPO2_SAMPLE_RATE_100 1
#define SPO2_SAMPLE_RATE_200 2
#define SPO2_SAMPLE_RATE_400 3
#define SPO2_SAMPLE_RATE_800 4
#define SPO2_SAMPLE_RATE_1000 5
#define SPO2_SAMPLE_RATE_1600 6
#define SPO2_SAMPLE_RATE_3200 7

/*	Multi-LED ���� (0x11-0x12) */
#define MLED_CONF_SLOT1 2    // bit2 �� 1����
#define MLED_CONF_SLOT2 6    // bit2 �� 1����
#define MLED_CONF_SLOT3 2    // bit2 �� 1����
#define MLED_CONF_SLOT4 6    // bit2 �� 1����
#define MLED_CONF_SLOT_LENGTH 3    // SLOTx[2:0] ���� 3bit

/*	Multi-LED ���� */
#define MLED_ACTIVE_RED 1
#define MLED_ACTIVE_IR 2
#define MLED_ACTIVE_GREEN 3

// LED������
#define SPO2_PULSE_WIDTH_69 0  // ������69us �ֱ���15bits
#define SPO2_PULSE_WIDTH_118 1 // ������118us �ֱ���16bits
#define SPO2_PULSE_WIDTH_215 2 // ������215us �ֱ���17bits
#define SPO2_PULSE_WIDTH_411 3 // ������4115us �ֱ���18bits

/***********************************************	MAX30101  ***********************************************/

/*	ͨ��״̬ ״̬�о� */
typedef enum {
	Max30101_ERROR = 0, Max30101_OK = 1
} Max30101_STATUS;

/*	���� ����  */
Max30101_STATUS Max30101_Init(I2C_HandleTypeDef *i2c);
Max30101_STATUS Max30101_ReadFifo(volatile uint32_t *pun_green_led, volatile uint32_t *pun_ir_led);
Max30101_STATUS Max30101_WriteReg(uint8_t uch_addr, uint8_t uch_data);
Max30101_STATUS Max30101_ReadReg(uint8_t uch_addr, uint8_t *puch_data);

/*	�ж� ����  */
Max30101_STATUS Max30101_ReadInterruptStatus(uint8_t *Status);
Max30101_STATUS Max30101_InterruptEnable(uint8_t Enable);
void Max30101_InterruptCallback(void);

//	FIFO���� ����
Max30101_STATUS Max30101_FifoConfiguration(uint8_t Address);
Max30101_STATUS Max30101_FifoSampleAveraging(uint8_t Value);
Max30101_STATUS Max30101_FifoRolloverEnable(uint8_t Enable);
Max30101_STATUS Max30101_FifoAlmostFullValue(uint8_t Value); // 17-32��������FIFO��׼������

//	ģʽ���� ����
Max30101_STATUS Max30101_ShutdownMode(uint8_t Enable);
Max30101_STATUS Max30101_Reset(void);
Max30101_STATUS Max30101_SetMode(uint8_t Mode);

//	SpO2���� ����
Max30101_STATUS Max30101_SpO2AdcRange(uint8_t Value);
Max30101_STATUS Max30101_SpO2SampleRate(uint8_t Value);
Max30101_STATUS Max30101_SpO2LedPulseWidth(uint8_t Value);

// Multi-LED����
Max30101_STATUS Max30101_MultiLedSlot1(uint8_t Value);
Max30101_STATUS Max30101_MultiLedSlot2(uint8_t Value);

//	LED ����Ŵ������� ����
Max30101_STATUS Max30101_Led1PulseAmplitude(uint8_t Value);
Max30101_STATUS Max30101_Led2PulseAmplitude(uint8_t Value);
Max30101_STATUS Max30101_Led3PulseAmplitude(uint8_t Value);

//	������ API
void Max30101_Task(void);
uint8_t Max30101_GetHeartRate(void);

/***********************************************	MAX30102  ***********************************************/

//	ͨ��״̬ ״̬�о�
typedef enum {
	MAX30102_ERROR = 0, MAX30102_OK = 1
} MAX30102_STATUS;

//	���� ����
MAX30102_STATUS Max30102_Init(I2C_HandleTypeDef *i2c);
MAX30102_STATUS Max30102_ReadFifo(volatile uint32_t *pun_red_led,
		volatile uint32_t *pun_ir_led);
MAX30102_STATUS Max30102_WriteReg(uint8_t uch_addr, uint8_t uch_data);
MAX30102_STATUS Max30102_ReadReg(uint8_t uch_addr, uint8_t *puch_data);

//	�ж� ����
MAX30102_STATUS Max30102_ReadInterruptStatus(uint8_t *Status);
MAX30102_STATUS Max30102_InterruptEnable(uint8_t Enable);
void Max30102_InterruptCallback(void);

//	FIFO���� ����
MAX30102_STATUS Max30102_FifoConfiguration(uint8_t Address);
MAX30102_STATUS Max30102_FifoSampleAveraging(uint8_t Value);
MAX30102_STATUS Max30102_FifoRolloverEnable(uint8_t Enable);
MAX30102_STATUS Max30102_FifoAlmostFullValue(uint8_t Value); // 17-32��������FIFO��׼������

//	ģʽ���� ����
MAX30102_STATUS Max30102_ShutdownMode(uint8_t Enable);
MAX30102_STATUS Max30102_Reset(void);
MAX30102_STATUS Max30102_SetMode(uint8_t Mode);

//	SpO2���� ����
MAX30102_STATUS Max30102_SpO2AdcRange(uint8_t Value);
MAX30102_STATUS Max30102_SpO2SampleRate(uint8_t Value);
MAX30102_STATUS Max30102_SpO2LedPulseWidth(uint8_t Value);

//	LED ����Ŵ������� ����
MAX30102_STATUS Max30102_Led1PulseAmplitude(uint8_t Value);
MAX30102_STATUS Max30102_Led2PulseAmplitude(uint8_t Value);

//	������ API
void Max30102_Task(void);
uint8_t Max30102_GetHeartRate(void);
float Max30102_GetSpO2Value(void);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//	���� ����
MAX30102_STATUS max30102s_Init(I2C_HandleTypeDef *i2c);
MAX30102_STATUS max30102s_ReadFifo(volatile uint32_t *pun_red_led,
		volatile uint32_t *pun_ir_led);
MAX30102_STATUS max30102s_WriteReg(uint8_t uch_addr, uint8_t uch_data);
MAX30102_STATUS max30102s_ReadReg(uint8_t uch_addr, uint8_t *puch_data);

//	�ж� ����
MAX30102_STATUS max30102s_ReadInterruptStatus(uint8_t *Status);
MAX30102_STATUS max30102s_InterruptEnable(uint8_t Enable);
void max30102s_InterruptCallback(void);

//	FIFO���� ����
MAX30102_STATUS max30102s_FifoConfiguration(uint8_t Address);
MAX30102_STATUS max30102s_FifoSampleAveraging(uint8_t Value);
MAX30102_STATUS max30102s_FifoRolloverEnable(uint8_t Enable);
MAX30102_STATUS max30102s_FifoAlmostFullValue(uint8_t Value); // 17-32��������FIFO��׼������

//	ģʽ���� ����
MAX30102_STATUS max30102s_ShutdownMode(uint8_t Enable);
MAX30102_STATUS max30102s_Reset(void);
MAX30102_STATUS max30102s_SetMode(uint8_t Mode);

//	SpO2���� ����
MAX30102_STATUS max30102s_SpO2AdcRange(uint8_t Value);
MAX30102_STATUS max30102s_SpO2SampleRate(uint8_t Value);
MAX30102_STATUS max30102s_SpO2LedPulseWidth(uint8_t Value);

//	LED ����Ŵ������� ����
MAX30102_STATUS max30102s_Led1PulseAmplitude(uint8_t Value);
MAX30102_STATUS max30102s_Led2PulseAmplitude(uint8_t Value);

//	������ API
void max30102s_Task(void);
uint8_t max30102s_GetHeartRate(void);
float max30102s_GetSpO2Value(void);


#endif /* MAX3010X_H_ */
