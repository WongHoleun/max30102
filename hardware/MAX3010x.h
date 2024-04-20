/**
 ******************************************************************************
 * 模块名称：MAX3010x 头文件
 * 文件名	：MAX3010x.h
 * 说明	：适用于MAX30101和MAX30102，提供用户自定义数据类型以及供外部调用的接口函数的声明
 * 版本号	：个人版
 * 修改记录：开源参考：https://msalamon.pl/palec-mi-pulsuje-pulsometr-Max30101-pod-kontrola-stm32/
 LED位置交换：https://github.com/aromring/Max30101_by_RF/issues/13#issue-601473302
 淘宝模块设计问题：http://bbs.eeworld.com.cn/thread-548367-1-1.html
 ******************************************************************************
 */

#ifndef MAX3010X_H_
#define MAX3010X_H_

#include "stm32f4xx_hal.h"
#include "ALGORITHM_HRANDSPO2.h"
#include "stdbool.h"

/***********************************************	共享配置  ***********************************************/
#define I2C_TIMEOUT 1	// 超时时间

#define MAX3010x_ADDRESS 0xAE          			//(0x57<<1)	传感器 IIC 写地址 读地址 0xAF
#define MAX3010x_FIFO_ALMOST_FULL_SAMPLES 17	//发出几乎全满中断时 未读样本数17
#define MAX3010x_IR_LED_CURRENT_LOW 0x00  		// 红外LED脉冲幅值最小 0mA
#define MAX3010x_RED_LED_CURRENT_LOW 0x00 		// 红色LED脉冲幅值最小 0mA
#define MAX3010x_GREEN_LED_CURRENT_LOW 0x00  	// 绿色 LED脉冲幅值最小 0mA （注意仅限 max30101 有绿 led）

#define MAX3010x_IR_LED_CURRENT_HIGH	0x32			//红外LED脉冲幅值最大10mA
#define MAX3010x_RED_LED_CURRENT_HIGH	0x32			//红色LED脉冲幅值最大10mA
#define MAX3010x_GREEN_LED_CURRENT_HIGH	0x32			//绿色 LED脉冲幅值 15mA4c

// 数据结构体
typedef struct {
    volatile uint32_t led1_buffer[BUFFER_SIZE]; // 红外LED传感器数据
    volatile uint32_t led2_buffer[BUFFER_SIZE]; // 红色LED传感器数据
    volatile uint16_t buffer_head;                      	// 缓存器头
    volatile uint16_t buffer_tail;                      	// 缓存器尾
    volatile uint16_t collected_samples;                	// 采集的样本数目
} Max3010xDataStruct;

// 状态机
typedef enum
{
	MAX3010x_STATE_BEGIN,// 状态1：手指没有施加在传感器上。红色LED灯关闭，红外灯处于最小值。红外LED是为了检测放在传感器上的手指。每当手指从传感器上移开时，程序就会通过熄灭LED灯而恢复到这个状态。
	MAX3010x_STATE_CALIBRATE,// 状态2：一旦手指被应用，缓冲区就需要被 "有效数据 "填满。默认情况下，计算是在过去5秒的样本上进行的。因此，校准，即填充缓冲区，将需要很长时间。
	MAX3010x_STATE_CALCULATE,// 状态3：一旦收集到足够数量的样本，就会根据Maxim算法进行计算。在这一步骤中，循环缓冲器被移位，与一秒钟对应的样本数量。
	MAX3010x_STATE_COLLECT_NEXT_PORTION // 状态4：在这种状态下，传感器处理程序处于直到收集下一部分样本，即从下一秒开始。之后，它进入3号状态，循环往复，直到手指从传感器上移开。
} Max3010xState;


/***********************************************	寄存器地址  ***********************************************/
// 状态
#define REG_INTR_STATUS_1 0x00 // 中断状态1 只读
#define REG_INTR_STATUS_2 0x01 // 中断状态2 只读
#define REG_INTR_ENABLE_1 0x02 // 中断使能1 读写
#define REG_INTR_ENABLE_2 0x03 // 中断使能2 读写
#define REG_FIFO_WR_PTR 0x04   // FIFO写指针：FIFO 写指针指向 Max30101 写入下一个样本的位置。对于推入 FIFO 的每个样本，该指针都会前进。
#define REG_OVF_COUNTER 0x05   // 溢出计数器：当FIFO满的时候，样本不会被推到FIFO上，样本会丢失。OVF_COUNTER计算丢失的样本数。它在0x1F处达到饱和。
#define REG_FIFO_RD_PTR 0x06   // FIFO读指针：FIFO读取指针指向处理器通过I2C接口从FIFO获取下一个样本的位置。
#define REG_FIFO_DATA 0x07     // FIFO数据寄存器：FIFO数据左对齐，MSB始终位于 bit17

// 配置
#define REG_FIFO_CONFIG 0x08 // FIFO配置
#define REG_MODE_CONFIG 0x09 // 模式配置
#define REG_SPO2_CONFIG 0x0A // SPO2配置
#define REG_LED1_PA 0x0C     // LED1 脉冲幅度 RED
#define REG_LED2_PA 0x0D     // LED2 脉冲幅度 IR
#define REG_LED3_PA 0x0E     // LED3 脉冲幅度 GREEN
#define REG_PILOT_PA 0x10
#define REG_MULTI_LED_CTRL1 0x11 // 多LED模式控制寄存器1
#define REG_MULTI_LED_CTRL2 0x12 // 多LED模式控制寄存器2

// 模具温度
#define REG_TEMP_INTR 0x1F   // 模具温度整数
#define REG_TEMP_FRAC 0x20   // 模具温度分数
#define REG_TEMP_CONFIG 0x21 // 模具温度配置
#define REG_PROX_INT_THRESH 0x30

// 零件编号
#define REG_REV_ID 0xFE  // 修订号
#define REG_PART_ID 0xFF // 零件编号

/*	中断状态1 (0x00) ；中断状态2 (0x01) ；中断使能1 (0x02)；中断使能2 (0x03)*/
// 只要读取中断状态寄存器，或读取触发中断的寄存器，中断就会被清除。
#define INT_A_FULL_BIT 7       // FIFO即将满的标志：当 FIFO 写指针剩余一定数量的可用空间时，会触发此中断——中断状态寄存器1 bit7
#define INT_PPG_RDY_BIT 6      // 新的 FIFO 数据就绪：当数据 FIFO 中有新样本时触发此中断——中断状态寄存器1  bit6
#define INT_ALC_OVF_BIT 5      // 环境光消除溢出：当 SpO2/HR 光电二极管的环境光消除功能达到其最大极限时，该中断将触发——中断状态寄存器1  bit5
#define INT_DIE_TEMP_RDY_BIT 1 // 内部温度就绪标志：处理器可以读取温度数据寄存器——中断状态寄存器2 bit1
#define INT_PWR_RDY_BIT 0      // 电源就绪标志：模块已通电并准备收集数据（只有状态寄存器有）——中断状态寄存器1 bit0
// 每个硬件中断源都可以在IC的软件寄存器中禁用。电源就绪中断无法禁用，因为模块的数字状态会在掉电情况下（低电源电压）重置，默认情况是所有中断都被禁用。

/*	FIFO 配置 (0x08) */
#define FIFO_CONF_SMP_AVE_BIT 7          // bit7 样本平均：为了减少数据吞吐量，通过设置该寄存器，可以在芯片上对相邻样本(每个单独通道中)进行平均和抽取。
#define FIFO_CONF_SMP_AVE_LENGHT 3       // 数据长度3  SMP_AVE[2:0]
#define FIFO_CONF_FIFO_ROLLOVER_EN_BIT 4 // bit4 FIFO满循环：设置为 (1)，则 FIFO填满时地址翻转为零并且 FIFO 继续填充新数据。设置 (0)，则 FIFO 不会更新，直到读取 FIFO_DATA 或更改 WRITE/READ 指针位置。
#define FIFO_CONF_FIFO_A_FULL_BIT 3      // bit3 FIFO几乎填满：中断发出时FIFO中剩余的数据样本（3字节/样本）的数量
#define FIFO_CONF_FIFO_A_FULL_LENGHT 4   // 数据长度4 FIFO_A_FULL[3:0]

// SMP_AVE[2:0] 每个 FIFO 样本的平均样本数
#define FIFO_SMP_AVE_1 0
#define FIFO_SMP_AVE_2 1
#define FIFO_SMP_AVE_4 2
#define FIFO_SMP_AVE_8 3
#define FIFO_SMP_AVE_16 4
#define FIFO_SMP_AVE_32 5 //[2:0]=101

/*	模式 配置 (0x09) */
#define MODE_SHDN_BIT 7    // bit7 关机控制：设置为1，可以使该部件进入省电模式，所有的寄存器都保留它们的值，所有的中断都被清除为零。
#define MODE_RESET_BIT 6   // bit6 复位控制：设置为1，所有的配置、阈值和数据寄存器都通过上电复位被重置为上电状态
#define MODE_MODE_BIT 2    // bit2 模式控制
#define MODE_MODE_LENGTH 3 // MODE[2:0] 模式控制位长度3bit

// 模式选择
#define MODE_HEART_RATE_MODE 2 // 心率模式，只使用红灯
#define MODE_SPO2_MODE 3       // 血氧模式，使用红灯和红外
#define MODE_MULTI_LED_MODE 7  // 混合模式，使用红灯和红外和绿光

/*	SpO2 配置 (0x0A) */
#define SPO2_CONF_ADC_RGE_BIT 6    // bit6 ADC范围控制
#define SPO2_CONF_ADC_RGE_LENGTH 2 // SPO2_ADC_RGE[1:0] 长度2bit
#define SPO2_CONF_SR_BIT 4         // bit4 采样率控制：有效采样率，一个样本由一个 红外 脉冲/转换和一个 红色 脉冲/转换组成。
#define SPO2_CONF_SR_LENGTH 3      // SPO2_SR[2:0] 长度3bit
#define SPO2_CONF_LED_PW_BIT 1     // bit1 LED 脉冲宽度控制和 ADC 分辨率：设置 LED 脉冲宽度，间接设置每个样本中 ADC 的积分时间。 ADC 分辨率与积分时间直接相关。
#define SPO2_CONF_LED_PW_LENGTH 2  // SPO2_SR[2:0] 长度2bit

// ADC范围 满量程 2048nA 4096nA 8192nA 16384nA
#define SPO2_ADC_RGE_2048 0
#define SPO2_ADC_RGE_4096 1
#define SPO2_ADC_RGE_8192 2
#define SPO2_ADC_RGE_16384 3

// 采样率：每秒样本数
#define SPO2_SAMPLE_RATE_50 0
#define SPO2_SAMPLE_RATE_100 1
#define SPO2_SAMPLE_RATE_200 2
#define SPO2_SAMPLE_RATE_400 3
#define SPO2_SAMPLE_RATE_800 4
#define SPO2_SAMPLE_RATE_1000 5
#define SPO2_SAMPLE_RATE_1600 6
#define SPO2_SAMPLE_RATE_3200 7

/*	Multi-LED 配置 (0x11-0x12) */
#define MLED_CONF_SLOT1 2    // bit2 槽 1配置
#define MLED_CONF_SLOT2 6    // bit2 槽 1配置
#define MLED_CONF_SLOT3 2    // bit2 槽 1配置
#define MLED_CONF_SLOT4 6    // bit2 槽 1配置
#define MLED_CONF_SLOT_LENGTH 3    // SLOTx[2:0] 长度 3bit

/*	Multi-LED 配置 */
#define MLED_ACTIVE_RED 1
#define MLED_ACTIVE_IR 2
#define MLED_ACTIVE_GREEN 3

// LED脉冲宽度
#define SPO2_PULSE_WIDTH_69 0  // 脉冲宽度69us 分辨率15bits
#define SPO2_PULSE_WIDTH_118 1 // 脉冲宽度118us 分辨率16bits
#define SPO2_PULSE_WIDTH_215 2 // 脉冲宽度215us 分辨率17bits
#define SPO2_PULSE_WIDTH_411 3 // 脉冲宽度4115us 分辨率18bits

/***********************************************	MAX30101  ***********************************************/

/*	通信状态 状态列举 */
typedef enum {
	Max30101_ERROR = 0, Max30101_OK = 1
} Max30101_STATUS;

/*	功能 函数  */
Max30101_STATUS Max30101_Init(I2C_HandleTypeDef *i2c);
Max30101_STATUS Max30101_ReadFifo(volatile uint32_t *pun_green_led, volatile uint32_t *pun_ir_led);
Max30101_STATUS Max30101_WriteReg(uint8_t uch_addr, uint8_t uch_data);
Max30101_STATUS Max30101_ReadReg(uint8_t uch_addr, uint8_t *puch_data);

/*	中断 函数  */
Max30101_STATUS Max30101_ReadInterruptStatus(uint8_t *Status);
Max30101_STATUS Max30101_InterruptEnable(uint8_t Enable);
void Max30101_InterruptCallback(void);

//	FIFO配置 函数
Max30101_STATUS Max30101_FifoConfiguration(uint8_t Address);
Max30101_STATUS Max30101_FifoSampleAveraging(uint8_t Value);
Max30101_STATUS Max30101_FifoRolloverEnable(uint8_t Enable);
Max30101_STATUS Max30101_FifoAlmostFullValue(uint8_t Value); // 17-32个样本在FIFO中准备就绪

//	模式配置 函数
Max30101_STATUS Max30101_ShutdownMode(uint8_t Enable);
Max30101_STATUS Max30101_Reset(void);
Max30101_STATUS Max30101_SetMode(uint8_t Mode);

//	SpO2配置 函数
Max30101_STATUS Max30101_SpO2AdcRange(uint8_t Value);
Max30101_STATUS Max30101_SpO2SampleRate(uint8_t Value);
Max30101_STATUS Max30101_SpO2LedPulseWidth(uint8_t Value);

// Multi-LED配置
Max30101_STATUS Max30101_MultiLedSlot1(uint8_t Value);
Max30101_STATUS Max30101_MultiLedSlot2(uint8_t Value);

//	LED 脉冲放大器配置 函数
Max30101_STATUS Max30101_Led1PulseAmplitude(uint8_t Value);
Max30101_STATUS Max30101_Led2PulseAmplitude(uint8_t Value);
Max30101_STATUS Max30101_Led3PulseAmplitude(uint8_t Value);

//	传感器 API
void Max30101_Task(void);
uint8_t Max30101_GetHeartRate(void);

/***********************************************	MAX30102  ***********************************************/

//	通信状态 状态列举
typedef enum {
	MAX30102_ERROR = 0, MAX30102_OK = 1
} MAX30102_STATUS;

//	功能 函数
MAX30102_STATUS Max30102_Init(I2C_HandleTypeDef *i2c);
MAX30102_STATUS Max30102_ReadFifo(volatile uint32_t *pun_red_led,
		volatile uint32_t *pun_ir_led);
MAX30102_STATUS Max30102_WriteReg(uint8_t uch_addr, uint8_t uch_data);
MAX30102_STATUS Max30102_ReadReg(uint8_t uch_addr, uint8_t *puch_data);

//	中断 函数
MAX30102_STATUS Max30102_ReadInterruptStatus(uint8_t *Status);
MAX30102_STATUS Max30102_InterruptEnable(uint8_t Enable);
void Max30102_InterruptCallback(void);

//	FIFO配置 函数
MAX30102_STATUS Max30102_FifoConfiguration(uint8_t Address);
MAX30102_STATUS Max30102_FifoSampleAveraging(uint8_t Value);
MAX30102_STATUS Max30102_FifoRolloverEnable(uint8_t Enable);
MAX30102_STATUS Max30102_FifoAlmostFullValue(uint8_t Value); // 17-32个样本在FIFO中准备就绪

//	模式配置 函数
MAX30102_STATUS Max30102_ShutdownMode(uint8_t Enable);
MAX30102_STATUS Max30102_Reset(void);
MAX30102_STATUS Max30102_SetMode(uint8_t Mode);

//	SpO2配置 函数
MAX30102_STATUS Max30102_SpO2AdcRange(uint8_t Value);
MAX30102_STATUS Max30102_SpO2SampleRate(uint8_t Value);
MAX30102_STATUS Max30102_SpO2LedPulseWidth(uint8_t Value);

//	LED 脉冲放大器配置 函数
MAX30102_STATUS Max30102_Led1PulseAmplitude(uint8_t Value);
MAX30102_STATUS Max30102_Led2PulseAmplitude(uint8_t Value);

//	传感器 API
void Max30102_Task(void);
uint8_t Max30102_GetHeartRate(void);
float Max30102_GetSpO2Value(void);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//	功能 函数
MAX30102_STATUS max30102s_Init(I2C_HandleTypeDef *i2c);
MAX30102_STATUS max30102s_ReadFifo(volatile uint32_t *pun_red_led,
		volatile uint32_t *pun_ir_led);
MAX30102_STATUS max30102s_WriteReg(uint8_t uch_addr, uint8_t uch_data);
MAX30102_STATUS max30102s_ReadReg(uint8_t uch_addr, uint8_t *puch_data);

//	中断 函数
MAX30102_STATUS max30102s_ReadInterruptStatus(uint8_t *Status);
MAX30102_STATUS max30102s_InterruptEnable(uint8_t Enable);
void max30102s_InterruptCallback(void);

//	FIFO配置 函数
MAX30102_STATUS max30102s_FifoConfiguration(uint8_t Address);
MAX30102_STATUS max30102s_FifoSampleAveraging(uint8_t Value);
MAX30102_STATUS max30102s_FifoRolloverEnable(uint8_t Enable);
MAX30102_STATUS max30102s_FifoAlmostFullValue(uint8_t Value); // 17-32个样本在FIFO中准备就绪

//	模式配置 函数
MAX30102_STATUS max30102s_ShutdownMode(uint8_t Enable);
MAX30102_STATUS max30102s_Reset(void);
MAX30102_STATUS max30102s_SetMode(uint8_t Mode);

//	SpO2配置 函数
MAX30102_STATUS max30102s_SpO2AdcRange(uint8_t Value);
MAX30102_STATUS max30102s_SpO2SampleRate(uint8_t Value);
MAX30102_STATUS max30102s_SpO2LedPulseWidth(uint8_t Value);

//	LED 脉冲放大器配置 函数
MAX30102_STATUS max30102s_Led1PulseAmplitude(uint8_t Value);
MAX30102_STATUS max30102s_Led2PulseAmplitude(uint8_t Value);

//	传感器 API
void max30102s_Task(void);
uint8_t max30102s_GetHeartRate(void);
float max30102s_GetSpO2Value(void);


#endif /* MAX3010X_H_ */
