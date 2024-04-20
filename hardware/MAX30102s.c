/**
****************************************************************************************
* @Description: 脉搏血氧仪和心率监测模块 驱动程序
* @Author: Haolin Wang
* @Date: 2023-03-20 15:55:29
* @LastEditTime: 2023-03-25 20:48:26
* @Note: 开源参考：https://msalamon.pl/palec-mi-pulsuje-pulsometr-max30102-pod-kontrola-stm32/
* LED位置交换问题：https://github.com/aromring/MAX30102_by_RF/issues/13#issue-601473302
* 淘宝模块设计问题：http://bbs.eeworld.com.cn/thread-548367-1-1.html
****************************************************************************************
*/

#include "main.h"
#include "i2c.h"
#include "MAX3010x.h"
#include "ALGORITHM_HRANDSPO2.h"
#include "stdio.h"

// 硬件 IIC 初始化
I2C_HandleTypeDef *i2c_max30102s;

// 全局变量
volatile uint32_t max30102s_led1_buffer[BUFFER_SIZE_MAX3010x] = {0};		// LED1 传感器数据
volatile uint32_t max30102s_led2_buffer[BUFFER_SIZE_MAX3010x] = {0};	// LED2 传感器数据
volatile uint16_t max30102s_buffer_head = 0;		// 缓存器头
volatile uint16_t max30102s_buffer_tail = 0;		// 缓存器尾
volatile uint16_t max30102s_collected_samples; // 采集的样本
uint8_t heart_rate_max30102s = 0;					// 心率值
int8_t hr_valid_max30102s = 0;					// 心率有效
float spo2_value_max30102s = 0;						// 血氧值
int8_t spo2_valid_max30102s = 0;					// 血氧值有效

// 状态机初始化
Max3010xState max30102s_state;

/**
****************************************************************************************
* @Funticon name: IIC写寄存器
* @Berif:
* @Note:
* @param {uint8_t} uch_addr	寄存器地址
* @param {uint8_t} uch_data	写入的数据头地址
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
* @Funticon name: IIC读寄存器
* @Berif:
* @Note:
* @param {uint8_t} uch_addr	IIC地址
* @param {uint8_t} *puch_data	读取的数据
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
* @Funticon name: 写寄存器指定位
* @Berif:
* @Note:
* @param {uint8_t} Register	寄存器地址
* @param {uint8_t} Bit			寄存器对应位
* @param {uint8_t} Value		修改值
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_WriteRegisterBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
	uint8_t tmp;
	if (MAX30102_OK != max30102s_ReadReg(Register, &tmp)) // 先获取对应寄存器的值，并暂存到tmp，并判断IIC是否正常读取
	{
		return MAX30102_ERROR;
	}

	/*将需要重新配置的位先清零再赋值*/
	tmp &= ~(1 << Bit);			  // 1左移bit位 再取反（修改位位0其他均为1），将需要修改的位和0做 “与” 运算达到某一位清零的效果
	tmp |= (Value & 0x01) << Bit; // 将修改值和1进行与运算，确保其他位为已知状态，再进行逻辑左移到对应位，并通过 “或”运算实现赋值

	if (MAX30102_OK != max30102s_WriteReg(Register, tmp)) // 将tmp的值写入寄存器，并判断IIC是否正常写入
	{
		return MAX30102_ERROR;
	}

	return MAX30102_OK;
}

/**
****************************************************************************************
* @Funticon name: IIC读FIFO数据
* @Berif:
* @Note:
* @param {volatile uint32_t} *pun_red_led	红色LED传感器数据变量地址
* @param {volatile uint32_t} *pun_ir_led	红外LED传感器数据变量地址
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_ReadFifo(volatile uint32_t *pun_led_1, volatile uint32_t *pun_led_2)
{
	uint8_t ach_i2c_data[6];

	/*FIFO数据读取 一个样本需读取读取6个IIC字节*/
	if (HAL_I2C_Mem_Read(i2c_max30102s, MAX3010x_ADDRESS, REG_FIFO_DATA, 1, ach_i2c_data, 6, I2C_TIMEOUT) != HAL_OK)
	{
		return MAX30102_ERROR;
	}

	/* 内存清理 */
	*pun_led_1 = 0;
	*pun_led_2 = 0;

	/* 红色LED数据处理 */
	*pun_led_1 += (unsigned char) ach_i2c_data[0] << 16;	// 数据叠加 左移16位并赋值
	*pun_led_1 += (unsigned char) ach_i2c_data[1] << 8;		// 数据叠加 左移8位并赋值
	*pun_led_1 += (unsigned char) ach_i2c_data[2];					// 数据叠加

	/* 红外LED数据处理 */
	*pun_led_2 += (unsigned char) ach_i2c_data[3] << 16;		// 数据叠加 左移16位并赋值
	*pun_led_2 += (unsigned char) ach_i2c_data[4] << 8;		// 数据叠加 左移8位并赋值
	*pun_led_2 += (unsigned char) ach_i2c_data[5];					// 数据叠加

	/* FIFO数据处理 FIFO DATA[23:18]未使用 进行清零 */
	*pun_led_1 &= 0x03FFFF;
	*pun_led_2 &= 0x03FFFF;

	return MAX30102_OK;
}

/**
****************************************************************************************
* @Funticon name: 设置中断使能寄存器
* @Berif: Enable：置1
* @Note:
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_InterruptEnable(uint8_t Enable)
{
	// 几乎全满 中断使能：FIFO 写指针剩余一定数量的可用空间时，触发中断使能
	if (MAX30102_OK != max30102s_WriteRegisterBit(REG_INTR_ENABLE_1, INT_A_FULL_BIT, Enable))
	{
		return MAX30102_ERROR;
	}

	// 新的 FIFO 数据就绪 中断使能：数据 FIFO 中有新样本时,触发中断使能
	if (MAX30102_OK != max30102s_WriteRegisterBit(REG_INTR_ENABLE_1, INT_PPG_RDY_BIT, Enable))
	{
		return MAX30102_ERROR;
	}

	return MAX30102_OK;
}

/**
****************************************************************************************
* @Funticon name: 读取中断状态寄存器
* @Berif:
* @Note:
* @param {uint8_t} *Status	中断标志位状态
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_ReadInterruptStatus(uint8_t *Status)
{
	uint8_t tmp;
	*Status = 0;

	/*读取中断状态寄存器1*/
	if (MAX30102_OK != max30102s_ReadReg(REG_INTR_STATUS_1, &tmp))
	{
		return MAX30102_ERROR;
	}
	*Status |= tmp & 0xE1; // 状态寄存器1 4个中断状态 与上 1110 0001（0xE1）对无效位清零，再通过或运算提取中断状态到 status

	return MAX30102_OK;
}

/**
****************************************************************************************
* @Funticon name: 中断回调函数
* @Berif: 外部中断触发时，进入此函数，获取传感器数据
* @Note:
* @return {*}
****************************************************************************************
*/
void max30102s_InterruptCallback(void)
{
	uint8_t Status;
	while (MAX30102_OK != max30102s_ReadInterruptStatus(&Status))
		; // 读取中断状态寄存器

	/*对 FIFO发出即将数据全满中断 的处理 */
	if (Status & (1 << INT_A_FULL_BIT)) // 判断是否产生数据将满中断
	{
		for (uint8_t i = 0; i < MAX3010x_FIFO_ALMOST_FULL_SAMPLES; i++)
		{
			while (MAX30102_OK != max30102s_ReadFifo(&max30102s_led1_buffer[max30102s_buffer_head], &max30102s_led2_buffer[max30102s_buffer_head])) {
				;
			}	// a+i等价&a[i], *(a+i)等价于a[i]
			max30102s_buffer_head = (max30102s_buffer_head + 1) % BUFFER_SIZE_MAX3010x; // 缓冲器头指针加一,通过求余运算符进行循环（求余%：如果a<b的话，a%b的商为0,余数就是a）
			max30102s_collected_samples++;							 // 继续采集样本
		}
	}

	/*对 FIFO发出新的数据就绪中断 的处理*/
	if (Status & (1 << INT_PPG_RDY_BIT)) // 判断是否产生 新数据就绪中断
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
* @Funticon name: FIFO配置
* @Berif:
* @Note:
* @param {uint8_t} Address	FIFO寄存器内部存储地址（bits）
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_FifoConfiguration(uint8_t Address)
{
	// FIFO写指针配置：FIFO_WR_PTR[4:0],高三位清零
	if (MAX30102_OK != max30102s_WriteReg(REG_FIFO_WR_PTR, (Address & 0x1F)))
	{
		return MAX30102_ERROR;
	}

	// FIFO溢出计数器配置：：OVF_COUNTER[4:0]
	if (MAX30102_OK != max30102s_WriteReg(REG_OVF_COUNTER, (Address & 0x1F)))
	{
		return MAX30102_ERROR;
	}

	// FIFO读指针配置：FIFO_RD_PTR[4:0]
	if (MAX30102_OK != max30102s_WriteReg(REG_FIFO_RD_PTR, (Address & 0x1F)))
	{
		return MAX30102_ERROR;
	}
	return MAX30102_OK;
}

/**
****************************************************************************************
* @Funticon name: FIFO样本平均配置
* @Berif:
* @Note:
* @param {uint8_t} Value	每个 FIFO 样本的平均样本数
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_FifoSampleAveraging(uint8_t Value)
{
	uint8_t tmp;

	/*获取当前FIFO配置寄存器状态设置*/
	if (MAX30102_OK != max30102s_ReadReg(REG_FIFO_CONFIG, &tmp))
	{
		return MAX30102_ERROR;
	}
	tmp &= ~(0x07);				// 处理为已知状态
	tmp |= (Value & 0x07) << 5; // 对bit[7:5]重新赋值

	/*修改FIFO配置寄存器 样本平均配置位*/
	if (MAX30102_OK != max30102s_WriteReg(REG_FIFO_CONFIG, tmp))
	{
		return MAX30102_ERROR;
	}
	return MAX30102_OK;
}

/**
****************************************************************************************
* @Funticon name: FIFO循环使能
* @Berif:
* @Note:
* @param {uint8_t} Enable	使能
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_FifoRolloverEnable(uint8_t Enable)
{
	return max30102s_WriteRegisterBit(REG_FIFO_CONFIG, FIFO_CONF_FIFO_ROLLOVER_EN_BIT, (Enable & 0x01));
}

/**
****************************************************************************************
* @Funticon name: FIFO几乎全满操作
* @Berif:
* @Note: 该寄存器设置发出中断时 FIFO 中剩余的数据样本数（3 字节/样本）。
* 例如，如果该字段设置为 0x0h，则当 FIFO 中剩余 0 个数据样本时（所有 32 个 FIFO 字都有未读数据），将发出中断。
* 此外，如果该字段设置为 0xFh ，则当 FIFO 中剩余 15 个数据样本（17 个 FIFO 数据样本有未读数据）时发出中断。
* @param {uint8_t} Value	使能
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_FifoAlmostFullValue(uint8_t Value)
{
	/*判断预设未读数据数量是否超出范围*/
	if (Value < 17)
	{
		Value = 17;
	}
	if (Value > 32)
	{
		Value = 32;
	}

	Value = 32 - Value; // 减去未读数据字节数 得到空数据数量
	uint8_t tmp;

	/* 读取fifo寄存器 */
	if (MAX30102_OK != max30102s_ReadReg(REG_FIFO_CONFIG, &tmp))
	{
		return MAX30102_ERROR;
	}
	tmp &= ~(0x0F); // 对FIFO_A_FULL[3:0]进行清零，确保状态已知

	/* 写入fifo寄存器 */
	tmp |= (Value & 0x0F); // 将设定值赋给临时储存区
	if (MAX30102_OK != max30102s_WriteReg(REG_FIFO_CONFIG, tmp))
	{
		return MAX30102_ERROR;
	}
	return MAX30102_OK;
}

/**
****************************************************************************************
* @Funticon name: 模式配置——传感器复位
* @Berif:
* @Note:
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_Reset(void)
{
	uint8_t tmp = 0xFF;
	/*执行复位*/
	if (MAX30102_OK != max30102s_WriteReg(REG_MODE_CONFIG, 0x40))
	{
		return MAX30102_ERROR;
	}

	/*等待复位完成*/
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
* @Funticon name: 模式配置——led工作模式设置
* @Berif:
* @Note:
* @param {uint8_t} Mode	工作模式
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_SetMode(uint8_t Mode)
{
	uint8_t tmp;
	/*获取传感器工作模式*/
	if (MAX30102_OK != max30102s_ReadReg(REG_MODE_CONFIG, &tmp))
	{
		return MAX30102_ERROR;
	}
	tmp &= ~(0x07); // 指定位清零

	/*工作模式设置*/
	tmp |= (Mode & 0x07); // 暂存设置位
	if (MAX30102_OK != max30102s_WriteReg(REG_MODE_CONFIG, tmp))
	{
		return MAX30102_ERROR;
	}
	return MAX30102_OK;
}

/**
****************************************************************************************
* @Funticon name: SpO2 配置
* @Berif:
* @Note:
* @param {uint8_t} Value	设置值
* @return {*}
****************************************************************************************
*/
MAX30102_STATUS max30102s_SpO2AdcRange(uint8_t Value)
{
	uint8_t tmp;

	/* 获取早期分辨率 */
	if (MAX30102_OK != max30102s_ReadReg(REG_SPO2_CONFIG, &tmp))
	{
		return MAX30102_ERROR;
	}
	tmp &= ~(0x03);
	tmp |= ((Value & 0x03) << 5); // SpO2_ADC_RGE bit[6:5]

	/* 重新设置分辨率 */
	if (MAX30102_OK != max30102s_WriteReg(REG_SPO2_CONFIG, tmp))
	{
		return MAX30102_ERROR;
	}
	return MAX30102_OK;
}

/* 采样率设置 */
MAX30102_STATUS max30102s_SpO2SampleRate(uint8_t Value)
{
	uint8_t tmp;

	/* 获取早期采样率 */
	if (MAX30102_OK != max30102s_ReadReg(REG_SPO2_CONFIG, &tmp))
	{
		return MAX30102_ERROR;
	}
	tmp &= ~(0x07);
	tmp |= ((Value & 0x07) << 2); // SpO2_SR bit[4:2]

	/* 重新设置分辨率 */
	if (MAX30102_OK != max30102s_WriteReg(REG_SPO2_CONFIG, tmp))
	{
		return MAX30102_ERROR;
	}
	return MAX30102_OK;
}

/*LED脉冲宽度设置*/
MAX30102_STATUS max30102s_SpO2LedPulseWidth(uint8_t Value)
{
	uint8_t tmp;

	/*获取早期脉冲宽度*/
	if (MAX30102_OK != max30102s_ReadReg(REG_SPO2_CONFIG, &tmp))
	{
		return MAX30102_ERROR;
	}
	tmp &= ~(0x03);
	tmp |= (Value & 0x03); // LED_PW bit[1:0]

	/* 重新设置LED脉冲宽度 */
	if (MAX30102_OK != max30102s_WriteReg(REG_SPO2_CONFIG, tmp))
	{
		return MAX30102_ERROR;
	}

	return MAX30102_OK;
}

/**
****************************************************************************************
* @Funticon name: LED 脉冲放大器配置
* @Berif:
* @Note: LED电流 == 寄存器设置值 * 0.2 （mA）
* @param {uint8_t} Value	设置值
* @return {*}
****************************************************************************************
*/
// LED1脉冲放大器设置
MAX30102_STATUS max30102s_Led1PulseAmplitude(uint8_t Value)
{
	if (MAX30102_OK != max30102s_WriteReg(REG_LED1_PA, Value))
	{
		return MAX30102_ERROR;
	}
	return MAX30102_OK;
}

/* LED2脉冲放大器设置 */
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
* @Funticon name: 人机交互
* @Berif: 用于主函数调用读取心率血氧
* @Note:
* @return {*}
****************************************************************************************
*/
/*心率获取*/
uint8_t max30102s_GetHeartRate(void)
{
	return heart_rate_max30102s;
}
/*血氧获取*/
float max30102s_GetSpO2Value(void)
{
	return spo2_value_max30102s;
}

/**
****************************************************************************************
* @Funticon name: 状态机程序段
* @Berif:
* @Note: 具有4种状态的状态机，同时在中断期间一直收集样本。
* @return {*}
****************************************************************************************
*/
void max30102s_Task(void)
{
	static bool flag = false;    //启动标记;
	switch (max30102s_state)
	{
		case MAX3010x_STATE_BEGIN: // 现态：准备采样状态
		{

			/*动作*/
			max30102s_collected_samples = 0; // 采集样本为0
			max30102s_buffer_tail = max30102s_buffer_head;		  // 缓存区头尾相连
			max30102s_Led1PulseAmplitude(MAX3010x_RED_LED_CURRENT_HIGH); // 红LED~7mA
			max30102s_Led2PulseAmplitude(MAX3010x_IR_LED_CURRENT_HIGH);	// 红外~7mA

			/*次态*/
			max30102s_state = MAX3010x_STATE_CALIBRATE; // 检测到手指接近 转换为校准态
			break;
		}	// MAX3010x_STATE_BEGIN

		case MAX3010x_STATE_CALIBRATE: // 现态：物体接触到传感器，采集过去5秒样本将有效数据填充缓冲区
		{

			/*条件*/
			if (max30102s_collected_samples > BUFFER_SIZE) // 采集的样本数超出缓冲区空间
			{
				/*次态*/
				max30102s_state = MAX3010x_STATE_CALCULATE; // 采集到一定数量的样本 转换为计算态
			}
			break;
		}	// MAX3010x_STATE_CALIBRATE

		case MAX3010x_STATE_CALCULATE: // 现态：收集到一定数量样本后，通过算法计算
		{

			/*动作*/
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

			max30102s_buffer_tail = (max30102s_buffer_tail + FS) % BUFFER_SIZE_MAX3010x; // 缓冲区尾
			max30102s_collected_samples = 0;						  // 收集到的样本数归零

			/*次态*/
			max30102s_state = MAX3010x_STATE_COLLECT_NEXT_PORTION; // 采集态：继续采集新的样本
			break;
		}	// MAX3010x_STATE_CALCULATE

		case MAX3010x_STATE_COLLECT_NEXT_PORTION: // 现态：传感器一直工作到收集下一部分样品，下一秒之后，它进入状态 3 并重复循环，直到物体离开传感器
		{
			/*条件 2*/
			if (max30102s_collected_samples > FS) // 已有样本超出每秒平均样本数
			{
				/*次态*/
				max30102s_state = MAX3010x_STATE_CALCULATE; // 对采集的样本进行算法处理
			}
			flag = true;
			break;
		}	// MAX3010x_STATE_COLLECT_NEXT_PORTION

	    default:
	    {
	        ;
	        break;
	    }	// 默认情况处理
	}	// switch
}

/**
****************************************************************************************
* @Funticon name: 传感器初始化
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

	/*软件复位*/
	if (MAX30102_OK != max30102s_Reset()) // 复位 MAX30102
	{
		return MAX30102_ERROR;
	}
	/*读寄存器*/
	if (MAX30102_OK != max30102s_ReadReg(0, &uch_dummy))
	{
		return MAX30102_ERROR;
	}
	/*FIFO配置：写指针,溢出计数器,读指针*/
	if (MAX30102_OK != max30102s_FifoConfiguration(0x00))
	{
		return MAX30102_ERROR;
	}
	/*FIFO采样平均*/
	if (MAX30102_OK != max30102s_FifoSampleAveraging(FIFO_SMP_AVE_1)) // 每个 FIFO 样本的平均样本数为4
	{
		return MAX30102_ERROR;
	}
	/*FIFO循环使能*/
	if (MAX30102_OK != max30102s_FifoRolloverEnable(0)) // 设置为0，FIFO不会更新，直到读取 FIFO_DATA 或更改 WRITE/READ 指针位置。
	{
		return MAX30102_ERROR;
	}
	/*FIFO满值设置*/
	if (MAX30102_OK != max30102s_FifoAlmostFullValue(MAX3010x_FIFO_ALMOST_FULL_SAMPLES)) //  未读样本X个时发出中断。
	{
		return MAX30102_ERROR;
	}
	/*模式设置*/
	if (MAX30102_OK != max30102s_SetMode(MODE_SPO2_MODE)) // 血氧模式
	{
		return MAX30102_ERROR;
	}
	/*ADC范围*/
	if (MAX30102_OK != max30102s_SpO2AdcRange(SPO2_ADC_RGE_8192)) // ADC满量程4096nA
	{
		return MAX30102_ERROR;
	}
	/*采样率*/
	if (MAX30102_OK != max30102s_SpO2SampleRate(SPO2_SAMPLE_RATE_1000)) // 采样率
	{
		return MAX30102_ERROR;
	}
	/*LED脉冲宽度*/
	if (MAX30102_OK != max30102s_SpO2LedPulseWidth(SPO2_PULSE_WIDTH_411)) // 脉冲宽度411us 分辨率18bits
	{
		return MAX30102_ERROR;
	}
	/*红色LED脉冲幅度*/
	if (MAX30102_OK != max30102s_Led1PulseAmplitude(MAX3010x_RED_LED_CURRENT_LOW)) // 红色LED脉冲幅值0mA
//	if (MAX30102_OK != max30102s_Led1PulseAmplitude(MAX3010x_RED_LED_CURRENT_HIGH)) // 红色LED脉冲幅值0mA
	{
		return MAX30102_ERROR;
	}
	/*红外LED脉冲幅度*/
	if (MAX30102_OK != max30102s_Led2PulseAmplitude(MAX3010x_IR_LED_CURRENT_LOW)) // 红外LED脉冲幅值0.2mA
//	if (MAX30102_OK != max30102s_Led2PulseAmplitude(MAX3010x_IR_LED_CURRENT_HIGH)) // 红外LED脉冲幅值0.2mA
	{
		return MAX30102_ERROR;
	}
	/*FIFO 中断使能 */
	if (MAX30102_OK != max30102s_InterruptEnable(1))
	{
		return MAX30102_ERROR;
	}
	max30102s_state = MAX3010x_STATE_BEGIN; // 状态机初始状态

	return MAX30102_OK;
}
