/**
 * @file adc_drv.h
 * @brief 双 ADC 注入电流采样、规则组慢信号 DMA 和采样诊断接口。
 */

#ifndef _ADC_DRV_H
#define _ADC_DRV_H

#include "main.h"

typedef struct
{
	float Vbus;       /**< 母线电压，单位 V。 */
	float Vr;         /**< 电位器 ADC 原始码值的浮点表示。 */
	uint16_t Temp;    /**< 温度通道 ADC 原始码值，尚未换算为摄氏度。 */

	float CurU;       /**< U 相瞬时电流，单位 A。 */
	float CurV;       /**< V 相重构电流，单位 A。 */
	float CurW;       /**< W 相瞬时电流，单位 A。 */

	int16_t OffsetCurU; /**< U 相电流采样零点 ADC 码值。 */
	int16_t OffsetCurV; /**< V 相由 U/W 相霍尔电流重构，不直接采样。 */
	int16_t OffsetCurW; /**< W 相电流采样零点 ADC 码值。 */
} ADC_Sample;

#define ADC_Sample_DEFAULTS {0}

typedef struct
{
	float Vbus; /**< 低通滤波后的母线电压，单位 V。 */
	float Temp; /**< 低通滤波后的温度 ADC 码值。 */

	float CurU; /**< 低通滤波后的 U 相电流，单位 A。 */
	float CurV; /**< 低通滤波后的 V 相电流，单位 A。 */
	float CurW; /**< 低通滤波后的 W 相电流，单位 A。 */

	int16_t OffsetCurU;
	int16_t OffsetCurV;
	int16_t OffsetCurW;
} ADC_Sample_F;

#define ADC_Sample_F_DEFAULTS {0}

typedef struct
{
	float Yn;      /**< 当前输出 y[n]。 */
	float Yn_last; /**< 上一拍输出 y[n-1]。 */
	float Xn;      /**< 当前输入 x[n]。 */
	float Xn_last; /**< 上一拍输入 x[n-1]。 */
	float Tau;     /**< 补偿环节时间常数。 */
} V_Comp, *P_Vcomp;

#define Voltage_Compensation_DEFAULTS {0, 0, 0, 0, 0}

#define ADC_SLOW_CHANNELS 3U
#define ADC_OFFSET_SAMPLES 1024U

extern uint16_t ADC_Slow[ADC_SLOW_CHANNELS]; /**< DMA: Vbus、温度、电位器；普通 SRAM。 */
extern volatile uint16_t AdcRawU, AdcRawW;
extern volatile uint32_t AdcCurrentSamples;
extern volatile uint32_t AdcSlowSamples;
extern volatile uint32_t AdcFault;
extern volatile uint32_t AdcControlCyclesMax; /**< ADC 回调最大 CPU 周期数。 */
extern volatile uint32_t AdcControlDeadlineMisses; /**< CCR 写入越过预期谷底的次数。 */
extern uint8_t NTC_ERROR;     /**< NTC 温度异常标志，由温度处理逻辑维护。 */

extern ADC_Sample AdcPara;
extern ADC_Sample_F AdcFilPara;

/** @brief 先启动 ADC2、ADC1，再启动 TIM1 内部触发；保持功率输出关闭。 */
HAL_StatusTypeDef AdcSamplingStart(void);

/** @brief 主循环等待注入回调累计 1024 组零电流采样，超时停机。 */
void AdcSampleOffset(void);

/** @brief 在 ADC1 注入完成中断读取 U/W、重构 V，并更新电流低通。 */
void GetAdc(void);

/**
 * @brief 把 NTC 通道 ADC 码值换算为温度。
 * @param ADC_value NTC 分压点的 ADC 原始码值。
 * @param temp 输出温度地址，单位由具体实现约定。
 * @note 本工程当前没有提供该函数的定义，调用前需确认是否由其他模块实现。
 */
void TempTrans(uint16_t ADC_value, float *temp);

#endif
