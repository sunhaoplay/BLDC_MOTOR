/**
 * @file adc_drv.h
 * @brief ADC DMA 采样结果、物理量换算和低通滤波的数据接口。
 *
 * ADC1 通过 DMA 循环更新 ADC_Value[]。驱动把原始码值换算为三相电流和母线
 * 电压，并保存原始量与滤波量两套结果，供 20 kHz FOC 控制和慢速保护逻辑使用。
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
	int16_t OffsetCurV; /**< V 相零点，当前两电阻采样实现未直接使用。 */
	int16_t OffsetCurW; /**< W 相电流采样零点 ADC 码值。 */
} ADC_Sample;

#define ADC_Sample_DEFAULTS {0, 0, 0, 0, 0, 0} // 初始化参数

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

#define ADC_Sample_F_DEFAULTS {0, 0, 0, 0, 0, 0} // 初始化参数

typedef struct
{
	float Yn;      /**< 当前输出 y[n]。 */
	float Yn_last; /**< 上一拍输出 y[n-1]。 */
	float Xn;      /**< 当前输入 x[n]。 */
	float Xn_last; /**< 上一拍输入 x[n-1]。 */
	float Tau;     /**< 补偿环节时间常数。 */
} V_Comp, *P_Vcomp;

#define Voltage_Compensation_DEFAULTS {0, 0, 0, 0, 0}

extern uint16_t ADC_Value[5]; /**< DMA 循环写入的五通道 ADC 原始码值。 */
extern uint8_t NTC_ERROR;     /**< NTC 温度异常标志，由温度处理逻辑维护。 */

extern ADC_Sample AdcPara;
extern ADC_Sample_F AdcFilPara;

/** @brief 在电机无电流时采集并计算 U/W 两相的 ADC 零点。 */
void AdcSampleOffset(void);

/** @brief 将最新 DMA 码值换算成物理量并更新低通滤波结果。 */
void GetAdc(void);

/**
 * @brief 把 NTC 通道 ADC 码值换算为温度。
 * @param ADC_value NTC 分压点的 ADC 原始码值。
 * @param temp 输出温度地址，单位由具体实现约定。
 * @note 本工程当前没有提供该函数的定义，调用前需确认是否由其他模块实现。
 */
void TempTrans(uint16_t ADC_value, float *temp);

#endif
