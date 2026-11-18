/**
 * @file adc_drv.c
 * @brief 实现 ADC 零点校准、物理量换算和一阶低通滤波。
 *
 * DMA 通道顺序由 CubeMX 配置决定：代码使用索引 0/3 采 U/W 相电流，索引 2
 * 采母线电压，索引 1 采温度，索引 4 采电位器。GetAdc() 在 TIM1 20 kHz
 * 更新中断中调用，因此换算和滤波必须保持短小、确定且不可阻塞。
 */

#include "task.h"
#include "motor_para.h"
ADC_Sample AdcPara = ADC_Sample_DEFAULTS;       /**< 当前一拍未经低通的采样物理量。 */
ADC_Sample_F AdcFilPara = ADC_Sample_F_DEFAULTS; /**< 当前一拍低通后的采样物理量。 */

uint16_t ADC_Value[5];

/**
 * @brief 在零电流条件下估计 U、W 两相电流采样的 ADC 偏置。
 *
 * 外层执行 10 次，每次等待并读取约 100 ms 后把该次末值累加，最终取 10 次平均。
 * 后续相电流换算使用 I=(Offset-ADC)*I_GAIN 消除运放和 ADC 的静态零漂。
 * @warning 校准期间功率级必须没有相电流，否则该电流会被误认为零点并带入控制环。
 */
void AdcSampleOffset(void)
{
	uint8_t i = 0;
	uint8_t j = 0;
	uint16_t Sum1 = 0;
	uint16_t Sum2 = 0;

	for (i = 0; i < 10; i++)
	{
		for (j = 0; j < 100; j++)
		{
			AdcPara.OffsetCurU = ADC_Value[0];
			AdcPara.OffsetCurW = ADC_Value[3];

			HAL_Delay(1);
		}
		Sum1 += AdcPara.OffsetCurU;
		Sum2 += AdcPara.OffsetCurW;
	}
	AdcFilPara.OffsetCurU = Sum1 / 10;
	AdcFilPara.OffsetCurW = Sum2 / 10;
}

/**
 * @brief 将 ADC DMA 原始码值换算为三相电流、母线电压和辅助通道值。
 *
 * U/W 相：I=(Offset-ADC)*I_GAIN；V 相由星形电机无中性线条件
 * Iu+Iv+Iw=0 重构，即 Iv=-Iu-Iw。母线电压使用 Vbus=ADC*VBUS_GAIN。
 */
static void AdcSample(void)
{
	AdcPara.CurU = (float)(AdcFilPara.OffsetCurU - ADC_Value[0]) * I_GAIN;
	AdcPara.CurW = (float)(AdcFilPara.OffsetCurW - ADC_Value[3]) * I_GAIN;
	AdcPara.CurV = -AdcPara.CurU - AdcPara.CurW;

	AdcPara.Vbus = (float)ADC_Value[2] * VBUS_GAIN;
	AdcPara.Temp = (float)ADC_Value[1];
	AdcPara.Vr = (float)ADC_Value[4];
}

/**
 * @brief 对电流、母线电压和温度执行一阶离散低通滤波。
 *
 * 通用形式为 y[n]=B*y[n-1]+A*x[n]。电流采用较高截止频率的 RUN 系数以保留
 * 控制带宽；母线电压和温度采用较低截止频率的 STOP 系数以抑制慢变量噪声。
 */
static void AdcSampleDeal(void)
{
	AdcFilPara.CurU = LPF_I_RUN_B * AdcFilPara.CurU + LPF_I_RUN_A * AdcPara.CurU;
	AdcFilPara.CurV = LPF_I_RUN_B * AdcFilPara.CurV + LPF_I_RUN_A * AdcPara.CurV;
	AdcFilPara.CurW = LPF_I_RUN_B * AdcFilPara.CurW + LPF_I_RUN_A * AdcPara.CurW;
	AdcFilPara.Vbus = AdcFilPara.Vbus * LPF_I_STOP_B + AdcPara.Vbus * LPF_I_STOP_A;
	AdcFilPara.Temp = AdcFilPara.Temp * LPF_I_STOP_B + AdcPara.Temp * LPF_I_STOP_A;
}

/**
 * @brief 更新本控制周期的原始采样量和滤波采样量。
 * @note 由 TIM1 更新中断每 50 us 调用一次；不得在此函数中加入延时或阻塞操作。
 */
void GetAdc(void)
{
	AdcSample();
	AdcSampleDeal();
}
