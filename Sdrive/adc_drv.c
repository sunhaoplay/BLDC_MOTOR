/**
 * @file adc_drv.c
 * @brief TIM1 OC4REF -> 双 ADC 同步注入 -> 电流闭环；1 kHz 规则组 DMA 采慢信号。
 */
#include "adc_drv.h"
#include "adc.h"
#include "tim.h"
#include "motor_ctrl.h"
#include "sen.h"
#include "foc_wave.h"

ADC_Sample AdcPara = ADC_Sample_DEFAULTS;
ADC_Sample_F AdcFilPara = ADC_Sample_F_DEFAULTS;
uint16_t ADC_Slow[ADC_SLOW_CHANNELS];
volatile uint16_t AdcRawU, AdcRawW;
volatile uint32_t AdcCurrentSamples;
volatile uint32_t AdcSlowSamples;
volatile uint32_t AdcFault;
volatile uint32_t AdcControlCyclesMax;
volatile uint32_t AdcControlDeadlineMisses;

static volatile uint32_t offset_count;
static volatile uint8_t offset_active;
static volatile uint8_t offset_ready;
static uint32_t offset_sum_u, offset_sum_w;
static volatile uint8_t slow_busy;
static uint16_t slow_divider;

#define ADC_SLOW_DIVIDER (PWM_FRQ / 1000U)
#define ADC_FAULT_SYNC       0x10000U
#define ADC_FAULT_CALIBRATE  0x20000U
#define ADC_FAULT_SLOW       0x40000U

static void AdcFail(uint32_t fault)
{
    AdcFault |= fault;
    MotorStop();
}

HAL_StatusTypeDef AdcSamplingStart(void)
{
    MotorStop();
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(&htim1);
    if (TIM1->ARR != TIM1_PERIOD - 1U)
        return HAL_ERROR;

    /* CH4 无引脚输出。峰顶前 0.5 us 上升，避免 ARR-1 产生过窄的 TRGO 脉冲。 */
    CLEAR_BIT(TIM1->CCER, TIM_CCER_CC4E);
    TIM1->CCR4 = TIM1->ARR - TIM1_SAMPLE_LEAD_TICKS;
    TIM1->EGR = TIM_EGR_UG;
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK)
        return HAL_ERROR;
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
        return HAL_ERROR;
    return HAL_TIM_Base_Start(&htim1);
}

void AdcSampleOffset(void)
{
    const uint32_t primask = __get_PRIMASK();
    const uint32_t start = HAL_GetTick();
    __disable_irq();
    offset_active = 0U;
    offset_ready = 0U;
    offset_count = 0U;
    offset_sum_u = offset_sum_w = 0U;
    __DMB();//数据内存屏障，保证上面的清零在置 offset_active=1 之前对中断可见（编译器/CPU 不重排）
    offset_active = 1U;
    __set_PRIMASK(primask);

    while (offset_ready == 0U)
    {
        if (AdcFault != 0U || (uint32_t)(HAL_GetTick() - start) >= 250U)
        {
            AdcFail(ADC_FAULT_CALIBRATE);
            Error_Handler();
        }
        HAL_Delay(1);
    }
}

void GetAdc(void)
{
    const uint16_t raw_u = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    const uint16_t raw_w = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    AdcRawU = raw_u;
    AdcRawW = raw_w;
    ++AdcCurrentSamples;
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JSTRT | ADC_FLAG_JEOC);//adc2是从机，没有开中断，需要手动清除标志位

    if (offset_active != 0U)
    {
        offset_sum_u += raw_u;
        offset_sum_w += raw_w;
        if (++offset_count == ADC_OFFSET_SAMPLES)
        {
            const int16_t u = (int16_t)((offset_sum_u + ADC_OFFSET_SAMPLES / 2U) / ADC_OFFSET_SAMPLES);
            const int16_t w = (int16_t)((offset_sum_w + ADC_OFFSET_SAMPLES / 2U) / ADC_OFFSET_SAMPLES);
            AdcPara.OffsetCurU = AdcFilPara.OffsetCurU = u;
            AdcPara.OffsetCurW = AdcFilPara.OffsetCurW = w;
            AdcFilPara.CurU = AdcFilPara.CurV = AdcFilPara.CurW = 0.0f;
            offset_active = 0U;
            __DMB();//保证前面写偏移/清电流对等待线程可见
            offset_ready = 1U;
        }
    }

    if (offset_ready == 0U)
        return;
    /* 保留原有电流符号定义，传感器增益及极性仍须通过实测确认。 */
    AdcPara.CurU = (float)((int32_t)AdcFilPara.OffsetCurU - raw_u) * I_GAIN;
    AdcPara.CurW = (float)((int32_t)AdcFilPara.OffsetCurW - raw_w) * I_GAIN;
    AdcPara.CurV = -AdcPara.CurU - AdcPara.CurW;
    AdcFilPara.CurU = LPF_I_RUN_B * AdcFilPara.CurU + LPF_I_RUN_A * AdcPara.CurU;
    AdcFilPara.CurW = LPF_I_RUN_B * AdcFilPara.CurW + LPF_I_RUN_A * AdcPara.CurW;
    AdcFilPara.CurV = -AdcFilPara.CurU - AdcFilPara.CurW;
}

static void AdcSlowStart(void)
{
    if (++slow_divider < ADC_SLOW_DIVIDER)
        return;
    slow_divider = 0U;
    if (slow_busy != 0U)
    {
        AdcFail(ADC_FAULT_SLOW);
        return;
    }
    slow_busy = 1U;
    /* 仅启动 ADC1 规则组；不调用会同时关闭注入采样的 HAL_ADC_Stop_DMA。 */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Slow, ADC_SLOW_CHANNELS) != HAL_OK)
    {
        AdcFail(ADC_FAULT_SLOW);
        return;
    }
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1)
        return;
    AdcPara.Vbus = (float)ADC_Slow[0] * VBUS_GAIN;
    AdcPara.Temp = ADC_Slow[1];
    AdcPara.Vr = (float)ADC_Slow[2];
    /* 更新频率从 20 kHz 改为 1 kHz，保持约 10 Hz 的慢信号低通。 */
    if (AdcSlowSamples == 0U)
    {
        AdcFilPara.Vbus = AdcPara.Vbus;
        AdcFilPara.Temp = (float)AdcPara.Temp;
    }
    else
    {
        AdcFilPara.Vbus += 0.06089863f * (AdcPara.Vbus - AdcFilPara.Vbus);
        AdcFilPara.Temp += 0.06089863f * ((float)AdcPara.Temp - AdcFilPara.Temp);
    }
    ++AdcSlowSamples;
    __DMB();
    slow_busy = 0U;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1 || AdcFault != 0U)
        return;
    const uint32_t start_cycles = DWT->CYCCNT;
    const uint32_t entry_count = TIM1->CNT;
    const uint32_t entry_down = TIM1->CR1 & TIM_CR1_DIR;
    if (__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_JEOC) == RESET)
    {
        AdcFail(ADC_FAULT_SYNC);
        return;
    }
    GetAdc();
    if (offset_ready != 0U)
        AxisDq();

    if (Init_Over)
    {
        //1.开环强托
        // QtMotor();
        //2.有感电流环驱动
        // CurCloseloop(0.5f, Motor.E_theta);
        //3.有感速度环驱动
        SpeedCloseloop(Para.SpeedRef, 1);
        // SpeedCloseloop(1200, 1);
        FocSvpwm();
        /* 采样完成时应处于下数，新的三相 CCR 应在随后的谷底前写完。 */
        const uint32_t count = TIM1->CNT;
        const uint32_t elapsed = DWT->CYCCNT - start_cycles;
        if (entry_down == 0U || (TIM1->CR1 & TIM_CR1_DIR) == 0U || count > entry_count || elapsed >= entry_count)
            ++AdcControlDeadlineMisses;
        /* 测速系数按 PWM_FRQ / SPEED_CNT 定义，使用同样的调用分频。
         * 放在 CCR 写入后，不推迟本拍占空比计算。
         */
        if (++EnCoSpeed.SpeedClock >= SPEED_CNT)
        {
            EnCoSpeed.SpeedClock = 0U;
            MotorAngleGet();
            MotorSpeedGet();
        }
        FocWaveCapture(AdcPara.CurU, AdcPara.CurV, AdcPara.CurW);
    }
    if (AdcFault == 0U)
        AdcSlowStart();
    const uint32_t cycles = DWT->CYCCNT - start_cycles;
    if (cycles > AdcControlCyclesMax)
        AdcControlCyclesMax = cycles;
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1 || hadc->Instance == ADC2)
        AdcFail(HAL_ADC_GetError(hadc) | ADC_FAULT_SLOW);
}
