/**
 * @file task.c
 * @brief 上电校准、ADC 驱动电流环启动，以及冻结波形的后台导出。
 */
#include "task.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "motor_ctrl.h"
#include "sen.h"
#include "foc_trace.h"

void task_init(void)
{
    Init_Over = false;
    MotorInit();
    MotorStop();
    if (UART_DMA_Init() != HAL_OK ||
        HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK ||
        AdcSamplingStart() != HAL_OK)
    {
        MotorStop();
        Error_Handler();
    }

    /* 定时器和 ADC 已运行，三相输出关闭，先等待霍尔电流传感器稳定。 */
    HAL_Delay(100);
    AdcSampleOffset();
    PostionToZeroDouble();
    if (AdcFault != 0U)
    {
        MotorStop();
        Error_Handler();
    }

    VariableClear();
    EnCoSpeed.EncoLastCnt = TIM3->CNT;
    EnCoSpeed.SpeedClock = 0U;
    Svpwm.Ualpha = Svpwm.Ubeta = 0.0f;
    SvpwmCal(&Svpwm);
    HAL_Delay(1); /* 等待中性占空比装载，再开放三相功率输出。 */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (AdcFault != 0U)
    {
        __set_PRIMASK(primask);
        MotorStop();
        Error_Handler();
    }
    MotorRun();
    (void)FocTraceArm(1U);
    __DMB();
    Init_Over = true;
    __set_PRIMASK(primask);
}

void task_loop(void)
{
    static uint32_t last_led;
    FocTraceService();
    const uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_led) >= 500U)
    {
        last_led = now;
        ToggleLed(LED_B);
    }
}
