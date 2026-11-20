/**
 * @file task.c
 * @brief 上电校准、ADC 驱动电流环启动，以及实时波形的 DMA 发送。
 */
#include "task.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "motor_ctrl.h"
#include "sen.h"
#include "foc_wave.h"

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

    (void)FocWaveInit(); /* 遥测失败只禁用波形，不阻止电机启动。 */
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
    __DMB();
    Init_Over = true;
    __set_PRIMASK(primask);
}

void task_loop(void)
{
    static uint32_t last_led;
    FocWaveService();
    const uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_led) >= 500U)
    {
        last_led = now;
        ToggleLed(LED_B);
    }
}
