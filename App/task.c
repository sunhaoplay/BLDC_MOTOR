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
#include "htmi_app.h"
/* 无需在线调试的上传诊断：每 500 ms 判断进展，每 4 s 用红灯闪次数报状态。
 * 仅在主循环操作 LED，不向串口插入诊断文本，也不等待发送。
 */
static void WaveLedService(uint32_t now)
{
    static uint32_t last_check, last_sent, last_errors, last_dropped;
    static uint32_t pattern_start;
    static uint8_t code = 3U;

    if ((uint32_t)(now - last_check) >= 500U)
    {
        last_check = now;
        const uint32_t sent = FocWaveSent;
        const uint32_t errors = FocWaveTxErrors;
        const uint32_t dropped = FocWaveDropped;
        uint8_t next;
        if (FocWaveFault == 1U)
            next = 1U; /* 上传速率超过配置的串口带宽。 */
        else if (FocWaveFault != 0U)
            next = 2U; /* 未取得串口发送权。 */
        else if (FocWaveEnabled == 0U || !Init_Over || FocWaveSamples == 0U)
            next = 3U; /* 未开始采集或已停止。 */
        else if (errors != last_errors)
            next = 5U; /* DMA 启动/传输错误。 */
        else if (sent == last_sent)
            next = 4U; /* 已有数据，但本窗口没有确认发送完成。 */
        else if (dropped != last_dropped)
            next = 6U; /* 发送在进行，但队列发生拥堵丢点。 */
        else
            next = 0U; /* 已确认有新的串口发送完成。 */

        last_sent = sent;
        last_errors = errors;
        last_dropped = dropped;
        if (next != code)
        {
            code = next;
            pattern_start = now;
        }
    }

    if (code == 0U)
        TurnOnLed(LED_G);
    else
        TurnOffLed(LED_G);

    const uint32_t phase = (uint32_t)(now - pattern_start) % 4000U;
    if (code != 0U && phase < (uint32_t)code * 400U && phase % 400U < 200U)
        TurnOnLed(LED_R);
    else
        TurnOffLed(LED_R);
}

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
    Motor.CtrlMode=SPEED_LOOP;
    Init_Over = true;
    __set_PRIMASK(primask);
}

void task_loop(void)
{
    static uint32_t last_led;
    static uint32_t last_wave_led;
    FocWaveService();
    const uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_wave_led) >= 100U)
    {
        last_wave_led = now;
        WaveLedService(now);
    }
    if ((uint32_t)(now - last_led) >= 500U)
    {
        last_led = now;
        ToggleLed(LED_B);
        // shell_send("%u,%.3f\r\n",TIM3->CNT,Motor.E_theta);

    }
    VrTask();
}
