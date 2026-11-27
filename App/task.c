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
#include "key_drv.h"
#include "fault.h"
typedef struct
{
	uint8_t run;                 /**< 就绪标志：1 表示等待主循环执行，0 表示未到期。 */
	uint16_t timCount;           /**< 当前倒计时，单位为 1 ms 调度节拍。 */
	uint16_t timRload;           /**< 周期重载值，任务执行周期约为 timRload ms。 */
	void (*pTaskFuncCb)(void);   /**< 到期后由主循环调用的无参数业务函数。 */
} TaskComps_t;

static TaskComps_t g_taskComps[] =
	{
#if VR_OR_PC
		{0, 1, 1, VrTask}, // 电位器处理任务
#else
		{0, 10, 10, RecevToVofa}, // 接收数据任务
#endif
		{0, 5, 5, HmiTask},		 // 按键处理任务
		//{0, 10, 10, SendToVofa}, // 发送数据任务
		{0, 1, 1, FaultCheck},	 // 故障检测任务

		/* 添加业务功能模块 */
};

#define TASK_NUM_MAX (sizeof(g_taskComps) / sizeof(g_taskComps[0])) /**< 静态任务表中的任务数量。 */

/**
 * @brief 扫描任务表并执行所有已经到期的后台任务。
 *
 * 执行前先清除 run 标志，因此同一任务在一次扫描中最多执行一次。如果回调为空，
 * 直接跳过该项。业务执行时间若超过任务周期，调度器不会补跑丢失的周期。
 * @note 由 task_loop() 在主循环上下文调用。
 */
static void TaskHandler(void)
{
	for (uint8_t i = 0; i < TASK_NUM_MAX; i++)
	{
		if (g_taskComps[i].run) // 判断时间片标志
		{
			g_taskComps[i].run = 0; // 标志清零
			if (g_taskComps[i].pTaskFuncCb == NULL)
			{
				continue;
			}
			g_taskComps[i].pTaskFuncCb(); // 执行调度业务功能模块
		}
	}
}

/**
 * @brief 更新所有后台任务的毫秒倒计时并设置就绪标志。
 *
 * 每次调用将非零 timCount 减一；减到零时置 run=1，并装回 timRload，形成周期任务。
 * @note 该回调由 SysTick_Handler() 间接调用，默认调用周期为 1 ms；函数只修改
 *       标志和计数，不直接执行任务，以缩短中断占用时间。
 */
static void TaskScheduleCb(void)
{
	for (uint8_t i = 0; i < TASK_NUM_MAX; i++)
	{
		if (g_taskComps[i].timCount)
		{
			g_taskComps[i].timCount--;
			if (g_taskComps[i].timCount == 0)
			{
				g_taskComps[i].run = 1;
				g_taskComps[i].timCount = g_taskComps[i].timRload;
			}
		}
	}
}

/**
 * @brief 将 1 ms 任务节拍回调注册到 SysTick 中断模块。
 * @note 注册完成前，SysTick_Handler() 检测到空指针会直接返回。
 */
static void AppInit()
{
	TaskScheduleCbReg(TaskScheduleCb);
}
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
    AppInit();
    (void)FocWaveInit(); /* 遥测失败只禁用波形，不阻止电机启动。 */
    /* 定时器和 ADC 已运行，三相输出关闭，先等待霍尔电流传感器稳定。 */
    HAL_Delay(100);
    AdcSampleOffset();
    Motor.CtrlMode = TO_ZERO;
    PostionToZeroDouble();
    if (AdcFault != 0U)
    {
        MotorStop();
        Error_Handler();
    }

    VariableClear();
    EnCoSpeed.EncoLastCnt = TIM3->CNT;
    EnCoSpeed.SpeedClock = 0U;
    // Svpwm.Ualpha = Svpwm.Ubeta = 0.0f;
    // SvpwmCal(&Svpwm);
    HAL_Delay(1); /* 等待中性占空比装载，再开放三相功率输出。 */
    Motor.CtrlMode = STOP;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (AdcFault != 0U)
    {
        __set_PRIMASK(primask);
        MotorStop();
        Error_Handler();
    }
    __DMB();
    Init_Over = true;
    __set_PRIMASK(primask);
}

void task_loop(void)
{
    static uint32_t last_led;
    static uint32_t last_wave_led;
    FocWaveService();
    const uint32_t now = HAL_GetTick();
	TaskHandler(); // 任务调度
}
