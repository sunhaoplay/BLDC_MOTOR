// /**
//  * @file task.c
//  * @brief 实现简单的前后台时间片调度器和整机上电初始化流程。
//  *
//  * SysTick 每 1 ms 调用 TaskScheduleCb()，仅递减计数并置位 run；主循环中的
//  * TaskHandler() 再执行已经就绪的业务函数。高速 FOC 环不经过本调度器，而是由
//  * TIM1 的 20 kHz 更新中断直接驱动。
//  */
#include "task.h"
#include "motor_para.h"
#include "motor_ctrl.h"
#include "sen.h"
#include "tim.h"
// typedef struct
// {
// 	uint8_t run;                 /**< 就绪标志：1 表示等待主循环执行，0 表示未到期。 */
// 	uint16_t timCount;           /**< 当前倒计时，单位为 1 ms 调度节拍。 */
// 	uint16_t timRload;           /**< 周期重载值，任务执行周期约为 timRload ms。 */
// 	void (*pTaskFuncCb)(void);   /**< 到期后由主循环调用的无参数业务函数。 */
// } TaskComps_t;

// static TaskComps_t g_taskComps[] =
// 	{
// #if VR_OR_PC
// 		{0, 1, 1, VrTask}, // 电位器处理任务
// #else
// 		{0, 10, 10, RecevToVofa}, // 接收数据任务
// #endif
// 		{0, 5, 5, HmiTask},		 // 按键处理任务
// 		//{0, 10, 10, SendToVofa}, // 发送数据任务
// 		{0, 1, 1, FaultCheck},	 // 故障检测任务

// 		/* 添加业务功能模块 */
// };

// #define TASK_NUM_MAX (sizeof(g_taskComps) / sizeof(g_taskComps[0])) /**< 静态任务表中的任务数量。 */

// /**
//  * @brief 扫描任务表并执行所有已经到期的后台任务。
//  *
//  * 执行前先清除 run 标志，因此同一任务在一次扫描中最多执行一次。如果回调为空，
//  * 直接跳过该项。业务执行时间若超过任务周期，调度器不会补跑丢失的周期。
//  * @note 由 task_loop() 在主循环上下文调用。
//  */
// static void TaskHandler(void)
// {
// 	for (uint8_t i = 0; i < TASK_NUM_MAX; i++)
// 	{
// 		if (g_taskComps[i].run) // 判断时间片标志
// 		{
// 			g_taskComps[i].run = 0; // 标志清零
// 			if (g_taskComps[i].pTaskFuncCb == NULL)
// 			{
// 				continue;
// 			}
// 			g_taskComps[i].pTaskFuncCb(); // 执行调度业务功能模块
// 		}
// 	}
// }

// /**
//  * @brief 更新所有后台任务的毫秒倒计时并设置就绪标志。
//  *
//  * 每次调用将非零 timCount 减一；减到零时置 run=1，并装回 timRload，形成周期任务。
//  * @note 该回调由 SysTick_Handler() 间接调用，默认调用周期为 1 ms；函数只修改
//  *       标志和计数，不直接执行任务，以缩短中断占用时间。
//  */
// static void TaskScheduleCb(void)
// {
// 	for (uint8_t i = 0; i < TASK_NUM_MAX; i++)
// 	{
// 		if (g_taskComps[i].timCount)
// 		{
// 			g_taskComps[i].timCount--;
// 			if (g_taskComps[i].timCount == 0)
// 			{
// 				g_taskComps[i].run = 1;
// 				g_taskComps[i].timCount = g_taskComps[i].timRload;
// 			}
// 		}
// 	}
// }

/**
 * @brief 初始化应用直接使用的按键、串口接收和 TIM1 更新中断。
 * @note CubeMX 已完成 GPIO、UART 和定时器底层配置，本函数只启动运行时功能。
 */
static void DrvInit()
{
	// DelayInit();
	// KeyDrvInit();
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); // receive interrrupt，空闲时间开启中断
	__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);	 // 避免中断已开启就进入中断
	HAL_TIM_Base_Start_IT(&htim1);				 // 开启定时器1中断
}

// /**
//  * @brief 将 1 ms 任务节拍回调注册到 SysTick 中断模块。
//  * @note 注册完成前，SysTick_Handler() 检测到空指针会直接返回。
//  */
// static void AppInit()
// {
// 	TaskScheduleCbReg(TaskScheduleCb);
// }

/**
 * @brief 完成应用层和电机控制系统的上电初始化。
 *
 * 顺序为：启动驱动与调度器、装载电机和控制参数、启动 ADC DMA、启动编码器、
 * 在无电流条件下校准相电流零点，最后执行机械零位校准。Init_Over 在 ADC 校准
 * 后置位，用于允许故障检测逻辑开始工作。
 * @warning 调零过程可能驱动电机，调用前必须保证功率级和机构处于可运行状态。
 */
void task_init(void)
{
	DrvInit();
	// AppInit();

	MotorInit(); // 电机参数初始化

	HAL_Delay(100);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC_Value, 5);		  // 开启ADC-DMA转换
	/* 启动 A/B 硬件计数 */
if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
{
    Error_Handler();
}
	AdcSampleOffset(); // 零点校准

	// Init_Over = 1;

	// Motor.CtrlMode = TO_ZERO;
	PostionToZeroDouble(); // 调零
    Init_Over=1;
    MotorRun();	// Motor.CtrlMode = STOP;
}

// /**
//  * @brief 执行一次后台任务扫描并发送一帧 VOFA 实时数据。
//  * @note 发送采用轮询方式，主循环速度会受到串口发送时间影响，但不会改变
//  *       由 TIM1 中断驱动的高速电机控制周期。
//  */
// void task_loop(void)
// {
// 	TaskHandler(); // 任务调度
//     SendToVofa(); // 发送数据任务
// }
