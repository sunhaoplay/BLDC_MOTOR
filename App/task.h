/**
 * @file task.h
 * @brief 应用任务调度器的公共入口及各业务模块的统一依赖头文件。
 *
 * 本文件把 CubeMX 创建的外设句柄、板级驱动和电机控制模块汇集到应用层，
 * 供 task.c 完成上电初始化和前后台任务调度。1 ms 的 SysTick 中断只负责
 * 产生任务就绪标志，具体业务函数在主循环中执行，避免在中断中运行耗时逻辑。
 */

#ifndef __TASK_H__
#define __TASK_H__

#include "main.h"
#include <stdlib.h>
// #include "delay.h"
#include "stm32f4xx_it.h"
// #include "hmi_app.h"
#include "led_drv.h"
#include "tim_drv.h"
#include "adc_drv.h"
// #include "key_drv.h"
// #include "motor_ctrl.h"

extern UART_HandleTypeDef huart1; /**< USART1 句柄，用于 VOFA 上位机通信。 */
extern TIM_HandleTypeDef htim1;   /**< TIM1 句柄，产生三相 PWM 和 20 kHz 控制中断。 */
extern ADC_HandleTypeDef hadc1;   /**< ADC1 句柄，通过 DMA 连续采集电流、电压等信号。 */
extern TIM_HandleTypeDef htim3;   /**< TIM3 句柄，工作在正交编码器接口模式。 */

/**
 * @brief 初始化板级驱动、调度器、电机参数、ADC DMA 和编码器。
 * @note 该函数在 CubeMX 完成时钟及外设初始化后调用，整个上电过程只调用一次。
 */
void task_init(void);

/**
 * @brief 运行一次后台任务调度并向 VOFA 发送监测数据。
 * @note 该函数由 main() 的无限循环反复调用，不应在其中加入永久阻塞操作。
 */
void task_loop(void);

extern volatile uint16_t AccMotionTim; /**< 运动过程使用的易失时间计数，可能被中断修改。 */

#endif
