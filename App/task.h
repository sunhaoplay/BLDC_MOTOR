/**
 * @file task.h
 * @brief 上电校准和后台波形导出的应用入口。
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

extern UART_HandleTypeDef huart1; /**< USART1 句柄，用于实时 JustFloat 波形。 */
extern TIM_HandleTypeDef htim1;   /**< TIM1 产生三相 PWM 和 OC4REF 内部采样触发。 */
extern ADC_HandleTypeDef hadc1;   /**< ADC1 主 ADC：注入组采电流，规则组采慢信号。 */
extern TIM_HandleTypeDef htim3;   /**< TIM3 句柄，工作在正交编码器接口模式。 */

/**
 * @brief 启动同步采样、零电流校准、编码器对齐，然后允许电流闭环。
 * @note 该函数在 CubeMX 完成时钟及外设初始化后调用，整个上电过程只调用一次。
 */
void task_init(void);

/**
 * @brief 批量启动波形 DMA 发送，并更新 LED；不等待串口发送完成。
 * @note 该函数由 main() 的无限循环反复调用，不应在其中加入永久阻塞操作。
 */
void task_loop(void);

extern volatile uint16_t AccMotionTim; /**< 运动过程使用的易失时间计数，可能被中断修改。 */

#endif
