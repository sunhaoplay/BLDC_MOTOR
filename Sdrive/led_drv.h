/**
 * @file led_drv.h
 * @brief 三通道低电平有效 RGB LED 的硬件抽象接口。
 *
 * 上层仅使用 0..2 的逻辑编号，不直接依赖具体 GPIO 端口和引脚。编号到颜色的
 * 实际映射由 led_drv.c 中的 g_gpioList[] 决定。
 */


#ifndef _LED_DRV_H
#define _LED_DRV_H

#include "main.h"

#define LED_B 0 /**< 逻辑通道 0；实际映射由 g_gpioList[] 决定。 */
#define LED_G 1 /**< 逻辑通道 1；实际映射由 g_gpioList[] 决定。 */
#define LED_R 2 /**< 逻辑通道 2；实际映射由 g_gpioList[] 决定。 */

/**
 * @brief 点亮指定 LED 通道。
 * @param ledNo LED 逻辑编号，有效范围 0..2；越界时函数直接返回。
 */
void TurnOnLed(uint8_t ledNo);

/**
 * @brief 熄灭指定 LED 通道。
 * @param ledNo LED 逻辑编号，有效范围 0..2；越界时函数直接返回。
 */
void TurnOffLed(uint8_t ledNo);

/**
 * @brief 翻转指定 LED 通道当前的 GPIO 输出电平。
 * @param ledNo LED 逻辑编号，有效范围 0..2；越界时函数直接返回。
 */
void ToggleLed(uint8_t ledNo);

void LedTask(uint8_t r, uint8_t g, uint8_t b);
#endif
