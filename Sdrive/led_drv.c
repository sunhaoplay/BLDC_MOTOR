/**
 * @file led_drv.c
 * @brief 实现 RGB LED 逻辑编号到 GPIO 的映射及低电平点亮操作。
 *
 * g_gpioList[] 的顺序是蓝、绿、红。硬件采用灌电流驱动，GPIO_RESET 点亮，
 * GPIO_SET 熄灭；上层无需关心这种反相逻辑。
 */

#include "led_drv.h"

typedef struct
{
	GPIO_TypeDef *gpio; /**< LED 通道所在 GPIO 端口。 */
	uint32_t pin;       /**< LED 通道对应的 GPIO 位掩码。 */
} Led_GPIO_t;

static Led_GPIO_t g_gpioList[] =
{
	{LED_B_GPIO_Port, LED_B_Pin},
    {LED_G_GPIO_Port, LED_G_Pin},
    {LED_R_GPIO_Port, LED_R_Pin}
};

#define LED_NUM_MAX (sizeof(g_gpioList) / sizeof(g_gpioList[0]))

/**
 * @brief 把指定 LED 引脚拉低以点亮通道。
 * @param ledNo g_gpioList[] 索引，越界时不访问 GPIO。
 */
void TurnOnLed(uint8_t ledNo)
{
	if (ledNo >= LED_NUM_MAX)
	{
		return;
	}
	HAL_GPIO_WritePin(g_gpioList[ledNo].gpio, g_gpioList[ledNo].pin, GPIO_PIN_RESET);
}

/**
 * @brief 把指定 LED 引脚拉高以熄灭通道。
 * @param ledNo g_gpioList[] 索引，越界时不访问 GPIO。
 */
void TurnOffLed(uint8_t ledNo)
{
	if (ledNo >= LED_NUM_MAX)
	{
		return;
	}
	HAL_GPIO_WritePin(g_gpioList[ledNo].gpio, g_gpioList[ledNo].pin, GPIO_PIN_SET);
}

/**
 * @brief 翻转指定 LED 引脚，可用于闪烁或状态指示。
 * @param ledNo g_gpioList[] 索引，越界时不访问 GPIO。
 */
void ToggleLed(uint8_t ledNo)
{
	if (ledNo >= LED_NUM_MAX)
	{
		return;
	}
	HAL_GPIO_TogglePin(g_gpioList[ledNo].gpio, g_gpioList[ledNo].pin);
}

/**
 * @brief 根据三个布尔量控制 RGB LED。
 * @param r 红色通道使能，非零点亮。
 * @param g 绿色通道使能，非零点亮。
 * @param b 蓝色通道使能，非零点亮。
 * @note 底层 LED 为低电平点亮，极性由 led_drv.c 封装。
 */
void LedTask(uint8_t r, uint8_t g, uint8_t b)
{
	/* 红 */
	if (r)
		TurnOnLed(LED_R);
	else
		TurnOffLed(LED_R);

	/* 绿 */
	if (g)
		TurnOnLed(LED_G);
	else
		TurnOffLed(LED_G);

	/* 蓝 */
	if (b)
		TurnOnLed(LED_B);
	else
		TurnOffLed(LED_B);
}


