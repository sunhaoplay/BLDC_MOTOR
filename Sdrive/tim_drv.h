/**
 * @file tim_drv.h
 * @brief TIM3 编码器位置/速度计算接口；电流环由 ADC 注入完成中断执行。
 */

#ifndef __TIM_DRV_H__
#define __TIM_DRV_H__

#include "main.h"

typedef struct
{
    int32_t EncoderCnt;   /**< 本次读取的 TIM3 编码器计数。 */
    int32_t EncoLastCnt;  /**< 上一次测速时的编码器计数。 */
    uint16_t SpeedClock;  /**< 可用于对 ADC 控制周期分频的测速计数器。 */
}ENCOSPEED;
extern ENCOSPEED EnCoSpeed;


/** @brief 根据 TIM3 计数计算 0..2PI 范围内的转子电角度。 */
void MotorAngleGet(void);

/** @brief 根据相邻采样的编码器计数差计算并滤波机械/电气转速。 */
void MotorSpeedGet(void);

#endif 
