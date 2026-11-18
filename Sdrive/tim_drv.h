/**
 * @file tim_drv.h
 * @brief 编码器位置/速度计算与 TIM1 高速控制中断接口。
 *
 * TIM3 以四倍频正交编码器模式累计位置，TIM1 更新中断以 20 kHz 触发采样、
 * 坐标变换和控制模式执行。速度计算按 SPEED_CNT 对高速中断进行分频。
 */

#ifndef __TIM_DRV_H__
#define __TIM_DRV_H__

#include "main.h"

typedef struct
{
    int32_t EncoderCnt;   /**< 本次读取的 TIM3 编码器计数。 */
    int32_t EncoLastCnt;  /**< 上一次测速时的编码器计数。 */
    uint16_t SpeedClock;  /**< TIM1 中断分频计数器，到 SPEED_CNT 时计算速度。 */
}ENCOSPEED;
extern ENCOSPEED EnCoSpeed;


/** @brief 根据 TIM3 计数计算 0..2PI 范围内的转子电角度。 */
void MotorAngleGet(void);

/** @brief 根据相邻采样的编码器计数差计算并滤波机械/电气转速。 */
void MotorSpeedGet(void);

#endif 
