/**
 * @file sen.h
 * @brief 基于增量编码器电角度的有感电流、速度、位置闭环及机械零位校准接口。
 *
 * 电流环是最内层，速度环输出 q 轴电流给定，位置环输出速度给定。调零函数通过
 * 固定定子电压矢量把转子吸合到已知方向，再把 TIM3 编码器计数清零。
 */

#ifndef __SEN_H
#define __SEN_H

#include "main.h"

/** @brief 使用编码器角度完成 d/q 电流闭环。 */
void CurCloseloop(float IqRef, float Theta);
/** @brief 运行有感速度环，并可选择斜坡给定。 */
void SpeedCloseloop(int16_t SpeedRef, uint8_t UseRamp);
/** @brief 运行环形编码器位置 P 环及其内层速度、电流环。 */
void PosCloseloop(void);
/** @brief 用单个定子电压矢量执行机械零位校准。 */
void PositionToZero(void);
/** @brief 用两个正交电压矢量分两步吸合后执行机械零位校准。 */
void PostionToZeroDouble(void);

#endif
