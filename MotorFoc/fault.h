/**
 * @file fault.h
 * @brief 母线、温度、堵转和速度异常诊断标志及周期检测接口。
 *
 * 每个故障使用独立位标志。检测函数采用连续计数去抖，只有异常持续达到配置
 * 时间后才停机，避免瞬时噪声误触发。
 */

#ifndef __FAULT_H
#define __FAULT_H

#include "main.h"

/** @brief 电机故障位集合。 */
typedef union
{
    struct
    {
        unsigned OverVbusFlag : 1;  /**< 母线过压。 */
        unsigned UnderVbusFlag : 1; /**< 母线欠压。 */
        unsigned OverTempFlag : 1;  /**< MOS/功率级过温。 */
        unsigned OverIBUSFlag : 1;  /**< 过流，当前 FaultCheck() 未置位。 */
        unsigned LockedFlag : 1;    /**< 堵转。 */
        unsigned OverSpeedFlag : 1; /**< 超速或实际转向与给定相反。 */
        unsigned OffsetFlag : 1;    /**< 电流零点校准异常，当前函数未置位。 */
    } bit;
    uint16_t Byte;                  /**< 故障位的 16 位整体视图。 */
} DiagFlag;

extern DiagFlag Fault; /**< 当前诊断标志。 */

/** @brief 执行一次全部软件保护检测并更新故障状态及指示灯。 */
void FaultCheck(void);

#endif
