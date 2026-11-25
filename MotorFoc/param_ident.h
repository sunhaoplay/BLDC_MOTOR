/**
 * @file param_ident.h
 * @brief 电机静态参数辨识（Rs / Ld / Lq）对外接口与结果对象。
 *
 * 本模块作为 Motor.CtrlMode = PARAM_IDENT(12) 的一种控制模式，挂在 20kHz TIM1
 * 控制中断的状态机上，依次完成定子电阻 Rs、d 轴增量电感 Ld、q 轴增量电感 Lq 的
 * 静态辨识。辨识仅在电机静止（Lq 需外部机械锁轴）条件下进行，结果只保存在 RAM
 * 全局对象 g_paramIdent 中，不自动写回任何控制参数，需人工读取后回填电机参数宏。
 */

#ifndef __PARAM_IDENT_H
#define __PARAM_IDENT_H

#include "stdint.h"
#include "stdbool.h"
#include "motor_para.h"

/** @brief 参数辨识整体运行结果状态。 */
typedef enum
{
    PARAM_IDENT_STATUS_IDLE = 0, /**< 空闲：尚未启动辨识。 */
    PARAM_IDENT_STATUS_RUNNING,  /**< 运行中：正在执行参数辨识。 */
    PARAM_IDENT_STATUS_SUCCESS,  /**< 成功：Rs、Ld、Lq 均辨识成功。 */
    PARAM_IDENT_STATUS_FAILED,   /**< 失败：启动条件或计算检查失败。 */
    PARAM_IDENT_STATUS_ABORTED   /**< 中断：运行期间被其他控制模式或故障打断。 */
} PARAM_IDENT_STATUS;

/** @brief 参数辨识状态机阶段，执行顺序为 Rs、Ld、Lq。 */
typedef enum
{
    PARAM_IDENT_STATE_IDLE = 0,  /**< 空闲状态。 */
    PARAM_IDENT_STATE_INIT,      /**< 零电压初始化，等待控制变量稳定。 */
    PARAM_IDENT_STATE_RS_RAMP,   /**< d 轴电流给定爬升到当前 Rs 采样点。 */
    PARAM_IDENT_STATE_RS_SETTLE, /**< 当前 Rs 电流点闭环稳定。 */
    PARAM_IDENT_STATE_RS_SAMPLE, /**< 采集当前电流点的 Id 和 Ud 平均值。 */
    PARAM_IDENT_STATE_RS_CALC,   /**< 三点直线拟合计算 Rs 和电压偏置。 */
    PARAM_IDENT_STATE_LD_PREPARE,/**< 建立 Ld 辨识偏置电流及中心电压。 */
    PARAM_IDENT_STATE_LD_EXCITE, /**< d 轴双极性电压激励并采集样本。 */
    PARAM_IDENT_STATE_LD_CALC,   /**< 最小二乘计算 Ld 和 d 轴电压偏置。 */
    PARAM_IDENT_STATE_LQ_PREPARE,/**< 转子锁定条件下建立 q 轴中心电压。 */
    PARAM_IDENT_STATE_LQ_EXCITE, /**< q 轴双极性电压激励并采集样本。 */
    PARAM_IDENT_STATE_LQ_CALC,   /**< 最小二乘计算 Lq 和 q 轴电压偏置。 */
    PARAM_IDENT_STATE_FINISH     /**< 辨识结束并关闭 PWM。 */
} PARAM_IDENT_STATE;

/** @brief 参数辨识启动、计算和中断错误码。 */
typedef enum
{
    PARAM_IDENT_ERROR_NONE = 0,        /**< 无错误。 */
    PARAM_IDENT_ERROR_NOT_STOP,        /**< 启动时电机控制模式不是 STOP。 */
    PARAM_IDENT_ERROR_FAULT_ACTIVE,    /**< 启动时已有系统故障。 */
    PARAM_IDENT_ERROR_CALCULATION,     /**< 拟合分母、样本或结果检查失败。 */
    PARAM_IDENT_ERROR_INTERRUPTED,     /**< 被其他控制模式中断。 */
    PARAM_IDENT_ERROR_ABORTED_BY_FAULT /**< 被现有系统故障中断。 */
} PARAM_IDENT_ERROR;

/** @brief 参数辨识结果和 Keil Watch 观察数据。 */
typedef struct
{
    uint8_t status;                         /**< PARAM_IDENT_STATUS 运行结果。 */
    uint8_t state;                          /**< PARAM_IDENT_STATE 当前阶段。 */
    uint8_t progress;                       /**< 辨识进度，范围 0～100。 */
    uint8_t error;                          /**< PARAM_IDENT_ERROR 错误码。 */
    float result_Rs;                        /**< 定子相电阻辨识结果，单位 ohm。 */
    float result_Ld;                        /**< d 轴增量电感辨识结果，单位 H。 */
    float result_Lq;                        /**< q 轴增量电感辨识结果，单位 H。 */
    float voltage_offset;                   /**< Rs 拟合得到的逆变器等效电压偏置，单位 V。 */
    float ld_voltage_offset;                /**< Ld 拟合截距对应的 d 轴电压偏置，单位 V。 */
    float lq_voltage_offset;                /**< Lq 拟合截距对应的 q 轴电压偏置，单位 V。 */
    uint16_t sample_count;                  /**< 当前采样阶段已累计的有效样本数量。 */
    float id_avg[PARAM_IDENT_RS_POINT_NUM]; /**< 三个 Rs 电流点的平均 d 轴电流，单位 A。 */
    float ud_avg[PARAM_IDENT_RS_POINT_NUM]; /**< 三个 Rs 电流点的平均实际 d 轴电压，单位 V。 */
} PARAM_IDENT_RESULT;

/** @brief 参数辨识全局结果对象，结果仅保存在 RAM 中。 */
extern volatile PARAM_IDENT_RESULT g_paramIdent;

/** @brief 检查启动条件并进入参数辨识模式。 */
void ParamIdent_Start(void);
/** @brief 在 20kHz 控制中断中运行辨识状态机。 */
void ParamIdent_Run20kHz(void);
/** @brief 清零上次辨识结果和运行累计量。 */
void ParamIdent_Reset(void);
/** @brief 查询参数辨识是否正在运行。 */
bool ParamIdent_IsRunning(void);
/** @brief 记录故障或控制模式切换导致的中断。 */
void ParamIdent_MonitorModeExit(void);

#endif

