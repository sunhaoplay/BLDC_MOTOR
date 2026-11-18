/**
 * @file motor_ctrl.h
 * @brief 电机控制顶层数据模型、模块汇总依赖和高速控制接口。
 *
 * 本头文件连接采样、FOC、PI、有感控制、HFI、SMO、磁链观测和故障保护。
 * motor_ctrl.c 根据 CtrlMode 在 20 kHz TIM1 中断中选择控制链，并维护不同
 * 无感观测器之间的角度、速度和 PI 状态连续性。
 */

#ifndef __MOTOR_CTRL_H
#define __MOTOR_CTRL_H

#include "stm32f407xx.h"
#include "task.h"
// #include "delay.h"
#include "arm_math.h"
#include "foc.h"
#include "motor_para.h"
#include "pid.h"
#include "main.h"
// #include "hmi_app.h"
// #include "filter.h"
#include "stdbool.h"
// #include "hfi.h"
// #include "flux.h"
// #include "sen.h"
// #include "smo.h"
// #include "fault.h"
// #include "param_ident.h"

/** @brief 电机物理参数、编码器反馈和当前控制模式。 */
typedef struct
{
    float M_theta;             /**< 转子机械角度，单位 rad，当前实现较少使用。 */
    float E_theta;             /**< 编码器转子电角度，单位 rad。 */
    float speed_M_rpm;         /**< 编码器机械转速，单位 rpm。 */
    float speed_E_rpm;         /**< 编码器电气转速，等于机械 rpm*P。 */
    volatile uint8_t CtrlMode; /**< motor_para.h 定义的当前控制模式。 */
    float Rs;                  /**< 定子相电阻，单位 ohm。 */
    float Ls;                  /**< 定子相电感，单位 H。 */
    float Ib;                  /**< 电流基值，供备用标幺 SMO 计算。 */
    float Vb;                  /**< 电压基值，供备用标幺 SMO 计算。 */
    uint8_t P;                 /**< 电机极对数。 */
} motor_p;

#define MOTOR_DEFAULTS {0.0f, 0.0f, 0.0f, 0.0f, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0}

/** @brief 全速域无感控制的观测器选择、融合角度和速度状态。 */
typedef struct
{
    volatile uint8_t Observer;      /**< 当前观测器：1=HFI，3=SMO 或磁链。 */
    volatile uint8_t Observer_last; /**< 上一控制周期观测器编号。 */
    volatile float theta;           /**< 最终供 Park/反 Park 使用的融合电角度，rad。 */
    volatile float Omega;           /**< 当前融合电角速度，rad/s。 */
    volatile float Speed_Fbk;       /**< 当前观测器电气速度反馈，rpm。 */
    float Speed_Ref;                /**< 速度外环电气速度给定，rpm。 */
    int16_t Speed_Max;              /**< 无感速度目标上限，机械 rpm。 */
    int16_t w1;                     /**< 低速迟滞边界，电气 rpm。 */
    int16_t w2;                     /**< 高速迟滞边界，电气 rpm。 */
} Sensorless_Control;

#define Sensorless_DEFAULTS {1, 1, 0, 0, 0, 0, 0, 0, 0}

/** @brief 给定值的离散斜坡发生器。 */
typedef struct
{
    float XieLv_X;        /**< 目标值。 */
    float XieLv_Y;        /**< 当前斜坡输出。 */
    float XieLv_Grad;     /**< 每次更新允许的增减量。 */
    uint8_t Timer_Count;  /**< 当前更新间隔计数。 */
    uint8_t Grad_Timer;   /**< 达到该计数后执行一次斜坡更新。 */
} GXieLv, *p_GXieLv;

#define GXieLv_DEFAULTS {0, 0, 0, 0, 0}

/** @brief 运行给定、IF 启动、观测器切换及 PI 参数集合。 */
typedef struct
{
    uint16_t IfFrq;       /**< 查表强拖分频计数。 */
    uint16_t IfAngle;     /**< 查表强拖角度索引 0..359。 */
    uint16_t Locktime;    /**< 保留的堵转检测计时。 */
    uint8_t SpeedCalTime; /**< 无感速度环分频计数。 */
    float IfTheta;        /**< IF 开环电角度，rad。 */
    float IfCurrent;      /**< IF 阶段 q 轴电流给定，A。 */
    int16_t SpeedRef;     /**< 用户机械速度给定，rpm。 */
    float CurRef;         /**< 用户 q 轴电流给定，A。 */
    int32_t PosRef;       /**< 用户位置给定，通常为 12 位 ADC/命令值。 */
    double RpmCmd;        /**< 保留的速度命令。 */
    float FluxErr;        /**< 磁链观测器切换角度误差。 */
    float HfiSmoErr;      /**< HFI/SMO 切换角度误差。 */
    float FocAngle;       /**< 保留的 FOC 角度。 */
    uint16_t FluxSpeedCnt;/**< 保留的磁链测速计数。 */
    uint8_t ToZeroState;  /**< 调零状态。 */
    uint16_t UpTime;      /**< 保留的上升计时。 */
    uint16_t DownTime;    /**< 保留的下降计时。 */
    uint8_t CalTime;      /**< 保留的计算计数。 */
    bool ObsSwFlag;       /**< 观测器切换标志。 */
    bool SmoSwFlag;       /**< SMO 切换标志。 */
    float SpdSenKp;       /**< 有感速度 PI 比例增益。 */
    float SpdSenKi;       /**< 有感速度 PI 积分增益。 */
    float SpdLessKp;      /**< 无感速度 PI 比例增益。 */
    float SpdLessKi;      /**< 无感速度 PI 积分增益。 */
} PARA;
extern PARA Para;

/** @brief 环形编码器位置控制器状态。 */
typedef struct
{
    int32_t P_Ref_Err; /**< 保留的给定误差。 */
    int32_t P_Ref_Old; /**< 上一位置给定。 */
    int32_t P_Fdb;     /**< TIM3 实际位置计数。 */
    int32_t P_Ref;     /**< 映射后的目标位置计数。 */
    int32_t P_Err;     /**< 位置误差。 */
    int32_t P_Err_Max; /**< 最大位置误差。 */
    float Kp;          /**< 位置比例增益。 */
    int32_t Ki;        /**< 位置积分增益，当前控制未启用。 */
    int16_t P_Out;     /**< 位置环输出的机械速度给定，rpm。 */
} POS_DRV;
extern POS_DRV Pos;

extern CLARKE Clarke;                 /**< 三相到 alpha/beta 电流变换对象。 */
extern PARK Park;                     /**< alpha/beta 到 d/q 电流变换对象。 */
extern IPARK Ipark;                   /**< d/q 到 alpha/beta 电压变换对象。 */
extern GXieLv IfFreqGXieLv;           /**< IF 电频率斜坡。 */
extern GXieLv SpdRefGXieLv;           /**< 机械速度给定斜坡。 */
extern IPARK IparkSmoPv;              /**< SMO 切换时的备用反 Park 对象。 */
extern bool SMO_flag;
extern bool SMO_Switch_flag;
extern bool HFI_Init_Angle_flag;      /**< HFI 初始角度及极性识别完成标志。 */
extern bool Init_Over;                /**< 整机初始化完成、允许故障检测标志。 */

extern bool HFI_Init_Angle_flag;
extern motor_p Motor;
extern const float ubeta_tab[360];    /**< 0.5*sin(theta) 强拖查表。 */
extern const float ualpha_tab[360];   /**< 0.5*cos(theta) 强拖查表。 */
extern Sensorless_Control SensorLess;

/** @brief 让斜坡输出按固定步长逼近目标值。 */
void GradXieLv(p_GXieLv pv);
/** @brief 计算空间矢量 PWM 扇区和 TIM1 比较值。 */
void SvpwmCal(M_SVPWM pv);
/** @brief 关闭三相 PWM 并清理控制动态状态。 */
void MotorStop(void);
/** @brief 启动 TIM1 三相互补 PWM 输出。 */
void MotorRun(void);
/** @brief 完成采样电流的 Clarke/Park 变换和所有观测器角度更新。 */
void AxisDq(void);
/** @brief 把反 Park 电压送入 SVPWM。 */
void FocSvpwm(void);
/** @brief 按正余弦表产生开环旋转定子电压矢量。 */
void QtMotor(void);
/** @brief 根据 Motor.CtrlMode 执行本周期控制链。 */
void MotorModeSel(void);
/** @brief 清零停机后不应保留的控制器和观测器动态状态。 */
void VariableClear(void);
/** @brief 运行 IF 开环角度加 q 轴电流闭环启动。 */
void IFStartControl(void);
/** @brief 清理并建立一次 IF 到高速观测器启动所需状态。 */
void IFObserverStartInit(void);

/** @brief 初始化电机参数、控制器、观测器和滤波器。 */
void MotorInit(void);
/** @brief 查表近似计算二维向量角度。 */
float my_atan2(float x, float y);
/** @brief 查表近似计算 atan(x)。 */
float my_atan(float x);
/** @brief 返回 -1、0 或 1 的符号值。 */
int8_t Sign(float x);
/** @brief 返回浮点绝对值。 */
float my_abs(float x);
/** @brief 滑模边界层饱和函数。 */
float Sat(float s, float delta);
/** @brief 返回两个浮点数中的较大者。 */
float my_max(float a, float b);

#endif
