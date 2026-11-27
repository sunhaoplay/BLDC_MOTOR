#include "param_ident.h"
#include "motor_ctrl.h"
#include "fault.h"
#include "sen.h"
/**
 * @file param_ident.c
 * @brief Rs、Ld、Lq 静态参数辨识模块，执行顺序为 Rs -> Ld -> Lq。
 *
 * 辨识结果只保存在 g_paramIdent 中，不自动更新电机、Flux 或 SMO 控制参数。
 * 辨识期间继续沿用现有 FaultCheck() 保护，Lq 辨识要求外部机械锁定转子。
 */

/** @brief 参数辨识结果对象，供控制逻辑和 Keil Watch 观察。 */
volatile PARAM_IDENT_RESULT g_paramIdent = {0};
// 可根据DTG值预定义成宏，DTG=50时用0.14f，DTG=100时用0.29f
#define DEAD_ZONE_COMP  0.14f
/** @brief 参数辨识状态机内部运行数据，不对其他模块开放。 */
typedef struct
{
    uint32_t state_count;             /**< 当前阶段运行周期计数。 */
    uint8_t rs_index;                 /**< 当前 Rs 电流采样点索引。 */
    uint8_t axis_polarity;            /**< 电感辨识双极性激励方向。 */
    bool axis_prev_valid;             /**< 上一周期电流数据是否有效。 */
    float id_ref;                     /**< d 轴电流渐变给定，单位 A。 */
    float iq_ref;                     /**< q 轴电流渐变给定，单位 A。 */
    float id_sum;                     /**< Rs 采样阶段 d 轴电流累加。 */
    float ud_sum;                     /**< Rs 采样阶段实际 d 轴电压累加。 */
    float start_theta;                /**< 启动辨识时的编码器电角度。 */
    float axis_center_sum;            /**< Ld/Lq 准备阶段 PI 中心电压累计值。 */
    float axis_center_voltage;        /**< Ld/Lq 双极性激励的中心电压。 */
    float axis_last_current;          /**< 上一有效周期的被辨识轴电流。 */
    float axis_voltage_delay_1;       /**< 最近一次下发的实际轴电压。 */
    float axis_voltage_delay_2;       /**< 与当前电流变化配对的延迟轴电压。 */
    float axis_sum_x;                 /**< 最小二乘 Σx。 */
    float axis_sum_y;                 /**< 最小二乘 Σy。 */
    float axis_sum_xx;                /**< 最小二乘 Σx²。 */
    float axis_sum_xy;                /**< 最小二乘 Σxy。 */
    uint8_t axis_voltage_delay_count; /**< 电压延迟队列有效数据数量。 */
} PARAM_IDENT_RUNTIME;

static PARAM_IDENT_RUNTIME g_paramIdentRuntime;

/** @brief Rs 辨识使用的三个 d 轴电流采样点。 */
static const float g_paramIdentRsCurrent[PARAM_IDENT_RS_POINT_NUM] =
{
    PARAM_IDENT_RS_CURRENT_1,
    PARAM_IDENT_RS_CURRENT_2,
    PARAM_IDENT_RS_CURRENT_3
};

/** @brief 直接电压注入时另一个轴的闭环控制选择。 */
#define PARAM_IDENT_CONTROL_NONE 0U   /**< d/q 轴均不使用电流 PI 覆盖电压指令。 */
#define PARAM_IDENT_CONTROL_D_AXIS 1U /**< d 轴维持零电流闭环。 */
#define PARAM_IDENT_CONTROL_Q_AXIS 2U /**< q 轴维持零电流闭环。 */

/** @brief 电感辨识的目标坐标轴。 */
#define PARAM_IDENT_AXIS_D 0U /**< d 轴，对应 Ld 辨识。 */
#define PARAM_IDENT_AXIS_Q 1U /**< q 轴，对应 Lq 辨识。 */

/**
 * @brief 判断计算结果是否为有限正数，用于 Rs、Ld、Lq 结果检查。
 * @param value 待检查的浮点数。
 * @return true 表示为有限正数；false 表示为 NaN、非正数或超出 FLT_MAX。
 */
static bool ParamIdent_IsFinitePositive(float value)
{
    // value==value 用于排除 NaN，上限使用 FLT_MAX 排除 +inf 或溢出
    return (value == value) && (value > 0.0f) && (value < 3.402823e38f);
}

/**
 * @brief 判断计算结果是否为有限数，用于拟合截距检查。
 * @param value 待检查的浮点数。
 * @return true 表示为有限数（允许负数）；false 表示为 NaN 或超出浮点范围。
 */
static bool ParamIdent_IsFinite(float value)
{
    // 电压偏置可正可负，因此只做正负两侧的有限性检查
    return (value == value) && (value < 3.402823e38f) && (value > -3.402823e38f);
}

/**
 * @brief 将标准化电压指令按实时母线电压换算为实际施加电压。
 * @param voltage_command 以 STAN_VOLTAGE_V 为基准的归一化电压指令。
 * @return 实际施加的相电压，单位 V。
 */
static float ParamIdent_ActualVoltage(float voltage_command)
{
    // 母线电压不是标准值时按比例还原实际电压，保证辨识结果准确
    return voltage_command * AdcPara.Vbus / STAN_VOLTAGE_V;
}

/**
 * @brief 使用现有 d/q 轴电流 PI 跟踪给定电流，并更新 SVPWM 输出。
 * @param id_ref d 轴电流给定，单位 A。
 * @param iq_ref q 轴电流给定，单位 A。
 */
static void ParamIdent_ApplyCurrent(float id_ref, float iq_ref)
{
    // d轴电流闭环
    pi_id.Ref = id_ref;
    pi_id.Fbk = Park.Ds;
    PiController((M_PI_Control)&pi_id);

    // q轴电流闭环
    pi_iq.Ref = iq_ref;
    pi_iq.Fbk = Park.Qs;
    PiController((M_PI_Control)&pi_iq);

    Ipark.Theta = Motor.E_theta;
    Ipark.Ds = pi_id.Out;
    Ipark.Qs = pi_iq.Out;
    IPARK_Cale((M_IPARK)&Ipark);
    FocSvpwm();
}

/**
 * @brief 在指定轴直接注入电压，另一轴可继续使用零电流 PI 闭环。
 * @param ud_command d 轴注入电压指令（归一化），d 轴为闭环轴时会被 PI 输出覆盖。
 * @param uq_command q 轴注入电压指令（归一化），q 轴为闭环轴时会被 PI 输出覆盖。
 * @param control_other_axis 需要 PI 闭环的轴：PARAM_IDENT_CONTROL_NONE/D_AXIS/Q_AXIS。
 */
static void ParamIdent_ApplyVoltage(float ud_command,
    float uq_command,
    uint8_t control_other_axis)
{
    if (control_other_axis == PARAM_IDENT_CONTROL_D_AXIS)
    {
        // Lq辨识时使用d轴PI把Id维持在零附近
        pi_id.Ref = 0.0f;
        pi_id.Fbk = Park.Ds;
        PiController((M_PI_Control)&pi_id);
        ud_command = pi_id.Out;
    }
    else if (control_other_axis == PARAM_IDENT_CONTROL_Q_AXIS)
    {
        // Ld辨识时使用q轴PI把Iq维持在零附近
        pi_iq.Ref = 0.0f;
        pi_iq.Fbk = Park.Qs;
        PiController((M_PI_Control)&pi_iq);
        uq_command = pi_iq.Out;
    }

    Ipark.Theta = Motor.E_theta;
    Ipark.Ds = ud_command;
    Ipark.Qs = uq_command;
    IPARK_Cale((M_IPARK)&Ipark);
    FocSvpwm();
}

/**
 * @brief 清零 Ld/Lq 公共拟合累计量，并记录激励开始时的轴电流。
 * @param initial_current 激励开始前的被辨识轴电流，单位 A。
 */
static void ParamIdent_ResetAxisSamples(float initial_current)
{
    g_paramIdentRuntime.axis_polarity = 0U;
    g_paramIdentRuntime.axis_prev_valid = false;
    g_paramIdentRuntime.axis_last_current = initial_current;
    g_paramIdentRuntime.axis_voltage_delay_1 = 0.0f;
    g_paramIdentRuntime.axis_voltage_delay_2 = 0.0f;
    g_paramIdentRuntime.axis_sum_x = 0.0f;
    g_paramIdentRuntime.axis_sum_y = 0.0f;
    g_paramIdentRuntime.axis_sum_xx = 0.0f;
    g_paramIdentRuntime.axis_sum_xy = 0.0f;
    g_paramIdentRuntime.axis_voltage_delay_count = 0U;
    g_paramIdent.sample_count = 0U;
}

/**
 * @brief 按固定步长把电流给定平滑调整到目标值，达到目标时返回 true。
 * @param reference 电流给定指针，就地修改。
 * @param target 目标电流值，单位 A。
 * @return true 表示已到达目标；false 表示本拍仍在爬升。
 */
static bool ParamIdent_RampReference(float *reference, float target)
{
    if (*reference < (target - PARAM_IDENT_CURRENT_RAMP_STEP))
    {
        *reference += PARAM_IDENT_CURRENT_RAMP_STEP;
        return false;
    }
    if (*reference > (target + PARAM_IDENT_CURRENT_RAMP_STEP))
    {
        *reference -= PARAM_IDENT_CURRENT_RAMP_STEP;
        return false;
    }

    // 误差已落在一步之内，直接赋值目标避免在目标附近抖动
    *reference = target;
    return true;
}

/**
 * @brief 在指定轴施加双极性电压并累计电感拟合样本。
 * @param axis 被激励轴：PARAM_IDENT_AXIS_D 或 PARAM_IDENT_AXIS_Q。
 * @param center_voltage 激励的中心电压（归一化）。
 * @param excite_voltage 双极性激励幅值（归一化）。
 * @param target_sample_count 目标有效样本数。
 * @return true 表示有效样本已集满；false 表示仍在激励采样。
 */
static bool ParamIdent_RunAxisExcitation(uint8_t axis,
    float center_voltage,
    float excite_voltage,
    uint16_t target_sample_count)
{
    float axis_current;
    float current_slope;
    float current_mid;
    float model_voltage;
    float axis_voltage;

    axis_current = (axis == PARAM_IDENT_AXIS_D) ? Park.Ds : Park.Qs;

    // CCR预装载使电压指令延迟生效，使用两级队列把ΔI与实际生效电压配对
    if (g_paramIdentRuntime.axis_prev_valid &&
        (g_paramIdentRuntime.axis_voltage_delay_count >= 2U))
    {
        // x为电流变化率，y为扣除Rs压降后的电感电压
        current_slope = (axis_current - g_paramIdentRuntime.axis_last_current) / TS;
        current_mid = 0.5f * (axis_current + g_paramIdentRuntime.axis_last_current);
        model_voltage = g_paramIdentRuntime.axis_voltage_delay_1 -
            g_paramIdent.result_Rs * current_mid;

        g_paramIdentRuntime.axis_sum_x += current_slope;
        g_paramIdentRuntime.axis_sum_y += model_voltage;
        g_paramIdentRuntime.axis_sum_xx += current_slope * current_slope;
        g_paramIdentRuntime.axis_sum_xy += current_slope * model_voltage;
        g_paramIdent.sample_count++;
    }

    // 达到目标有效样本数后停止继续切换激励极性
    if (g_paramIdent.sample_count >= target_sample_count)
    {
        return true;
    }

    // 每个20kHz控制周期翻转一次激励极性
    axis_voltage = center_voltage;
    if (g_paramIdentRuntime.axis_polarity == 0U)
    {
        axis_voltage += excite_voltage;
        // 根据被辨识轴的电流极性叠加死区补偿电压
        if (axis_current > 0.01f)       // 正电流，电压往上补
        {
         axis_voltage += DEAD_ZONE_COMP;
        }
    else if (axis_current < -0.01f)  // 负电流，电压往下补
        {
        axis_voltage -= DEAD_ZONE_COMP;
        }
        // ±0.01A死区不补偿，避免过零附近振荡
        g_paramIdentRuntime.axis_polarity = 1U;
    }
    else
    {
        if (axis_current > 0.01f)       // 正电流，电压往上补
        {
         axis_voltage += DEAD_ZONE_COMP;
        }
    else if (axis_current < -0.01f)  // 负电流，电压往下补
        {
        axis_voltage -= DEAD_ZONE_COMP;
        }
        // ±0.01A死区不补偿，避免过零附近振荡
        axis_voltage -= excite_voltage;
        g_paramIdentRuntime.axis_polarity = 0U;
    }

    // 被辨识轴直接注入电压，正交轴继续维持零电流闭环
    if (axis == PARAM_IDENT_AXIS_D)
    {
        ParamIdent_ApplyVoltage(axis_voltage, 0.0f, PARAM_IDENT_CONTROL_Q_AXIS);
    }
    else
    {
        ParamIdent_ApplyVoltage(0.0f, axis_voltage, PARAM_IDENT_CONTROL_D_AXIS);
    }

    // 保存本周期数据，下一周期用于电压与电流变化延迟配对
    g_paramIdentRuntime.axis_last_current = axis_current;
    if (g_paramIdentRuntime.axis_voltage_delay_count == 0U)
    {
        g_paramIdentRuntime.axis_voltage_delay_1 = ParamIdent_ActualVoltage(axis_voltage);
        g_paramIdentRuntime.axis_voltage_delay_count = 1U;
    }
    else
    {
        g_paramIdentRuntime.axis_voltage_delay_2 = g_paramIdentRuntime.axis_voltage_delay_1;
        g_paramIdentRuntime.axis_voltage_delay_1 = ParamIdent_ActualVoltage(axis_voltage);
        g_paramIdentRuntime.axis_voltage_delay_count = 2U;
    }
    g_paramIdentRuntime.axis_prev_valid = true;

    return false;
}

/**
 * @brief 使用带截距的一元最小二乘拟合计算增量电感和等效电压偏置。
 * @param inductance 输出：拟合斜率，即增量电感，单位 H。
 * @param voltage_offset 输出：拟合截距，即该轴等效电压偏置，单位 V。
 * @return true 表示拟合有效；false 表示样本不足、分母过小或结果非法。
 */
static bool ParamIdent_CalculateInductance(float *inductance,
    float *voltage_offset)
{
    float sample_count;
    float denominator;

    if (g_paramIdent.sample_count < 2U)
    {
        return false;
    }

    sample_count = (float)g_paramIdent.sample_count;

    // 分母为NΣx²-(Σx)²，过小时无法完成可靠拟合
    denominator = sample_count * g_paramIdentRuntime.axis_sum_xx -
        g_paramIdentRuntime.axis_sum_x * g_paramIdentRuntime.axis_sum_x;
    if (my_abs(denominator) <= 1.0e-12f)
    {
        return false;
    }

    // 斜率为电感，截距为该轴的等效电压偏置
    *inductance = (sample_count * g_paramIdentRuntime.axis_sum_xy -
        g_paramIdentRuntime.axis_sum_x * g_paramIdentRuntime.axis_sum_y) /
        denominator;
    *voltage_offset = (g_paramIdentRuntime.axis_sum_y -
        (*inductance) * g_paramIdentRuntime.axis_sum_x) / sample_count;

    return ParamIdent_IsFinitePositive(*inductance) &&
        ParamIdent_IsFinite(*voltage_offset);
}

/**
 * @brief 结束辨识，清零电压和 PWM 输出，并按现有故障状态决定是否返回 STOP。
 * @param success true 表示辨识成功；false 表示失败。
 * @param error 结束时的错误码（成功时传 PARAM_IDENT_ERROR_NONE）。
 */
static void ParamIdent_Finish(bool success, PARAM_IDENT_ERROR error)
{
    // 先清零d/q、αβ电压指令及PWM比较值
    Ipark.Ds = 0.0f;
    Ipark.Qs = 0.0f;
    Ipark.Alpha = 0.0f;
    Ipark.Beta = 0.0f;
    Svpwm.Ualpha = 0.0f;
    Svpwm.Ubeta = 0.0f;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    g_paramIdent.state = PARAM_IDENT_STATE_FINISH;
    g_paramIdent.progress = success ? 100U : g_paramIdent.progress;
    g_paramIdent.error = (uint8_t)error;
    g_paramIdent.status = success ? PARAM_IDENT_STATUS_SUCCESS : PARAM_IDENT_STATUS_FAILED;

    // MotorStop()沿用现有停机过程；已有故障时不得把M_ERR覆盖为STOP
    MotorStop();
    if ((Motor.CtrlMode == PARAM_IDENT) && (Fault.Byte == 0U))
    {
        Motor.CtrlMode = STOP;
    }
}

/**
 * @brief 清零上次辨识结果、状态机计数和全部拟合累计量。
 */
void ParamIdent_Reset(void)
{
    uint8_t i;

    g_paramIdent.status = PARAM_IDENT_STATUS_IDLE;
    g_paramIdent.state = PARAM_IDENT_STATE_IDLE;
    g_paramIdent.progress = 0U;
    g_paramIdent.error = PARAM_IDENT_ERROR_NONE;
    g_paramIdent.result_Rs = 0.0f;
    g_paramIdent.result_Ld = 0.0f;
    g_paramIdent.result_Lq = 0.0f;
    g_paramIdent.voltage_offset = 0.0f;
    g_paramIdent.ld_voltage_offset = 0.0f;
    g_paramIdent.lq_voltage_offset = 0.0f;
    g_paramIdent.sample_count = 0U;

    for (i = 0U; i < PARAM_IDENT_RS_POINT_NUM; i++)
    {
        g_paramIdent.id_avg[i] = 0.0f;
        g_paramIdent.ud_avg[i] = 0.0f;
    }

    g_paramIdentRuntime.state_count = 0U;
    g_paramIdentRuntime.rs_index = 0U;
    g_paramIdentRuntime.axis_polarity = 0U;
    g_paramIdentRuntime.axis_prev_valid = false;
    g_paramIdentRuntime.id_ref = 0.0f;
    g_paramIdentRuntime.iq_ref = 0.0f;
    g_paramIdentRuntime.id_sum = 0.0f;
    g_paramIdentRuntime.ud_sum = 0.0f;
    g_paramIdentRuntime.start_theta = 0.0f;
    g_paramIdentRuntime.axis_center_sum = 0.0f;
    g_paramIdentRuntime.axis_center_voltage = 0.0f;
    g_paramIdentRuntime.axis_last_current = 0.0f;
    g_paramIdentRuntime.axis_voltage_delay_1 = 0.0f;
    g_paramIdentRuntime.axis_voltage_delay_2 = 0.0f;
    g_paramIdentRuntime.axis_sum_x = 0.0f;
    g_paramIdentRuntime.axis_sum_y = 0.0f;
    g_paramIdentRuntime.axis_sum_xx = 0.0f;
    g_paramIdentRuntime.axis_sum_xy = 0.0f;
    g_paramIdentRuntime.axis_voltage_delay_count = 0U;
}

/**
 * @brief 返回参数辨识状态机是否处于运行状态。
 * @return true 表示 status 为 PARAM_IDENT_STATUS_RUNNING。
 */
bool ParamIdent_IsRunning(void)
{
    return g_paramIdent.status == PARAM_IDENT_STATUS_RUNNING;
}

/**
 * @brief 检查 STOP 和无故障启动条件，通过后进入 PARAM_IDENT 并使能 PWM。
 */
void ParamIdent_Start(void)
{
    // 非STOP状态不改变当前控制模式和PWM状态
    if (Motor.CtrlMode != STOP)
    {
        if (ParamIdent_IsRunning())
        {
            return;
        }
        ParamIdent_Reset();
        g_paramIdent.status = PARAM_IDENT_STATUS_FAILED;
        g_paramIdent.error = PARAM_IDENT_ERROR_NOT_STOP;
        return;
    }

    // 已存在系统故障时只记录启动失败，不进入辨识模式
    if (Fault.Byte != 0U)
    {
        ParamIdent_Reset();
        g_paramIdent.status = PARAM_IDENT_STATUS_FAILED;
        g_paramIdent.error = PARAM_IDENT_ERROR_FAULT_ACTIVE;
        return;
    }

    // 清除上次结果，记录启动角度并进入初始化阶段
    ParamIdent_Reset();
    g_paramIdentRuntime.start_theta = Motor.E_theta;
    g_paramIdent.status = PARAM_IDENT_STATUS_RUNNING;
    g_paramIdent.state = PARAM_IDENT_STATE_INIT;
    Motor.CtrlMode = PARAM_IDENT;
    MotorRun();
}

/**
 * @brief 监测控制模式是否离开 PARAM_IDENT，并记录普通中断或故障中断。
 */
void ParamIdent_MonitorModeExit(void)
{
    if (ParamIdent_IsRunning() && (Motor.CtrlMode != PARAM_IDENT))
    {
        g_paramIdent.status = PARAM_IDENT_STATUS_ABORTED;
        g_paramIdent.state = PARAM_IDENT_STATE_FINISH;
        if ((Motor.CtrlMode == M_ERR) || (Fault.Byte != 0U))
            g_paramIdent.error = PARAM_IDENT_ERROR_ABORTED_BY_FAULT;
        else
            g_paramIdent.error = PARAM_IDENT_ERROR_INTERRUPTED;
    }
}

/**
 * @brief 20kHz 参数辨识状态机：依次完成 Rs、Ld 和 Lq 静态辨识。
 */
void ParamIdent_Run20kHz(void)
{
    float target_current;
    float sum_x;
    float sum_y;
    float sum_xx;
    float sum_xy;
    float denominator;
    float inductance_result;
    float voltage_offset_result;
    uint8_t i;
    bool id_ready;
    bool iq_ready;

    switch (g_paramIdent.state)
    {
    // 初始化：保持零电压20ms，等待电流PI和PWM变量稳定
    case PARAM_IDENT_STATE_INIT:
        ParamIdent_ApplyVoltage(0.0f, 0.0f, PARAM_IDENT_CONTROL_NONE);
        g_paramIdentRuntime.state_count++;
        g_paramIdent.progress = 5U;
        if (g_paramIdentRuntime.state_count >= PARAM_IDENT_INIT_COUNT)
        {
            g_paramIdentRuntime.state_count = 0U;
            g_paramIdent.state = PARAM_IDENT_STATE_RS_RAMP;
        }
        break;

    // Rs爬升：把d轴电流给定平滑调整到当前采样点
    case PARAM_IDENT_STATE_RS_RAMP:
        target_current = g_paramIdentRsCurrent[g_paramIdentRuntime.rs_index];
        if (g_paramIdentRuntime.id_ref < (target_current - PARAM_IDENT_CURRENT_RAMP_STEP))
        {
            g_paramIdentRuntime.id_ref += PARAM_IDENT_CURRENT_RAMP_STEP;
        }
        else if (g_paramIdentRuntime.id_ref > (target_current + PARAM_IDENT_CURRENT_RAMP_STEP))
        {
            g_paramIdentRuntime.id_ref -= PARAM_IDENT_CURRENT_RAMP_STEP;
        }
        else
        {
            g_paramIdentRuntime.id_ref = target_current;
        }

        ParamIdent_ApplyCurrent(g_paramIdentRuntime.id_ref, 0.0f);
        g_paramIdent.progress = (uint8_t)(10U + (uint8_t)(g_paramIdentRuntime.rs_index * 15U));

        if (g_paramIdentRuntime.id_ref == target_current)
        {
            g_paramIdentRuntime.state_count = 0U;
            g_paramIdent.state = PARAM_IDENT_STATE_RS_SETTLE;
        }
        break;

    // Rs稳定：保持当前d轴电流点100ms
    case PARAM_IDENT_STATE_RS_SETTLE:
        target_current = g_paramIdentRsCurrent[g_paramIdentRuntime.rs_index];
        ParamIdent_ApplyCurrent(target_current, 0.0f);
        g_paramIdentRuntime.state_count++;
        if (g_paramIdentRuntime.state_count >= PARAM_IDENT_RS_SETTLE_COUNT)
        {
            g_paramIdentRuntime.state_count = 0U;
            g_paramIdentRuntime.id_sum = 0.0f;
            g_paramIdentRuntime.ud_sum = 0.0f;
            g_paramIdent.sample_count = 0U;
            g_paramIdent.state = PARAM_IDENT_STATE_RS_SAMPLE;
        }
        break;

    // Rs采样：累加50ms的实际Id和按母线电压换算后的Ud
    case PARAM_IDENT_STATE_RS_SAMPLE:
        target_current = g_paramIdentRsCurrent[g_paramIdentRuntime.rs_index];
        ParamIdent_ApplyCurrent(target_current, 0.0f);
        g_paramIdentRuntime.id_sum += Park.Ds;
        g_paramIdentRuntime.ud_sum += ParamIdent_ActualVoltage(Ipark.Ds);
        g_paramIdentRuntime.state_count++;
        g_paramIdent.sample_count = (uint16_t)g_paramIdentRuntime.state_count;

        if (g_paramIdentRuntime.state_count >= PARAM_IDENT_RS_SAMPLE_COUNT)
        {
            g_paramIdent.id_avg[g_paramIdentRuntime.rs_index] =
                g_paramIdentRuntime.id_sum / (float)PARAM_IDENT_RS_SAMPLE_COUNT;
            g_paramIdent.ud_avg[g_paramIdentRuntime.rs_index] =
                g_paramIdentRuntime.ud_sum / (float)PARAM_IDENT_RS_SAMPLE_COUNT;

            g_paramIdentRuntime.rs_index++;
            g_paramIdentRuntime.state_count = 0U;
            if (g_paramIdentRuntime.rs_index < PARAM_IDENT_RS_POINT_NUM)
            {
                g_paramIdent.state = PARAM_IDENT_STATE_RS_RAMP;
            }
            else
            {
                g_paramIdent.state = PARAM_IDENT_STATE_RS_CALC;
            }
        }
        break;

    // Rs计算：对三组Id/Ud平均值做带截距直线拟合
    case PARAM_IDENT_STATE_RS_CALC:
        sum_x = 0.0f;
        sum_y = 0.0f;
        sum_xx = 0.0f;
        sum_xy = 0.0f;
        for (i = 0U; i < PARAM_IDENT_RS_POINT_NUM; i++)
        {
            sum_x += g_paramIdent.id_avg[i];
            sum_y += g_paramIdent.ud_avg[i];
            sum_xx += g_paramIdent.id_avg[i] * g_paramIdent.id_avg[i];
            sum_xy += g_paramIdent.id_avg[i] * g_paramIdent.ud_avg[i];
        }

        denominator = (float)PARAM_IDENT_RS_POINT_NUM * sum_xx - sum_x * sum_x;
        if (my_abs(denominator) <= 1.0e-12f)
        {
            ParamIdent_Finish(false, PARAM_IDENT_ERROR_CALCULATION);
            break;
        }

        // 斜率即Rs，截距即逆变器等效电压偏置
        g_paramIdent.result_Rs =
            ((float)PARAM_IDENT_RS_POINT_NUM * sum_xy - sum_x * sum_y) / denominator;
        g_paramIdent.voltage_offset =
            (sum_y - g_paramIdent.result_Rs * sum_x) / (float)PARAM_IDENT_RS_POINT_NUM;

        if (!ParamIdent_IsFinitePositive(g_paramIdent.result_Rs))
        {
            ParamIdent_Finish(false, PARAM_IDENT_ERROR_CALCULATION);
            break;
        }

        g_paramIdentRuntime.state_count = 0U;
        g_paramIdentRuntime.axis_center_sum = 0.0f;
        g_paramIdent.sample_count = 0U;
        g_paramIdent.progress = 65U;
        g_paramIdent.state = PARAM_IDENT_STATE_LD_PREPARE;
        break;

    // Ld准备：建立d轴偏置电流，并平均d轴PI输出作为中心电压
    case PARAM_IDENT_STATE_LD_PREPARE:
        id_ready = ParamIdent_RampReference(&g_paramIdentRuntime.id_ref,
            PARAM_IDENT_LD_BIAS_CURRENT);
        iq_ready = ParamIdent_RampReference(&g_paramIdentRuntime.iq_ref, 0.0f);
        if (id_ready && iq_ready)
        {
            g_paramIdentRuntime.state_count++;
        }
        else
        {
            g_paramIdentRuntime.state_count = 0U;
            g_paramIdentRuntime.axis_center_sum = 0.0f;
        }

        ParamIdent_ApplyCurrent(g_paramIdentRuntime.id_ref, g_paramIdentRuntime.iq_ref);
        // 只平均斜坡到位后的后50ms中心电压，跳过刚到位时的瞬态
        if (g_paramIdentRuntime.state_count >
            (PARAM_IDENT_RS_SETTLE_COUNT - PARAM_IDENT_AXIS_CENTER_SAMPLE_COUNT))
        {
            g_paramIdentRuntime.axis_center_sum += Ipark.Ds;
        }
        g_paramIdent.progress = 70U;

        if (g_paramIdentRuntime.state_count >= PARAM_IDENT_RS_SETTLE_COUNT)
        {
            g_paramIdentRuntime.axis_center_voltage =
                g_paramIdentRuntime.axis_center_sum /
                (float)PARAM_IDENT_AXIS_CENTER_SAMPLE_COUNT;
            ParamIdent_ResetAxisSamples(Park.Ds);
            g_paramIdent.state = PARAM_IDENT_STATE_LD_EXCITE;
        }
        break;

    // Ld激励：d轴叠加双极性电压，q轴维持零电流闭环
    case PARAM_IDENT_STATE_LD_EXCITE:
        if (ParamIdent_RunAxisExcitation(PARAM_IDENT_AXIS_D,
            g_paramIdentRuntime.axis_center_voltage,
            PARAM_IDENT_LD_EXCITE_VOLTAGE,
            PARAM_IDENT_LD_SAMPLE_COUNT))
        {
            g_paramIdent.progress = 82U;
            g_paramIdent.state = PARAM_IDENT_STATE_LD_CALC;
        }
        else
        {
            g_paramIdent.progress = (uint8_t)(70U +
                ((uint32_t)g_paramIdent.sample_count * 12U /
                 PARAM_IDENT_LD_SAMPLE_COUNT));
        }
        break;

    // Ld计算：拟合d轴增量电感和等效电压偏置
    case PARAM_IDENT_STATE_LD_CALC:
        if (!ParamIdent_CalculateInductance(&inductance_result,
            &voltage_offset_result))
        {
            ParamIdent_Finish(false, PARAM_IDENT_ERROR_CALCULATION);
            break;
        }
        g_paramIdent.result_Ld = inductance_result;
        g_paramIdent.ld_voltage_offset = voltage_offset_result;
        g_paramIdentRuntime.state_count = 0U;
        g_paramIdentRuntime.axis_center_sum = 0.0f;
        g_paramIdent.sample_count = 0U;
        g_paramIdent.progress = 83U;
        g_paramIdent.state = PARAM_IDENT_STATE_LQ_PREPARE;
        break;

    // Lq准备：外部锁定转子，Id/Iq回零并平均q轴PI中心电压
    case PARAM_IDENT_STATE_LQ_PREPARE:
        id_ready = ParamIdent_RampReference(&g_paramIdentRuntime.id_ref, 0.0f);
        iq_ready = ParamIdent_RampReference(&g_paramIdentRuntime.iq_ref,
            PARAM_IDENT_LQ_BIAS_CURRENT);
        if (id_ready && iq_ready)
        {
            g_paramIdentRuntime.state_count++;
        }
        else
        {
            g_paramIdentRuntime.state_count = 0U;
            g_paramIdentRuntime.axis_center_sum = 0.0f;
        }

        ParamIdent_ApplyCurrent(g_paramIdentRuntime.id_ref, g_paramIdentRuntime.iq_ref);
        // 只平均斜坡到位后的后50ms中心电压，跳过刚到位时的瞬态
        if (g_paramIdentRuntime.state_count >
            (PARAM_IDENT_RS_SETTLE_COUNT - PARAM_IDENT_AXIS_CENTER_SAMPLE_COUNT))
        {
            g_paramIdentRuntime.axis_center_sum += Ipark.Qs;
        }
        g_paramIdent.progress = 85U;

        if (g_paramIdentRuntime.state_count >= PARAM_IDENT_RS_SETTLE_COUNT)
        {
            g_paramIdentRuntime.axis_center_voltage =
                g_paramIdentRuntime.axis_center_sum /
                (float)PARAM_IDENT_AXIS_CENTER_SAMPLE_COUNT;
            ParamIdent_ResetAxisSamples(Park.Qs);
            g_paramIdent.state = PARAM_IDENT_STATE_LQ_EXCITE;
        }
        break;

    // Lq激励：q轴叠加双极性电压，d轴维持零电流闭环
    case PARAM_IDENT_STATE_LQ_EXCITE:
        if (ParamIdent_RunAxisExcitation(PARAM_IDENT_AXIS_Q,
            g_paramIdentRuntime.axis_center_voltage,
            PARAM_IDENT_LQ_EXCITE_VOLTAGE,
            PARAM_IDENT_LQ_SAMPLE_COUNT))
        {
            g_paramIdent.progress = 98U;
            g_paramIdent.state = PARAM_IDENT_STATE_LQ_CALC;
        }
        else
        {
            g_paramIdent.progress = (uint8_t)(85U +
                ((uint32_t)g_paramIdent.sample_count * 13U /
                 PARAM_IDENT_LQ_SAMPLE_COUNT));
        }
        break;

    // Lq计算：拟合q轴增量电感和等效电压偏置，成功后停机
    case PARAM_IDENT_STATE_LQ_CALC:
        if (!ParamIdent_CalculateInductance(&inductance_result,
            &voltage_offset_result))
        {
            ParamIdent_Finish(false, PARAM_IDENT_ERROR_CALCULATION);
            break;
        }
        g_paramIdent.result_Lq = inductance_result;
        g_paramIdent.lq_voltage_offset = voltage_offset_result;
        ParamIdent_Finish(true, PARAM_IDENT_ERROR_NONE);
        break;

    // 非法状态统一按计算失败结束辨识
    default:
        ParamIdent_Finish(false, PARAM_IDENT_ERROR_CALCULATION);
        break;
    }
}


// 把这几个变量加到全局（方便 Watch 观察）
volatile uint16_t  PP_Start_Cnt = 0;       // 起步编码器值
volatile uint16_t  PP_End_Cnt = 0;         // 停机后编码器值
volatile int16_t  PP_Diff_Cnt = 0;        // 编码器差值 (End - Start)

void paramidentParis(void)
{
    static uint8_t  pp_state = 0;          // 0=起步, 1=强拖, 2=停机
    static uint32_t spin_timer = 0;
    static float    current_voltage = 0.0f;
    static float    total_elec_angle = 0.0f;

    // 角度平滑累加（保持你测出来的不到2秒一圈的速度）
    Test_Spin_Angle += 4.0f * TS; 
    if (Test_Spin_Angle >= PIX2)
    {
        Test_Spin_Angle -= PIX2; 
    }

    switch (pp_state)
    {
        case 0: // 【阶段 0：软启动与预定位，用时 1 秒】
            current_voltage += 0.001f; 
            if (current_voltage > 0.8f) current_voltage = 0.8f; // 防烧管限幅

            spin_timer++;
            if (spin_timer >= 20000U) // 1 秒后
            {
                PP_Start_Cnt = TIM3->CNT; // 记录起点
                total_elec_angle = 0.0f;
                spin_timer = 0;
                pp_state = 1;             // 进入强拖阶段
            }
            break;

        case 1: // 【阶段 1：强拖旋转，刚好转 4 个电周期】
            total_elec_angle += 4.0f * TS; 
            
            if (total_elec_angle >= (8.0f * PIX2)) // 满 4 圈电角度
            {
                current_voltage = 0.0f; // 电压立刻归零
                spin_timer = 0;
                pp_state = 2;           // 进入停机等待
            }
            break;

            case 2: // 【阶段 2：等待转子停稳，记录终点】
            spin_timer++;
            if (spin_timer >= 10000U) // 停机后等 0.5 秒，彻底停稳
            {
                PP_End_Cnt = TIM3->CNT; // 记录终点
                
                // 1. 计算差值
                PP_Diff_Cnt = (int16_t)(PP_End_Cnt - PP_Start_Cnt); 
                
                // 2. 取绝对值，防止逆时针转导致负数崩溃
                int32_t abs_diff = (PP_Diff_Cnt > 0) ? PP_Diff_Cnt : -PP_Diff_Cnt;
                
                // 3. 防呆保护
                if (abs_diff > 50) 
                {
                    g_paramIdent.result_PP = (uint16_t)((8 * 4096 + (abs_diff / 2)) / abs_diff);
                }
                else
                {
                    g_paramIdent.result_PP = 0; // 没动，报错
                }

                MotorStop();
                Motor.CtrlMode = STOP;
                pp_state = 0; // 恢复状态，方便下次再测
            }
            break;
    }

    // ============ 电压输出 ============
    Ipark.Theta = Test_Spin_Angle;
    Ipark.Ds = current_voltage; 
    Ipark.Qs = 0.0f;
    IPARK_Cale((M_IPARK)&Ipark);
    FocSvpwm();
}

