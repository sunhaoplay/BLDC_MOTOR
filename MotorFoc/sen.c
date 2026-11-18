/**
 * @file sen.c
 * @brief 实现编码器有感 FOC 的调零、电流环、速度环和位置环。
 *
 * 闭环级联关系为：位置误差 -> 速度给定 -> q 轴电流给定 -> q 轴电压；d 轴
 * 电流固定为零。电流环使用 20 kHz 控制角度，速度和位置环通过软件分频降频。
 */

#include "motor_ctrl.h"
#include "sen.h"
#include "motor_para.h"
/**
 * @brief 用固定 alpha 轴电压吸合转子并把当前位置定义为编码器零点。
 *
 * 先使能六路互补 PWM，输出 Ualpha=1、Ubeta=0 的静止电压矢量并等待转子稳定，
 * 然后清零 TIM3->CNT 和 Motor.E_theta，最后关闭 PWM 输出。
 * @warning 函数包含秒级阻塞延时并会产生保持转矩，只能在停机校准流程调用。
 */
void PositionToZero(void)
{
    TIM1->CCER |= 0x5555;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_Delay(100);

    Svpwm.Ualpha = 1; // 固定定子电压矢量，使转子磁极对齐 alpha 轴。
    Svpwm.Ubeta = 0;
    SvpwmCal((M_SVPWM)&Svpwm);
    HAL_Delay(1000);
    TIM3->CNT = 0;     // 把吸合后的机械位置定义为编码器零点。
    Motor.E_theta = 0;
    HAL_Delay(500);

    TIM1->CCER &= 0xAAAA;
    HAL_Delay(500);
}

/**
 * @brief 先后施加 beta、alpha 两个电压矢量后执行编码器调零。
 *
 * 第一次吸合让转子离开可能的不确定初始位置，第二次吸合到最终 alpha 轴方向，
 * 可减小单次吸合受静摩擦或初始夹角影响产生的零位误差。最后清零编码器并停机。
 * @warning 函数会驱动电机且包含多个阻塞延时，机构必须允许校准运动。
 */
void PostionToZeroDouble(void)
{
    TIM1->CCER |= 0x5555;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    HAL_Delay(100);

    Svpwm.Ualpha = 0.0f;
    Svpwm.Ubeta = 0.5f; // 第一次沿 beta 轴吸合。
    SvpwmCal((M_SVPWM)&Svpwm);
    HAL_Delay(1000);

    Svpwm.Ualpha = 0.5f; // 第二次沿 alpha 轴吸合到最终零位。
    Svpwm.Ubeta = 0.0f;
    SvpwmCal((M_SVPWM)&Svpwm);
    HAL_Delay(1000);

    TIM3->CNT = 0;
    Motor.E_theta = 0;

    HAL_Delay(1000);

    TIM1->CCER &= 0xAAAA;
    HAL_Delay(1000);

    MotorStop();
}

/**
 * @brief 使用给定电角度完成有感 d/q 电流双闭环。
 * @param IqRef q 轴电流给定，单位 A，正负决定转矩方向。
 * @param Theta 编码器换算得到的转子电角度，单位 rad。
 *
 * 表贴式 PMSM 常用 IdRef=0，使定子电流主要用于产生转矩。两个 PI 输出 Ud/Uq，
 * 再经反 Park 变换形成供 SVPWM 使用的 Ualpha/Ubeta。
 */
void CurCloseloop(float IqRef, float Theta)
{
    pi_id.Ref = 0.0f;
    pi_id.Fbk = Park.Ds;
    PiController((M_PI_Control)&pi_id);

    pi_iq.Ref = IqRef;
    pi_iq.Fbk = Park.Qs;
    PiController((M_PI_Control)&pi_iq);

    Ipark.Theta = Theta;
    Ipark.Ds = pi_id.Out;
    Ipark.Qs = pi_iq.Out;
    IPARK_Cale((M_IPARK)&Ipark);
}

/**
 * @brief 运行有感速度 PI，并把输出作为内层 q 轴电流给定。
 * @param SpeedRef 机械速度给定，单位 rpm。
 * @param UseRamp 1 使用斜坡限制速度变化；0 直接给定并限为 +/-100 rpm。
 *
 * 斜坡模式在速度误差过大时暂停斜坡计数，避免给定继续远离实际速度。速度 PI
 * 使用电气 rpm，因此给定乘极对数 P；PI 输出再低通后送入 CurCloseloop()。
 */
void SpeedCloseloop(int16_t SpeedRef, uint8_t UseRamp)
{
    float final_spd_ref = 0.0f;

    if (UseRamp)
    {
		SpdRefGXieLv.XieLv_Grad = SPD_REF_OD;
		SpdRefGXieLv.Grad_Timer = SPD_REF_TIMER;
        SpdRefGXieLv.XieLv_X = SpeedRef;
        SpdRefGXieLv.Timer_Count++;
        if (my_abs(pi_spd.Ref - pi_spd.Fbk) >= 1000)
            SpdRefGXieLv.Timer_Count--;

        if (SpdRefGXieLv.Timer_Count > SpdRefGXieLv.Grad_Timer)
        {
            SpdRefGXieLv.Timer_Count = 0;
            GradXieLv((p_GXieLv)&SpdRefGXieLv);
        }
        final_spd_ref = Limit_Sat(SpdRefGXieLv.XieLv_Y, SPD_REF_MAX, -SPD_REF_MAX);
    }
    else
    {
        final_spd_ref = (float)SpeedRef;
        final_spd_ref = Limit_Sat(final_spd_ref, 100, -100);
    }

    // 速度环按 SPEED_CNT 分频运行，电流环仍在每个高速控制周期执行。
    if (++Para.SpeedCalTime >= SPEED_CNT)
    {
        Para.SpeedCalTime = 0;

        pi_spd.Ref = final_spd_ref * Motor.P;
        pi_spd.Fbk = Motor.speed_E_rpm;

        PiController((M_PI_Control)&pi_spd);
        pi_spd.OutF = pi_spd.OutF * LPF_I_RUN_B + pi_spd.Out * LPF_I_RUN_A;
    }

    CurCloseloop(pi_spd.OutF, Motor.E_theta);
}

/* 环形编码器位置范围及位置环输出约束。 */
#define ENC_MIN 1
#define ENC_MAX 3999
#define ENC_RANGE (ENC_MAX - ENC_MIN + 1)
#define POS_MAX_SPEED 3000
#define POS_I_LIMIT 500

/**
 * @brief 运行环形位置 P 控制，并级联有感速度环和电流环。
 *
 * 12 位 Para.PosRef 的 0..4095 被反向映射到编码器 3999..1。位置误差按环形空间
 * 取最短路径：超过半圈时加减 ENC_RANGE。每四次调用更新一次位置 P 输出并限制
 * 为 +/-POS_MAX_SPEED rpm，随后每拍调用 SpeedCloseloop()。
 * @note 积分位置项保留在注释代码中，当前实际控制器仅使用比例项 Pos.Kp*err。
 */
void PosCloseloop(void)
{
    static uint32_t PosLoopDivCnt = 0;
    float SpeedRef = 0.0f;

    Pos.P_Fdb = TIM3->CNT;

    if (++PosLoopDivCnt >= 4)
    {
        PosLoopDivCnt = 0;

        // 0..4095 反向线性映射到有效编码器区间 3999..1。
        uint32_t t = ((uint32_t)Para.PosRef * (ENC_RANGE - 1)) >> 12;
        uint16_t mapped = (uint16_t)(ENC_MAX - t);
        Pos.P_Ref = (int32_t)mapped;

        // 环形位置取最短方向，避免跨零点时误走接近一整圈。
        int32_t err = (int32_t)mapped - (int32_t)Pos.P_Fdb;
        int32_t half = ENC_RANGE / 2;
        if (err > half)
            err -= ENC_RANGE;
        else if (err < -half)
            err += ENC_RANGE;

        float up = Pos.Kp * (float)err;

        /*
        Pos.P_Integral += Pos.Ki * (float)err;
        Pos.P_Integral = Limit_Sat(Pos.P_Integral, POS_I_LIMIT, -POS_I_LIMIT);
        SpeedRef = up + Pos.P_Integral;
        */

        SpeedRef = up;
        Pos.P_Out = (int16_t)Limit_Sat(SpeedRef, POS_MAX_SPEED, -POS_MAX_SPEED);
    }

    SpeedCloseloop(Pos.P_Out, 0);
}
