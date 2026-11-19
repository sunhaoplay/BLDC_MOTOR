/**
 * @file foc.c
 * @brief 实现 Clarke/Park/反 Park 坐标变换和七段式 SVPWM 的简化计算。
 *
 * 变换采用幅值不变形式。SVPWM 先由 alpha/beta 电压的三个投影符号确定扇区，
 * 再计算对称三相调制量并映射到中心对齐 PWM 的比较寄存器。
 */

#include "motor_ctrl.h"
#include "arm_math.h"
#include <math.h>
SVPWM Svpwm = SVPWM_DEFAULTS;

/**
 * @brief 将三相电流投影到两相静止 alpha/beta 坐标。
 * @param pv Clarke 输入输出对象。
 *
 * 在 Iu+Iv+Iw=0 条件下采用 alpha=Iu，
 * beta=Iu/sqrt(3)+2*Iv/sqrt(3)。常数 0.57735027 和 1.1547004 分别是
 * 1/sqrt(3) 与 2/sqrt(3)。
 */
void ClarkeCale(M_CLARKE pv)
{
    pv->Alpha = pv->Iu;
    pv->Beta = (pv->Iu * 0.57735027f) + (pv->Iv * 1.1547004f);
}

/**
 * @brief 将静止 alpha/beta 矢量旋转到转子同步 d/q 坐标。
 * @param pv Park 输入输出对象，Theta 为电角度 rad。
 *
 * d=alpha*cos(theta)+beta*sin(theta)；
 * q=beta*cos(theta)-alpha*sin(theta)。d 轴与转子磁链对齐，q 轴负责转矩。
 */
void ParkCale(M_PARK pv)
{
    pv->Ds = arm_cos_f32(pv->Theta) * pv->Alpha + arm_sin_f32(pv->Theta) * pv->Beta;
    pv->Qs = arm_cos_f32(pv->Theta) * pv->Beta - arm_sin_f32(pv->Theta) * pv->Alpha;
}

/**
 * @brief 将旋转 d/q 电压矢量反变换到静止 alpha/beta 坐标。
 * @param pv 反 Park 输入输出对象，Theta 必须与电流 Park 变换使用同一电角度定义。
 *
 * alpha=d*cos(theta)-q*sin(theta)；
 * beta=d*sin(theta)+q*cos(theta)。
 */
void IPARK_Cale(M_IPARK pv)
{
    pv->Alpha = arm_cos_f32(pv->Theta) * pv->Ds - arm_sin_f32(pv->Theta) * pv->Qs;
    pv->Beta = arm_sin_f32(pv->Theta) * pv->Ds + arm_cos_f32(pv->Theta) * pv->Qs;
}

/**
 * @brief 将 alpha/beta 电压指令转换为三相中心对齐 PWM 比较值。
 * @param pv SVPWM 输入、状态和输出对象。
 *
 * 三个投影 u1/u2/u3 的符号组合确定六个 60 度扇区。每个扇区使用同一对
 * 有效空间矢量的对称七段式排列，简化后得到 Ta/Tb/Tc。最后先乘
 * SVPWM_KM_BACKW 归一化，再按实际 ARR 映射比较值；饱和时统一缩放三相，
 * 保持电压矢量方向并在峰顶附近保留固定采样窗口。
 */
void SvpwmCal(M_SVPWM pv)
{
    // 三个相隔 60 度的投影；sqrt(3)/2 近似为 0.8660254。
    pv->u1 = pv->Ubeta;
    pv->u2 = pv->Ubeta * 0.5f + pv->Ualpha * 0.8660254f;
    pv->u3 = pv->u2 - pv->u1;

    // 根据三个投影的符号编码空间矢量所在扇区 1..6。
    pv->VecSector = 3;
    pv->VecSector = (pv->u2 > 0) ? (pv->VecSector - 1) : pv->VecSector;
    pv->VecSector = (pv->u3 > 0) ? (pv->VecSector - 1) : pv->VecSector;
    pv->VecSector = (pv->u1 < 0) ? (7 - pv->VecSector) : pv->VecSector;

    // 对置扇区共用一组代数式，生成三相对称调制量。
    if ((pv->VecSector == 1) || (pv->VecSector == 4))
    {
        pv->Ta = pv->u2;
        pv->Tb = pv->u1 - pv->u3;
        pv->Tc = -pv->u2;
    }
    else if ((pv->VecSector == 2) || (pv->VecSector == 5))
    {
        pv->Ta = pv->u3 + pv->u2;
        pv->Tb = pv->u1;
        pv->Tc = -pv->u1;
    }
    else if ((pv->VecSector == 3) || (pv->VecSector == 6))
    {
        pv->Ta = pv->u3;
        pv->Tb = -pv->u3;
        pv->Tc = -(pv->u1 + pv->u2);
    }
    else // 非法扇区退化为零电压矢量，三相比较值最终位于 50%。
    {
        pv->Ta = 0;
        pv->Tb = 0;
        pv->Tc = 0;
    }

    /* 按实际 ARR 映射，统一缩放三相以保持矢量方向并保留峰顶采样窗口。
     * 预留的窗口会降低最大可用电压；不能逐相硬截断，否则会改变电压矢量。
     */
    const float midpoint = (float)TIM1->ARR * 0.5f;
    const float guard = (float)(TIM1_SAMPLE_LEAD_TICKS + TIM1_SAMPLE_GUARD_TICKS);
    const float available = midpoint - guard;
    float da = pv->Ta * SVPWM_KM_BACKW * midpoint;
    float db = pv->Tb * SVPWM_KM_BACKW * midpoint;
    float dc = pv->Tc * SVPWM_KM_BACKW * midpoint;
    float peak = fmaxf(fabsf(da), fmaxf(fabsf(db), fabsf(dc)));

    if (!isfinite(peak) || !isfinite(da) || !isfinite(db) || !isfinite(dc) || available <= 0.0f)
    {
        da = db = dc = 0.0f;
    }
    else if (peak > available)
    {
        const float scale = available / peak;
        da *= scale;
        db *= scale;
        dc *= scale;
    }
    /* 四舍五入避免浮点误差把窗口边界的 588 计数截成 587。 */
    TIM1->CCR1 = (uint16_t)(midpoint + da + 0.5f);
    TIM1->CCR2 = (uint16_t)(midpoint + db + 0.5f);
    TIM1->CCR3 = (uint16_t)(midpoint + dc + 0.5f);
}
