/**
 * @file fault.c
 * @brief 实现带时间确认的母线电压、堵转、转速方向和功率级温度保护。
 *
 * FaultCheck() 由 1 ms 后台任务调用，所以各计数阈值按毫秒理解。确认故障后
 * 立即关闭 PWM、设置 M_ERR 模式并用 RGB 灯提示；过温具有独立恢复阈值和延时。
 */
#include "motor_para.h"
#include "motor_ctrl.h"
#include "fault.h"
DiagFlag Fault;

uint32_t MotorOVState = 0;     /**< 母线过压连续计数。 */
uint32_t MotorLVState = 0;     /**< 母线欠压连续计数。 */
uint32_t Locktime = 0;         /**< 堵转条件连续计数。 */
uint32_t OverSpeedTime = 0;    /**< 超速条件连续计数。 */
uint32_t DownSpeedTime = 0;    /**< 实际转向相反的连续计数。 */
uint32_t MotorOtpState = 0;    /**< 过温条件连续计数。 */
uint32_t MotorDisOtoState = 0; /**< 从过温状态恢复的连续计数。 */

/**
 * @brief 对所有软件保护条件执行一次采样、延时确认和故障处置。
 *
 * 过压/欠压、堵转、超速/反向和过温均要求连续满足条件一定次数。Init_Over
 * 置位前不检查电压和堵转，避免上电校准阶段误报。正常状态下 Fault.Byte 清零，
 * 绿色 LED 以 400 ms 周期、约 50% 占空比闪烁。
 * @note 调用周期必须保持约 1 ms，否则所有时间阈值都会按比例变化。
 */
void FaultCheck(void)
{
    static uint16_t i;

    if (Init_Over) // ADC 零点和电机初始化完成后才启用运行保护。
    {
        // 母线电压连续高于上限 OV_PROTECT_TIME ms 后触发过压停机。
        if (AdcPara.Vbus > HIHG_VOLTAGE_V)
        {
            if (MotorOVState < OV_PROTECT_TIME)
            {
                MotorOVState++;
            }
            else
            {
                Fault.bit.OverVbusFlag = 1;
                MotorStop();
                Motor.CtrlMode = M_ERR;
                LedTask(1, 0, 1); // 紫色表示过压。
            }
        }
        else
        {
            MotorOVState = 0;
        }

        // 母线电压连续低于下限 LV_PROTECT_TIME ms 后触发欠压停机。
        if (AdcPara.Vbus < LOW_VOLTAGE_V)
        {
            if (MotorLVState < LV_PROTECT_TIME)
            {
                MotorLVState++;
            }
            else
            {
                Fault.bit.UnderVbusFlag = 1;
                MotorStop();
                Motor.CtrlMode = M_ERR;
                LedTask(1, 1, 0); // 黄色表示欠压。
            }
        }
        else
        {
            MotorLVState = 0;
        }

        // 反馈速度低于给定一半且 q 轴电流很大，视为可能堵转并开始累计。
        if (my_abs(pi_spd.Fbk) < my_abs(pi_spd.Ref / 2))
        {
            if (my_abs(Park.Qs) >= LOCK_CUR)
            {
                Locktime++;
            }
        }
        else
        {
            Locktime = 0;
        }

        if (Locktime == LOCK_TIME)
        {
            MotorStop();
            Fault.bit.LockedFlag = 1;
            Locktime = 0;
            Motor.CtrlMode = M_ERR;
            LedTask(1, 0, 0); // 红色表示堵转。
        }
    }

    // 速度足够高时才检测严重超调和实际转向相反，避开零速附近噪声。
    if (my_abs(pi_spd.Fbk) > 400 * Motor.P)
    {
        if (pi_spd.err < 0) // 负误差表示反馈速度高于给定。
        {
            if (my_abs(pi_spd.err) > my_abs(pi_spd.Ref))
            {
                if (OverSpeedTime < SPEED_PRO_TIME)
                {
                    OverSpeedTime++;
                }
                else
                {
                    MotorStop();
                    Fault.bit.OverSpeedFlag = 1;
                    Motor.CtrlMode = M_ERR;
                    LedTask(0, 0, 1); // 蓝色表示速度异常。
                }
            }
            else
            {
                OverSpeedTime = 0;
            }
        }
        if ((Sign(pi_spd.Ref) + Sign(pi_spd.Fbk)) == 0) // 给定与反馈符号相反。
        {
            if (DownSpeedTime < SPEED_PRO_TIME)
            {
                DownSpeedTime++;
            }
            else
            {
                MotorStop();
                Fault.bit.OverSpeedFlag = 1;
                Motor.CtrlMode = M_ERR;
                LedTask(0, 0, 1);
            }
        }
         else
        {
            DownSpeedTime = 0;
        }
    }

    // NTC 温度升高时分压 ADC 码值下降；低于阈值并持续后判为过温。
    if (AdcPara.Temp < MOS_TEMP_OVER_THD_ADC)
    {
        if (MotorOtpState < MOS_TEMP_OVER_TIME)
        {
            MotorOtpState++;
        }
        else
        {
            MotorStop();
            Fault.bit.OverTempFlag = 1;
            Motor.CtrlMode = M_ERR;
            MotorDisOtoState = 0;
        }
    }
    else if (AdcPara.Temp > RSM_MOS_TEMP_OVER_THD_ADC) // 较低温度恢复阈值形成迟滞。
    {
        if (Fault.bit.OverTempFlag == 1)
        {
            if (MotorDisOtoState < MOS_TEMP_OVER_TIME)
            {
                MotorDisOtoState++;
            }
            else
            {
                Fault.bit.OverTempFlag = 0;
                MotorOtpState = 0;
            }
        }
        else
        {
            MotorOtpState = 0;
        }
    }

    if (Motor.CtrlMode != M_ERR)
    {
        i++;
        Fault.Byte = 0;

        if (i < 200)
            LedTask(0, 1, 0); // 系统正常时绿色 LED 闪烁。
        else
            LedTask(0, 0, 0);

        i %= 400;
    }
}
