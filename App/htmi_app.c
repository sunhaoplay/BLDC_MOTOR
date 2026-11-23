/**
 * @file hmi_app.c
 * @brief 实现 USART1/VOFA 协议、本地按键与电位器的人机交互逻辑。
 *
 * 接收方向采用 HAL 单字节中断，把以 0xFA 结束的命令帧交给后台任务解析；
 * 发送方向采用 JustFloat 协议，即连续发送若干个小端 float，末尾追加固定四字节。
 * 本模块还把按键事件映射为电机控制模式，并根据模式解释电位器的物理含义。
 */

#include "htmi_app.h"
#include "led_drv.h"
#include "string.h"
#include "motor_ctrl.h"

/**
 * @brief 按控制模式把电位器采样值映射为对应给定量。
 *
 * 速度模式写 Para.SpeedRef，电流/IF 模式写 Para.CurRef，位置模式写
 * Para.PosRef；Limit_Sat() 对给定范围限幅，防止电位器端点产生越界指令。
 */
void VrTask(void)
{
	static float AdcVr = 0;

	switch (Motor.CtrlMode)
	{
	case STOP:
	{
		Para.SpeedRef = 0;
		Para.CurRef = 0;
		break;
	}
	case SPEED_LOOP:
	{
		AdcVr = (float)AdcPara.Vr;
		Para.SpeedRef = (int16_t)AdcVr;
		Para.SpeedRef = Limit_Sat(Para.SpeedRef, SPD_REF_MAX, 100.0f);
		break;
	}
	case CUR_LOOP:
	{
		AdcVr = (float)(AdcPara.Vr / 2000.0f);
		Para.CurRef = (float)AdcVr;
		Para.CurRef = Limit_Sat(Para.CurRef, 1.0f, 0.05f);
		break;
	}
	case POS_LOOP:
	{
		Para.PosRef = (int32_t)AdcPara.Vr;
		break;
	}
	case IF_CTRL:
	{
		AdcVr = (float)(AdcPara.Vr / 1000.0f);
		Para.CurRef = (float)AdcVr;
		Para.CurRef = Limit_Sat(Para.CurRef, 4.0f, 0.05f);
		break;
	}

	case HFI_CTRL:
	{
		AdcVr = (float)(AdcPara.Vr / 10.0f);
		Para.SpeedRef = (int16_t)AdcVr;
		Para.SpeedRef = Limit_Sat(Para.SpeedRef, HFI_SPD_REF_MAX, 10.0f);
		break;
	}

	case HFI_SMO:
	case HFI_FLUX:
	case IF_SMO:
	case IF_FLUX:
	{
		AdcVr = (float)AdcPara.Vr;
		Para.SpeedRef = (int16_t)AdcVr;
		Para.SpeedRef = Limit_Sat(Para.SpeedRef, SPD_REF_MAX, 500.0f);
		break;
	}
	case TO_ZERO:
	{
		break;
	}
	default:
		break;
	}
}