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
#include "key_drv.h"
#include "sen.h"
#include "param_ident.h"
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

/**
 * @brief 把短按、双击和长按事件映射为启动、停机、调零和控制模式切换。
 *
 * 按键 1/2 选择控制模式，按键 3 提供停机、IF+磁链和调零功能，按键 4 长按
 * 关闭 PWR 保持信号。位置模式使用有感速度 PI 参数，其余模式使用无感参数。
 */
void HmiTask(void)
{
	uint8_t keyVal;

	keyVal = GetKeyVal();

	switch (keyVal)
	{
	case KEY4_SHORT_PRESS:
	{
		// 清除之前的残留状态
 		Test_Spin_Angle = 0.0f; 

		// 启动强拖
		MotorRun(); 
		Motor.CtrlMode = OPEN_LOOP_SPIN;
	}
	break;

	case KEY4_DOUBLE_PRESS:
	{
		// ParamIdent_Start();
	}
	break;

	case KEY4_LONG_PRESS:
	{
		HAL_GPIO_WritePin(PWR_GPIO_Port, PWR_Pin, GPIO_PIN_RESET); // 长按关机
	}
	break;
	case KEY1_SHORT_PRESS:
	{
		// MotorRun();				  // 启动PWM
		// Motor.CtrlMode = HFI_SMO; // 低速HFI，高速滑膜
		ParamIdent_Start();
	}
	break;

	case KEY1_DOUBLE_PRESS:
	{
		MotorRun();				 // 启动PWM
		Motor.CtrlMode = IF_SMO; // 低速IF高速SMO
	}
	break;

	case KEY1_LONG_PRESS:
	{
		MotorRun();				   // 启动PWM
		Motor.CtrlMode = HFI_FLUX; // 低速HFI，高速磁链
	}
	break;

	case KEY2_SHORT_PRESS:
	{
		MotorRun();					 // 启动PWM
		Motor.CtrlMode = SPEED_LOOP; // 有感，速度&电流闭环
	}
	break;

	case KEY2_DOUBLE_PRESS:
	{
		MotorRun();				   // 启动PWM
		Motor.CtrlMode = CUR_LOOP; // 有感，电流闭环
	}
	break;

	case KEY2_LONG_PRESS:
	{
		MotorRun();				   // 启动PWM
		Motor.CtrlMode = POS_LOOP; // 有感，位置闭环
	}
	break;

	case KEY3_SHORT_PRESS:
	{
		MotorStop();
		Motor.CtrlMode = STOP;
	}
	break;

	case KEY3_DOUBLE_PRESS:
	{
		MotorRun();				  // 启动PWM
		Motor.CtrlMode = IF_FLUX; // 低速IF，高速FLUX
	}
	break;

	case KEY3_LONG_PRESS:
	{
		Motor.CtrlMode = TO_ZERO;
		PostionToZeroDouble(); // 二次调零
		Motor.CtrlMode = STOP;
	}
	break;

	default:
		break;
	}

	if (Motor.CtrlMode == POS_LOOP)
	{
		pi_spd.Kp = Para.SpdSenKp;
		pi_spd.Ki = Para.SpdSenKi;
	}
	else
	{
		pi_spd.Kp = Para.SpdLessKp;
		pi_spd.Ki = Para.SpdLessKi;
	}
}