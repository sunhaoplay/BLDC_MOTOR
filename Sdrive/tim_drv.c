/**
 * @file tim_drv.c
 * @brief 实现编码器角度/速度估算和 20 kHz FOC 控制中断调度。
 *
 * TIM3 提供正交编码器位置。测速时利用无符号模减法得到相邻计数差，再显式修正
 * 正反向回绕；TIM1 更新回调依次完成 ADC 采样、坐标变换、分频测速和模式控制。
 */

#include "task.h"
#include "motor_para.h"
#include "tim_drv.h"
#include "motor_ctrl.h"
#include "sen.h"
ENCOSPEED EnCoSpeed;

/**
 * @brief 根据编码器计数增量计算机械转速和电气转速。
 *
 * 计数差 Diff 先修正到 [-TIM3_PULSE/2, TIM3_PULSE/2]，避免计数器跨零时产生
 * 巨大速度尖峰。机械转速公式为 rpm=Diff*(ENCODER_FRQ/ENCODE_PULSE)，其中
 * ENCODER_FRQ 已包含每分钟 60 秒及测速调用频率；随后用一阶低通抑制量化噪声。
 * 电气转速等于机械转速乘极对数 Motor.P。
 */
void MotorSpeedGet(void)
{
	static float SpeedMrpmFil = 0.0f;

	EnCoSpeed.EncoderCnt = TIM3->CNT; // 当前CNT的值+溢出次数*定时器的脉冲触发数

	int32_t Diff = (int32_t)((uint32_t)EnCoSpeed.EncoderCnt - (uint32_t)EnCoSpeed.EncoLastCnt); // 利用模算术

	// 通用回绕处理（适用于上溢或下溢）
	if (Diff > (int32_t)(TIM3_PULSE / 2))
		Diff -= (int32_t)TIM3_PULSE;
	else if (Diff < -(int32_t)(TIM3_PULSE / 2))
		Diff += (int32_t)TIM3_PULSE;

	float SpeedMrpmTemp = ((float)Diff * ((float)ENCODER_FRQ / (float)ENCODE_PULSE));

	SpeedMrpmFil = (LPF_U_A * SpeedMrpmTemp) + (LPF_U_B * SpeedMrpmFil);

	Motor.speed_M_rpm = SpeedMrpmFil;

	Motor.speed_E_rpm = Motor.speed_M_rpm * Motor.P; // 电角速度 = 机械角速度 * 极对数
	EnCoSpeed.EncoLastCnt = EnCoSpeed.EncoderCnt;
}

/**
 * @brief 把编码器机械位置换算为一个周期内的转子电角度。
 *
 * theta_e=CNT/ENCODE_PULSE*2PI*P。乘极对数 P 后可能跨越多个电周期，
 * while 循环将结果回绕到 (0,2PI]，供 Park 变换使用。
 */
void MotorAngleGet(void)
{
	Motor.E_theta = (float)TIM3->CNT / ENCODE_PULSE * PIX2 * Motor.P; // CNT/4096(编码器转一圈计数值)*2PI*极对数

	while (Motor.E_theta >= PIX2)
		Motor.E_theta -= PIX2;
	while (Motor.E_theta < 0.0f)
		Motor.E_theta += PIX2;
}

/**
 * @brief HAL 定时器周期完成回调，执行一个高速电机控制周期。
 * @param htim 产生更新事件的定时器句柄；本函数只处理 TIM1。
 *
 * TIM1 每 50 us 进入一次：先更新 ADC 物理量，再完成电流坐标变换；每累计
 * SPEED_CNT 次更新一次编码器速度，最后由 MotorModeSel() 执行当前控制模式。
 * @warning 该函数运行在中断上下文，所有被调函数都必须在下次 TIM1 中断前结束。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == (&htim1)) // FOC控制		20KHz
	{
		GetAdc(); // 获取电流
		AxisDq(); // 坐标轴变换

		// EnCoSpeed.SpeedClock++;
		// if (EnCoSpeed.SpeedClock == SPEED_CNT) // 计算一次电机转速
		// {
		// 	EnCoSpeed.SpeedClock %= SPEED_CNT;
		// 	MotorSpeedGet(); // 速度获取
		// }
		// MotorModeSel(); // 控制模式选择
      if(Init_Over==0)
	  {
		return;
	  }
	 //1.开环强托
		// QtMotor(); // 强拖模式
		// FocSvpwm(); // SVPWM计算
	// 2.有感电流环
	    CurCloseloop(0.3, Motor.E_theta);
		FocSvpwm(); // SVPWM计算
	}
}
