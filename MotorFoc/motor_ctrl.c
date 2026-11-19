/**
 * @file motor_ctrl.c
 * @brief 电机控制顶层状态机、IF 启动和多观测器全速域融合实现。
 *
 * 每个 TIM1 控制周期先在 AxisDq() 更新电流坐标和观测器，再由 MotorModeSel()
 * 执行当前闭环并更新 SVPWM。无感模式在低速 HFI 或 IF 与高速 SMO/磁链之间
 * 采用迟滞阈值、角度同步和 PI 状态复位，降低切换时的电流与转矩冲击。
 */

#include "motor_ctrl.h"

// FOC 坐标变换与电机运行状态对象
CLARKE /******************/ Clarke = CLARKE_DEFAULTS;
PARK /********************/ Park = PARK_DEFAULTS;
IPARK /*******************/ Ipark = IPARK_DEFAULTS;
motor_p /*****************/ Motor = MOTOR_DEFAULTS;

// 频率、速度、电流给定的斜率规划对象
GXieLv /******************/ IfFreqGXieLv = GXieLv_DEFAULTS;
GXieLv /******************/ SpdRefGXieLv = GXieLv_DEFAULTS;
GXieLv /******************/ IdqRefGXieLv = GXieLv_DEFAULTS;

// 无感观测器、运行参数与位置控制对象
Sensorless_Control /******/ SensorLess = Sensorless_DEFAULTS;
IPARK /*******************/ IparkSmoPv = IPARK_DEFAULTS;
PARA /********************/ Para;
POS_DRV /*****************/ Pos;

// FLUX_DRV                 flux = FLUX_DEFAULTS;

bool Init_Over = 0;
bool HFI_Init_Angle_flag = 0;

/**
 * @brief 让当前斜坡输出以固定步长逼近目标值。
 * @param pv 斜坡对象，XieLv_X 为目标、XieLv_Y 为当前输出、XieLv_Grad 为步长。
 *
 * 当误差绝对值大于一步时只增减一个步长；进入一步范围后直接赋目标，避免
 * 在目标两侧往复振荡。调用间隔由上层 Timer_Count/Grad_Timer 决定。
 */
void GradXieLv(p_GXieLv pv)
{
	if (pv->XieLv_X < (pv->XieLv_Y - pv->XieLv_Grad)) // XieLv_Grad 减梯度值时候
	{
		pv->XieLv_Y = pv->XieLv_Y - pv->XieLv_Grad;
	}
	else if (pv->XieLv_X > (pv->XieLv_Y + pv->XieLv_Grad)) // 加梯度值时候
	{
		pv->XieLv_Y = pv->XieLv_Y + pv->XieLv_Grad;
	}
	else
	{
		pv->XieLv_Y = pv->XieLv_X;
	}
}

/**
 * @brief 立即禁止 TIM1 三相主/互补 PWM、清零比较值并复位控制动态状态。
 *
 * CCER 与 0xAAAA 相与会清除各通道输出使能位；CCR1..3 置零后调用
 * VariableClear()，确保下一次启动不继承 PI 积分或观测器历史。
 */
void MotorStop(void)
{
	// 关闭三相互补 PWM 输出，防止停机后继续驱动功率管
	TIM1->CCER &= 0xAAAA; // 不使能PWM输出通道OC1/OC1N/OC2/OC2N/OC3/OC3N

	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;

	// VariableClear(); // 变量清零
}

/**
 * @brief 使能 TIM1 三相主通道和互补通道 PWM 输出。
 * @note 本函数只开放功率输出，不选择控制模式，也不校验故障状态。
 */
void MotorRun(void)
{
	// Fault_Detection();				//启动前先进行故障检测
	// 使能三相上下桥 PWM 输出，后续占空比由 SVPWM 更新
	TIM1->CCER |= 0x5555;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

/**
 * @brief 按 360 点正余弦表产生幅值 0.5 V 的开环旋转电压矢量。
 *
 * 每 QT_FRQ 个 20 kHz 周期角度索引增加 1 度，因此电角频率约为
 * PWM_FRQ/(QT_FRQ*360)。该模式没有电流闭环，只适合受控调试或预定位。
 */
void QtMotor(void)
{
	Para.IfFrq++;
	if (Para.IfFrq == QT_FRQ) // 强拖频率
	{
		Para.IfFrq = 0;
		Ipark.Alpha = ualpha_tab[Para.IfAngle];
		Ipark.Beta = ubeta_tab[Para.IfAngle];
		Para.IfAngle++;
		if (Para.IfAngle > 359)
			Para.IfAngle = 0;
	}
}

/**
 * @brief 把电流 PI 经反 Park 得到的 alpha/beta 电压送入 SVPWM。
 *
 * SvpwmCal() 最终直接更新 TIM1 CCR1/2/3，因此本函数应在每个运行控制模式
 * 完成电流环后调用一次。
 */
void FocSvpwm(void)
{
	// 电流环输出的 d/q 电压经 IPark 后送入 SVPWM 更新三相占空比
	Svpwm.Ualpha = Ipark.Alpha; // IPARK的数据传入SVPWM
	Svpwm.Ubeta = Ipark.Beta;
	SvpwmCal((M_SVPWM)&Svpwm); // 将Alpha和Beta电压带入计算SVPWM的占空比
}


/**
 * @brief 更新一个高速周期的电流坐标、编码器角度和全部相关观测器状态。
 *
 * 数据流为 ADC 三相电流 -> Clarke -> 观测器/编码器角度选择 -> Park。磁链模型
 * 每拍后台更新，融合模式再执行相应 HFI/SMO/IF 切换逻辑。最终 Park.Theta
 * 严格按当前模式选择，得到供内层电流 PI 使用的 Park.Ds/Park.Qs。
 * @note 由 TIM1 更新中断在 GetAdc() 后每 50 us 调用。
 */
void AxisDq(void)
{
	// FOC 主采样链路：三相电流 -> Clarke -> 角度选择 -> Park
	Para.SpeedCalTime++;

	// 1. 采样三相电流并做 Clarke 变换
	Clarke.Iu = AdcFilPara.CurU;
	Clarke.Iv = AdcFilPara.CurV;
	Clarke.Iw = AdcFilPara.CurW;
	ClarkeCale((M_CLARKE)&Clarke);

	MotorAngleGet(); // 获取编码器角度

	// // 2. 运行磁链观测器，供 Flux 控制模式或切换过程使用
	// FluxCale(&flux, Clarke.Alpha, Clarke.Beta, Ipark.Alpha, Ipark.Beta);

	// // 3. 根据控制模式更新无感角度或切换状态
	// if (Motor.CtrlMode == HFI_CTRL)
	// {
	// 	HfiAngleCale((p_HFI)&HFI);
	// }
	// else if (Motor.CtrlMode == HFI_SMO)
	// {
	// 	Process_HFI_SMO(); // HFI+SMO 处理
	// }
	// else if (Motor.CtrlMode == HFI_FLUX)
	// {
	// 	Process_HFI_FLUX(); // HFI+Flux 处理
	// }
	// else if (Motor.CtrlMode == IF_SMO)
	// {
	// 	Process_IF_SMO();
	// }
	// else if (Motor.CtrlMode == IF_FLUX)
	// {
	// 	Process_IF_FLUX();
	// }

	// // 4. 选择 Park 变换使用的电角度来源
	// if ((QT_IF_MODE == 3) && ((Motor.CtrlMode == IF_CTRL) || ((Motor.CtrlMode == IF_SMO || Motor.CtrlMode == IF_FLUX) && (SensorLess.Observer == 1))))
	// {
	// 	Park.Theta = Para.FocAngle;
	// }
	// else if (Motor.CtrlMode == HFI_CTRL)
	// {
	// 	Park.Theta = PllHfiPara.Theta; // 高频注入角度
	// }
	// else if (Para.SmoSwFlag)
	// {
	// 	Park.Theta = PllSmoPara.Angle; // 滑膜观测角度
	// }
	// else if (Motor.CtrlMode == HFI_SMO || Motor.CtrlMode == HFI_FLUX || Motor.CtrlMode == IF_SMO || Motor.CtrlMode == IF_FLUX)
	// {
	// 	Park.Theta = SensorLess.theta; // 观测角度
	// }
	// else
	// {
	// 	Park.Theta = Motor.E_theta; // 编码器角度
	// }
    Park.Theta = Motor.E_theta; // 编码器角度
	// 5. Park 变换，得到 d/q 轴电流反馈
	Park.Alpha = Clarke.Alpha;
	Park.Beta = Clarke.Beta;
	ParkCale((M_PARK)&Park);
}

/**
 * @brief 清除停机后不应保留的 PI、斜坡、观测器和切换动态状态。
 *
 * 该函数不修改 Motor.CtrlMode 和静态配置增益，只清除运行历史。包括四个 PI
 * 积分、IF/速度斜坡、反 Park/SVPWM 缓存、HFI/SMO PLL、电流观测状态、磁链
 * 积分以及全速域切换计数，确保下一次启动从确定的低速默认状态开始。
 */
void VariableClear(void)
{
	// 停机或重新启动前清除运行状态，避免旧积分和旧角度影响下次启动
	// FluxFlag = 0;
	Para.FluxSpeedCnt = 0;
	Para.RpmCmd = 0;
	Para.FocAngle = 0;

	Motor.speed_M_rpm = 0; // 电机机械角速度清零
	Motor.speed_E_rpm = 0; // 电机电角速度清零

	// Motor.CtrlMode = 0;	   // 电机控制模式清零

	Para.IfFrq = 0; // 强拖频率计数清零

	// 速度、电流、位置 PI 控制器状态清零
	pi_spd.Ref = 0;
	pi_spd.Fbk = 0;
	pi_spd.err = 0;
	pi_spd.err_l = 0;
	pi_spd.Out = 0;
	pi_spd.OutF = 0;
	pi_spd.integrator = 0;

	pi_id.Ref = 0;
	pi_id.Fbk = 0;
	pi_id.err = 0;
	pi_id.Out = 0;
	pi_id.OutF = 0;
	pi_id.err_l = 0;
	pi_id.integrator = 0;

	pi_iq.Ref = 0;
	pi_iq.Fbk = 0;
	pi_iq.err = 0;
	pi_iq.Out = 0;
	pi_iq.OutF = 0;
	pi_iq.err_l = 0;
	pi_iq.integrator = 0;

	pi_pos.Ref = 0;
	pi_pos.Fbk = 0;
	pi_pos.err = 0;
	pi_pos.Out = 0;
	pi_pos.OutF = 0;
	pi_pos.err_l = 0;
	pi_pos.integrator = 0;

	// IF、速度、电流给定斜率规划状态清零
	IfFreqGXieLv.Timer_Count = 0;
	Para.IfTheta = 0;
	IfFreqGXieLv.XieLv_Y = 0;

	SpdRefGXieLv.Timer_Count = 0;
	SpdRefGXieLv.XieLv_Y = 0;

	IdqRefGXieLv.Timer_Count = 0;
	IdqRefGXieLv.XieLv_Y = 0;

	Ipark.Alpha = 0;
	Ipark.Beta = 0;
	Ipark.Ds = 0;
	Ipark.Qs = 0;

	// 逆 Park 与 SVPWM 输出缓存清零
	Svpwm.Ualpha = 0;
	Svpwm.Ubeta = 0;

	// SMO/HFI PLL 状态清零
	// PllSmoPara.Theta = 0;
	// PllSmoPara.Angle = 0;
	// PllSmoPara.Theta_Err = 0;
	// PllSmoPara.Theta_Err_last = 0;
	// PllSmoPara.Omega = 0;
	// PllSmoPara.Omega_F = 0;
	// PllSmoPara.Omega2 = 0;
	// PllSmoPara.Omega2_F = 0;
	// PllSmoPara.Theta_last = 0;

	// PllHfiPara.Theta = 0;
	// PllHfiPara.Theta_Err = 0;
	// PllHfiPara.Theta_Err_last = 0;
	// PllHfiPara.Omega = 0;
	// PllHfiPara.Omega_F = 0;
	// PllHfiPara.Omega2 = 0;
	// PllHfiPara.Omega2_F = 0;
	// PllHfiPara.Theta_last = 0;
	// PllHfiPara.Angle = 0;

	// HFI 与 SMO 内部采样/估算状态清零
	// HFI_Init_Angle_flag = 0;
	// HFI.Ialpha_last = 0;
	// HFI.Ibeta_last = 0;
	// HFI.Id_last = 0;
	// HFI.Iq_last = 0;
	// HFI.Uin = 0;
	// AngleSmoPara.Est_Ialpha = 0;
	// AngleSmoPara.Est_Ibeta = 0;
	// AngleSmoPara.Ialpha_Err = 0;
	// AngleSmoPara.Ibeta_Err = 0;
	// AngleSmoPara.Zalpha = 0;
	// AngleSmoPara.Zbeta = 0;
	// AngleSmoPara.Ealpha = 0;
	// AngleSmoPara.Ebeta = 0;
	// HfiUinFlag = 0;

	// 无感观测器状态回到低速默认状态
	SensorLess.Observer = 1;
	SensorLess.Observer_last = 1;
	SensorLess.theta = 0;
	SensorLess.Omega = 0;
	SensorLess.Speed_Fbk = 0;
	SensorLess.Speed_Ref = 0;

	// FluxReset(&flux);

	// 观测器切换计数与标志清零
	Para.SpeedCalTime = 0;
	Para.UpTime = 0;
	Para.DownTime = 0;
	Para.CalTime = 0;
	Para.ObsSwFlag = 0;
	Para.SmoSwFlag = 0;
}