/**
 * @file motor_ctrl.c
 * @brief 电机控制顶层状态机、IF 启动和多观测器全速域融合实现。
 *
 * 每个 ADC1 注入采样周期先在 AxisDq() 更新电流坐标，再执行电流闭环和 SVPWM。
 * 无感模式的扩展接口在低速 HFI 或 IF 与高速 SMO/磁链之间
 * 采用迟滞阈值、角度同步和 PI 状态复位，降低切换时的电流与转矩冲击。
 */

#include "motor_ctrl.h"
#include "sen.h"
#include "param_ident.h"
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

volatile bool Init_Over = 0;
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
 * @brief 禁止电流闭环和三相功率输出，保留 TIM1 计数及 CH4 内部采样触发。
 */
void MotorStop(void)
{
	// Init_Over = false;
	TIM1->CCER &= ~MOTOR_PWM_CHANNEL_MASK;
	TIM1->CCR1 = TIM1_PERIOD_HALF;
	TIM1->CCR2 = TIM1_PERIOD_HALF;
	TIM1->CCR3 = TIM1_PERIOD_HALF;
	VariableClear();
}

/**
 * @brief 使能 TIM1 三相主通道和互补通道 PWM 输出。
 * @note 只在没有采样故障时开放功率输出，不选择控制模式。
 */
void MotorRun(void)
{
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	if (AdcFault != 0U)
	{
		__set_PRIMASK(primask);
		return;
	}
	/* TIM1 已由采样驱动启动；直接切换使能位，避免反复 Start 导致 HAL_BUSY。 */
	TIM1->CCER |= MOTOR_PWM_CHANNEL_MASK;
	__HAL_TIM_MOE_ENABLE(&htim1);
	__set_PRIMASK(primask);
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
 * @note 由 ADC1 注入完成回调在 GetAdc() 后每 50 us 调用。
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


/**
 * @brief 根据 Motor.CtrlMode 执行本周期对应的控制算法和 PWM 更新。
 *
 * 有感模式调用速度/电流/位置级联环；IF 模式选择查表强拖或 IF 电流闭环；HFI、
 * HFI+SMO、HFI+Flux、IF+SMO、IF+Flux 分别进入对应启动与观测器状态机。
 * 所有产生电压指令的模式最终调用 FocSvpwm()。未知模式执行 MotorStop()。
 */
void MotorModeSel(void)
{
	ParamIdent_MonitorModeExit();

	// 控制入口：根据 CtrlMode 执行对应闭环，再统一进入 SVPWM 输出
	switch (Motor.CtrlMode)
	{
	case SPEED_LOOP:
	{
		SpeedCloseloop(Para.SpeedRef, 1);
		FocSvpwm(); // SVPWM计算
		break;
	}

	case CUR_LOOP:
	{
		CurCloseloop(Para.CurRef, Motor.E_theta);
		FocSvpwm(); // SVPWM计算
		break;
	}

	case POS_LOOP:
	{
		PosCloseloop();
		FocSvpwm(); // SVPWM计算
		break;
	}

	// case IF_CTRL:
	// {
	// 	switch (QT_IF_MODE)
	// 	{
	// 	case 1: // 强拖至角度
	// 	{
	// 		QtMotor();	// 电机强拖控制
	// 		FocSvpwm(); // SVPWM计算
	// 		if (Para.IfAngle == QT_ANGLE)
	// 		{
	// 			MotorStop(); // 停机
	// 			Para.IfAngle = 0;
	// 			Para.IfFrq = 0;
	// 		}
	// 		break;
	// 	}
	// 	case 2: // 强拖旋转
	// 	{
	// 		QtMotor();	// 电机强拖控制
	// 		FocSvpwm(); // SVPWM计算
	// 		break;
	// 	}
	// 	case 3: // IF启动
	// 	{
	// 		IFStartControl(); //	I/F流频比控制
	// 		FocSvpwm();
	// 		break;
	// 	}
	// 	default:
	// 		break;
	// 	}
	// 	break;
	// }

	// case HFI_CTRL:
	// {
	// 	if (HFI_Init_Angle_flag) // 初始角度检测完毕后
	// 	{
	// 		HfiSpdCloseLoop(); // HFI速度闭环模式
	// 	}
	// 	else
	// 	{
	// 		HfiInitTheta(); // HFI初始角度检测
	// 	}

	// 	FocSvpwm(); // SVPWM计算
	// 	break;
	// }

	// case HFI_SMO: // 低速HFI，高速SMO的全速域无感控制
	// {
	// 	LessHfiSmoControl();
	// 	FocSvpwm(); // SVPWM计算
	// 	break;
	// }

	// case HFI_FLUX: // 低速HFI，高速FLUX的全速域无感控制
	// {
	// 	LessHfiFluxControl();
	// 	FocSvpwm();
	// 	break;
	// }

	// case IF_SMO:
	// {
	// 	if (SensorLess.Observer == 1)
	// 	{
	// 		// 低速阶段使用 IF 开环角度和固定电流启动
	// 		// IFObserverThetaRampCale();
	// 		// CurCloseloop(1.0f, Para.FocAngle);
	// 	}
	// 	else
	// 	{
	// 		// 高速阶段使用 SMO 速度闭环
	// 		// SensorLess.Speed_Ref = GetObserverSpeedRef();
	// 		// SmoObserver(Park.Ds, Park.Qs);
	// 	}
	// 	FocSvpwm();
	// 	Para.SpeedCalTime %= SPD_CAL_PERIOD;
	// 	break;
	// }

	// case IF_FLUX:
	// {
	// 	if (SensorLess.Observer == 1)
	// 	{
	// 		// 低速阶段使用 IF 开环角度和固定电流启动
	// 		// IFObserverThetaRampCale();
	// 		// CurCloseloop(1.0f, Para.FocAngle);
	// 	}
	// 	else
	// 	{
	// 		// 高速阶段使用 Flux 速度闭环
	// 		// SensorLess.Speed_Ref = GetObserverSpeedRef();
	// 		// FluxObserver(Park.Ds, Park.Qs);
	// 	}
	// 	FocSvpwm();
	// 	Para.SpeedCalTime %= SPD_CAL_PERIOD;
	// 	break;
	// }
	case TO_ZERO:
	{
		break;
	}

	case PARAM_IDENT:
	{
		ParamIdent_Run20kHz();
		break;
	}
    // 强拖模式 
    case OPEN_LOOP_SPIN:
    {
		paramidentParis();//极对数辨识
        // OpenLoopSpinControl();
        break;
    }
	default:
	{
		MotorStop(); // 停机
		break;
	}
	}
}