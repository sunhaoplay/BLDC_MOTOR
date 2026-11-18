/**
 * @file foc.h
 * @brief FOC 坐标变换和空间矢量 PWM 的数据结构与计算接口。
 *
 * 三相电流先经 Clarke 变换到静止 alpha/beta 坐标，再按转子电角度经 Park
 * 变换到旋转 d/q 坐标；PI 输出的 d/q 电压经反 Park 变回 alpha/beta，最后由
 * SVPWM 换算为 TIM1 三相比较值。
 */

#ifndef __FOC_H
#define __FOC_H

// #include "delay.h"

/** @brief Clarke 变换的三相输入与 alpha/beta 输出。 */
typedef struct
{
	float Iu;    /**< U 相电流，单位 A。 */
	float Iv;    /**< V 相电流，单位 A。 */
	float Iw;    /**< W 相电流，单位 A；当前变换式不直接使用。 */
	float Alpha; /**< 静止坐标 alpha 轴分量，单位 A。 */
	float Beta;  /**< 静止坐标 beta 轴分量，单位 A。 */
} CLARKE, *M_CLARKE;

/** @brief Park 变换的静止坐标输入、角度和旋转坐标输出。 */
typedef struct
{
	float Alpha; /**< 静止 alpha 轴分量。 */
	float Beta;  /**< 静止 beta 轴分量。 */
	float Theta; /**< 坐标系电角度，单位 rad。 */
	float Ds;    /**< 旋转 d 轴分量。 */
	float Qs;    /**< 旋转 q 轴分量。 */
} PARK, *M_PARK;

/** @brief 反 Park 变换的 d/q 输入、角度和 alpha/beta 输出。 */
typedef struct
{
	float Alpha; /**< 计算得到的静止 alpha 轴分量。 */
	float Beta;  /**< 计算得到的静止 beta 轴分量。 */
	float Theta; /**< 坐标系电角度，单位 rad。 */
	float Ds;    /**< 输入 d 轴分量。 */
	float Qs;    /**< 输入 q 轴分量。 */
} IPARK, *M_IPARK;

#define CLARKE_DEFAULTS {0, 0, 0, 0, 0} /**< Clarke 状态全零初始化。 */
#define PARK_DEFAULTS {0, 0, 0, 0, 0}   /**< Park 状态全零初始化。 */
#define IPARK_DEFAULTS {0, 0, 0, 0, 0}  /**< 反 Park 状态全零初始化。 */

/**
 * @brief 当两个有效矢量时间之和超过周期 s 时，按比例压缩 x 和 y。
 * @note 比例缩放保持 x:y 不变，从而保持合成电压矢量方向，仅降低幅值。
 */
#define MODIFY_TX_TY(x, y, z, s) \
	{                            \
		z = x + y;               \
		if (x + y > s)           \
		{                        \
			x = x / z * s;       \
			y = y / z * s;       \
		}                        \
	}

/** @brief SVPWM 输入、中间投影量、输出比较量和扇区。 */
typedef struct
{
	float Ualpha;      /**< alpha 轴电压指令，单位 V。 */
	float Ubeta;       /**< beta 轴电压指令，单位 V。 */
	float Ta;          /**< A 相归一化调制量，随后换算为 CCR1。 */
	float Tb;          /**< B 相归一化调制量，随后换算为 CCR2。 */
	float Tc;          /**< C 相归一化调制量，随后换算为 CCR3。 */
	float u1;          /**< 扇区判定投影量 Ubeta。 */
	float u2;          /**< 扇区判定投影量 0.5*Ubeta+sqrt(3)/2*Ualpha。 */
	float u3;          /**< 扇区判定投影量 u2-u1。 */
	float T0;          /**< 零矢量作用时间，当前简化算法未直接使用。 */
	uint8_t VecSector; /**< 空间矢量所在扇区，正常范围 1..6。 */
} SVPWM, *M_SVPWM;

#define SVPWM_DEFAULTS {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} /**< SVPWM 状态全零初始化。 */

extern SVPWM Svpwm; /**< 全局 SVPWM 计算对象。 */

/** @brief 计算空间矢量扇区和三相 TIM1 比较值。 */
void SvpwmCal(M_SVPWM pv);
/** @brief 把三相电流变换到静止 alpha/beta 坐标。 */
void ClarkeCale(M_CLARKE pv);
/** @brief 把 alpha/beta 分量旋转到转子同步 d/q 坐标。 */
void ParkCale(M_PARK pv);
/** @brief 把 d/q 电压反变换到静止 alpha/beta 坐标。 */
void IPARK_Cale(M_IPARK pv);

#endif
