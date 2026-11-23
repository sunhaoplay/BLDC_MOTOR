/**
 * @file hmi_app.h
 * @brief 人机交互、VOFA 串口协议以及按键/电位器业务接口。
 *
 * 上位机可通过 USART1 修改控制器参数和工作模式；设备同时把 13 路关键状态量
 * 按 JustFloat 格式发回 VOFA。按键和电位器提供脱离上位机时的本地控制入口。
 */

#ifndef __HMI_APP_H__
#define __HMI_APP_H__

#include "main.h"


/** @brief 在 IEEE 754 单精度数和 32 位无符号整数之间共享同一组位。 */
typedef union
{
    float fdata;          /**< 待拆分的单精度浮点数。 */
    unsigned long ldata;  /**< 与 fdata 位模式相同的 32 位整数视图。 */
} FloatLongType;

/** @brief 在 IEEE 754 单精度数和四个原始字节之间转换。 */
typedef union
{
    float f_data;       /**< 浮点数视图。 */
    uint8_t c_data[4];  /**< 小端字节视图，供串口协议解析。 */
} FloatToChar;

extern FloatToChar fc;




/** @brief 检查并解析一帧已经接收完成的 VOFA 控制命令。 */
void RecevToVofa(void);


/**
 * @brief 根据当前控制模式把电位器 ADC 值换算成速度、电流或位置给定。
 */
void VrTask(void);



#endif
