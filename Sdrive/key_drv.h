/**
 * @file key_drv.h
 * @brief 四路低电平按键的短按、双击和长按事件编码接口。
 *
 * 调用者周期性执行 GetKeyVal()，驱动内部利用 HAL_GetTick() 消抖并维护状态机。
 * 返回码低位区分按键编号，高位区分短按、双击和长按事件。
 */
#ifndef _KEY_DRV_H_
#define _KEY_DRV_H_

#include <main.h>

#define KEY1_SHORT_PRESS   0X01 /**< 按键 1 单击事件码。 */
#define KEY1_DOUBLE_PRESS  0X51 /**< 按键 1 双击事件码。 */
#define KEY1_LONG_PRESS    0X81 /**< 按键 1 长按释放事件码。 */
#define KEY2_SHORT_PRESS   0X02 /**< 按键 2 单击事件码。 */
#define KEY2_DOUBLE_PRESS  0X52 /**< 按键 2 双击事件码。 */
#define KEY2_LONG_PRESS    0X82 /**< 按键 2 长按释放事件码。 */
#define KEY3_SHORT_PRESS   0X03 /**< 按键 3 单击事件码。 */
#define KEY3_DOUBLE_PRESS  0X53 /**< 按键 3 双击事件码。 */
#define KEY3_LONG_PRESS    0X83 /**< 按键 3 长按释放事件码。 */
#define KEY4_SHORT_PRESS   0X04 /**< 按键 4 单击事件码。 */
#define KEY4_DOUBLE_PRESS  0X54 /**< 按键 4 双击事件码。 */
#define KEY4_LONG_PRESS    0X84 /**< 按键 4 长按释放事件码。 */

/**
 * @brief 开启 GPIOB 时钟并把四个按键引脚配置为上拉输入。
 * @note 按键按下时引脚被拉低，松开时由内部上拉保持高电平。
 */
void KeyDrvInit(void);
/**
 * @brief 扫描四个按键并返回本次产生的第一个完整按键事件。
 * @return 0 表示无事件；0x01..0x04 为短按，0x51..0x54 为双击，
 *         0x81..0x84 为长按释放。
 * @note 为保证 10 ms 消抖和 300 ms 双击窗口准确，应以稳定的短周期反复调用。
 */
uint8_t GetKeyVal(void);

#endif
