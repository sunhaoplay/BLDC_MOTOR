/**
 * @file key_drv.c
 * @brief 实现四路按键的消抖、单击/双击/长按识别状态机。
 *
 * 每个按键拥有独立状态和时间戳。GPIO 为上拉输入，所以读到 0 表示按下。
 * 单击只有在双击等待窗口结束后才上报；长按事件在持续时间达到阈值并释放时上报。
 */
#include "key_drv.h"

typedef struct
{
	GPIO_TypeDef *gpio; /**< 按键所在 GPIO 端口。 */
	uint32_t pin;       /**< 对应 HAL GPIO_PIN_x 位掩码。 */
} Key_GPIO_t;

static Key_GPIO_t g_gpioList[] = 
{
	{GPIOB, GPIO_PIN_12},   // key1
	{GPIOB, GPIO_PIN_13},   // key2
	{GPIOB, GPIO_PIN_14},   // key3
    {GPIOB, GPIO_PIN_11},   // ONOFF
};

#define KEY_NUM_MAX (sizeof(g_gpioList) / sizeof(g_gpioList[0]))

typedef enum
{
	KEY_RELEASE = 0, /**< 当前松开，或正在等待双击窗口结束。 */
	KEY_CONFIRM,     /**< 检测到低电平，等待消抖时间确认。 */
	KEY_SHORTPRESS,  /**< 已确认按下，等待释放或长按超时。 */
	KEY_LONGPRESS    /**< 已达到长按时间，等待释放后上报。 */
} KEY_STATE;

#define CONFIRM_TIME                 10       // 按键消抖时间窗10ms
#define DOUBLE_CLICK_INTERVAL_TIME   300      // 两次抬起时间窗300ms，用来判断是否双击
#define LONGPRESS_TIME               2000     // 长按时间窗2000ms

typedef struct
{
	KEY_STATE keyState;             /**< 当前状态机状态。 */
	uint8_t singleClickNum;         /**< 双击窗口内已经完成的短按次数。 */
	uint64_t firstIoChangeSysTime;  /**< 本次首次检测到按下的毫秒时间戳。 */
	uint64_t firstReleaseSysTime;   /**< 第一次短按释放的毫秒时间戳。 */
} Key_Info_t;

static Key_Info_t g_keyInfo[KEY_NUM_MAX];

/**
 * @brief 配置四个按键为 GPIOB 上拉输入。
 * @note 引脚映射来自 g_gpioList[]，数组顺序同时决定返回事件中的按键编号。
 */
void KeyDrvInit(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	GPIO_InitTypeDef gpioInitStruct;
	gpioInitStruct.Mode = GPIO_MODE_INPUT;
	gpioInitStruct.Pull = GPIO_PULLUP;
	
	for (uint8_t i = 0; i < KEY_NUM_MAX; i++)
	{
		gpioInitStruct.Pin = g_gpioList[i].pin;
		HAL_GPIO_Init(g_gpioList[i].gpio, &gpioInitStruct);
	}
}

/**
 * @brief 推进一步指定按键的事件识别状态机。
 * @param keyIndex 按键数组索引，有效范围 0..KEY_NUM_MAX-1。
 * @return 0 表示事件尚未完成，否则返回该按键的短按、双击或长按事件码。
 *
 * 状态流为 RELEASE -> CONFIRM -> SHORTPRESS；持续按下超过 LONGPRESS_TIME 后进入
 * LONGPRESS。第一次短按释放后保留 singleClickNum，若窗口内再次释放则立即上报
 * 双击，否则窗口超时后上报单击。
 */
static uint8_t KeyScan(uint8_t keyIndex)
{
	uint64_t curSysTime;
	uint8_t keyPress;
	
	keyPress = HAL_GPIO_ReadPin(g_gpioList[keyIndex].gpio, g_gpioList[keyIndex].pin);

	switch (g_keyInfo[keyIndex].keyState)
	{
		case KEY_RELEASE:                                          //  释放状态：判断有无按键按下
			if (!keyPress)                                         //  有按键按下
			{ 
				g_keyInfo[keyIndex].keyState = KEY_CONFIRM;        //  获取系统运行时间
				g_keyInfo[keyIndex].firstIoChangeSysTime = HAL_GetTick();
				break;
			}

			if (g_keyInfo[keyIndex].singleClickNum != 0)
			{
				curSysTime = HAL_GetTick();
				if (curSysTime - g_keyInfo[keyIndex].firstReleaseSysTime > DOUBLE_CLICK_INTERVAL_TIME)
				{
					g_keyInfo[keyIndex].singleClickNum = 0;
					return (keyIndex + 1); //  返回单击按键码值，四个按键短按对应0x01, 0x02, 0x03, 0x04
				}
			}

			break;
			
		case KEY_CONFIRM:
			if (!keyPress)
			{
				curSysTime = HAL_GetTick();
				if (curSysTime - g_keyInfo[keyIndex].firstIoChangeSysTime > CONFIRM_TIME) //  超过时间窗，切换到短按
				{
					g_keyInfo[keyIndex].keyState = KEY_SHORTPRESS;
				}
			}
			else
			{
				g_keyInfo[keyIndex].keyState = KEY_RELEASE;
			}
			break;
			
		case KEY_SHORTPRESS:           //  短按状态：继续判定是短按，还是双击，还是长按
			if (keyPress)
			{
				g_keyInfo[keyIndex].keyState = KEY_RELEASE;
				
				g_keyInfo[keyIndex].singleClickNum++;
				if (g_keyInfo[keyIndex].singleClickNum == 1)
				{
					g_keyInfo[keyIndex].firstReleaseSysTime = HAL_GetTick();
					break;
				}
				else
				{
					curSysTime = HAL_GetTick();
					if (curSysTime - g_keyInfo[keyIndex].firstReleaseSysTime <= DOUBLE_CLICK_INTERVAL_TIME) //  超过双击时间间隔窗，返回双击码值
					{
						g_keyInfo[keyIndex].singleClickNum = 0;
						return (keyIndex + 0x51); //  返回按键码值，四个按键双击对应0x51, 0x52, 0x53, 0x54
					}
				}
			}
			else
			{
				curSysTime = HAL_GetTick();
				if (curSysTime - g_keyInfo[keyIndex].firstIoChangeSysTime > LONGPRESS_TIME)
				{	
					g_keyInfo[keyIndex].keyState = KEY_LONGPRESS;
				}
			}
			break;
		case KEY_LONGPRESS:
			if (keyPress)
			{
				g_keyInfo[keyIndex].keyState = KEY_RELEASE;
				return (keyIndex + 0x81); //返回按键码值，四个按键长按对应0x81, 0x82, 0x83, 0x84
			}
			break;
		default:
			g_keyInfo[keyIndex].keyState = KEY_RELEASE;
			break;
	}
	return 0;
}

/**
 * @brief 依次扫描四个按键并返回第一个非零事件。
 * @return 0 表示本轮无完整事件，否则返回 key_drv.h 定义的事件码。
 * @note 同一扫描周期若多个按键同时产生事件，只返回数组顺序最靠前的一个。
 */
uint8_t GetKeyVal(void)
{
	uint8_t res = 0;

	for (uint8_t i = 0; i < KEY_NUM_MAX; i++)
	{
		res = KeyScan(i);
		if (res != 0)
		{
			break;
		}
	}
	return res;
}
