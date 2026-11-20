#ifndef FOC_WAVE_H
#define FOC_WAVE_H

#include "main.h"

/* 20 kHz / 5 = 4 kHz；3 个 float + JustFloat 帧尾，每帧 16 字节。 */
#define FOC_WAVE_DIVIDER       5U
#define FOC_WAVE_BUFFER_FRAMES 128U
#define FOC_WAVE_BATCH_FRAMES  16U

extern volatile uint8_t FocWaveEnabled;
extern volatile uint32_t FocWaveFault;    /**< 0=正常；1=配置/带宽不足；2=串口占用失败。 */
extern volatile uint32_t FocWaveSamples;  /**< 成功入队的采样组数。 */
extern volatile uint32_t FocWaveSent;     /**< DMA 完整发送的采样组数。 */
extern volatile uint32_t FocWaveDropped;  /**< 队满丢弃的组数，仅 ADC 回调写入。 */
extern volatile uint32_t FocWaveTxDropped; /**< 发送失败而丢弃的组数，仅主循环写入。 */
extern volatile uint32_t FocWaveTxErrors;

/** UART_DMA_Init 后、允许电流环前，仅初始化一次；失败只禁用波形。 */
HAL_StatusTypeDef FocWaveInit(void);
/** ADC 控制回调调用：固定分频后复制同一拍三相电流，不调用 HAL 串口。 */
void FocWaveCapture(float iu, float iv, float iw);
/** 主循环调用：直接从 SRAM 队列批量启动 DMA，无等待、无字符串格式化。 */
void FocWaveService(void);

#endif
