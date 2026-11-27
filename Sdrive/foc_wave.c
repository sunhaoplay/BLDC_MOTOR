/** @brief 连续三通道波形：ADC 单生产者，主循环单消费者，USART1 DMA 零拷贝。 */
#include "foc_wave.h"
#include "motor_ctrl.h"
#include "usart.h"
#include <float.h>
#include <stddef.h>
#include "param_ident.h"
typedef struct
{
    float channels[3];
    uint32_t tail;
} FOC_WAVE_FRAME;

_Static_assert(sizeof(float) == 4U && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128,
               "JustFloat requires IEEE 754 binary32");
_Static_assert(sizeof(FOC_WAVE_FRAME) == 16U && offsetof(FOC_WAVE_FRAME, tail) == 12U,
               "JustFloat frame must contain exactly 16 bytes");
_Static_assert(FOC_WAVE_BUFFER_FRAMES >= FOC_WAVE_BATCH_FRAMES &&
               (FOC_WAVE_BUFFER_FRAMES & (FOC_WAVE_BUFFER_FRAMES - 1U)) == 0U,
               "Wave buffer size must be a power of two and fit one batch");
_Static_assert(FOC_WAVE_BATCH_FRAMES > 0U && FOC_WAVE_BATCH_FRAMES * 16U <= 65535U,
               "Wave batch must fit the HAL DMA length");
_Static_assert(FOC_WAVE_DIVIDER > 0U && PWM_FRQ % FOC_WAVE_DIVIDER == 0U,
               "Wave divider must divide the nominal control frequency");
_Static_assert(FOC_WAVE_DEFAULT_VIEW <= FOC_WAVE_VIEW_DQ_SPEED,
               "Unknown default wave view");

static FOC_WAVE_FRAME frames[FOC_WAVE_BUFFER_FRAMES]; /* 普通 SRAM，可供 DMA 读取。 */
static volatile uint32_t write_pos, read_pos;
static uint32_t capture_clock;
static uint16_t tx_frames;
volatile uint8_t FocWaveEnabled;
volatile uint32_t FocWaveView = FOC_WAVE_DEFAULT_VIEW;
volatile uint32_t FocWaveFault;
volatile uint32_t FocWaveSamples, FocWaveSent, FocWaveDropped, FocWaveTxDropped, FocWaveTxErrors;

HAL_StatusTypeDef FocWaveInit(void)
{
    FocWaveEnabled = 0U;
    FocWaveView = FOC_WAVE_DEFAULT_VIEW;
    FocWaveFault = 0U;
    write_pos = read_pos = capture_clock = 0U;
    tx_frames = 0U;
    FocWaveSamples = FocWaveSent = FocWaveDropped = FocWaveTxDropped = FocWaveTxErrors = 0U;
    /* 8N1 每字节占 10 bit，最多使用名义串口带宽的 80%。 */
    const uint32_t bit_rate = (PWM_FRQ / FOC_WAVE_DIVIDER) * sizeof(FOC_WAVE_FRAME) * 10U;
    if (bit_rate > huart1.Init.BaudRate * 8U / 10U)
    {
        FocWaveFault = 1U;
        return HAL_ERROR;
    }
    if (uart1_stream_enable() != HAL_OK)
    {
        FocWaveFault = 2U;
        return HAL_BUSY;
    }
    __DMB();
    FocWaveEnabled = 1U;
    return HAL_OK;
}

void FocWaveCapture(float iu, float iv, float iw)
{
    if (FocWaveEnabled == 0U)
    {
        capture_clock = 0U;
        return;
    }
    if (++capture_clock < FOC_WAVE_DIVIDER)
        return;
    capture_clock = 0U;

    const uint32_t pos = write_pos;
    const uint32_t consumed = read_pos;
    __DMB();
    if ((uint32_t)(pos - consumed) >= FOC_WAVE_BUFFER_FRAMES)
    {
        ++FocWaveDropped; /* 绝不等待串口，也不覆盖 DMA 尚未发送完的数据。 */
        return;
    }
    FOC_WAVE_FRAME *frame = &frames[pos & (FOC_WAVE_BUFFER_FRAMES - 1U)];
    switch (FocWaveView)
    {
    case FOC_WAVE_VIEW_FILTERED:
        frame->channels[0] = AdcFilPara.CurU;
        frame->channels[1] = AdcFilPara.CurV;
        frame->channels[2] = AdcFilPara.CurW;
        break;
    case FOC_WAVE_VIEW_DQ_SPEED:
        // frame->channels[0] = Park.Ds;
        // frame->channels[1] = Park.Qs;
        // frame->channels[2] = Motor.speed_M_rpm;
        // frame->channels[2] = (float)TIM3->CNT;
        // frame->channels[0] = g_paramIdent.result_Ld;
        frame->channels[0] = (float)g_paramIdent.result_PP;
        frame->channels[1] = g_paramIdent.result_Lq;
        frame->channels[2] = g_paramIdent.result_Rs;
        // frame->channels[1] = (float)g_paramIdent.progress;
        // frame->channels[2] = (float)g_paramIdent.sample_count;
        break;
    default:
        frame->channels[0] = iu;
        frame->channels[1] = iv;
        frame->channels[2] = iw;
        break;
    }
    frame->tail = 0x7F800000U; /* STM32 小端字节序：00 00 80 7F。 */
    ++FocWaveSamples;
    __DMB();
    write_pos = pos + 1U;
}

void FocWaveService(void)
{
    if (FocWaveFault != 0U)
        return;
    if (tx_frames != 0U)
    {
        const UART1_STREAM_STATE state = uart1_stream_state();
        if (state == UART1_STREAM_BUSY || state == UART1_STREAM_ABORTING)
            return;
        if (state == UART1_STREAM_DONE)
            FocWaveSent += tx_frames;
        else
        {
            /* 此块可能已部分到达上位机；按整块丢失统计，不重发旧电流。 */
            FocWaveTxDropped += tx_frames;
            ++FocWaveTxErrors;
        }
        __DMB();
        read_pos += tx_frames; /* 完成/中止确认之后，生产者才能复用这块 SRAM。 */
        tx_frames = 0U;
    }

    const uint32_t pos = read_pos;
    const uint32_t produced = write_pos;
    __DMB();
    uint32_t count = produced - pos;
    if (count == 0U || (count < FOC_WAVE_BATCH_FRAMES && Init_Over && FocWaveEnabled != 0U))
        return;
    if (count > FOC_WAVE_BATCH_FRAMES)
        count = FOC_WAVE_BATCH_FRAMES;
    const uint32_t index = pos & (FOC_WAVE_BUFFER_FRAMES - 1U);
    const uint32_t contiguous = FOC_WAVE_BUFFER_FRAMES - index;
    if (count > contiguous)
        count = contiguous;

    const HAL_StatusTypeDef status = uart1_stream_send((const uint8_t *)&frames[index],
                                                       (uint16_t)(count * sizeof(FOC_WAVE_FRAME)));
    if (status == HAL_OK)
        tx_frames = (uint16_t)count;
    else if (status != HAL_BUSY)
    {
        FocWaveTxDropped += count;
        ++FocWaveTxErrors;
        __DMB();
        read_pos = pos + count;
    }
}
