#ifndef FOC_TRACE_H
#define FOC_TRACE_H

#include "main.h"

#define FOC_TRACE_POINTS 1024U

typedef struct
{
    uint32_t tick;
    uint16_t adc_u, adc_w;
    float iu, iv, iw;
    float id, iq;
    float theta;
} FOC_TRACE_SAMPLE;

typedef enum
{
    FOC_TRACE_IDLE,
    FOC_TRACE_RECORDING,
    FOC_TRACE_READY,
    FOC_TRACE_SENDING
} FOC_TRACE_STATE;

extern FOC_TRACE_SAMPLE FocTraceBuffer[FOC_TRACE_POINTS];
extern volatile FOC_TRACE_STATE FocTraceState;
extern volatile uint16_t FocTraceCount;
/** 调试器写入 1..65535，在主循环请求再抓取一次；数值为记录分频。 */
extern volatile uint32_t FocTraceRequest;

/** 仅主循环调用；缓存空闲时启动一次记录，divider=1 表示每拍记录。 */
uint8_t FocTraceArm(uint16_t divider);
/** 仅电流控制中断调用，无格式化、无串口发送。 */
void FocTraceRecord(void);
/** 主循环发送冻结数据为 CSV，串口繁忙时直接返回。 */
void FocTraceService(void);

#endif
