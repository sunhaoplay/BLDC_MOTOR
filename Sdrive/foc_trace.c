/** @brief 固定周期记录 1024 点，冻结后从主循环经 USART1 导出 CSV。 */
#include "foc_trace.h"
#include "adc_drv.h"
#include "motor_ctrl.h"
#include "usart.h"

_Static_assert(sizeof(FOC_TRACE_SAMPLE) == 32U, "Trace samples must occupy 32 bytes");
FOC_TRACE_SAMPLE FocTraceBuffer[FOC_TRACE_POINTS];
volatile FOC_TRACE_STATE FocTraceState;
volatile uint16_t FocTraceCount;
volatile uint32_t FocTraceRequest;
static uint16_t trace_divider, trace_clock;
static uint16_t send_index;

uint8_t FocTraceArm(uint16_t divider)
{
    if (divider == 0U || FocTraceState != FOC_TRACE_IDLE)
        return 0U;
    FocTraceCount = 0U;
    trace_divider = divider;
    trace_clock = 0U;
    __DMB();
    FocTraceState = FOC_TRACE_RECORDING;
    return 1U;
}

void FocTraceRecord(void)
{
    if (FocTraceState != FOC_TRACE_RECORDING || ++trace_clock < trace_divider)
        return;
    trace_clock = 0U;
    FOC_TRACE_SAMPLE *sample = &FocTraceBuffer[FocTraceCount];
    sample->tick = AdcCurrentSamples;
    sample->adc_u = AdcRawU;
    sample->adc_w = AdcRawW;
    sample->iu = AdcPara.CurU;
    sample->iv = AdcPara.CurV;
    sample->iw = AdcPara.CurW;
    sample->id = Park.Ds;
    sample->iq = Park.Qs;
    sample->theta = Park.Theta;
    ++FocTraceCount;
    if (FocTraceCount == FOC_TRACE_POINTS)
    {
        __DMB();
        FocTraceState = FOC_TRACE_READY;
    }
}

void FocTraceService(void)
{
    if (FocTraceRequest != 0U && FocTraceState == FOC_TRACE_IDLE)
    {
        uint32_t divider = FocTraceRequest;
        if (divider > 65535U)
            divider = 65535U;
        if (FocTraceArm((uint16_t)divider) != 0U)
            FocTraceRequest = 0U;
    }
    /* 故障或手动停机时也允许导出已经记录的部分。 */
    if (!Init_Over && FocTraceState == FOC_TRACE_RECORDING)
    {
        __DMB();
        FocTraceState = FOC_TRACE_READY;
    }
    if (uart1_tx_busy() != 0U)
        return;
    if (FocTraceState == FOC_TRACE_READY)
    {
        static const uint8_t header[] = "tick,adc_u,adc_w,iu_A,iv_A,iw_A,id_A,iq_A,theta_rad\r\n";
        if (shell_send_raw(header, sizeof(header) - 1U) == HAL_OK)
        {
            send_index = 0U;
            FocTraceState = FOC_TRACE_SENDING;
        }
        return;
    }
    if (FocTraceState != FOC_TRACE_SENDING)
        return;
    if (send_index == FocTraceCount)
    {
        FocTraceState = FOC_TRACE_IDLE;
        return;
    }
    const FOC_TRACE_SAMPLE *sample = &FocTraceBuffer[send_index];
    if (shell_send("%lu,%u,%u,%.5f,%.5f,%.5f,%.5f,%.5f,%.6f\r\n",
                   (unsigned long)sample->tick, (unsigned)sample->adc_u, (unsigned)sample->adc_w,
                   (double)sample->iu, (double)sample->iv, (double)sample->iw,
                   (double)sample->id, (double)sample->iq, (double)sample->theta) > 0)
        ++send_index;
}
