#include "rtklib_port.h"
#include "main.h"      // 提供 HAL_GetTick()

double rtklib_gettime(void) {
    /*
     * Not wall/GPS time — uptime only. RTCM adjweek() falls back to timeget() when
     * rtcm->time is zero; rtklib_app seeds rtcm->time from rover RAWX to fix GPS week.
     */
    return (double)HAL_GetTick() / 1000.0;
}

double rtklib_get_gpst_seconds(void)
{
    /* Keep behavior compatible with existing embedded stub. */
    return rtklib_gettime();
}

uint32_t rtklib_get_tick_ms(void)
{
    return HAL_GetTick();
}

void rtklib_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}
