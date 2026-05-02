#ifndef RTKLIB_APP_H
#define RTKLIB_APP_H

#include <stdint.h>

/*
 * Debug text ($PRTKMON / $HB / $PRTK):
 *  0 — USART3 PC4/PC5: CH340 + RTCM/NTRIP + debug text; USART2 ST-Link mirrors same debug (115200).
 *  1 — USART1 PA9 (mixes with rover UBX — rarely useful).
 */
#ifndef RTK_DEBUG_PORT_UART1
#define RTK_DEBUG_PORT_UART1 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

void rtklib_init(void);
void rtklib_process(void);
/*
 * Send UBX-CFG on USART1 at link_baud (MCU peripheral must match receiver).
 * CFG-PRT always programs the rover UART to 115200 8N1 UBX; ends with MCU back on 115200.
 */
void rtklib_rover_ubx_link_train(uint32_t link_baud);
int rtklib_get_base_obs_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* RTKLIB_APP_H */
