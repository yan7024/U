#ifndef RTKLIB_APP_H
#define RTKLIB_APP_H

#include <stdint.h>

/*
 * Default route: one F9P UART feeds UBX RAWX + SFRBX + NAV-PVT to USART1.
 * USART3 is debug text by default, not an RTCM dependency.
 *
 * Debug text ($PRTKMON / $HB / $PRTK):
 *  0 = USART3 PC4/PC5, mirrored to USART2 ST-Link VCP.
 *  1 = USART1 PA9 (shares the rover UBX link; only for special probing).
 */
#ifndef RTK_DEBUG_PORT_UART1
#define RTK_DEBUG_PORT_UART1 0
#endif

/* 1 = enable USART3 RTCM/NTRIP input. Default 0 = pure F9P UBX SPP. */
#ifndef MCU_USE_USART3_RTCM
#define MCU_USE_USART3_RTCM 0
#endif

/* 1 = copy USART3 RTCM observations into rover obs for replay tests. */
#ifndef MCU_REPLAY_RTCM_OBS
#define MCU_REPLAY_RTCM_OBS 0
#endif

/*
 * 0 = SPP by pntpos() and publish $PRTK latitude/longitude/height.
 * 1 = RTK by rtkpos(); requires base RTCM and a full rebuild.
 */
#ifndef MCU_SOLVE_RTK
#define MCU_SOLVE_RTK 0
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
