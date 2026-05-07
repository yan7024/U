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

/* 1 = USART3 RTCM 观测回放进 rover（tools/replay_rtcm_to_mcu.py）；默认 0 = F9P UBX @ USART1 */
#ifndef MCU_REPLAY_RTCM_OBS
#define MCU_REPLAY_RTCM_OBS 0
#endif

/*
 * 0（默认，本工程 Release/Debug 已写死）：仅用 pntpos 做单点解（SPP），串口 $PRTK 输出经纬度。
 * 1：有基站 MSM 时走 rtkpos 差分（需改 .cproject 去掉 MCU_SOLVE_RTK=0 并改为 1 后全量重编）。
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
