#ifndef RTKLIB_PORT_H
#define RTKLIB_PORT_H
#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stddef.h>

/*
 * USART1: PA10 RX rover UBX in; PA9 TX UBX CFG to rover (+ debug if RTK_DEBUG_PORT_UART1=1).
 * USART2: PA2/PA3 — ST-Link VCP; RX sunk; debug lines mirrored here when RTK_DEBUG_PORT_UART1=0 (see rtklib_port_debug_send).
 * USART3: PC4 TX / PC5 RX (L476RCT LQFP64) — USB-TTL: $PRTKMON / $HB / $PRTK + RTCM when RTK_DEBUG_PORT_UART1=0.
 */

// 时间接口：返回当前 GPS 秒周内秒（0 ~ 604800）
double rtklib_gettime(void);
double rtklib_get_gpst_seconds(void);
uint32_t rtklib_get_tick_ms(void);
void rtklib_delay_ms(uint32_t ms);

// 串口数据读取（非阻塞，从环形缓冲区取数据）
int rtklib_uart_read(uint8_t *buf, int size);
void rtklib_serial_set_read_port(uint8_t port);
void rtklib_serial_feed_usart1_isr(uint8_t byte);
void rtklib_serial_feed_usart3_isr(uint8_t byte);
void rtklib_serial_reset_usart1_ring(void);
void rtklib_port_uart1_send(const uint8_t *data, uint16_t len);
void rtklib_port_uart2_send(const uint8_t *data, uint16_t len);
void rtklib_port_uart3_send(const uint8_t *data, uint16_t len);
/* Routes to USART1 or USART3 per RTK_DEBUG_PORT_UART1 (see rtklib_app.h). */
void rtklib_port_debug_send(const uint8_t *data, uint16_t len);
void rtklib_port_allocator_reset(void);
uint32_t rtklib_serial_get_rx1_total(void);
uint32_t rtklib_serial_get_rx2_total(void);
uint32_t rtklib_serial_get_ubx_sync_total(void);
uint32_t rtklib_serial_get_rtcm_preamble_total(void);
uint32_t rtklib_serial_get_rtcm_preamble_uart1_total(void);

// 如果 DONT_USE_MALLOC=1，则需要提供静态内存分配接口
void *rtklib_malloc(size_t size);
void *rtklib_calloc(size_t n, size_t size);
void *rtklib_realloc(void *ptr, size_t size);
void rtklib_free(void *ptr);

// 调试打印宏（可空或重定向到 printf）
#define TRACE_PRINTF(...)   // 调试期可改为 printf(__VA_ARGS__)

#endif
