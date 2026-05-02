#include "rtklib_port.h"
#include "rtklib_app.h"
#include "usart.h"
#include <stdlib.h>
#include <string.h>

/*
 * USART3 application TX: RX uses HAL_UART_Receive_IT; avoid HAL_UART_Transmit (HAL_BUSY vs IRQ RX).
 * Poll TXE/TC and write TDR (same idea as USART1 queue drain).
 */
static void uart_blocking_send(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len)
{
    USART_TypeDef *ux;
    uint16_t i;

    if (huart == NULL || data == NULL || len == 0u) {
        return;
    }
    ux = huart->Instance;
    for (i = 0u; i < len; i++) {
        while (!__HAL_UART_GET_FLAG(huart, UART_FLAG_TXE)) {
        }
        ux->TDR = data[i];
    }
    while (!__HAL_UART_GET_FLAG(huart, UART_FLAG_TC)) {
    }
}

/*
 * USART1 debug TX must be the only logical writer to TDR (HAL_UART_Transmit on huart1 is forbidden).
 * A small ring queues lines from main context; one drain loop emits bytes so frames never interleave.
 * libc stderr/stdout is discarded via strong _write() so fprintf chains cannot corrupt the wire.
 */
#define UART1_TXQ_SZ 1024u

static uint8_t s_uart1_txq[UART1_TXQ_SZ];
static uint16_t s_uart1_txq_wr;
static uint16_t s_uart1_txq_rd;

static int uart1_txq_push_all(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (data == NULL || len == 0u) {
        return 0;
    }
    for (i = 0u; i < len; i++) {
        uint16_t next = (uint16_t)((s_uart1_txq_wr + 1u) % UART1_TXQ_SZ);
        if (next == s_uart1_txq_rd) {
            return -1;
        }
        s_uart1_txq[s_uart1_txq_wr] = data[i];
        s_uart1_txq_wr = next;
    }
    return 0;
}

static void uart1_txq_drain_blocking(void)
{
    USART_TypeDef *ux = huart1.Instance;

    for (;;) {
        uint8_t b;
        if (s_uart1_txq_rd == s_uart1_txq_wr) {
            break;
        }
        b = s_uart1_txq[s_uart1_txq_rd];
        s_uart1_txq_rd = (uint16_t)((s_uart1_txq_rd + 1u) % UART1_TXQ_SZ);

        while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE)) {
        }
        ux->TDR = b;
    }
    while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)) {
    }
}

void rtklib_port_allocator_reset(void)
{
    /* toolchain heap; nothing to reset */
}

void *rtklib_port_malloc(size_t size)
{
    if (size == 0u) return NULL;
    return malloc(size);
}

void *rtklib_port_calloc(size_t n, size_t size)
{
    if (n == 0u || size == 0u) return NULL;
    return calloc(n, size);
}

void *rtklib_port_realloc(void *ptr, size_t size)
{
    if (size == 0u) return NULL;
    return realloc(ptr, size);
}

void rtklib_port_free(void *ptr)
{
    free(ptr);
}

void *rtklib_malloc(size_t size)
{
    return rtklib_port_malloc(size);
}

void *rtklib_calloc(size_t n, size_t size)
{
    return rtklib_port_calloc(n, size);
}

void *rtklib_realloc(void *ptr, size_t size)
{
    return rtklib_port_realloc(ptr, size);
}

void rtklib_free(void *ptr)
{
    rtklib_port_free(ptr);
}

void rtklib_port_uart3_send(const uint8_t *data, uint16_t len)
{
    uart_blocking_send(&huart3, data, len);
}

void rtklib_port_uart2_send(const uint8_t *data, uint16_t len)
{
    uart_blocking_send(&huart2, data, len);
}

void rtklib_port_debug_send(const uint8_t *data, uint16_t len)
{
#if RTK_DEBUG_PORT_UART1
    rtklib_port_uart1_send(data, len);
#else
    rtklib_port_uart3_send(data, len);
#endif
}

void rtklib_port_uart1_send(const uint8_t *data, uint16_t len)
{
    if (len == 0u || data == NULL) {
        return;
    }
    while (uart1_txq_push_all(data, len) != 0) {
        uart1_txq_drain_blocking();
    }
    uart1_txq_drain_blocking();
}

/*
 * Strong symbol: discard nano/newlib stdout/stderr so no hidden UART traffic via printf/fprintf.
 * (Weak _write lives in syscalls.c from Cube.)
 */
int _write(int file, char *ptr, int len);

int _write(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    if (len < 0) {
        return -1;
    }
    return len;
}
