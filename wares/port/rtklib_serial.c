#include "rtklib_port.h"
#include "usart.h"

/* Ring buffers in main SRAM (USART RX is IT byte mode; keep off SRAM2 for simplest IRQ path). */
#define RX_BUF_SIZE 2048
volatile uint16_t usart1_rx_head = 0u;
volatile uint16_t usart1_rx_tail = 0u;
uint8_t usart1_rx_buffer[RX_BUF_SIZE];

volatile uint16_t usart2_rx_head = 0u;
volatile uint16_t usart2_rx_tail = 0u;
uint8_t usart2_rx_buffer[RX_BUF_SIZE];

static uint8_t s_read_port = 1u;
static volatile uint32_t s_rx1_total = 0u;
static volatile uint32_t s_rx2_total = 0u;
static volatile uint32_t s_ubx_sync_total = 0u;
static volatile uint32_t s_rtcm_preamble_total = 0u; /* USART3 base stream: 0xD3 */
static volatile uint32_t s_rtcm_preamble_uart1_total = 0u; /* USART1: 0xD3 (diag) */
static uint8_t s_ubx_sync_state = 0u;

void rtklib_serial_reset_usart1_ring(void)
{
    usart1_rx_head = 0u;
    usart1_rx_tail = 0u;
    s_rx1_total = 0u;
    s_ubx_sync_total = 0u;
    s_rtcm_preamble_uart1_total = 0u;
    s_ubx_sync_state = 0u;
}

void rtklib_serial_set_read_port(uint8_t port)
{
    s_read_port = (port == 2u) ? 2u : 1u;
}

void rtklib_serial_feed_usart1_isr(uint8_t byte)
{
    uint16_t next = (uint16_t)((usart1_rx_head + 1u) % RX_BUF_SIZE);
    s_rx1_total++;
    if (byte == 0xD3u) {
        s_rtcm_preamble_uart1_total++;
    }
    if (s_ubx_sync_state == 0u) {
        s_ubx_sync_state = (byte == 0xB5u) ? 1u : 0u;
    } else {
        if (byte == 0x62u) {
            s_ubx_sync_total++;
        }
        s_ubx_sync_state = (byte == 0xB5u) ? 1u : 0u;
    }
    if (next != usart1_rx_tail) {
        usart1_rx_buffer[usart1_rx_head] = byte;
        usart1_rx_head = next;
    }
}

void rtklib_serial_feed_usart3_isr(uint8_t byte)
{
    uint16_t next = (uint16_t)((usart2_rx_head + 1u) % RX_BUF_SIZE);
    s_rx2_total++;
    if (byte == 0xD3u) {
        s_rtcm_preamble_total++;
    }
    if (next != usart2_rx_tail) {
        usart2_rx_buffer[usart2_rx_head] = byte;
        usart2_rx_head = next;
    }
}

uint32_t rtklib_serial_get_rx1_total(void)
{
    return s_rx1_total;
}

uint32_t rtklib_serial_get_rx2_total(void)
{
    return s_rx2_total;
}

uint32_t rtklib_serial_get_ubx_sync_total(void)
{
    return s_ubx_sync_total;
}

uint32_t rtklib_serial_get_rtcm_preamble_total(void)
{
    return s_rtcm_preamble_total;
}

uint32_t rtklib_serial_get_rtcm_preamble_uart1_total(void)
{
    return s_rtcm_preamble_uart1_total;
}

int rtklib_uart_read(uint8_t *buf, int size) {
    int i = 0;
    if (s_read_port == 2u) {
        while (i < size && usart2_rx_head != usart2_rx_tail) {
            buf[i++] = usart2_rx_buffer[usart2_rx_tail++];
            if (usart2_rx_tail >= RX_BUF_SIZE) usart2_rx_tail = 0;
        }
    } else {
        while (i < size && usart1_rx_head != usart1_rx_tail) {
            buf[i++] = usart1_rx_buffer[usart1_rx_tail++];
            if (usart1_rx_tail >= RX_BUF_SIZE) usart1_rx_tail = 0;
        }
    }
    return i;
}
