#ifndef VISION_LINK_HAL_H
#define VISION_LINK_HAL_H

#include "stm32f4xx_hal.h"
#include "vision_link.h"

#define VL_HAL_RX_RING_SIZE 256u

typedef struct {
    UART_HandleTypeDef *uart;
    vl_parser_t parser;
    uint8_t irq_rx_byte;
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    uint8_t rx_ring[VL_HAL_RX_RING_SIZE];
    vl_observation_t latest_observation;
    uint8_t observation_ready;
    vl_ack_t latest_ack;
    uint8_t ack_ready;
    uint32_t last_byte_ms;
    uint32_t last_observation_ms;
    uint32_t last_ack_ms;
    uint32_t valid_frame_count;
    uint32_t crc_error_count;
    uint32_t format_error_count;
    volatile uint32_t ring_overflow_count;
} vl_hal_link_t;

void vl_hal_init(vl_hal_link_t *link, UART_HandleTypeDef *uart);
HAL_StatusTypeDef vl_hal_start_rx(vl_hal_link_t *link);
void vl_hal_uart_rx_cplt_isr(vl_hal_link_t *link, UART_HandleTypeDef *uart);
void vl_hal_uart_error_isr(vl_hal_link_t *link, UART_HandleTypeDef *uart);
void vl_hal_process(vl_hal_link_t *link, uint32_t now_ms);
int vl_hal_take_observation(vl_hal_link_t *link, vl_observation_t *out);
int vl_hal_take_ack(vl_hal_link_t *link, vl_ack_t *out);
HAL_StatusTypeDef vl_hal_send_command(vl_hal_link_t *link, uint8_t seq,
                                      const vl_command_t *command);
int vl_hal_observation_is_fresh(const vl_hal_link_t *link, uint32_t now_ms,
                                uint32_t timeout_ms);

#endif
