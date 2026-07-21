#include "vision_link_hal.h"

#include <string.h>

void vl_hal_init(vl_hal_link_t *link, UART_HandleTypeDef *uart) {
    if (link == NULL) {
        return;
    }
    memset(link, 0, sizeof(*link));
    link->uart = uart;
    vl_parser_init(&link->parser);
}

HAL_StatusTypeDef vl_hal_start_rx(vl_hal_link_t *link) {
    if (link == NULL || link->uart == NULL) {
        return HAL_ERROR;
    }
    return HAL_UART_Receive_IT(link->uart, &link->irq_rx_byte, 1u);
}

void vl_hal_uart_rx_cplt_isr(vl_hal_link_t *link, UART_HandleTypeDef *uart) {
    uint16_t next;
    if (link == NULL || uart == NULL || uart != link->uart) {
        return;
    }
    next = (uint16_t)((link->rx_head + 1u) % VL_HAL_RX_RING_SIZE);
    if (next == link->rx_tail) {
        link->ring_overflow_count++;
    } else {
        link->rx_ring[link->rx_head] = link->irq_rx_byte;
        link->rx_head = next;
    }
    (void)HAL_UART_Receive_IT(link->uart, &link->irq_rx_byte, 1u);
}

void vl_hal_uart_error_isr(vl_hal_link_t *link, UART_HandleTypeDef *uart) {
    if (link == NULL || uart == NULL || uart != link->uart) {
        return;
    }
    (void)HAL_UART_Receive_IT(link->uart, &link->irq_rx_byte, 1u);
}

void vl_hal_process(vl_hal_link_t *link, uint32_t now_ms) {
    uint8_t byte;
    vl_frame_t frame;
    vl_parse_result_t result;
    if (link == NULL) {
        return;
    }

    if (link->parser.state != VL_WAIT_SOF1
        && (uint32_t)(now_ms - link->last_byte_ms) > 20u) {
        vl_parser_timeout(&link->parser);
        link->format_error_count++;
    }

    while (link->rx_tail != link->rx_head) {
        byte = link->rx_ring[link->rx_tail];
        link->rx_tail = (uint16_t)((link->rx_tail + 1u) % VL_HAL_RX_RING_SIZE);
        link->last_byte_ms = now_ms;
        result = vl_parser_push_byte(&link->parser, byte, &frame);
        if (result == VL_PARSE_FRAME) {
            link->valid_frame_count++;
            if (frame.type == VL_TYPE_VISION_OBSERVATION
                && vl_decode_observation(&frame, &link->latest_observation)) {
                link->last_observation_ms = now_ms;
                link->observation_ready = 1u;
            }
        } else if (result == VL_PARSE_BAD_CRC) {
            link->crc_error_count++;
        } else if (result < VL_PARSE_NONE) {
            link->format_error_count++;
        }
    }
}

int vl_hal_take_observation(vl_hal_link_t *link, vl_observation_t *out) {
    if (link == NULL || out == NULL || link->observation_ready == 0u) {
        return 0;
    }
    *out = link->latest_observation;
    link->observation_ready = 0u;
    return 1;
}

int vl_hal_observation_is_fresh(const vl_hal_link_t *link, uint32_t now_ms,
                                uint32_t timeout_ms) {
    if (link == NULL || link->valid_frame_count == 0u) {
        return 0;
    }
    if ((link->latest_observation.flags & VL_FLAG_TARGET_VALID) == 0u) {
        return 0;
    }
    return (uint32_t)(now_ms - link->last_observation_ms) <= timeout_ms;
}
