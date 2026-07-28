#ifndef HC05_NODE_H
#define HC05_NODE_H

#include "stm32f4xx_hal.h"

void hc05_node_init(UART_HandleTypeDef *uart);
void hc05_node_process(uint32_t now_ms);
void hc05_node_rx_complete_isr(UART_HandleTypeDef *uart);
void hc05_node_error_isr(UART_HandleTypeDef *uart);

#endif
