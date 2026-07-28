#include "hc05_node.h"

#include <stdbool.h>
#include <string.h>

#include "main.h"
#include "../../../shared/hc05_at.h"
#include "../../../shared/hc05_protocol.h"

/* Set to 1 only while the HC-05 KEY/EN pin was high before power-on. */
#ifndef HC05_TEST_MODE_AT
#define HC05_TEST_MODE_AT 0u
#endif
#ifndef HC05_AT_SCRIPT
#define HC05_AT_SCRIPT HC05_AT_QUERY
#endif
/* Replace after querying HC-05 A: e.g. "1234,56,ABCDEF". */
#ifndef HC05_PEER_ADDRESS
#define HC05_PEER_ADDRESS "0000,00,000000"
#endif

#define HC05_NODE_ID 2u
#define RX_BUFFER_SIZE 128u
#define PING_PERIOD_MS 500u
#define LINK_TIMEOUT_MS 1500u
#define TX_ACTIVITY_PULSE_MS 60u
#define RX_ACTIVITY_PULSE_MS 250u

static UART_HandleTypeDef *g_uart;
static volatile uint8_t g_rx_isr_byte;
static volatile uint8_t g_rx_buffer[RX_BUFFER_SIZE];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static volatile bool g_rx_overflow;
#if !HC05_TEST_MODE_AT
static uint32_t g_last_ping_ms;
static uint32_t g_last_valid_rx_ms;
static uint32_t g_tx_activity_until_ms;
static uint32_t g_rx_activity_until_ms;
static uint32_t g_seen_rx_byte_count;
static uint16_t g_next_sequence;
static hc05_parser_t g_parser;
#endif

/* Inspect these globals from Keil Watch. */
volatile uint32_t g_tx_ping_count;
volatile uint32_t g_tx_pong_count;
volatile uint32_t g_rx_valid_count;
volatile uint32_t g_rx_ping_count;
volatile uint32_t g_rx_pong_count;
volatile uint32_t g_rx_bad_count;
volatile uint32_t g_rx_overflow_count;
volatile uint32_t g_link_timeout_count;
volatile uint32_t g_rx_byte_count;
volatile uint16_t g_last_rx_sequence;
volatile uint8_t g_last_rx_node;
volatile bool g_link_alive;
hc05_at_session_t g_at_session;

#if !HC05_TEST_MODE_AT
static void uart_send(const uint8_t *data, uint16_t length)
{
    (void)HAL_UART_Transmit(g_uart, (uint8_t *)data, length, 50u);
}
#endif

#if HC05_TEST_MODE_AT
static void uart_send_text(const char *text)
{
    (void)HAL_UART_Transmit(g_uart, (uint8_t *)text, (uint16_t)strlen(text), 50u);
    (void)HAL_UART_Transmit(g_uart, (uint8_t *)"\r\n", 2u, 50u);
}
#endif

#if !HC05_TEST_MODE_AT
static void send_frame(uint8_t type, uint16_t sequence)
{
    uint8_t raw[HC05_FRAME_SIZE];
    hc05_frame_t frame;

    frame.type = type;
    frame.node_id = HC05_NODE_ID;
    frame.sequence = sequence;
    (void)hc05_frame_pack(&frame, raw);
    uart_send(raw, HC05_FRAME_SIZE);
}
#endif

static bool rx_pop(uint8_t *byte)
{
    uint16_t tail = g_rx_tail;
    if (tail == g_rx_head) {
        return false;
    }
    *byte = g_rx_buffer[tail];
    g_rx_tail = (uint16_t)((tail + 1u) % RX_BUFFER_SIZE);
    return true;
}

#if !HC05_TEST_MODE_AT
static void set_leds(uint32_t now_ms)
{
    /* PB0/PB1 are active-low user LEDs on the OpenCTR H60 F407 board. */
    /*
     * PB0: slow heartbeat while no valid frame has arrived; solid on when the
     * protocol link is alive.  This also proves that the flashed firmware is
     * actually running even when UART RX is completely silent.
     */
    HAL_GPIO_WritePin(
        HC05_LINK_LED_GPIO_Port,
        HC05_LINK_LED_Pin,
        (g_link_alive || (((now_ms / 500u) % 2u) == 0u))
            ? GPIO_PIN_RESET : GPIO_PIN_SET);

    /*
     * PB1: a short flash for every local PING transmission.  Receiving any
     * UART byte holds it on longer, so physical RX activity remains visible
     * even if the bytes fail frame/CRC validation.
     */
    HAL_GPIO_WritePin(HC05_ACTIVITY_LED_GPIO_Port, HC05_ACTIVITY_LED_Pin,
                      (((int32_t)(g_rx_activity_until_ms - now_ms) > 0) ||
                       ((int32_t)(g_tx_activity_until_ms - now_ms) > 0))
                      ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void process_transparent_byte(uint8_t byte, uint32_t now_ms)
{
    hc05_frame_t frame;
    hc05_parse_result_t result = hc05_parser_push(&g_parser, byte, &frame);

    if (result == HC05_PARSE_BAD) {
        g_rx_bad_count++;
        return;
    }
    if (result != HC05_PARSE_FRAME) {
        return;
    }

    g_rx_valid_count++;
    g_last_rx_sequence = frame.sequence;
    g_last_rx_node = frame.node_id;
    g_last_valid_rx_ms = now_ms;
    g_link_alive = true;
    if (frame.type == HC05_FRAME_PING) {
        g_rx_ping_count++;
        send_frame(HC05_FRAME_PONG, frame.sequence);
        g_tx_pong_count++;
    } else if (frame.type == HC05_FRAME_PONG) {
        g_rx_pong_count++;
    } else {
        g_rx_bad_count++;
    }
}
#endif

void hc05_node_init(UART_HandleTypeDef *uart)
{
    g_uart = uart;
#if !HC05_TEST_MODE_AT
    hc05_parser_init(&g_parser);
#endif
#if HC05_TEST_MODE_AT
    hc05_at_init(&g_at_session, HC05_AT_SCRIPT, HC05_PEER_ADDRESS);
#endif
    (void)HAL_UART_Receive_IT(g_uart, (uint8_t *)&g_rx_isr_byte, 1u);
}

void hc05_node_process(uint32_t now_ms)
{
    uint8_t byte;

#if !HC05_TEST_MODE_AT
    if (g_seen_rx_byte_count != g_rx_byte_count) {
        g_seen_rx_byte_count = g_rx_byte_count;
        g_rx_activity_until_ms = now_ms + RX_ACTIVITY_PULSE_MS;
    }
#endif

    if (g_rx_overflow) {
        g_rx_overflow = false;
        g_rx_tail = g_rx_head;
#if !HC05_TEST_MODE_AT
        hc05_parser_init(&g_parser);
#endif
        g_rx_overflow_count++;
    }
    while (rx_pop(&byte)) {
#if HC05_TEST_MODE_AT
        hc05_at_rx_byte(&g_at_session, byte);
#else
        process_transparent_byte(byte, now_ms);
#endif
    }
#if HC05_TEST_MODE_AT
    {
        const char *command;
        if (hc05_at_poll(&g_at_session, now_ms, &command)) {
            uart_send_text(command);
        }
    }
#else
    if ((uint32_t)(now_ms - g_last_ping_ms) >= PING_PERIOD_MS) {
        send_frame(HC05_FRAME_PING, g_next_sequence++);
        g_tx_ping_count++;
        g_tx_activity_until_ms = now_ms + TX_ACTIVITY_PULSE_MS;
        g_last_ping_ms = now_ms;
    }
    if (g_link_alive &&
        ((uint32_t)(now_ms - g_last_valid_rx_ms) >= LINK_TIMEOUT_MS)) {
        g_link_alive = false;
        g_link_timeout_count++;
    }
    set_leds(now_ms);
#endif
}

void hc05_node_rx_complete_isr(UART_HandleTypeDef *uart)
{
    uint16_t head;
    uint16_t next;

    if (uart != g_uart) {
        return;
    }
    head = g_rx_head;
    next = (uint16_t)((head + 1u) % RX_BUFFER_SIZE);
    if (next == g_rx_tail) {
        g_rx_overflow = true;
    } else {
        g_rx_buffer[head] = g_rx_isr_byte;
        g_rx_head = next;
    }
    g_rx_byte_count++;
    (void)HAL_UART_Receive_IT(g_uart, (uint8_t *)&g_rx_isr_byte, 1u);
}

void hc05_node_error_isr(UART_HandleTypeDef *uart)
{
    if (uart == g_uart) {
        (void)HAL_UART_Receive_IT(g_uart, (uint8_t *)&g_rx_isr_byte, 1u);
    }
}
