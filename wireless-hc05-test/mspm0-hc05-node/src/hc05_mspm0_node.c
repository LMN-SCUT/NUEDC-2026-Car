/* LP-MSPM0G3507 HC-05 test node. J14 selects PA9 for BP UART RX.
 * PA8=TX, PA9=RX, PB14=onboard user LED.
 */

#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../../shared/hc05_at.h"
#include "../../shared/hc05_protocol.h"

/* Set to 1 only while the HC-05 KEY/EN pin was high before power-on. */
#ifndef HC05_TEST_MODE_AT
#define HC05_TEST_MODE_AT 0u
#endif
#ifndef HC05_AT_SCRIPT
#define HC05_AT_SCRIPT HC05_AT_CONFIG_MASTER_AUTO
#endif
/* Replace after querying HC-05 B: e.g. "1234,56,ABCDEF". */
#ifndef HC05_PEER_ADDRESS
#define HC05_PEER_ADDRESS "98D3,02,9687D2"
#endif

#define HC05_NODE_ID 1u
#define RX_BUFFER_SIZE 128u
#define PING_PERIOD_MS 500u
#define LINK_TIMEOUT_MS 1500u
#define ACTIVITY_PULSE_MS 80u

static volatile uint8_t g_rx_buffer[RX_BUFFER_SIZE];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static volatile bool g_rx_overflow;
static volatile uint32_t g_millis;
#if !HC05_TEST_MODE_AT
static uint32_t g_last_ping_ms;
static uint32_t g_last_valid_rx_ms;
static uint32_t g_activity_until_ms;
static uint16_t g_next_sequence;
static hc05_parser_t g_parser;
#endif

/* Inspect these globals from CCS Watch. */
volatile uint32_t g_tx_ping_count;
volatile uint32_t g_tx_pong_count;
volatile uint32_t g_rx_valid_count;
volatile uint32_t g_rx_ping_count;
volatile uint32_t g_rx_pong_count;
volatile uint32_t g_rx_bad_count;
volatile uint32_t g_rx_overflow_count;
volatile uint32_t g_link_timeout_count;
volatile uint16_t g_last_rx_sequence;
volatile uint8_t g_last_rx_node;
volatile bool g_link_alive;
hc05_at_session_t g_at_session;

#if !HC05_TEST_MODE_AT
static void uart_send(const uint8_t *data, uint16_t length)
{
    uint16_t index;
    for (index = 0u; index < length; ++index) {
        DL_UART_Main_transmitDataBlocking(UART_HC05_INST, data[index]);
    }
}
#endif

#if HC05_TEST_MODE_AT
static void uart_send_text(const char *text)
{
    while (*text != '\0') {
        DL_UART_Main_transmitDataBlocking(UART_HC05_INST, (uint8_t)*text++);
    }
    DL_UART_Main_transmitDataBlocking(UART_HC05_INST, '\r');
    DL_UART_Main_transmitDataBlocking(UART_HC05_INST, '\n');
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
static void process_transparent_byte(uint8_t byte)
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
    g_last_valid_rx_ms = g_millis;
    g_link_alive = true;
    g_activity_until_ms = g_millis + ACTIVITY_PULSE_MS;

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

static void update_led(uint32_t now_ms)
{
    if (!g_link_alive) {
        /* Slow blink means firmware runs but no valid wireless frame arrived. */
        if ((now_ms / 500u) % 2u == 0u) {
            DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_D1_PIN);
        } else {
            DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_D1_PIN);
        }
    } else if ((int32_t)(g_activity_until_ms - now_ms) > 0) {
        DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_D1_PIN);
    } else {
        DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_D1_PIN);
    }
}
#endif

int main(void)
{
    uint8_t byte;

    SYSCFG_DL_init();
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_D1_PIN);
    (void)DL_SYSTICK_config(CPUCLK_FREQ / 1000u);
#if !HC05_TEST_MODE_AT
    hc05_parser_init(&g_parser);
#endif
#if HC05_TEST_MODE_AT
    hc05_at_init(&g_at_session, HC05_AT_SCRIPT, HC05_PEER_ADDRESS);
#endif
    NVIC_ClearPendingIRQ(UART_HC05_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_HC05_INST_INT_IRQN);

    while (1) {
        uint32_t now_ms = g_millis;

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
            process_transparent_byte(byte);
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
            g_last_ping_ms = now_ms;
        }
        if (g_link_alive &&
            ((uint32_t)(now_ms - g_last_valid_rx_ms) >= LINK_TIMEOUT_MS)) {
            g_link_alive = false;
            g_link_timeout_count++;
        }
        update_led(now_ms);
#endif
        __WFI();
    }
}

void UART_HC05_INST_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART_HC05_INST) == DL_UART_MAIN_IIDX_RX) {
        uint16_t head = g_rx_head;
        uint16_t next = (uint16_t)((head + 1u) % RX_BUFFER_SIZE);
        uint8_t byte = DL_UART_Main_receiveData(UART_HC05_INST);
        if (next == g_rx_tail) {
            g_rx_overflow = true;
        } else {
            g_rx_buffer[head] = byte;
            g_rx_head = next;
        }
    }
}

void SysTick_Handler(void)
{
    g_millis++;
}
