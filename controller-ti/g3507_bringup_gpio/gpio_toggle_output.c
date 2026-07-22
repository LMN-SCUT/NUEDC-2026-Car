/* MSP-LITO-G3507 <-> K230 protocol-v1 smoke test. */

#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../../shared/protocol/c/vision_link.h"

#define UART_RX_BUFFER_SIZE 256U
#define COMMAND_SET_MODE    0x01U
#define COMMAND_MODE        0x01U
#define ACK_STATUS_OK       0x00U
#define COMMAND_MAX_SENDS   3U
#define PARSER_TIMEOUT_MS   20U
#define OBSERVATION_STALE_MS 150U
#define LINK_LOSS_MS        500U

static volatile uint8_t g_uartRxBuffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t g_uartRxHead = 0U;
static volatile uint16_t g_uartRxTail = 0U;
static volatile bool g_uartRxOverflow = false;

/* Kept global so these values can be inspected easily in the CCS debugger. */
volatile uint32_t g_validFrameCount = 0U;
volatile uint32_t g_observationCount = 0U;
volatile uint32_t g_heartbeatCount = 0U;
volatile uint32_t g_badFrameCount = 0U;
volatile uint32_t g_uartRxByteCount = 0U;
volatile uint32_t g_uartRxOverflowCount = 0U;
volatile uint32_t g_parserTimeoutCount = 0U;
volatile bool g_ackReceived = false;
volatile bool g_modeConfirmed = false;
volatile bool g_observationFresh = false;
volatile bool g_linkAlive = false;
volatile vl_observation_t g_latestObservation;
volatile vl_heartbeat_t g_latestHeartbeat;
volatile uint8_t g_commandSendCount = 0U;

static uint8_t g_commandSeq = 0U;
static volatile uint32_t g_millis = 0U;
static uint32_t g_lastByteMs = 0U;
static uint32_t g_lastValidFrameMs = 0U;
static uint32_t g_lastObservationMs = 0U;

static bool uart_rx_pop(uint8_t *byte)
{
    uint16_t tail;

    if (byte == NULL) {
        return false;
    }

    tail = g_uartRxTail;
    if (tail == g_uartRxHead) {
        return false;
    }

    *byte = g_uartRxBuffer[tail];
    g_uartRxTail = (uint16_t)((tail + 1U) % UART_RX_BUFFER_SIZE);
    return true;
}

static void uart_send_bytes(const uint8_t *data, size_t length)
{
    size_t index;

    for (index = 0U; index < length; ++index) {
        DL_UART_Main_transmitDataBlocking(UART_VISION_INST, data[index]);
    }
}

static void send_set_mode_command(void)
{
    uint8_t frame[VL_MAX_FRAME_SIZE];
    size_t frameLength;
    const vl_command_t command = {
        .command_id = COMMAND_SET_MODE,
        .mode = COMMAND_MODE,
        .arg0 = 0,
        .arg1 = 0,
        .arg2 = 0,
    };

    frameLength = vl_pack_command(
        g_commandSeq, &command, frame, sizeof(frame));
    if (frameLength != 0U) {
        uart_send_bytes(frame, frameLength);
        g_commandSendCount++;
    }
}

static void process_protocol_frame(const vl_frame_t *frame)
{
    vl_ack_t ack;
    vl_heartbeat_t heartbeat;
    vl_observation_t observation;

    g_validFrameCount++;
    g_lastValidFrameMs = g_millis;
    g_linkAlive = true;

    if (frame->type == VL_TYPE_VISION_OBSERVATION) {
        if (vl_decode_observation(frame, &observation) != 0) {
            g_latestObservation = observation;
            g_observationCount++;
            g_lastObservationMs = g_millis;
            g_observationFresh =
                ((observation.flags & VL_FLAG_TARGET_VALID) != 0U);
            if (observation.mode == COMMAND_MODE) {
                g_modeConfirmed = true;
            }
        }
    } else if (frame->type == VL_TYPE_HEARTBEAT) {
        if (vl_decode_heartbeat(frame, &heartbeat) != 0) {
            g_latestHeartbeat = heartbeat;
            g_heartbeatCount++;

            if (g_ackReceived && g_modeConfirmed) {
                /* Full duplex protocol PASS: blink D1 at the heartbeat rate. */
                DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_USER_LED_PIN);
            } else {
                DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_USER_LED_PIN);
            }

            if ((!g_ackReceived) &&
                (g_commandSendCount < COMMAND_MAX_SENDS)) {
                send_set_mode_command();
            }
        }
    } else if (frame->type == VL_TYPE_ACK) {
        if ((vl_decode_ack(frame, &ack) != 0) &&
            (ack.request_type == VL_TYPE_COMMAND) &&
            (ack.request_seq == g_commandSeq) &&
            (ack.status == ACK_STATUS_OK)) {
            g_ackReceived = true;
        }
    }
}

int main(void)
{
    uint8_t byte;
    vl_frame_t frame;
    vl_parser_t parser;

    SYSCFG_DL_init();
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_USER_LED_PIN);

    vl_parser_init(&parser);
    (void) DL_SYSTICK_config(CPUCLK_FREQ / 1000U);
    NVIC_ClearPendingIRQ(UART_VISION_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_VISION_INST_INT_IRQN);

    while (1) {
        uint32_t now = g_millis;

        if (g_uartRxOverflow) {
            g_uartRxOverflow = false;
            g_uartRxTail = g_uartRxHead;
            vl_parser_timeout(&parser);
            g_badFrameCount++;
            g_uartRxOverflowCount++;
        }

        while (uart_rx_pop(&byte)) {
            vl_parse_result_t result =
                vl_parser_push_byte(&parser, byte, &frame);

            g_lastByteMs = now;

            if (result == VL_PARSE_FRAME) {
                process_protocol_frame(&frame);
            } else if (result < VL_PARSE_NONE) {
                g_badFrameCount++;
            }
        }

        if ((parser.state != VL_WAIT_SOF1) &&
            ((uint32_t) (now - g_lastByteMs) > PARSER_TIMEOUT_MS)) {
            vl_parser_timeout(&parser);
            g_badFrameCount++;
            g_parserTimeoutCount++;
        }

        if (g_observationFresh &&
            ((uint32_t) (now - g_lastObservationMs) >
             OBSERVATION_STALE_MS)) {
            g_observationFresh = false;
        }

        if (g_linkAlive &&
            ((uint32_t) (now - g_lastValidFrameMs) > LINK_LOSS_MS)) {
            g_linkAlive = false;
            g_observationFresh = false;
            g_ackReceived = false;
            g_modeConfirmed = false;
            g_commandSendCount = 0U;
            DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_USER_LED_PIN);
        }

        __WFI();
    }
}

void UART_VISION_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_VISION_INST)) {
        case DL_UART_MAIN_IIDX_RX: {
            uint16_t head = g_uartRxHead;
            uint16_t next =
                (uint16_t)((head + 1U) % UART_RX_BUFFER_SIZE);
            uint8_t byte = DL_UART_Main_receiveData(UART_VISION_INST);

            g_uartRxByteCount++;

            if (next == g_uartRxTail) {
                g_uartRxOverflow = true;
            } else {
                g_uartRxBuffer[head] = byte;
                g_uartRxHead = next;
            }
            break;
        }

        default:
            break;
    }
}

void SysTick_Handler(void)
{
    g_millis++;
}
