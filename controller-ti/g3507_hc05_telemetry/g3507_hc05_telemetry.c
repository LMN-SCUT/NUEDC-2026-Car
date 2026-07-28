#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#define ENCODER_COUNT 4U
#define TELEMETRY_PERIOD_MS 100U
#define TELEMETRY_BUFFER_SIZE 256U

enum {
    ENCODER_LF = 0,
    ENCODER_LR,
    ENCODER_RF,
    ENCODER_RR
};

static const int8_t g_quadrature_table[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
};

volatile uint32_t g_millis;
volatile int32_t g_encoder_total[ENCODER_COUNT];
volatile int32_t g_encoder_period[ENCODER_COUNT];
volatile uint32_t g_encoder_invalid[ENCODER_COUNT];
volatile uint8_t g_encoder_state[ENCODER_COUNT];

/* These counters are useful in CCS Watch while the serial terminals run. */
volatile uint32_t g_telemetry_sequence;
volatile uint32_t g_hc05_lines_sent;
volatile uint32_t g_debug_lines_sent;
volatile uint32_t g_last_checksum;

static uint8_t read_encoder_state(uint32_t index)
{
    uint32_t pins;
    uint8_t state;

    switch (index) {
        case ENCODER_LF:
            pins = DL_GPIO_readPins(GPIO_ENCODER_B_PORT,
                GPIO_ENCODER_B_LF_A_PIN | GPIO_ENCODER_B_LF_B_PIN);
            state = ((pins & GPIO_ENCODER_B_LF_A_PIN) ? 2U : 0U) |
                    ((pins & GPIO_ENCODER_B_LF_B_PIN) ? 1U : 0U);
            break;
        case ENCODER_LR:
            pins = DL_GPIO_readPins(GPIO_ENCODER_B_PORT,
                GPIO_ENCODER_B_LR_A_PIN | GPIO_ENCODER_B_LR_B_PIN);
            state = ((pins & GPIO_ENCODER_B_LR_A_PIN) ? 2U : 0U) |
                    ((pins & GPIO_ENCODER_B_LR_B_PIN) ? 1U : 0U);
            break;
        case ENCODER_RF:
            pins = DL_GPIO_readPins(GPIO_ENCODER_A_PORT,
                GPIO_ENCODER_A_RF_A_PIN | GPIO_ENCODER_A_RF_B_PIN);
            state = ((pins & GPIO_ENCODER_A_RF_A_PIN) ? 2U : 0U) |
                    ((pins & GPIO_ENCODER_A_RF_B_PIN) ? 1U : 0U);
            break;
        default:
            pins = DL_GPIO_readPins(GPIO_ENCODER_A_PORT,
                GPIO_ENCODER_A_RR_A_PIN | GPIO_ENCODER_A_RR_B_PIN);
            state = ((pins & GPIO_ENCODER_A_RR_A_PIN) ? 2U : 0U) |
                    ((pins & GPIO_ENCODER_A_RR_B_PIN) ? 1U : 0U);
            break;
    }

    return state;
}

static void update_encoder(uint32_t index)
{
    uint8_t previous = g_encoder_state[index];
    uint8_t current = read_encoder_state(index);
    int8_t delta = g_quadrature_table[(previous << 2U) | current];

    if ((previous ^ current) == 3U) {
        g_encoder_invalid[index]++;
    }
    g_encoder_state[index] = current;
    g_encoder_total[index] += delta;
    g_encoder_period[index] += delta;
}

static void sample_all_encoders(void)
{
    uint32_t index;
    for (index = 0U; index < ENCODER_COUNT; index++) {
        update_encoder(index);
    }
}

static bool append_char(char *buffer, uint32_t *length, char value)
{
    if (*length >= (TELEMETRY_BUFFER_SIZE - 1U)) {
        return false;
    }
    buffer[(*length)++] = value;
    return true;
}

static bool append_text(char *buffer, uint32_t *length, const char *text)
{
    while (*text != '\0') {
        if (!append_char(buffer, length, *text++)) {
            return false;
        }
    }
    return true;
}

static bool append_u32(char *buffer, uint32_t *length, uint32_t value)
{
    char digits[10];
    uint32_t count = 0U;

    if (value == 0U) {
        return append_char(buffer, length, '0');
    }
    while (value != 0U) {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    while (count != 0U) {
        if (!append_char(buffer, length, digits[--count])) {
            return false;
        }
    }
    return true;
}

static bool append_i32(char *buffer, uint32_t *length, int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        if (!append_char(buffer, length, '-')) {
            return false;
        }
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }
    return append_u32(buffer, length, magnitude);
}

static char hex_digit(uint8_t value)
{
    value &= 0x0FU;
    return (value < 10U) ? (char)('0' + value) : (char)('A' + value - 10U);
}

static bool append_encoder(char *buffer, uint32_t *length, const char *name,
    int32_t period, int32_t total, uint32_t invalid)
{
    return append_char(buffer, length, ',') &&
        append_text(buffer, length, name) &&
        append_char(buffer, length, '=') &&
        append_i32(buffer, length, period) &&
        append_char(buffer, length, '/') &&
        append_i32(buffer, length, total) &&
        append_char(buffer, length, '/') &&
        append_u32(buffer, length, invalid);
}

static uint32_t build_telemetry_line(char *buffer, uint32_t sequence,
    uint32_t now_ms, const int32_t period[ENCODER_COUNT],
    const int32_t total[ENCODER_COUNT],
    const uint32_t invalid[ENCODER_COUNT])
{
    uint32_t length = 0U;
    uint32_t index;
    uint8_t checksum = 0U;

    if (!append_text(buffer, &length, "$TEL,") ||
        !append_u32(buffer, &length, sequence) ||
        !append_text(buffer, &length, ",ms=") ||
        !append_u32(buffer, &length, now_ms) ||
        !append_encoder(buffer, &length, "LF", period[ENCODER_LF],
            total[ENCODER_LF], invalid[ENCODER_LF]) ||
        !append_encoder(buffer, &length, "LR", period[ENCODER_LR],
            total[ENCODER_LR], invalid[ENCODER_LR]) ||
        !append_encoder(buffer, &length, "RF", period[ENCODER_RF],
            total[ENCODER_RF], invalid[ENCODER_RF]) ||
        !append_encoder(buffer, &length, "RR", period[ENCODER_RR],
            total[ENCODER_RR], invalid[ENCODER_RR])) {
        return 0U;
    }

    /* NMEA-style XOR: bytes after '$' and before '*'. */
    for (index = 1U; index < length; index++) {
        checksum ^= (uint8_t)buffer[index];
    }
    g_last_checksum = checksum;

    if (!append_char(buffer, &length, '*') ||
        !append_char(buffer, &length, hex_digit(checksum >> 4U)) ||
        !append_char(buffer, &length, hex_digit(checksum)) ||
        !append_char(buffer, &length, '\r') ||
        !append_char(buffer, &length, '\n')) {
        return 0U;
    }
    buffer[length] = '\0';
    return length;
}

static void uart_send(UART_Regs *uart, const char *data, uint32_t length)
{
    uint32_t index;
    for (index = 0U; index < length; index++) {
        DL_UART_Main_transmitDataBlocking(uart, (uint8_t)data[index]);
    }
}

int main(void)
{
    int32_t period[ENCODER_COUNT];
    int32_t total[ENCODER_COUNT];
    uint32_t invalid[ENCODER_COUNT];
    char line[TELEMETRY_BUFFER_SIZE];
    uint32_t previous_report_ms = 0U;
    uint32_t index;

    SYSCFG_DL_init();
    (void)DL_SYSTICK_config(CPUCLK_FREQ / 1000U);

    for (index = 0U; index < ENCODER_COUNT; index++) {
        g_encoder_state[index] = read_encoder_state(index);
    }
    NVIC_ClearPendingIRQ(GPIO_ENCODER_A_INT_IRQN);
    NVIC_ClearPendingIRQ(GPIO_ENCODER_B_INT_IRQN);
    NVIC_EnableIRQ(GPIO_ENCODER_A_INT_IRQN);
    NVIC_EnableIRQ(GPIO_ENCODER_B_INT_IRQN);

    while (1) {
        uint32_t now = g_millis;

        if ((uint32_t)(now - previous_report_ms) >= TELEMETRY_PERIOD_MS) {
            uint32_t length;

            __disable_irq();
            for (index = 0U; index < ENCODER_COUNT; index++) {
                period[index] = g_encoder_period[index];
                total[index] = g_encoder_total[index];
                invalid[index] = g_encoder_invalid[index];
                g_encoder_period[index] = 0;
            }
            __enable_irq();

            length = build_telemetry_line(line, g_telemetry_sequence++, now,
                period, total, invalid);
            if (length != 0U) {
                uart_send(UART_HC05_INST, line, length);
                g_hc05_lines_sent++;
                uart_send(UART_DEBUG_INST, line, length);
                g_debug_lines_sent++;
            }
            previous_report_ms = now;
        }
        __WFI();
    }
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case GPIO_ENCODER_A_INT_IIDX:
        case GPIO_ENCODER_B_INT_IIDX:
            sample_all_encoders();
            break;
        default:
            break;
    }
}

void SysTick_Handler(void)
{
    g_millis++;
}
