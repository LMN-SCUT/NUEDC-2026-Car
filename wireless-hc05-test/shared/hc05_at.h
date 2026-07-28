#ifndef HC05_AT_H
#define HC05_AT_H

#include <stdbool.h>
#include <stdint.h>

#define HC05_AT_MAX_STEPS 11u
#define HC05_AT_RESPONSE_SIZE 96u

typedef enum {
    HC05_AT_QUERY = 0,
    HC05_AT_CONFIG_SLAVE,
    HC05_AT_CONFIG_MASTER,
    HC05_AT_VERIFY_MASTER,
    HC05_AT_PAIR_MASTER,
    HC05_AT_CONFIG_MASTER_AUTO
} hc05_at_script_t;

typedef struct {
    hc05_at_script_t script;
    const char *peer_address;
    uint8_t step;
    bool waiting;
    bool complete;
    uint32_t sent_ms;
    uint32_t next_ms;
    uint32_t success_count;
    uint32_t error_count;
    uint32_t timeout_count;
    uint32_t rx_byte_count;
    uint32_t echo_count;
    uint16_t response_length;
    bool response_complete;
    bool response_has_content;
    bool inquiry_found;
    char current_command[40];
    char last_response[HC05_AT_RESPONSE_SIZE];
    char inquiry_result[HC05_AT_RESPONSE_SIZE];
    char responses[HC05_AT_MAX_STEPS][HC05_AT_RESPONSE_SIZE];
} hc05_at_session_t;

void hc05_at_init(hc05_at_session_t *session, hc05_at_script_t script,
                  const char *peer_address);
bool hc05_at_poll(hc05_at_session_t *session, uint32_t now_ms,
                  const char **command);
void hc05_at_rx_byte(hc05_at_session_t *session, uint8_t byte);

#endif
