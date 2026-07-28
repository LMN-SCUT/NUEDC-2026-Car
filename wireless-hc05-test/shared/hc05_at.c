#include "hc05_at.h"

#include <string.h>

#define HC05_AT_COMMAND_INTERVAL_MS 300u
#define HC05_AT_RESPONSE_TIMEOUT_MS 1000u
#define HC05_AT_PAIR_TIMEOUT_MS 25000u

static const char *const g_query_commands[] = {
    "AT", "AT+VERSION?", "AT+ADDR?", "AT+UART?", "AT+ROLE?"
};
static const char *const g_slave_commands[] = {
    "AT", "AT+ROLE=0", "AT+PSWD=1234", "AT+UART=115200,0,0"
};
static const char *const g_master_commands[] = {
    "AT", "AT+ROLE=1", "AT+CMODE=0", "AT+BIND=", "AT+UART=115200,0,0"
};
static const char *const g_master_verify_commands[] = {
    "AT", "AT+ROLE?", "AT+CMODE?", "AT+BIND?", "AT+UART?"
};
static const char *const g_master_auto_commands[] = {
    "AT", "AT+ROLE=1", "AT+CMODE=1", "AT+PSWD=1234",
    "AT+UART=115200,0,0"
};
static const char *const g_master_pair_commands[] = {
    "AT",
    "AT+RMAAD",
    "AT+ROLE=1",
    "AT+IAC=9E8B33",
    "AT+CLASS=0",
    "AT+PSWD=1234",
    "AT+INIT",
    "AT+INQM=0,9,5",
    "AT+INQ",
    "AT+PAIR=",
    "AT+LINK="
};

static char normalized_address_char(char value)
{
    if (value == ':') {
        return ',';
    }
    if ((value >= 'a') && (value <= 'f')) {
        return (char)(value - ('a' - 'A'));
    }
    return value;
}

static bool inquiry_result_is_peer(const hc05_at_session_t *session,
                                   const char *line)
{
    const char *peer;
    const char *address;

    if ((session->peer_address == 0) || (strncmp(line, "+INQ:", 5u) != 0)) {
        return false;
    }
    peer = session->peer_address;
    address = line + 5u;
    while (*peer != '\0') {
        if ((*address == '\0') ||
            (normalized_address_char(*peer) !=
             normalized_address_char(*address))) {
            return false;
        }
        peer++;
        address++;
    }
    return true;
}

static uint8_t command_count(hc05_at_script_t script)
{
    if (script == HC05_AT_CONFIG_SLAVE) {
        return (uint8_t)(sizeof(g_slave_commands) / sizeof(g_slave_commands[0]));
    }
    if (script == HC05_AT_CONFIG_MASTER) {
        return (uint8_t)(sizeof(g_master_commands) / sizeof(g_master_commands[0]));
    }
    if (script == HC05_AT_VERIFY_MASTER) {
        return (uint8_t)(sizeof(g_master_verify_commands) /
                         sizeof(g_master_verify_commands[0]));
    }
    if (script == HC05_AT_PAIR_MASTER) {
        return (uint8_t)(sizeof(g_master_pair_commands) /
                         sizeof(g_master_pair_commands[0]));
    }
    if (script == HC05_AT_CONFIG_MASTER_AUTO) {
        return (uint8_t)(sizeof(g_master_auto_commands) /
                         sizeof(g_master_auto_commands[0]));
    }
    return (uint8_t)(sizeof(g_query_commands) / sizeof(g_query_commands[0]));
}

static const char *command_for_step(hc05_at_session_t *session)
{
    const char *command;

    if (session->script == HC05_AT_CONFIG_SLAVE) {
        command = g_slave_commands[session->step];
    } else if (session->script == HC05_AT_CONFIG_MASTER) {
        command = g_master_commands[session->step];
    } else if (session->script == HC05_AT_VERIFY_MASTER) {
        command = g_master_verify_commands[session->step];
    } else if (session->script == HC05_AT_PAIR_MASTER) {
        command = g_master_pair_commands[session->step];
    } else if (session->script == HC05_AT_CONFIG_MASTER_AUTO) {
        command = g_master_auto_commands[session->step];
    } else {
        command = g_query_commands[session->step];
    }

    (void)strncpy(session->current_command, command,
                  sizeof(session->current_command) - 1u);
    session->current_command[sizeof(session->current_command) - 1u] = '\0';
    if ((session->script == HC05_AT_CONFIG_MASTER) && (session->step == 3u)) {
        if ((session->peer_address != 0) && (strlen(session->peer_address) <= 28u)) {
            (void)strncat(session->current_command, session->peer_address,
                          sizeof(session->current_command) - strlen(session->current_command) - 1u);
        }
    }
    if ((session->script == HC05_AT_PAIR_MASTER) &&
        ((session->step == 9u) || (session->step == 10u))) {
        if ((session->peer_address != 0) && (strlen(session->peer_address) <= 28u)) {
            (void)strncat(session->current_command, session->peer_address,
                          sizeof(session->current_command) -
                          strlen(session->current_command) - 1u);
            if (session->step == 9u) {
                (void)strncat(session->current_command, ",20",
                              sizeof(session->current_command) -
                              strlen(session->current_command) - 1u);
            }
        }
    }
    return session->current_command;
}

void hc05_at_init(hc05_at_session_t *session, hc05_at_script_t script,
                  const char *peer_address)
{
    (void)memset(session, 0, sizeof(*session));
    session->script = script;
    session->peer_address = peer_address;
}

bool hc05_at_poll(hc05_at_session_t *session, uint32_t now_ms,
                  const char **command)
{
    if ((session->complete) || (command == 0)) {
        return false;
    }

    if (session->waiting) {
        if (session->response_complete) {
            if (session->step < HC05_AT_MAX_STEPS) {
                (void)strncpy(session->responses[session->step],
                              session->last_response,
                              HC05_AT_RESPONSE_SIZE - 1u);
                session->responses[session->step][HC05_AT_RESPONSE_SIZE - 1u] = '\0';
            }
            if ((strncmp(session->last_response, "ERROR", 5u) == 0) ||
                (strncmp(session->last_response, "FAIL", 4u) == 0)) {
                session->error_count++;
            } else {
                session->success_count++;
            }
            session->step++;
            session->waiting = false;
            session->next_ms = now_ms + HC05_AT_COMMAND_INTERVAL_MS;
        } else if ((uint32_t)(now_ms - session->sent_ms) >=
                   ((session->script == HC05_AT_PAIR_MASTER)
                        ? HC05_AT_PAIR_TIMEOUT_MS
                        : HC05_AT_RESPONSE_TIMEOUT_MS)) {
            session->timeout_count++;
            session->step++;
            session->waiting = false;
            session->next_ms = now_ms + HC05_AT_COMMAND_INTERVAL_MS;
        } else {
            return false;
        }
    }
    if (session->step >= command_count(session->script)) {
        session->complete = true;
        return false;
    }
    if ((int32_t)(now_ms - session->next_ms) < 0) {
        return false;
    }

    session->response_length = 0u;
    session->last_response[0] = '\0';
    session->response_complete = false;
    session->response_has_content = false;
    session->inquiry_found = false;
    session->inquiry_result[0] = '\0';
    *command = command_for_step(session);
    session->sent_ms = now_ms;
    session->waiting = true;
    return true;
}

void hc05_at_rx_byte(hc05_at_session_t *session, uint8_t byte)
{
    size_t line_length;

    if ((!session->waiting) || (session->response_complete)) {
        return;
    }
    session->rx_byte_count++;

    if ((byte == '\n') || (byte == '\r')) {
        if (!session->response_has_content) {
            return;
        }

        session->last_response[session->response_length] = '\0';
        line_length = strlen(session->last_response);

        /*
         * Some HC-05/serial paths echo the transmitted AT command before the
         * module's real reply.  Do not count that echo as a successful reply;
         * clear the line and continue waiting for OK/+VERSION/+ADDR/etc.
         */
        if ((line_length == strlen(session->current_command)) &&
            (strcmp(session->last_response, session->current_command) == 0)) {
            session->echo_count++;
            session->response_length = 0u;
            session->last_response[0] = '\0';
            session->response_has_content = false;
            return;
        }

        if (strcmp(session->current_command, "AT+INQ") == 0) {
            if (strncmp(session->last_response, "+INQ:", 5u) == 0) {
                if (inquiry_result_is_peer(session, session->last_response)) {
                    (void)strncpy(session->inquiry_result,
                                  session->last_response,
                                  sizeof(session->inquiry_result) - 1u);
                    session->inquiry_result[sizeof(session->inquiry_result) - 1u] = '\0';
                    session->inquiry_found = true;
                }
                session->response_length = 0u;
                session->last_response[0] = '\0';
                session->response_has_content = false;
                return;
            }
            if ((strcmp(session->last_response, "OK") == 0) &&
                session->inquiry_found) {
                (void)strncpy(session->last_response,
                              session->inquiry_result,
                              sizeof(session->last_response) - 1u);
                session->last_response[sizeof(session->last_response) - 1u] = '\0';
                session->response_length =
                    (uint16_t)strlen(session->last_response);
            }
        }

        session->response_complete = true;
        return;
    }

    if (session->response_length < (sizeof(session->last_response) - 1u)) {
        session->last_response[session->response_length++] = (char)byte;
        session->last_response[session->response_length] = '\0';
    }
    session->response_has_content = true;
}
