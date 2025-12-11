#include "comm.h"
#include "usbd_cdc_if.h"
#include "actio.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define USB_RX_BUFFER_SIZE 128
static char usb_rx_buffer[USB_RX_BUFFER_SIZE];

// ==================== Événements en attente d'ACK ====================

typedef struct {
    char evt[32];
    char info[32];
    uint32_t lastSentTimestamp; // en ms
    uint8_t retryCount;
    uint8_t active;
} PendingEvent;

#define MAX_PENDING_EVENTS 4
#define EVENT_RESEND_INTERVAL 1000 // ms

static PendingEvent pendingEvents[MAX_PENDING_EVENTS];

// ==================== Fonctions utilitaires ====================

void send_event(int idx) {
    char msg[128];
    sprintf(msg, "EVENT;%s;%s\n", pendingEvents[idx].evt, pendingEvents[idx].info);
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}

void queue_event(const char *evt, const char *info) {
    for (int i = 0; i < MAX_PENDING_EVENTS; ++i) {
        if (!pendingEvents[i].active) {
            strncpy(pendingEvents[i].evt, evt, sizeof(pendingEvents[i].evt) - 1);
            pendingEvents[i].evt[sizeof(pendingEvents[i].evt)-1] = '\0';
            strncpy(pendingEvents[i].info, info, sizeof(pendingEvents[i].info) - 1);
            pendingEvents[i].info[sizeof(pendingEvents[i].info)-1] = '\0';
            pendingEvents[i].lastSentTimestamp = HAL_GetTick();
            pendingEvents[i].retryCount = 0;
            pendingEvents[i].active = 1;
            send_event(i);
            break;
        }
    }
}

void check_event_timeouts(void) {
    uint32_t now = HAL_GetTick();
    for (int i = 0; i < MAX_PENDING_EVENTS; ++i) {
        if (pendingEvents[i].active && now - pendingEvents[i].lastSentTimestamp > EVENT_RESEND_INTERVAL) {
            pendingEvents[i].lastSentTimestamp = now;
            pendingEvents[i].retryCount++;
            send_event(i);
        }
    }
}

// ==================== Traitement du protocole ====================

void USBProtocol_ProcessCommand(char *cmd) {
    char response[256];
    char *type = strtok(cmd, ";");
    char *arg1 = strtok(NULL, ";");
    char *arg2 = strtok(NULL, ";");
    char *arg3 = strtok(NULL, ";");

    if (!type || !arg1 || !arg2) return;

    if (strcmp(type, "GET") == 0) {
        if (strcmp(arg1, "SERVO") == 0 && strcmp(arg2, "POS") == 0) {
            if (!arg3) {
                snprintf(response, sizeof(response), "KO;SET;%s;%s\n", arg1, arg2);
                CDC_Transmit_FS((uint8_t*)response, strlen(response));
                return;
            }
            int n = atoi(arg3);
            if (n <= 0) {
                snprintf(response, sizeof(response), "SET;%s;%s;0\n", arg1, arg2);
                CDC_Transmit_FS((uint8_t*)response, strlen(response));
                return;
            }

            size_t off = 0;
            int success = 1;
            off += snprintf(response + off, sizeof(response) - off, "SET;%s;%s;%d", arg1, arg2, n);

            for (int i = 0; i < n; ++i) {
                char *servoIdStr = strtok(NULL, ";");
                if (!servoIdStr) {
                    success = 0;
                    break;
                }
                int servoId = atoi(servoIdStr);
                int pos = getServoPos(servoId);

                /* append \";id;pos\" for each servo, check remaining buffer */
                int wrote = snprintf(response + off, sizeof(response) - off, ";%d;%d", servoId, pos);
                if (wrote < 0 || (size_t)wrote >= sizeof(response) - off) {
                    success = 0;
                    break;
                }
                off += wrote;
            }

            if (!success) {
                snprintf(response, sizeof(response), "KO;SET;%s;%s\n", arg1, arg2);
            } else {
                /* terminate with newline */
                if (off < sizeof(response) - 1) {
                    response[off++] = '\n';
                    response[off] = '\0';
                } else {
                    /* fallback if no room for newline */
                    response[sizeof(response) - 1] = '\0';
                }
            }
            CDC_Transmit_FS((uint8_t*)response, strlen(response));
        } else if (strcmp(arg1, "RELAY") == 0 && strcmp(arg2, "STATE") == 0) {
            if (!arg3) {
                snprintf(response, sizeof(response), "KO;SET;%s;%s\n", arg1, arg2);
                CDC_Transmit_FS((uint8_t*)response, strlen(response));
                return;
            }
            int n = atoi(arg3);
            if (n <= 0) {
                snprintf(response, sizeof(response), "SET;%s;%s;0\n", arg1, arg2);
                CDC_Transmit_FS((uint8_t*)response, strlen(response));
                return;
            }

            size_t off = 0;
            int success = 1;
            off += snprintf(response + off, sizeof(response) - off, "SET;%s;%s;%d", arg1, arg2, n);

            for (int i = 0; i < n; ++i) {
                char *relayIdStr = strtok(NULL, ";");
                if (!relayIdStr) {
                    success = 0;
                    break;
                }
                int relayId = atoi(relayIdStr);
                int state = getRelayState(relayId);

                int wrote = snprintf(response + off, sizeof(response) - off, ";%d;%d", relayId, state);
                if (wrote < 0 || (size_t)wrote >= sizeof(response) - off) {
                    success = 0;
                    break;
                }
                off += wrote;
            }

            if (!success) {
                snprintf(response, sizeof(response), "KO;SET;%s;%s\n", arg1, arg2);
            } else {
                if (off < sizeof(response) - 1) {
                    response[off++] = '\n';
                    response[off] = '\0';
                } else {
                    response[sizeof(response) - 1] = '\0';
                }
            }
            CDC_Transmit_FS((uint8_t*)response, strlen(response));
        }

    } else if (strcmp(type, "SET") == 0 && arg3) {
        int success = 1;
        if (strcmp(arg1, "TIR") == 0 && strcmp(arg2, "ARM") == 0) {
            int val = atoi(arg3);
            if(val){
               armTirette();
            } else {
               disarmTirette();
            }
            sprintf(response, "%s;%s;%s;%d\n", success ? "OK" : "KO", arg1, arg2, val);
            CDC_Transmit_FS((uint8_t*)response, strlen(response));
        } else if (strcmp(arg1, "SERVO") == 0 && strcmp(arg2, "POS") == 0){
            int n = atoi(arg3);
            size_t off = 0;
            off += snprintf(response + off, sizeof(response) - off, "%s;%s;%s;%d", "OK", arg1, arg2, n);

            for (int i = 0; i < n; ++i) {
                char *servoIdStr = strtok(NULL, ";");
                char *posStr = strtok(NULL, ";");
                char *angleStr = strtok(NULL, ";");
                if (!servoIdStr || !posStr || !angleStr) {
                    success = 0;
                    break;
                }
                int servoId = atoi(servoIdStr);
                int pos = atoi(posStr);
                int angle = atoi(angleStr);
                setServoPosValue(servoId, pos, angle);

                int wrote = snprintf(response + off, sizeof(response) - off, ";%d;%d", servoId, pos);
                if (wrote < 0 || (size_t)wrote >= sizeof(response) - off) {
                    success = 0;
                    break;
                }
                off += wrote;
            }

            if (!success) {
                snprintf(response, sizeof(response), "KO;%s;%s\n", arg1, arg2);
            } else {
                if (off < sizeof(response) - 1) {
                    response[off++] = '\n';
                    response[off] = '\0';
                } else {
                    response[sizeof(response) - 1] = '\0';
                }
            }
            CDC_Transmit_FS((uint8_t*)response, strlen(response));
        }
    } else if (strcmp(type, "MOV") == 0) {
        int success = 1;
        if (strcmp(arg1, "SERVO") == 0) {
            int n = atoi(arg2);
            size_t off = 0;
            off += snprintf(response + off, sizeof(response) - off, "%s;%s;%s;%d", "OK", arg1, arg2, n);

            for (int i = 0; i < n; ++i) {
                char *servoIdStr = strtok(NULL, ";");
                char *posStr = strtok(NULL, ";");
                if (!servoIdStr || !posStr) {
                    success = 0;
                    break;
                }
                int servoId = atoi(servoIdStr);
                int pos = atoi(posStr);
                moveServo(servoId, pos);

                int wrote = snprintf(response + off, sizeof(response) - off, ";%d;%d", servoId, pos);
                if (wrote < 0 || (size_t)wrote >= sizeof(response) - off) {
                    success = 0;
                    break;
                }
                off += wrote;
            }

            if (!success) {
                snprintf(response, sizeof(response), "KO;MOV;%s;%s\n", arg1, arg2);
            } else {
                if (off < sizeof(response) - 1) {
                    response[off++] = '\n';
                    response[off] = '\0';
                } else {
                    response[sizeof(response) - 1] = '\0';
                }
            }
            CDC_Transmit_FS((uint8_t*)response, strlen(response));
        } else if (strcmp(arg1, "RELAY") == 0) {
            int n = atoi(arg2);
            size_t off = 0;
            off += snprintf(response + off, sizeof(response) - off, "%s;%s;%s;%d", "OK", arg1, arg2, n);

            for (int i = 0; i < n; ++i) {
                char *relayIdStr = strtok(NULL, ";");
                char *stateStr = strtok(NULL, ";");
                if (!relayIdStr || !stateStr) {
                    success = 0;
                    break;
                }
                int relayId = atoi(relayIdStr);
                int state = atoi(stateStr);
                moveRelay(relayId, state);

                int wrote = snprintf(response + off, sizeof(response) - off, ";%d;%d", relayId, state);
                if (wrote < 0 || (size_t)wrote >= sizeof(response) - off) {
                    success = 0;
                    break;
                }
                off += wrote;
            }

            if (!success) {
                snprintf(response, sizeof(response), "KO;MOV;%s;%s\n", arg1, arg2);
            } else {
                if (off < sizeof(response) - 1) {
                    response[off++] = '\n';
                    response[off] = '\0';
                } else {
                    response[sizeof(response) - 1] = '\0';
                }
            }
            CDC_Transmit_FS((uint8_t*)response, strlen(response));
        }
    } else if (strcmp(type, "ACK") == 0) {
        // Réception d'un ACK d'évènement
        for (int i = 0; i < MAX_PENDING_EVENTS; ++i) {
            if (pendingEvents[i].active &&
                strcmp(pendingEvents[i].evt, arg1) == 0 &&
                strcmp(pendingEvents[i].info, arg2) == 0) {
                pendingEvents[i].active = 0;
                break;
            }
        }
    }
}

void USBProtocol_Receive(uint8_t* Buf, uint32_t Len) {
    static size_t rx_len = 0;
    if (Len == 0) return;

    if (Len + rx_len >= USB_RX_BUFFER_SIZE) {
        rx_len = 0;
        return;
    }

    memcpy(usb_rx_buffer + rx_len, Buf, Len);
    rx_len += Len;

    size_t start = 0;
    for (size_t i = 0; i < rx_len; ++i) {
        if (usb_rx_buffer[i] == '\n') {
            /* determine line length and null-terminate, trimming trailing \\r */
            size_t line_len = (i > start) ? (i - start) : 0;
            if (line_len > 0 && usb_rx_buffer[start + line_len - 1] == '\r') {
                usb_rx_buffer[start + line_len - 1] = '\0';
            } else {
                usb_rx_buffer[start + line_len] = '\0';
            }

            char *line = &usb_rx_buffer[start];
            if (line[0] != '\0') {
                USBProtocol_ProcessCommand(line);
            }

            start = i + 1;
        }
    }

    if (start == 0) {
        return;
    }

    size_t remaining = rx_len - start;
    if (remaining > 0) {
        memmove(usb_rx_buffer, usb_rx_buffer + start, remaining);
    }
    rx_len = remaining;
}