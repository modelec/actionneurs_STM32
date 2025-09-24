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
    char msg[64];
    sprintf(msg, "EVENT;%s;%s\n", pendingEvents[idx].evt, pendingEvents[idx].info);
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}

void queue_event(const char *evt, const char *info) {
    for (int i = 0; i < MAX_PENDING_EVENTS; ++i) {
        if (!pendingEvents[i].active) {
            strncpy(pendingEvents[i].evt, evt, sizeof(pendingEvents[i].evt) - 1);
            strncpy(pendingEvents[i].info, info, sizeof(pendingEvents[i].info) - 1);
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
    char response[128];
    char *type = strtok(cmd, ";");
    char *arg1 = strtok(NULL, ";");
    char *arg2 = strtok(NULL, ";");
    char *arg3 = strtok(NULL, ";");

    if (!type || !arg1 || !arg2) return;

    if (strcmp(type, "GET") == 0) {
        if (strncmp(arg1, "SERVO", 5) == 0 && strcmp(arg2, "POS") == 0) {
        	int n = atoi(arg1 + 5);
            int pos = getServoPos(n);
            sprintf(response, "SET;%s;%s;%d\n", arg1, arg2, pos);
            CDC_Transmit_FS((uint8_t*)response, strlen(response));
        }

    } else if (strcmp(type, "SET") == 0 && arg3) {
        int val = atoi(arg3);
        bool success = 1;
        if (strcmp(arg1, "TIR") == 0 && strcmp(arg2, "ARM") == 0) {
            if(val){
            	armTirette();
            } else {
            	disarmTirette();
            }
        } else if (strncmp(arg1, "SERVO", 5) == 0 && strncmp(arg2, "POS", 3) == 0){
            int n = atoi(arg1 + 5);
            int pos = atoi(arg2 + 3);
			setServoPosValue(n, pos, val);
        }
        sprintf(response, "%s;%s;%s;%d\n", success ? "OK" : "KO", arg1, arg2, val);
        CDC_Transmit_FS((uint8_t*)response, strlen(response));
    } else if (strcmp(type, "MOV") == 0) {
        bool success = 1;
        if (strncmp(arg1, "SERVO", 5) == 0) {
            int n = atoi(arg1 + 5);
            int pos = atoi(arg2 + 3);
            if(n == 5){
            	if(pos == 1){
            		PCA9685_ContinuousServoRun(n);
            	} else {
            		PCA9685_ContinuousServoStop(n);
            	}
            } else {
            	moveServo(n, pos);
            }
        } else if (strncmp(arg1, "RELAY", 5) == 0) {
        	int n = atoi(arg1 + 5);
            int state = atoi(arg2);
            moveRelay(n, state);
        }
        sprintf(response, "%s;%s;%s\n", success ? "OK" : "KO", arg1, arg2);
        CDC_Transmit_FS((uint8_t*)response, strlen(response));

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
    if (Len >= USB_RX_BUFFER_SIZE) return;
    memcpy(usb_rx_buffer, Buf, Len);
    usb_rx_buffer[Len] = '\0';

    char *line = strtok((char*)usb_rx_buffer, "\n");
    while (line != NULL) {
        USBProtocol_ProcessCommand(line);
        line = strtok(NULL, "\n");
    }
}
