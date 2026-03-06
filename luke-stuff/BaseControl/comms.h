#ifndef COMMS_H
#define COMMS_H

#include "data_structs.h"

#include <esp_now.h>
#include <WiFi.h>

void setupCommsReceiver(esp_now_recv_cb_t cb);
void setupCommsSender(esp_now_send_cb_t cb, uint8_t broadcastAdd[]);

#endif