#pragma once
#include "wyconfig.h"
#include "../master/globals.h"

void wy_v3_node_master_setup();
void wy_v3_node_master_loop();

void rx();
void sendMessage(String outgoing);
void onReceive(String rxdata);
void debugMessage(String message);

// Function declarations
BatteryStatus getBatteryStatus();

// Screen timeout stuff
void heltec_power_off();
void toggleScreen();
void turnScreenOn();
void turnScreenOff();

// ESP-NOW stuff
void initEspNow(const uint8_t *peerAddress);
void sendEspNowData(SensorData *data);
bool isDataReceived();
SensorData getReceivedData();
