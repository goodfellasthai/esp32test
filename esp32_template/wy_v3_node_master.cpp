#include "wy_v3_node_master.h"

#if defined(WYMAN_LORAV3)

#pragma once

#include <SPI.h>
#include <LoRa.h>

// Define frequency for Thailand
#define RF_FREQUENCY 920E6 // 920 MHz

// Define board-specific pins
#if defined(WYMAN_LORAV2)
  // Heltec V2 pin configuration
  #define LORA_SS    18
  #define LORA_RST   14
  #define LORA_DIO0  26
#elif defined(WYMAN_LORAV3)
  // Heltec V3 pin configuration
  #define LORA_SS    8
  #define LORA_RST   12
  #define LORA_DIO0  14
#else
  #error "Please define WYMAN_LORAV2 or WYMAN_LORAV3 to match your board version."
#endif

void wy_v3_node_master_setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialize LoRa module
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(RF_FREQUENCY)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initialization Successful");
}

void wy_v3_node_master_loop() {
  static uint8_t txNumber = 0;
  static bool awaitingResponse = false;
  static unsigned long lastSendTime = 0;
  const unsigned long interval = 2000; // 2 seconds

  if (!awaitingResponse && millis() - lastSendTime > interval) {
    // Send "Ping" message
    txNumber++;
    String message = "Ping " + String(txNumber);
    LoRa.beginPacket();
    LoRa.print(message);
    LoRa.endPacket();
    Serial.print("Sent: ");
    Serial.println(message);
    awaitingResponse = true;
    lastSendTime = millis();
  }

  // Check for incoming messages
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }
    Serial.print("Received: ");
    Serial.println(received);

    if (received.startsWith("Pong")) {
      // Received a Pong response
      awaitingResponse = false;
    } else if (received.startsWith("Ping")) {
      // Received a Ping, send Pong response
      String response = "Pong " + received.substring(5);
      LoRa.beginPacket();
      LoRa.print(response);
      LoRa.endPacket();
      Serial.print("Sent: ");
      Serial.println(response);
    }
  }
}

#endif // defined(WYMAN_LORAV3)