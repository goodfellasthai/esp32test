#include "wy_node_master.h"
#if defined(WYMAN_LORA_GENERIC) && defined(WY_NODE_MASTER)
#pragma once
// LoRa.h conflicts with the board functions
#include <heltec.h>
//#include <LoRa.h>
#include <WiFi.h>

// Node name
const String NODE_NAME = "node_master";

// LoRa parameters
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26
//#define BAND 915E6 // Set for your region and research a good default is 915 MHz
//923–925 MHz range is less congested compared to 2.4 GHz
//Lower frequencies, such as 433 MHz, might offer longer range but face higher noise levels and are not standardized for LoRaWAN in Thailand
//AS923-2 provides easier integration with cloud services and IoT platforms supporting LoRaWAN
//The lower frequency allows signals to travel farther compared to higher-frequency bands like 5 GHz or 60 GHz
//2.4 GHz signals are better at penetrating walls and other obstacles compared to higher frequencies.
//The 2.4 GHz band has fewer non-overlapping channels (typically 3 in Wi-Fi) compared to the 5 GHz band.
#define BAND 923E6 // 923 MHz for AS923-2 - recommended for Thailand


// Wi-Fi parameters
const char *ssid = "Somyong_2.4G";
const char *password = "0806264123";

void wy_node_master_setup() {
  //Serial.begin(115200);
  //Serial.println("IN NODE MASTER SETUP BEFORE WiFi");
  // Initialize Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");

  // Initialize Heltec with LoRa enabled don't initialise serial or display already done in main script and generic
  Heltec.begin(false /*DisplayEnable*/, true /*LoRaEnable*/, false /*SerialEnable*/);

  // Initialize LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  // Start LoRa with PA_BOOST enabled
  Serial.print("Starting LoRa");
  while (!LoRa.begin(BAND, true)) {
    delay(500);
    Serial.print(".");
    //Serial.println("Starting LoRa failed!");
    //while (1);
  }
  delay(100); // Small delay to allow LoRa module to stabilize
  Serial.println("\nLoRa initialised!");
}

void wy_node_master_loop() {
  // Handle incoming LoRa messages
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String message = "";
    while (LoRa.available()) {
      message += (char)LoRa.read();
    }
    int rssi = LoRa.packetRssi();

    // Log received message
    Serial.println("\nReceived from LoRa: " + message + " | RSSI: " + String(rssi));

    // Process message
    if (message.startsWith("node_")) {
      // Example: Handle RSSI updates or telemetry
      if (message.indexOf("RSSI:") >= 0) {
        Serial.println("\nRSSI update received: " + message);
      } else {
        Serial.println("\nForwarding message: " + message);

        // Forward the message to other nodes
        LoRa.beginPacket();
        LoRa.print(message);
        LoRa.endPacket();
      }
    }
  }

  // Send periodic status updates
  static unsigned long lastStatusUpdate = 0;
  if (millis() - lastStatusUpdate > 5000) { // Every 5 seconds ping the current nodes status
    lastStatusUpdate = millis();
    String statusUpdate = NODE_NAME + ": Status OK";
    LoRa.beginPacket();
    LoRa.print(statusUpdate);
    LoRa.endPacket();
    //Serial.println("Sent status update: " + statusUpdate);
    Serial.print(".");
  }
}

#endif