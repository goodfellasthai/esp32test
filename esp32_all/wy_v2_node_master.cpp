#include "wy_v2_node_master.h"

#if defined(WYMAN_LORAV2)

#pragma once

// LORA SPI
#include <SPI.h>
#include <LoRa.h>
// DISPLAY I2C
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// OLED display dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define MAX_LINES 6      // Maximum number of lines visible on the display
// I2C address for the LoRaV2 OLED
#define OLED_ADDRESS 0x3C // Replace with your OLED's I2C address
// Pins for the OLED LoRaV2 I2C
#define OLED_RESET 16 // Set to -1 if not used LoRaV2
#define OLED_SDA 4 // LoRaV2
#define OLED_SCL 15 // LoRaV2
// Create an SSD1306 object for I2C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
String messageBuffer[MAX_LINES]; // Buffer to store lines
int currentLine = 0;             // Tracks the current line for scrolling

// Define LoRaV2 board-specific Pins
#define csPin       18    // GPIO18 - LoRaV2 radio chip select
#define resetPin    14    // GPIO14 - LoRaV2 radio reset
#define irqPin      26    // GPIO26 - LoRaV2 radio interrupt

// Define LoRaV2 parameters
#define PAUSE               5           // Pause in seconds between transmits
#define FREQUENCY           915.0       // Frequency in MHz
#define BANDWIDTH           250.0       // Bandwidth in kHz
#define SPREADING_FACTOR    9           // Spreading factor (5-12)
#define TRANSMIT_POWER      0           // Transmit power in dBm

String outgoing;              // outgoing message
String monitormsg;            // use for message to monitor on serial or display

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of base station
//byte localAddress = 0xDD;     // address of drone device
//byte localAddress = 0xCC;     // address of node device
byte destination = 0xFF;      // destination to send to FF is broadcast to everyone or can specify specific address
long lastSendTime = 0;        // last send time
int interval = PAUSE * 1000;          // converts the pause time to milliseconds

void wy_v2_node_master_setup() {

    // Set up display
    // Set custom SDA and SCL pins for I2C
    Wire.begin(OLED_SDA, OLED_SCL); 
    // Initialize the display with the specified I2C address
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;); // Loop forever if initialization fails
    }
    // Clear the display buffer set the universal text information and display a confirmation
    display.clearDisplay();
    display.setTextSize(1);              // Set text size (1 = small, 2 = medium, etc.)
    display.setTextColor(SSD1306_WHITE); // Set text color to white
    display.setCursor(0, 0);             // Set cursor to top-left corner
    debugMessage("Display initialised");

    // Setup LoRaV2
    LoRa.setSpreadingFactor(SPREADING_FACTOR); // Set spreading factor
    LoRa.setSignalBandwidth(BANDWIDTH * 1E3);  // Convert kHz to Hz
    LoRa.setTxPower(TRANSMIT_POWER);           // Set transmit power (in dBm)
    // Set custom pins
    LoRa.setPins(csPin, resetPin, irqPin);      // set CS, reset, IRQ pin
    // Initialize LoRa with the frequency (convert MHz to Hz)
    if (!LoRa.begin(FREQUENCY * 1E6)) {
      Serial.println("LoRa init failed. Check your connections.");
      while (true);                             // if failed, do nothing
    }
    // Display configuration serial and OLED
    debugMessage("LoRa initialised");
    debugMessage("Frequency: " + String(FREQUENCY) + " MHz");
    debugMessage("Bandwidth: " + String(BANDWIDTH) + " kHz");
    debugMessage("Spreading Factor: " + String(SPREADING_FACTOR) );
    debugMessage("Transmit Pwr: " + String(TRANSMIT_POWER) + " dBm");
    currentLine = 0;  // Reset the screen for a refresh will delay for pause time on first loop before TX and display will refresh on current line reset
}

void wy_v2_node_master_loop() {

    // Check if it's time to send a message as the first message will always wait the pause time as no last send time
    if (millis() - lastSendTime > interval) {
        String message = "HeLoRa World!";               // Message to send
        unsigned long startTime = millis();             // Record the start time
        sendMessage(message);                           // Send the message
        unsigned long duration = millis() - startTime;  // Calculate duration
        // Send the monitor message to serial and display
        monitormsg = "TX [" + String(msgCount) + "] OK (" + String(duration) + " ms)";
        //Serial.println(monitormsg);   // Debug TX
        //display.println(monitormsg);  // Display TX dont use F as in setup that is fixed for flash this is dynamic could also use sprintf
        debugMessage(monitormsg); // Send to serial and display
        display.display();                        // Send buffer to the display, required for OLED
        lastSendTime = millis();                  // Update the last send time
        //interval = random(2000) + 1000;         // Set a random interval between 2-3 seconds
    }

    // parse for a packet, and call onReceive with the result:
    onReceive(LoRa.parsePacket());

}

void sendMessage(String outgoing) {
    LoRa.beginPacket();                   // start packet
    LoRa.write(destination);              // add destination address
    LoRa.write(localAddress);             // add sender address
    LoRa.write(msgCount);                 // add message ID
    LoRa.write(outgoing.length());        // add payload length
    LoRa.print(outgoing);                 // add payload
    LoRa.endPacket();                     // finish packet and send it
    msgCount++;                           // increment message ID
}

void onReceive(int packetSize) {
    if (packetSize == 0) return;          // if there's no packet, return

    // read packet header bytes:
    int recipient = LoRa.read();          // recipient address
    byte sender = LoRa.read();            // sender address
    byte incomingMsgId = LoRa.read();     // incoming msg ID
    byte incomingLength = LoRa.read();    // incoming msg length

    String incoming = "";

    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }

    if (incomingLength != incoming.length()) {   // check length for error
      Serial.println("error: message length does not match length");
      return;                             // skip rest of function
    }

    // if the recipient isn't this device or broadcast,
    if (recipient != localAddress && recipient != 0xFF) {
      Serial.println("This message is not for me.");
      return;                             // skip rest of function
    }

    // if message is for this device, or broadcast, print details:
    Serial.println("Received from: 0x" + String(sender, HEX));
    Serial.println("Sent to: 0x" + String(recipient, HEX));
    Serial.println("Message ID: " + String(incomingMsgId));
    Serial.println("Message length: " + String(incomingLength));
    Serial.println("Message: " + incoming);
    Serial.println("RSSI: " + String(LoRa.packetRssi()));
    Serial.println("Snr: " + String(LoRa.packetSnr()));
    Serial.println();
}

// Send debug to serial and display
void debugMessage(String message) {
    // Send to Serial first
    Serial.println(message);
    // Add new message to the buffer
    if (currentLine < MAX_LINES) {
        messageBuffer[currentLine] = message; // Add to the next available line
        currentLine++;
    } else {
        // Shift all lines up by 1 (scroll effect)
        for (int i = 1; i < MAX_LINES; i++) {
            messageBuffer[i - 1] = messageBuffer[i];
        }
        messageBuffer[MAX_LINES - 1] = message; // Add the new message to the last line
    }

    // Render all lines to display
    display.clearDisplay();
    for (int i = 0; i < currentLine; i++) {
        display.setCursor(0, i * 10); // Adjust line spacing as needed
        display.println(messageBuffer[i]);
    }
    display.display();
}

#endif // defined(WYMAN_LORAV2)