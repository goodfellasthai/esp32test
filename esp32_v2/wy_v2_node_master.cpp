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
// For MAC address and ESP Now
#include <WiFi.h>
#include <esp_now.h>

// OLED display dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define MAX_LINES 6      // Maximum number of lines visible on the display
// I2C address for the LoRaV2 OLED
#define OLED_ADDRESS 0x3C // Replace with your OLED's I2C address
// Pins for the OLED LoRaV2 I2C
#define OLED_RESET 16 // LoRaV2
//#define OLED_RESET 21 // LoRaV3
#define OLED_SDA 4 // LoRaV2
//#define OLED_SDA 17 // LoRaV3
#define OLED_SCL 15 // LoRaV2
//#define OLED_SCL 18 // LoRaV3

// Create an SSD1306 object for I2C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
String messageBuffer[MAX_LINES]; // Buffer to store lines
int currentLine = 0;             // Tracks the current line for scrolling

// Define LoRaV2 board-specific Pins
#define csPin       18    // GPIO18 - LoRaV2 radio chip select
#define resetPin    14    // GPIO14 - LoRaV2 radio reset
#define irqPin      26    // GPIO26 - LoRaV2 radio interrupt

// Define LoRaV2 parameters
#define PAUSE               10           // Pause in seconds between transmits
#define FREQUENCY           915.0       // Frequency in MHz
//#define FREQUENCY           433.0       // Frequency in MHz v3 only
#define BANDWIDTH           250.0       // Bandwidth in kHz
#define SPREADING_FACTOR    9           // Spreading factor (5-12)
#define TRANSMIT_POWER      0           // Transmit power in dBm

// LoRa Declarations
String outgoing;              // outgoing message
String monitormsg;            // use for message to monitor on serial or display
int msgCount = 0;            // count of outgoing messages
long loraLastSendTime = 0;        // Last send time
uint64_t tx_time;             // Transaction time
int interval = PAUSE * 1000;  // converts the pause time to milliseconds

// V2 LED stuff
#define LED_PIN 25  // GPIO for onboard LED

// Screen timeout stuff
bool isScreenOn = true;  // Screen state
unsigned long lastActivityTime = 0;  // Last time the screen was active
const unsigned long SCREEN_TIMEOUT = 30000;  // 30 seconds timeout
#define PRG_BTN 0  // GPIO0 for PRG button
#define POWER_LONGPRESS 3000 // Must longpress PRG 3 seconds to shutdown, V2 3000 works but needs to be shorted on V3 to avoid bootmode

// ESP-NOW Stuff
SensorData espnData = {51.5074, -0.1278, 30.0, 1013.25}; // Sample data
unsigned long espnLastSendTime = 0; // Can use when we seperate out but ESPN works on LoRa PAUSE time
static SensorData receivedData;
static bool dataReceivedFlag = false;


void wy_v2_node_master_setup() {

    // ESP-NOW Stuff
    // Initialize ESP-NOW
    uint8_t broadcastAddress[] = BC_ESPN;
    initEspNow(broadcastAddress);

    // V2 LED Stuff
    pinMode(LED_PIN, OUTPUT);

    // Set up display
    // Set custom SDA and SCL pins for I2C
    Wire.begin(OLED_SDA, OLED_SCL); 
    // Initialize the display with the specified I2C address
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;); // Loop forever if initialization fails
    }
    // Set the universal font information and start cursor point
    display.setTextSize(1);              // Set text size (1 = small, 2 = medium, etc.)
    display.setTextColor(SSD1306_WHITE); // Set text color to white
    display.setCursor(0, 0);             // Set cursor to top-left corner

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
    // Display battery status
    BatteryStatus status = getBatteryStatus();
    String batteryText = String(status.percentage) + "%";
    String voltageText = String(status.voltage, 2) + "V";
    display.clearDisplay();
    debugMessage("Disp. Init.");
    debugMessage("LoRa Init");
    debugMessage("F:" + String(FREQUENCY, 1) + "MHz " + "B:" + String(BANDWIDTH, 1) + "kHz");
    debugMessage("Spread:" + String(SPREADING_FACTOR) + " TXP:" + String(TRANSMIT_POWER) + "dBm");
    String macAddress = WiFi.macAddress();
    debugMessage("MAC:" + macAddress );
    debugMessage("Battery: " + String(batteryText) + " " + String(voltageText) );
    currentLine = 0;  // Reset the screen for a refresh will delay for pause time on first loop before TX and display will refresh on current line reset

    // Screen timeout stuff
    pinMode(PRG_BTN, INPUT_PULLUP);  // PRG button setup
}

void wy_v2_node_master_loop() {

    // Screen timeout stuff
    static unsigned long buttonPressTime = 0;  // Tracks button press duration
    static bool buttonHeld = false;           // Tracks if the button is being held
    bool buttonState = digitalRead(PRG_BTN);
    // Long press detection (power off)
    if (buttonState == LOW) {  // Button is pressed
        if (buttonPressTime == 0) {
            buttonPressTime = millis();  // Start tracking press time
        } else if (!buttonHeld && millis() - buttonPressTime > POWER_LONGPRESS) {  // Long press detected
            buttonHeld = true;  // Mark the button as held
            Serial.println("Long press detected! Powering off...");
            heltec_power_off();  // Power off the device and put into deep sleep
        }
    } else {  // Button is released
        if (buttonPressTime > 0 && !buttonHeld) {  // Short press detected
            if (millis() - buttonPressTime < POWER_LONGPRESS) {
                Serial.println("Short press detected! Toggling screen...");
                toggleScreen();  // Toggle screen on/off
            }
        }
        buttonPressTime = 0;  // Reset press time
        buttonHeld = false;   // Reset button held state
    }
    // Screen timeout logic
    if (isScreenOn && millis() - lastActivityTime > SCREEN_TIMEOUT) {
        Serial.println("Screen timeout! Turning off screen...");
        turnScreenOff();
    }

    // Check if it's time to send a message as the first message will always wait the pause time as no last send time
    if (millis() - loraLastSendTime > interval) {
        // Send ESP-NOW first
        sendEspNowData(&espnData);

        // Then send LoRa
        String message = "Transmit test message from LoRaV2";  
        analogWrite(LED_PIN, 128);  // 50% brightness   
        sendMessage(message);   // Send the message
        analogWrite(LED_PIN, 0);    // Turn off
    }

    // ESPN Receive
    // Check for received data
    if (isDataReceived()) {
        SensorData received = getReceivedData();
        Serial.println("Received Data:");
        Serial.print("GPS Latitude: "); Serial.println(received.gps_latitude);
        Serial.print("GPS Longitude: "); Serial.println(received.gps_longitude);
        Serial.print("Altitude: "); Serial.println(received.altitude);
        Serial.print("Pressure: "); Serial.println(received.pressure);
    }

    // LoRa Receive
    onReceive(LoRa.parsePacket());

}

void sendMessage(String outgoing) {
    uint8_t len = outgoing.length();
    BatteryStatus batteryStatus = getBatteryStatus(); // Get battery status (percentage and voltage)
    uint8_t batteryLevel = batteryStatus.percentage;  // Extract battery percentage
    uint16_t voltage = (uint16_t)(batteryStatus.voltage * 100); // Scale voltage to an integer (e.g., 4.12V -> 412)

    uint8_t payload[len + 7]; // Allocate space for header + battery level + voltage + payload

    // Prepare the header and increase the message count
    msgCount++;
    payload[0] = BC_LORA;  // BC_LORA address
    payload[1] = LOCAL_ADDRESS; // Sender address
    payload[2] = msgCount;     // Message ID
    payload[3] = len;          // Payload length
    payload[4] = batteryLevel; // Battery level
    payload[5] = voltage >> 8; // High byte of voltage
    payload[6] = voltage & 0xFF; // Low byte of voltage

    // Add the payload data (the message string)
    outgoing.getBytes(payload + 7, len + 1); // Convert String to binary array starting from payload[7]

    // Start transmission
    tx_time = millis();
    // V2: int16_t status = radio.transmit(payload, len + 7); // Transmit header + battery data + payload
    // Begin LoRa packet
    LoRa.beginPacket();
    // Write payload to the LoRa packet
    for (uint8_t i = 0; i < len + 7; i++) {
        LoRa.write(payload[i]);
    }
    // End packet and transmit
    bool result = LoRa.endPacket();
    tx_time = millis() - tx_time;
    loraLastSendTime = millis();                  // Update the last send time
   
    // Debug output
    // V3: if (_radiolib_status == RADIOLIB_ERR_NONE) {
    if (result) {
        #if defined(LORA_DEBUG)
          Serial.println("Payload Debug:");
          Serial.print("Hex: ");
          for (size_t i = 0; i < len + 7; i++) {
              Serial.printf("%02X ", payload[i]); // Print as hex
          }
          Serial.println();
          Serial.print("ASCII: ");
          for (size_t i = 0; i < len + 7; i++) {
              if (payload[i] >= 32 && payload[i] <= 126) {
                  Serial.print((char)payload[i]); // Printable ASCII
              } else {
                  Serial.print('.');
              }
          }
          Serial.printf(" [Transmitted Battery Info: %d%%, Voltage: %.2fV]\n", batteryLevel, voltage / 100.0); // Display battery data
        #endif
        monitormsg = "TX[" + String(msgCount) + "]:OK:" + String((int)tx_time) + "ms";
    } else {
        monitormsg = "TX [" + String(msgCount) + "] Fail";
    }
    debugMessage(monitormsg);
}

void onReceive(int packetSize) {
    if (packetSize == 0) return;  // If no packet, return

    // Read packet header bytes
    byte recipient = LoRa.read();             // BC_LORA address
    byte sender = LoRa.read();                // Sender address
    byte incomingMsgId = LoRa.read();         // Message ID
    byte incomingLength = LoRa.read();        // Payload length
    byte batteryLevel = LoRa.read();          // Battery percentage
    uint16_t voltage = (LoRa.read() << 8) | LoRa.read(); // Combine high and low bytes for voltage

    // Read the payload data (message string)
    String incoming = "";
    while (LoRa.available()) {
        incoming += (char)LoRa.read();
    }

    // Validate the payload length
    if (incomingLength != incoming.length()) {
        Serial.println("Error: Message length does not match payload length");
        return;
    }

    // Check if the message is for this device or broadcast
    if (recipient != LOCAL_ADDRESS && recipient != BC_LORA) {
        Serial.println("This message is not for me.");
        return;
    }

    #if defined(LORA_DEBUG)
      // Debugging: Display parsed packet details
      Serial.println("=== Received Packet ===");
      Serial.println("To: 0x" + String(recipient, HEX));
      Serial.println("From: 0x" + String(sender, HEX));
      Serial.println("Message ID: " + String(incomingMsgId));
      Serial.println("Message Length: " + String(incomingLength));
      Serial.println("Battery Level: " + String(batteryLevel) + "%");
      Serial.println("Voltage: " + String(voltage / 100.0, 2) + "V");
      Serial.println("Message: " + incoming);
      Serial.println("RSSI: " + String(LoRa.packetRssi()) + " dBm");
      Serial.println("SNR: " + String(LoRa.packetSnr()) + " dB");
      Serial.println("========================\n");
    #endif
}

// Send debug to serial and display
void debugMessage(String message) {
    // Send to Serial
    Serial.println(message);
    // Add the new message to the buffer
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
    // Render debug messages on the display
    display.clearDisplay();
    for (int i = 0; i < currentLine; i++) {
        display.setCursor(0, i * 10); // Adjust line spacing as needed
        display.println(messageBuffer[i]);
    }    
    // Display battery status
    BatteryStatus status = getBatteryStatus();
    String batteryText = String(status.percentage) + "%";
    String voltageText = String(status.voltage, 2) + "V";
    // Calculate width of the text to right-align
    int16_t x1, y1; // Variables to store bounding box coordinates (not used here)
    uint16_t textWidth, textHeight;  // Dimensions of the text
    // Get width of battery percentage
    display.getTextBounds(batteryText, 0, 0, &x1, &y1, &textWidth, &textHeight);
    display.setCursor(SCREEN_WIDTH - textWidth - 5, 0); // Right-align for percentage
    display.print(batteryText);
    // Get width of battery voltage
    display.getTextBounds(voltageText, 0, 0, &x1, &y1, &textWidth, &textHeight);
    display.setCursor(SCREEN_WIDTH - textWidth - 5, 10); // Right-align for voltage
    display.print(voltageText);

    // Render the updated screen
    display.display();
}

//LoRaV2
#define BATTERY_PIN 37         // VBAT_ADC
#define ADC_CTRL_PIN 21        // VBAT_CTRL
#define MAX_ADC_READING 4095   // 12-bit ADC
#define VOLTAGE_DIVIDER_RATIO 3.58 // Fine-tuned ratio
#define BATTERY_MIN_VOLTAGE 3.0     // Minimum battery voltage
#define BATTERY_MAX_VOLTAGE 4.2     // Maximum battery voltage
#define REF_VOLTAGE 3.3 // Replace with the correct reference voltage if different

BatteryStatus getBatteryStatus() {
    // Enable voltage divider
    pinMode(ADC_CTRL_PIN, OUTPUT);
    digitalWrite(ADC_CTRL_PIN, LOW);
    delay(10); // Stabilize
    // Read ADC value
    int rawAdc = analogRead(BATTERY_PIN);
    // Disable voltage divider
    digitalWrite(ADC_CTRL_PIN, HIGH);
    // Calculate pin voltage
    float pinVoltage = (rawAdc / (float)MAX_ADC_READING) * REF_VOLTAGE;
    // Calculate actual battery voltage
    float batteryVoltage = pinVoltage * VOLTAGE_DIVIDER_RATIO;
    // Debugging
    #if defined(BATTERY_DEBUG)
      Serial.println("=== Battery Debugging ===");
      Serial.println("Raw ADC Value: " + String(rawAdc));
      Serial.println("Pin Voltage: " + String(pinVoltage, 2) + " V");
      Serial.println("Calculated Battery Voltage: " + String(batteryVoltage, 2) + " V");
    #endif
    // Calculate battery percentage
    uint8_t percentage = 0;
    if (batteryVoltage >= BATTERY_MAX_VOLTAGE) {
        percentage = 100;
    } else if (batteryVoltage > BATTERY_MIN_VOLTAGE) {
        percentage = (uint8_t)((batteryVoltage - BATTERY_MIN_VOLTAGE) / 
                               (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE) * 100);
    }
    return {percentage, batteryVoltage};
}

// Screen timeout stuff
void heltec_power_off() {
    Serial.println("Powering off..."); // Never appears as goes into bootloader mode on V3
    esp_deep_sleep_start();  // Put ESP32 into deep sleep
}

void toggleScreen() {
    if (isScreenOn) {
        turnScreenOff();
    } else {
        turnScreenOn();
    }
}

void turnScreenOn() {
    isScreenOn = true;
    display.ssd1306_command(SSD1306_DISPLAYON);  // Wake the display from sleep mode
    lastActivityTime = millis();  // Reset activity timer
    Serial.println("Screen turned on");
}

void turnScreenOff() {
    isScreenOn = false;
    display.ssd1306_command(SSD1306_DISPLAYOFF);  // Put the display in sleep mode
    Serial.println("Screen turned off to save battery");
}

// ESP-NOW stuff
// Callback when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Last Packet Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}
// Callback when data is received
//void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
//    memcpy(&receivedData, incomingData, sizeof(receivedData));
//    dataReceivedFlag = true;
//}
void onDataRecv(const esp_now_recv_info *info, const uint8_t *data, int dataLen) {
    // Print the MAC address of the sender
    Serial.print("Received data from: ");
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", info->src_addr[i]);
        if (i < 5) Serial.print(":");
    }
    Serial.println();

    // Print the data length
    Serial.print("Data length: ");
    Serial.println(dataLen);

    // Print the raw data
    Serial.print("Raw data: ");
    for (int i = 0; i < dataLen; i++) {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();

    // Validate the length and copy data
    if (dataLen == sizeof(SensorData)) {
        memcpy(&receivedData, data, sizeof(SensorData));
        dataReceivedFlag = true;  // Set flag to indicate new data
    } else {
        Serial.println("Error: Data length mismatch!");
    }
}
// Initialize ESP-NOW
void initEspNow(const uint8_t *peerAddress) {
    WiFi.mode(WIFI_STA); // Ensure Wi-Fi is in Station mode
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register callbacks for send and receive
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    // Add peer for broadcasting
    Serial.print("Adding peer with address: ");
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", peerAddress[i]);
        if (i < 5) Serial.print(":");
    }
    Serial.println();
    
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peerAddress, 6); // Use the provided peer address
    peerInfo.channel = 0;  // Set to 0 for default Wi-Fi channel
    peerInfo.encrypt = false; // Disable encryption

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
    } else {
        Serial.println("Peer added successfully");
    }
}
// Send data using ESP-NOW
void sendEspNowData(SensorData *data) {
    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)data, sizeof(SensorData));
    if (result == ESP_OK) {
        Serial.println("Data sent successfully");
    } else {
        Serial.println("Error sending data");
    }
}
// Check if data is received
bool isDataReceived() {
    return dataReceivedFlag;
}
// Get the received data
SensorData getReceivedData() {
    dataReceivedFlag = false; // Reset the flag
    return receivedData;
}

#endif // defined(WYMAN_LORAV2)