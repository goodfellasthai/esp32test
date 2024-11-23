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
/*// LoRaV3 Pins
//V3
// 'PRG' Button
#ifndef BUTTON
#define BUTTON    GPIO_NUM_0
#endif
// LED pin & PWM parameters
#define LED_PIN   GPIO_NUM_35
#define LED_FREQ  5000
#define LED_CHAN  0
#define LED_RES   8
// External power control
#define VEXT      GPIO_NUM_36
// Battery voltage measurement
#define VBAT_CTRL GPIO_NUM_37
#define VBAT_ADC  GPIO_NUM_1
// SPI pins
#define SS        GPIO_NUM_8
#define MOSI      GPIO_NUM_10
#define MISO      GPIO_NUM_11
#define SCK       GPIO_NUM_9
// Radio pins
#define irqPin      14
#define resetPin  12
#define csPin 13
// Display pins
#define OLED_SDA  17
#define OLED_SCL  18
#define OLED_RESET 21*/

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

// Debug settings
//#define LORA_DEBUG
//#define BATTERY_DEBUG
#define ESPNOW_DEBUG

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

String outgoing;              // outgoing message
String monitormsg;            // use for message to monitor on serial or display

int msgCount = 0;            // count of outgoing messages
// Adress info
byte localAddress = 0xBB;     // address of base station
//byte localAddress = 0xDD;     // address of drone device
//byte localAddress = 0xCC;     // address of node device
byte destination = 0xFF;      // destination to send to FF is broadcast to everyone or can specify specific address

long lastSendTime = 0;        // Last send time
uint64_t tx_time;             // Transaction time
int interval = PAUSE * 1000;  // converts the pause time to milliseconds

// V2 LED stuff
#define LED_PIN 25  // GPIO for onboard LED

// Screen timeout stuff
bool isScreenOn = true;  // Screen state
unsigned long lastActivityTime = 0;  // Last time the screen was active
const unsigned long SCREEN_TIMEOUT = 20000;  // 20 seconds timeout
#define PRG_BTN 0  // GPIO0 for PRG button
#define POWER_LONGPRESS 3000 // Must longpress PRG 3 seconds to shutdown, V2 3000 works but needs to be shorted on V3 to avoid bootmode

void wy_v2_node_master_setup() {

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
    // Compact display of settings and battery level
    //float batteryPercentage = getBatteryPercentage();
    // Display battery status
    BatteryStatus status = getBatteryStatus();
    String batteryText = String(status.percentage) + "%";
    String voltageText = String(status.voltage, 2) + "V";
    display.clearDisplay();
    debugMessage("Disp. Init.");
    debugMessage("LoRa Init");
    debugMessage("Frequency:" + String(FREQUENCY, 1) + "MHz");
    debugMessage("Bandwidth:" + String(BANDWIDTH, 1) + "kHz");
    debugMessage("Spread:" + String(SPREADING_FACTOR) + " TXP:" + String(TRANSMIT_POWER) + "dBm");
    debugMessage("Battery: " + String(batteryText) + "% " + String(voltageText) );
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
    if (millis() - lastSendTime > interval) {
        String message = "Transmit test message LoRaV2";  
        //tx_time = millis();             // Record the start time
        analogWrite(LED_PIN, 128);  // 50% brightness   
        sendMessage(message);   // Send the message
        analogWrite(LED_PIN, 0);    // Turn off
        //tx_time = millis() - tx_time;  // Calculate duration
        // Send the monitor message to serial and display
        //monitormsg = "TX[" + String(msgCount) + "]:OK:" + String((int)tx_time) + "ms";
        //Serial.println(monitormsg);   // Debug TX
        //display.println(monitormsg);  // Display TX dont use F as in setup that is fixed for flash this is dynamic could also use sprintf
        ////debugMessage(monitormsg); // Send to serial and display
        /////display.display();                        // Send buffer to the display, required for OLED
        //lastSendTime = millis();                  // Update the last send time
        //interval = random(2000) + 1000;         // Set a random interval between 2-3 seconds
    }

    // parse for a packet, and call onReceive with the result:
    onReceive(LoRa.parsePacket());

}

void sendMessage(String outgoing) {
    /*LoRa.beginPacket();                   // start packet
    LoRa.write(destination);              // add destination address
    LoRa.write(localAddress);             // add sender address
    LoRa.write(msgCount);                 // add message ID
    LoRa.write(outgoing.length());        // add payload length
    LoRa.print(outgoing);                 // add payload
    LoRa.endPacket();                     // finish packet and send it
    msgCount++;                           // increment message ID */

    uint8_t len = outgoing.length();
    BatteryStatus batteryStatus = getBatteryStatus(); // Get battery status (percentage and voltage)
    uint8_t batteryLevel = batteryStatus.percentage;  // Extract battery percentage
    uint16_t voltage = (uint16_t)(batteryStatus.voltage * 100); // Scale voltage to an integer (e.g., 4.12V -> 412)

    uint8_t payload[len + 7]; // Allocate space for header + battery level + voltage + payload

    // Prepare the header
    payload[0] = destination;  // Destination address
    payload[1] = localAddress; // Sender address
    payload[2] = msgCount;     // Message ID
    payload[3] = len;          // Payload length
    payload[4] = batteryLevel; // Battery level
    payload[5] = voltage >> 8; // High byte of voltage
    payload[6] = voltage & 0xFF; // Low byte of voltage

    // Add the payload data (the message string)
    outgoing.getBytes(payload + 7, len + 1); // Convert String to binary array starting from payload[7]

    // Start transmission
    msgCount++;
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
    lastSendTime = millis();                  // Update the last send time
   
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
    /*if (packetSize == 0) return;          // if there's no packet, return

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
    Serial.println();*/

    if (packetSize == 0) return;  // If no packet, return

    // Read packet header bytes
    int recipient = LoRa.read();              // Destination address
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
    if (recipient != localAddress && recipient != 0xFF) {
        Serial.println("This message is not for me.");
        return;
    }

    #if defined(LORA_DEBUG)
      // Debugging: Display parsed packet details
      Serial.println("=== Received Packet ===");
      Serial.println("From: 0x" + String(sender, HEX));
      Serial.println("To: 0x" + String(recipient, HEX));
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
 /*   // Send to Serial first
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
    display.display();*/

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
    /*for (int i = 0; i < currentLine; i++) {
        display.drawString(0, i * 10, messageBuffer[i]); // Adjust line spacing as needed
    }*/
    for (int i = 0; i < currentLine; i++) {
        display.setCursor(0, i * 10); // Adjust line spacing as needed
        display.println(messageBuffer[i]);
    }    
    // Display battery status
    BatteryStatus status = getBatteryStatus();
    String batteryText = String(status.percentage) + "%";
    String voltageText = String(status.voltage, 2) + "V";
    /*int batteryWidth = display.getStringWidth(batteryText);
    display.drawString(display.width() - batteryWidth - 5, 0, batteryText); // Battery percentage
    int voltageWidth = display.getStringWidth(voltageText);
    display.drawString(display.width() - voltageWidth - 5, 12, voltageText); // Battery voltage*/
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

#endif // defined(WYMAN_LORAV2)