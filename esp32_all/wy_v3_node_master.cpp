#include "wy_v3_node_master.h"

#if defined(WYMAN_LORAV3)

#pragma once

// Turns the 'PRG' button into the power button, long press is off 
#define HELTEC_POWER_BUTTON   // must be before "#include <heltec_unofficial.h>"
#include <heltec_unofficial.h>
// LORA STUFF
// Pause between transmited packets in seconds.
// Set to zero to only transmit a packet when pressing the user button
// Will not exceed 1% duty cycle, even if you set a lower value.
#define PAUSE               10           // Pause in seconds between transmits
// Frequency in MHz. Keep the decimal point to designate float.
// Check your own rules and regulations to see what is legal where you are.
//#define FREQUENCY           866.3       // for Europe
// #define FREQUENCY           905.2       // for US
#define FREQUENCY           433.0       // for LoraV3 SX1278 - 915 for the SX1276 but thats only at ESP32 NOT S3
// LoRa bandwidth. Keep the decimal point to designate float.
// Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0 kHz.
#define BANDWIDTH           250.0
// Number from 5 to 12. Higher means slower but higher "processor gain",
// meaning (in nutshell) longer range and more robust against interference. 
#define SPREADING_FACTOR    9
// Transmit power in dBm. 0 dBm = 1 mW, enough for tabletop-testing. This value can be
// set anywhere between -9 dBm (0.125 mW) to 22 dBm (158 mW). Note that the maximum ERP
// (which is what your antenna maximally radiates) on the EU ISM band is 25 mW, and that
// transmissting without an antenna can damage your hardware.
// Europe (EU): 14 dBm (25 mW)
// US: Up to 20-30 dBm, depending on specific frequency bands.
#define TRANSMIT_POWER      0

// OLED Stuff
#define MAX_LINES 6      // Maximum number of lines visible on the display
String messageBuffer[MAX_LINES]; // Buffer to store lines
int currentLine = 0;             // Tracks the current line for scrolling

// LoRa declarations
String rxdata;                // use for received data
String monitormsg;            // use for message to monitor on serial or display

volatile bool rxFlag = false;
int msgCount = 0;
// Adress info
byte localAddress = 0xBB;     // address of base station
//byte localAddress = 0xDD;     // address of drone device
//byte localAddress = 0xCC;     // address of node device
byte destination = 0xFF;      // destination to send to FF is broadcast to everyone or can specify specific address

long lastSendTime = 0;        // Last send time
uint64_t tx_time;             // Transaction time

void wy_v3_node_master_setup() {
  heltec_setup();

  // Initialize OLED
  display.init();
  display.setFont(ArialMT_Plain_10);
  display.flipScreenVertically();

  // Begin radio looks like that needs done before radio configuration
  RADIOLIB_OR_HALT(radio.begin());
  //both.println("Radio Init...");

  // Set the callback function for received packets
  radio.setDio1Action(rx);

  // Set radio parameters
  RADIOLIB_OR_HALT(radio.setFrequency(FREQUENCY));
  RADIOLIB_OR_HALT(radio.setBandwidth(BANDWIDTH));
  RADIOLIB_OR_HALT(radio.setSpreadingFactor(SPREADING_FACTOR));
  RADIOLIB_OR_HALT(radio.setOutputPower(TRANSMIT_POWER));

  display.clear();
  debugMessage("Display initialised");
  debugMessage("LoRa initialised");
  debugMessage("Frequency:" + String(FREQUENCY, 1) + "MHz");
  debugMessage("Bandwidth:" + String(BANDWIDTH, 1) + "kHz");
  debugMessage("Spread:" + String(SPREADING_FACTOR) + " TXP:" + String(TRANSMIT_POWER) + "dBm");
  BatteryStatus batteryStatus = getBatteryStatus();
  uint8_t batteryLevel = batteryStatus.percentage;
  uint16_t voltage = (uint16_t)(batteryStatus.voltage * 100);
  debugMessage("Battery: " + String(batteryLevel) + "% " + String(voltage / 100.0, 2) + "V");
  currentLine = 0;  // Reset the screen for a refresh will delay for pause time on first loop before TX and display will refresh on current line reset

  // Start receiving
  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
}

void wy_v3_node_master_loop() {
  heltec_loop();
  
  if ( (millis() - lastSendTime) > (PAUSE * 1000) )  {
    //both.printf("TX [%s] ", String(msgCount).c_str());
    radio.clearDio1Action();

    String message = "Transmit test message LoRaV3";              
    heltec_led(50); // 50% brightness is plenty for this LED
    sendMessage(message);
    heltec_led(0);

    lastSendTime = millis();

    radio.setDio1Action(rx);
    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
  }

  // If a packet was received, display it and the RSSI and SNR
  if (rxFlag) {
    rxFlag = false;
    //radio.readData(rxdata);
    onReceive(rxdata);
    if (_radiolib_status == RADIOLIB_ERR_NONE) {
      both.printf("RX [%s]\n", rxdata.c_str());
      both.printf("  RSSI: %.2f dBm\n", radio.getRSSI());
      both.printf("  SNR: %.2f dB\n", radio.getSNR());
    }
    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
  }
}

// Can't do Serial or display things here, takes too much time for the interrupt
void rx() {
  rxFlag = true;
}

void sendMessage(String outgoing) {
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
    int16_t status = radio.transmit(payload, len + 7); // Transmit header + battery data + payload
    tx_time = millis() - tx_time;

    // Debug output
    if (_radiolib_status == RADIOLIB_ERR_NONE) {
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
        Serial.printf(" [Battery: %d%%, Voltage: %.2fV]\n", batteryLevel, voltage / 100.0); // Display battery data
        monitormsg = "TX[" + String(msgCount) + "]:OK:" + String((int)tx_time) + "ms";
    } else {
        Serial.printf("Fail (%d)\n", _radiolib_status);
        monitormsg = "TX [" + String(msgCount) + "] Fail (" + String(_radiolib_status) + ")";
    }
    debugMessage(monitormsg);
}

void onReceive(String rxdata) {
    if (rxdata.length() == 0) {
        Serial.println("Error: No data received.");
        return;
    }

    size_t len = rxdata.length();
    if (len < 7) { // Ensure the message has at least a header + battery data
        Serial.println("Error: Insufficient data length.");
        return;
    }

    // Convert the String to a buffer for parsing
    uint8_t buffer[len];
    rxdata.getBytes(buffer, len + 1);

    // Parse header
    uint8_t recipient = buffer[0];
    uint8_t sender = buffer[1];
    uint8_t msgId = buffer[2];
    uint8_t messageLen = buffer[3];
    uint8_t batteryLevel = buffer[4];        // Battery level
    uint16_t voltage = (buffer[5] << 8) | buffer[6]; // Combine high and low bytes for voltage

    // Verify the payload length
    if (len - 7 != messageLen) {
        Serial.println("Error: Message length mismatch!");
        return;
    }

    // Extract the message
    String message = "";
    for (size_t i = 7; i < len; i++) {
        message += (char)buffer[i];
    }

    // Check if the message is for this device
    if (recipient != localAddress && recipient != 0xFF) {
        Serial.println("Message not for me.");
        return;
    }

    // Debug output
    debugMessage("From: 0x" + String(sender, HEX));
    debugMessage("To: 0x" + String(recipient, HEX));
    debugMessage("ID: " + String(msgId));
    debugMessage("Battery Level: " + String(batteryLevel) + "%");
    debugMessage("Voltage: " + String(voltage / 100.0, 2) + "V");
    debugMessage("Message: " + message);
}


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
    display.clear();
    for (int i = 0; i < currentLine; i++) {
        display.drawString(0, i * 10, messageBuffer[i]); // Adjust line spacing as needed
    }
    // Display battery status
    BatteryStatus status = getBatteryStatus();
    String batteryText = String(status.percentage) + "%";
    String voltageText = String(status.voltage, 2) + "V";
    int batteryWidth = display.getStringWidth(batteryText);
    display.drawString(display.width() - batteryWidth - 5, 0, batteryText); // Battery percentage
    int voltageWidth = display.getStringWidth(voltageText);
    display.drawString(display.width() - voltageWidth - 5, 12, voltageText); // Battery voltage

    // Render the updated screen
    display.display();
}

//LoRaV3
// Pin definitions
#define BATTERY_PIN VBAT_ADC    // GPIO1 (VBAT_ADC)
#define ADC_CTRL_PIN VBAT_CTRL  // GPIO37 (VBAT_CTRL)
#define MAX_ADC_READING 4095    // 12-bit ADC
#define REF_VOLTAGE 3.29        // Adjust based on measured reference voltage
#define VOLTAGE_DIVIDER_RATIO 5.16
#define BATTERY_MIN_VOLTAGE 3.0 // Minimum battery voltage
#define BATTERY_MAX_VOLTAGE 4.2 // Maximum battery voltage

BatteryStatus getBatteryStatus() {
    // Enable voltage divider (if needed)
    pinMode(ADC_CTRL_PIN, OUTPUT);
    digitalWrite(ADC_CTRL_PIN, LOW);
    delay(10);

    // Read ADC value
    int rawAdc = analogRead(BATTERY_PIN);

    // Disable voltage divider (if needed)
    digitalWrite(ADC_CTRL_PIN, HIGH);

    // Calculate pin voltage
    float pinVoltage = (rawAdc / (float)MAX_ADC_READING) * REF_VOLTAGE;

    // Calculate battery voltage
    float batteryVoltage = pinVoltage * VOLTAGE_DIVIDER_RATIO;

    // Calculate battery percentage
    uint8_t batteryPercentage;
    if (batteryVoltage <= BATTERY_MIN_VOLTAGE) {
        batteryPercentage = 0; // Empty
    } else if (batteryVoltage >= BATTERY_MAX_VOLTAGE) {
        batteryPercentage = 100; // Full
    } else {
        batteryPercentage = (uint8_t)(((batteryVoltage - BATTERY_MIN_VOLTAGE) / 
                                       (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100);
    }

    // Return battery status
    return {batteryPercentage, batteryVoltage};
}

#endif // defined(WYMAN_LORAV3)