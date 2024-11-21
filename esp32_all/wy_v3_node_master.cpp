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
#define PAUSE               5           // Pause in seconds between transmits
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

// LoRa decalrations
String rxdata;                // use for received data
String monitormsg;            // use for message to monitor on serial or display

volatile bool rxFlag = false;
int msgCount = 0;
uint64_t last_tx = 0;
uint64_t tx_time;
//uint64_t minimum_pause = PAUSE (1000);

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

  // Compact display of settings and battery level
  //float batteryPercentage = getBatteryPercentage();
  float batteryPercentage = 66;
  display.clear();
  debugMessage("Display initialised");
  debugMessage("LoRa initialised");
  debugMessage("Frequency:" + String(FREQUENCY, 1) + "MHz");
  debugMessage("Bandwidth:" + String(BANDWIDTH, 1) + "kHz");
  debugMessage("Spread:" + String(SPREADING_FACTOR) + " TXP:" + String(TRANSMIT_POWER) + "dBm");
  debugMessage("Battery:" + String(batteryPercentage, 1) + "%");
  currentLine = 0;  // Reset the screen for a refresh will delay for pause time on first loop before TX and display will refresh on current line reset

  // Start receiving
  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
}

void wy_v3_node_master_loop() {
  heltec_loop();
  
  if ( (millis() - last_tx) > (PAUSE * 1000) )  {
    //both.printf("TX [%s] ", String(msgCount).c_str());
    radio.clearDio1Action();
    heltec_led(50); // 50% brightness is plenty for this LED
    tx_time = millis();
    RADIOLIB(radio.transmit(String(msgCount++).c_str()));
    tx_time = millis() - tx_time;
    heltec_led(0);
    // Construct monitormsg based on transmission result
    if (_radiolib_status == RADIOLIB_ERR_NONE) {
        //both.printf("OK (%d ms)\n", (int)tx_time);
        monitormsg = "TX [" + String(msgCount) + "] OK (" + String((int)tx_time) + " ms)";
    } else {
        //both.printf("fail (%d)\n", _radiolib_status);
        monitormsg = "TX [" + String(msgCount) + "] Fail (" + String(_radiolib_status) + ")";
    }
    debugMessage(monitormsg);
    last_tx = millis();

    radio.setDio1Action(rx);
    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
  }

  // If a packet was received, display it and the RSSI and SNR
  if (rxFlag) {
    rxFlag = false;
    radio.readData(rxdata);
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
    display.clear();
    for (int i = 0; i < currentLine; i++) {
        display.drawString(0, i * 10, messageBuffer[i]); // Adjust line spacing as needed this is fine for Arial 10 font
        //display.println(messageBuffer[i]);
    }
    display.display();
}


#endif // defined(WYMAN_LORAV3)