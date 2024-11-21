#include "wy_v3_node_master.h"

#if defined(WYMAN_LORAV3)

#pragma once

// Turns the 'PRG' button into the power button, long press is off 
#define HELTEC_POWER_BUTTON   // must be before "#include <heltec_unofficial.h>"
#include <heltec_unofficial.h>
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

String rxdata;
volatile bool rxFlag = false;
int counter = 0;
uint64_t last_tx = 0;
uint64_t tx_time;
//uint64_t minimum_pause = PAUSE (1000);

void wy_v3_node_master_setup() {
  heltec_setup();

  // Initialize OLED
  display.init();
  display.setFont(ArialMT_Plain_10);
  display.flipScreenVertically();

 /* // Read and calculate battery percentage
  float batteryPercentage = getBatteryPercentage();
  // Display battery percentage on OLED
  display.drawString(0, 0, "Booting...");
  display.drawString(0, 12, "Battery: " + String(batteryPercentage, 1) + "%");
  display.display();
  delay(2000); // Pause for 2 seconds to display information
    */
  display.clear();
  display.drawString(0, 0,"Next string 1...");
  display.drawString(0, 10,"Next string 2...");
  display.drawString(0, 20,"Next string 3...");
  display.drawString(0, 30,"Next string 4...");
  display.drawString(0, 40,"Next string 5...");
  display.drawString(0, 50,"Next string 6...");
  display.display();
  delay(5000); // 5 seconds for screen testing

  // Begin radio looks like that needs done before radio configuration
  RADIOLIB_OR_HALT(radio.begin());
  //both.println("Radio Init...");

  // Set the callback function for received packets
  radio.setDio1Action(rx);

  // Set radio parameters
  /*both.printf("Frequency: %.2f MHz\n", FREQUENCY);
  RADIOLIB_OR_HALT(radio.setFrequency(FREQUENCY));
  both.printf("Bandwidth: %.1f kHz\n", BANDWIDTH);
  RADIOLIB_OR_HALT(radio.setBandwidth(BANDWIDTH));
  both.printf("Spreading Factor: %i\n", SPREADING_FACTOR);
  RADIOLIB_OR_HALT(radio.setSpreadingFactor(SPREADING_FACTOR));
  both.printf("TX power: %i dBm\n", TRANSMIT_POWER);
  RADIOLIB_OR_HALT(radio.setOutputPower(TRANSMIT_POWER));*/

  // Compact debug display for radio parameters
  //float batteryPercentage = getBatteryPercentage();
  float batteryPercentage = 66;
  display.clear();
  //display.setFont(ArialMT_Plain_10);
  // First row: Frequency and Bandwidth
  display.drawString(0, 0, "FQ:" + String(FREQUENCY, 1) + "MHz BW:" + String(BANDWIDTH, 1) + "kHz");
  // Second row: Spreading Factor and TX Power
  display.drawString(0, 12, "SF:" + String(SPREADING_FACTOR) + " TXP:" + String(TRANSMIT_POWER) + "dBm");
  // Third row: Battery percentage
  display.drawString(0, 24, "Batt:" + String(batteryPercentage, 1) + "%");
  // Send buffer to display
  display.display();
  delay(5000);

  // Start receiving
  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
  //display.clear();
}

void wy_v3_node_master_loop() {
  heltec_loop();
  
  //bool tx_legal = millis() > last_tx + minimum_pause;
  // Transmit a packet every PAUSE seconds or when the button is pressed
  //if ((PAUSE && tx_legal && millis() - last_tx > (PAUSE * 1000)) || button.isSingleClick()) {
  if ( (millis() - last_tx) > (PAUSE * 1000) )  {
    // In case of button click, tell user to wait
    //if (!tx_legal) {
    //  both.printf("Legal limit, wait %i sec.\n", (int)((minimum_pause - (millis() - last_tx)) / 1000) + 1);
    //  return;
    //}
    both.printf("TX [%s] ", String(counter).c_str());
    radio.clearDio1Action();
    heltec_led(50); // 50% brightness is plenty for this LED
    tx_time = millis();
    RADIOLIB(radio.transmit(String(counter++).c_str()));
    tx_time = millis() - tx_time;
    heltec_led(0);
    if (_radiolib_status == RADIOLIB_ERR_NONE) {
      both.printf("OK (%i ms)\n", (int)tx_time);
    } else {
      both.printf("fail (%i)\n", _radiolib_status);
    }
    // Maximum 1% duty cycle
    //minimum_pause = tx_time * 100;
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


#endif // defined(WYMAN_LORAV3)