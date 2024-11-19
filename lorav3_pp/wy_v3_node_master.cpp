#include "wy_v3_node_master.h"

#if defined(WYMAN_LORAV3)

#pragma once

// Turns the 'PRG' button into the power button, long press is off 
#define HELTEC_POWER_BUTTON   // must be before "#include <heltec_unofficial.h>"
#include <heltec_unofficial.h>

// Pause between transmited packets in seconds.
// Set to zero to only transmit a packet when pressing the user button
// Will not exceed 1% duty cycle, even if you set a lower value.
#define PAUSE               10 // Reduced to 2 seconds to match v2 and commincation interval

// Frequency in MHz. Keep the decimal point to designate float.
// Check your own rules and regulations to see what is legal where you are.
// #define FREQUENCY           866.3       // for Europe
// #define FREQUENCY           905.2       // for US
#define FREQUENCY 433E6 // 920 MHz in Hz for Thailand to match my ping frequency from v2

// LoRa bandwidth. Keep the decimal point to designate float.
// Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0 kHz.
#define BANDWIDTH           125.0

// Number from 5 to 12. Higher means slower but higher "processor gain",
// meaning (in nutshell) longer range and more robust against interference. 
#define SPREADING_FACTOR    5

// Transmit power in dBm. 0 dBm = 1 mW, enough for tabletop-testing. This value can be
// set anywhere between -9 dBm (0.125 mW) to 22 dBm (158 mW). Note that the maximum ERP
// (which is what your antenna maximally radiates) on the EU ISM band is 25 mW, and that
// transmissting without an antenna can damage your hardware.
#define TRANSMIT_POWER      10

String rxdata;
volatile bool rxFlag = false;
long counter = 1;
uint64_t last_tx = 0;
uint64_t tx_time;
uint64_t minimum_pause;

void wy_v3_node_master_setup() {
  heltec_setup();
  //both.println("Radio init");
  RADIOLIB_OR_HALT(radio.begin());
  // Set the callback function for received packets

  // Custom radio
  radio.setCodingRate(5); // For Heltec V3
  radio.setPreambleLength(8); // For Heltec V3
  //radio.setCRC(true);

  radio.setDio1Action(rx);
  // Set radio parameters
  both.printf("Frequency: %.2f MHz\n", FREQUENCY / 1E6);
  RADIOLIB_OR_HALT(radio.setFrequency(FREQUENCY/ 1E6));
  both.printf("Bandwidth: %.1f kHz\n", BANDWIDTH);
  RADIOLIB_OR_HALT(radio.setBandwidth(BANDWIDTH));
  both.printf("Spreading Factor: %i\n", SPREADING_FACTOR);
  RADIOLIB_OR_HALT(radio.setSpreadingFactor(SPREADING_FACTOR));
  both.printf("TX power: %i dBm\n", TRANSMIT_POWER);
  RADIOLIB_OR_HALT(radio.setOutputPower(TRANSMIT_POWER));

}

void wy_v3_node_master_loop() {
  heltec_loop();
  
  bool tx_legal = millis() > last_tx + minimum_pause;
  // Transmit a packet every PAUSE seconds or when the button is pressed
  if ((PAUSE && tx_legal && millis() - last_tx > (PAUSE * 1000)) || button.isSingleClick()) {
    // In case of button click, tell user to wait
    if (!tx_legal) {
      both.printf("Legal limit, wait %i sec.\n", (int)((minimum_pause - (millis() - last_tx)) / 1000) + 1);
      return;
    }

    // SEND
    both.printf("TX [%s] ", String(counter).c_str());
    radio.clearDio1Action();
    heltec_led(50); // 50% brightness is plenty for this LED
    tx_time = millis();
    //RADIOLIB(radio.transmit(("Pong " + String(counter++)).c_str()));
    radio.transmit(("Pong " + String(counter)).c_str());
    tx_time = millis() - tx_time;
    heltec_led(0);
    if (_radiolib_status == RADIOLIB_ERR_NONE) {
      both.printf("OK (%i ms)\n", (int)tx_time);
    } else {
      both.printf("Fail (%i)\n", _radiolib_status);
    }
    // Maximum 1% duty cycle
    minimum_pause = tx_time * 100;
    last_tx = millis();

    // RECEIVE
    radio.setDio1Action(rx);
    //RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
    both.printf("RX [%s] ", String(counter++).c_str());
    if (_radiolib_status == RADIOLIB_ERR_NONE) {
      both.printf("OK\n");
    } else {
      both.printf("Fail\n");
    }
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
    //RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
  }
}

// Can't do Serial or display things here, takes too much time for the interrupt
void rx() {
  rxFlag = true;
  // Debug statement
  Serial.println("Packet received interrupt triggered");
}


#endif // defined(WYMAN_LORAV3)