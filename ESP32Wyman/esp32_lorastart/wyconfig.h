#pragma once
#ifndef wyconfig_h

  #define wyconfig_h

  #define ENGLISH_POTATO

  //// GLOBAL INCLUDES
  //#include <Arduino.h>
  #include <SPI.h>
  #include <LoRa.h>
  #include <Wire.h> // For I2C communication
  #include <Adafruit_GFX.h> // Graphics library LoRa
  #include <Adafruit_SSD1306.h> // OLED display library LoRa
  #include "wy_ic2scan.h" // IC2 Scanner

  //// BOARD TARGETS (COMMENT OUT AS NEEDED)
  #define WYMAN_LORAV2
  //#define WYMAN_LORAV3
  //#define WYMAN_ESP32C6147
  //#define WYMAN_M5STICKCP2

  //// GLOBAL DECLARATIONS
  #define WYMAN_VERSION "v1.0.0"

  //// HARDWARE NAMES
  #if defined(WYMAN_LORAV2)
    #define HARDWARE_NAME "LoRa ESP32 V2 0.96 OLED 64x128"
  #elif defined(WYMAN_LORAV3)
    #define HARDWARE_NAME "LoRa ESP32 V3 0.96 OLED 64x128"
  #elif defined(WYMAN_ESP32C6147)
    #define HARDWARE_NAME "ESP32-C6FH4 1.47 TFT 172x320 SD"
  #elif defined(WYMAN_M5STICKCP2)
    #define HARDWARE_NAME "M5StickC Plus2 1.14 TFT 135x240"  
  #endif

  //// DEVICE DEFINITIONS
  #if defined(WYMAN_LORAV2)
    // OLED display width and height
    #define SCREEN_WIDTH 128
    #define SCREEN_HEIGHT 64
    #define OLED_ADDRESS 0x3C
    // Define the pins used by the LoRa transceiver module SPI
    #define SCK     5   // GPIO5  -- SX1278's SCK
    #define MISO    19  // GPIO19 -- SX1278's MISO
    #define MOSI    27  // GPIO27 -- SX1278's MOSI
    #define SS      18  // GPIO18 -- SX1278's CS
    #define RST     14  // GPIO14 -- SX1278's RESET
    #define DI0     26  // GPIO26 -- SX1278's IRQ(Interrupt Request)
    // OLED reset pin
    #define OLED_RESET -1
    // Declare the display object (do not define it here)
    extern Adafruit_SSD1306 display;
    // Declare the counter variable (do not define it here)
    extern int counter;

  #elif defined(WYMAN_LORAV3)
    // OLED display width and height
    #define SCREEN_WIDTH 128
    #define SCREEN_HEIGHT 64
    // Define the pins used by the LoRa transceiver module
    #define SCK     5   // GPIO5  -- SX1278's SCK
    #define MISO    19  // GPIO19 -- SX1278's MISO
    #define MOSI    27  // GPIO27 -- SX1278's MOSI
    #define SS      18  // GPIO18 -- SX1278's CS
    #define RST     14  // GPIO14 -- SX1278's RESET
    #define DI0     26  // GPIO26 -- SX1278's IRQ(Interrupt Request)
    // OLED reset pin
    #define OLED_RESET -1 

  #elif defined(WYMAN_ESP32C6147)
  // Placeholder

  #lif defined(WYMAN_M5STICKCP2)
    // Placeholder

  //// COMPLETE DEVICE DEFINTION
  #endif
//// COMPLETED MAIN CONFIGURATION 
#endif
