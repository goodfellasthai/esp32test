#pragma once

#ifndef wyconfig_h

  #define wyconfig_h

  #define ENGLISH_POTATO

  //// GLOBAL INCLUDES
  #include <Arduino.h>

  //// BOARD TARGETS (COMMENT OUT AS NEEDED)
  #define WYMAN_ESP32C6147
  //#define WYMAN_M5STICKCP2

  //// GLOBAL DECLARATIONS
  #define WYMAN_VERSION "v1.0.0"

  //// HARDWARE NAMES
  #ifdef WYMAN_M5STICKCP2
    #define HARDWARE_NAME "M5StickC Plus2"
  #elif defined(WYMAN_ESP32C6147)
    #define HARDWARE_NAME "ESP32-C6FH4 1.47LCD SD"
  #endif

  //// BOARD FEATURES
  #if defined(WYMAN_M5STICKCP2)
    #define HAS_PWR_MGMT
    #define HAS_BUTTONS
    #define HAS_SCREEN
    #define HAS_SD
  #elif defined(WYMAN_ESP32C6147)
    #define HAS_BUTTONS
    #define HAS_SCREEN
    #define HAS_SD
  #endif

  //// POWER MANAGEMENT
  #ifdef HAS_PWR_MGMT
    #if defined(WYMAN_M5STICKCP2)
      #include "AXP192.h"
      //// Prevent StickCP2 from turning off when disconnect USB cable
      #define POWER_HOLD_PIN 4
    #elif defined(WYMAN_ESP32C6147)
      //// No power management
    #endif
  #endif

  //// BUTTON DEFINITIONS
  #ifdef HAS_BUTTONS
    #if defined(WYMAN_M5STICKCP2)
      #define A_BTN 37
      #define B_BTN 39
      #define C_BTN 35
    #elif defined(WYMAN_ESP32C6147)
      #define A_BTN 0
    #endif
  #endif

  //// DISPLAY DEFINITIONS
  #ifdef HAS_SCREEN
    #if defined(WYMAN_M5STICKCP2)
      #define LARGESPIFFS
      #define TFT_MISO -1  // Master in only required for touch or input i.e. SD
      #define TFT_MOSI 15 // Master out only required not touch
      #define TFT_SCLK 13
      #define TFT_CS 5
      #define TFT_DC 14
      #define TFT_RST 12
      #define TFT_BL 27
      #define TFT_TOUCH_CS -1
      #define TFT_WIDTH 135
      #define TFT_HEIGHT 240
      #define TFT_SPI_FREQUENCY  40000000
      #define TFT_PWM_FREQUENCY 1000
      #define TFT_PWM_RESOLUTION 10
      #define TFT_HORIZONTAL 1
      #define TFT_VERTICAL 0
      #define TFT_OFFSET_X 34
      #define TFT_OFFSET_Y 0    
     #elif defined(WYMAN_ESP32C6147)
      #define TFT_MISO -1  // Master in only required for touch or input i.e. SD
      #define TFT_MOSI 6 // Master out only required not touch
      #define TFT_SCLK 7
      #define TFT_CS 14
      #define TFT_DC 15
      #define TFT_RST 21
      #define TFT_BL 22
      #define TFT_TOUCH_CS -1
      #define TFT_WIDTH 172
      #define TFT_HEIGHT 320
      #define TFT_SPI_FREQUENCY  40000000
      #define TFT_PWM_FREQUENCY 1000
      #define TFT_PWM_RESOLUTION 10
      #define TFT_HORIZONTAL 1
      #define TFT_VERTICAL 0
      #define TFT_OFFSET_X 34
      #define TFT_OFFSET_Y 0    
    #endif
  #endif

  //// SD DEFINITIONS
  #ifdef HAS_SD
    #if defined(WYMAN_M5STICKCP2)
      #define LARGESPIFFS // Defines the device has having large SPIFFS to check if no SD card present
      #define SD_MISO -1 // Assume no grove SD
      #define SD_CS -1  // Assume no grove SD
    #elif defined(WYMAN_ESP32C6147)
      #define SD_CS 4    
      #define SD_MISO 5  // Master in only required for touch or input i.e. SD
    #endif
  #endif

//// COMPLETED MAIN CONFIGURATION 
#endif