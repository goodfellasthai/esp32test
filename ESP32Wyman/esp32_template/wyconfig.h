#pragma once
#ifndef wyconfig_h

  #define wyconfig_h

  #define ENGLISH_POTATO

  //// GLOBAL INCLUDES
  //#include <Arduino.h>
  #include "heltec_demo.h"

  //// BOARD TARGETS (COMMENT OUT AS NEEDED)
  //#define WYMAN_M5STICKCP2
  //#define WYMAN_ESP32C6147
  #define WYMAN_LORAV3

  //// GLOBAL DECLARATIONS
  #define WYMAN_VERSION "v1.0.0"

  //// HARDWARE NAMES
  #ifdef WYMAN_M5STICKCP2
    #define HARDWARE_NAME "M5StickC Plus2 1.14 TFT 135x240"
  #elif defined(WYMAN_ESP32C6147)
    #define HARDWARE_NAME "ESP32-C6FH4 1.47 TFT 172x320 SD"
  #elif defined(WYMAN_LORAV3)
    #define HARDWARE_NAME "LoRa ESP32 V3 0.96 OLED 64x128"
  #endif

  //// BOARD FEATURES
  #if defined(WYMAN_M5STICKCP2)
    #define HAS_SCREEN
  #elif defined(WYMAN_ESP32C6147)
    #define HAS_SCREEN
  #elif defined(WYMAN_LORAV3)
    #define HAS_SCREEN
  #endif

  //// DISPLAY DEFINITIONS
  #ifdef HAS_SCREEN
    #if defined(WYMAN_M5STICKCP2)
      // Placeholder
     #elif defined(WYMAN_ESP32C6147)
      // Placeholder
    #elif defined(WYMAN_LORAV3)
      // Placeholder
    #endif
  #endif

//// COMPLETED MAIN CONFIGURATION 
#endif