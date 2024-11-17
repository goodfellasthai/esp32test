#pragma once
#ifndef wyconfig_h

  #define wyconfig_h

  #define ENGLISH_POTATO

  //// GLOBAL INCLUDES
  #include <Arduino.h>

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
  #elif defined(WYMAN_M5STICKCP2)
    #define HARDWARE_NAME "M5StickC Plus2 1.14 TFT 135x240"  
  #elif defined(WYMAN_ESP32C6147)
    #define HARDWARE_NAME "ESP32-C6FH4 1.47 TFT 172x320 SD"
  #endif

  //// DEVICE DEFINITIONS
  #if defined(WYMAN_LORAV2)
    // Placeholder

  #elif defined(WYMAN_LORAV3)
    // Placeholder

  #elif defined(WYMAN_M5STICKCP2)
    // Placeholder

  #elif defined(WYMAN_ESP32C6147)
    // Placeholder

  //// COMPLETE DEVICE DEFINTION
  #endif
//// COMPLETED MAIN CONFIGURATION 
#endif
