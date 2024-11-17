#pragma once
#ifndef wyconfig_h

  #define wyconfig_h

  #define ENGLISH_POTATO

  //// BOARD TARGETS (COMMENT OUT AS NEEDED)
  #define WYMAN_LORAV2
  //#define WYMAN_LORAV3
  //#define WYMAN_ESP32C6147
  //#define WYMAN_M5STICKCP2

  //// GLOBALINCLUDES BASED ON BOARD
  #include <Arduino.h>
  #include "wy_ic2scan.h" // Not board target dependant so can add here to test global code
  #if defined(WYMAN_LORAV2)
    #include "wy_v2_factory.h"
  #elif defined(WYMAN_LORAV3)
    #include "wy_v3_simple.h"
  #endif

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
