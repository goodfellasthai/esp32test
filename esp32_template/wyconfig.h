#pragma once
#ifndef wyconfig_h
// Called by all module headers
  #define wyconfig_h

  #define ENGLISH_POTATO

  //// BOARD TARGETS (COMMENT OUT AS NEEDED)
  //#define WYMAN_LORAV2
  //#define WYMAN_LORAV3
  #define WYMAN_LORA_GENERIC // Should only require a board change to rebuild no configuration
  //#define WYMAN_M5STICKCP2
  //#define WYMAN_ESP32C6147

  //// GLOBAL INCLUDES BASED ON BOARD ALWAYS INCLUDE ARDUINO FOR INO SERIAL OR OTHER FUNCS
  #include <Arduino.h>
  #include "wy_ic2scan.h" // Not board target dependant so can add or repeat here to test global modules
  #if defined(WYMAN_LORAV2)
    #include "wy_v2_factory.h"
  #elif defined(WYMAN_LORAV3)
    #include "wy_v3_simple.h"
  #elif defined(WYMAN_LORA_GENERIC)
    #include "wy_generic.h"
  #elif defined(WYMAN_M5STICKCP2)
    #include "wy_cp2.h"
  #elif defined(WYMAN_ESP32C6147)
    // Placeholder
  #endif

  //// GLOBAL DECLARATIONS
  #define WYMAN_VERSION "v1.0.0"

  //// HARDWARE NAMES
  #if defined(WYMAN_LORAV2)
    #define HARDWARE_NAME "LoRa ESP32 V2 0.96 OLED 64x128"
  #elif defined(WYMAN_LORAV3)
    #define HARDWARE_NAME "LoRa ESP32 V3 0.96 OLED 64x128"
  #elif defined(WYMAN_LORA_GENERIC)
    #define HARDWARE_NAME "LoRa ESP32 Generic V2/V3"
  #elif defined(WYMAN_M5STICKCP2)
    #define HARDWARE_NAME "M5StickC Plus2 1.14 TFT 135x240"  
  #elif defined(WYMAN_ESP32C6147)
    #define HARDWARE_NAME "ESP32-C6FH4 1.47 TFT 172x320 SD"
  #endif

  //// DEVICE DEFINITIONS
  #if defined(WYMAN_LORAV2)
    #define BUTTON_PIN 0
    #define BATTERY_PIN 37

  #elif defined(WYMAN_LORAV3)
    #define BUTTON_PIN 0
    #define BATTERY_PIN 37

  #elif defined(WYMAN_LORA_GENERIC)
    #define BUTTON_PIN 0
    #define BATTERY_PIN 37

  #elif defined(WYMAN_M5STICKCP2)
    // Placeholder

  #elif defined(WYMAN_ESP32C6147)
    // Placeholder

  //// COMPLETE DEVICE DEFINTION
  #endif
//// COMPLETED MAIN CONFIGURATION 
#endif
