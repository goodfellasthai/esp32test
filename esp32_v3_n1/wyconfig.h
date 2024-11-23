#pragma once
#ifndef wyconfig_h
// Called by all module headers
  #define wyconfig_h

  #define ENGLISH_POTATO

  //// BOARD TARGETS (COMMENT OUT AS NEEDED)
  //#define WYMAN_LORAV2
  #define WYMAN_LORAV3
  //#define WYMAN_M5STICKCP2
  //#define WYMAN_ESP32C6147

  //// LORA NODE NUMBER (COMMENT OUT AS NEEDED)
  #define WY_NODE_01
  //#define WY_NODE_02

  //// LORA DEBUG OPTIONS (COMMENT OUT AS NEEDED)
  //#define LORA_DEBUG
  //#define BATTERY_DEBUG
  #define ESPNOW_DEBUG

  //// GLOBAL INCLUDES BASED ON BOARD ALWAYS INCLUDE ARDUINO FOR INO SERIAL OR OTHER FUNCS
  #include <Arduino.h>
  #if defined(WYMAN_LORAV2)
    #if defined(WY_NODE_01) || defined(WY_NODE_02)
       #include "wy_v2_node_master.h"
    #endif
  #elif defined(WYMAN_LORAV3)
    #if defined(WY_NODE_01) || defined(WY_NODE_02)
       #include "wy_v3_node_master.h"
    #endif
  #elif defined(WYMAN_M5STICKCP2)
    #include "m5/wy_cp2.h"
  #elif defined(WYMAN_ESP32C6147)
    // Placeholder
  #endif

  //// GLOBAL DECLARATIONS
  #define WYMAN_VERSION "v1.0.0"

  //// HARDWARE NAMES
  #if defined(WYMAN_LORAV2)
    #define HARDWARE_NAME "LoRa ESP32 V2 Semtech SX1276 915mhz 0.96 OLED 64x128"
  #elif defined(WYMAN_LORAV3)
    #define HARDWARE_NAME "LoRa ESP32-S3 V3 Semtech SX1278 433mhz 0.96 OLED 64x128"
  #elif defined(WYMAN_M5STICKCP2)
    #define HARDWARE_NAME "M5StickC Plus2 1.14 TFT 135x240"  
  #elif defined(WYMAN_ESP32C6147)
    #define HARDWARE_NAME "ESP32-C6FH4 1.47 TFT 172x320 SD"
  #endif

  //// DEVICE DEFINITIONS
  #if defined(WYMAN_LORAV2)
    #if defined(WY_NODE_01)
      #define MAC_ADDRESS {0xA0, 0xDD, 0x6C, 0x98, 0x13, 0xF0}
      #define LOCAL_ADDRESS 0x21
      #define BC_LORA 0xFF
      #define BC_ESPN {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
    #elif defined(WY_NODE_02)
      #define MAC_ADDRESS {0x22, 0x22, 0x22, 0x22, 0x22, 0x22}
      #define LOCAL_ADDRESS 0x22
      #define BC_LORA 0xFF
      #define BC_ESPN {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
    #endif
  #elif defined(WYMAN_LORAV3)
    #if defined(WY_NODE_01)
      #define MAC_ADDRESS {0x64, 0xE8, 0x33, 0x69, 0x09, 0xF8}
      #define LOCAL_ADDRESS 0x31
      #define BC_LORA 0xFF
      #define BC_ESPN {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
    #elif defined(WY_NODE_02)
      #define MAC_ADDRESS {0x32, 0x32, 0x32, 0x32, 0x32, 0x32}
      #define LOCAL_ADDRESS 0x32
      #define BC_LORA 0xFF
      #define BC_ESPN {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
    #endif
  #elif defined(WYMAN_M5STICKCP2)
    // Placeholder

  #elif defined(WYMAN_ESP32C6147)
    // Placeholder

  //// COMPLETE DEVICE DEFINTION
  #endif

  // MAC DIRECTORY
  #define V2_NODE_01 {0xA0, 0xDD, 0x6C, 0x98, 0x13, 0xF0}
  #define V2_NODE_02 {0x22, 0x22, 0x22, 0x22, 0x22, 0x22}
  #define V3_NODE_01 {0x64, 0xE8, 0x33, 0x69, 0x09, 0xF8}
  #define V3_NODE_02 {0x32, 0x32, 0x32, 0x32, 0x32, 0x32}

//// COMPLETED MAIN CONFIGURATION 
#endif
