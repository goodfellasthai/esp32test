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

  //// LORA NODE PURPOSE (COMMENT OUT AS NEEDED)
  #define WY_NODE_MASTER
  //#define WY_NODE_DRONE
  //#define WY_NODE_01

  //// GLOBAL INCLUDES BASED ON BOARD ALWAYS INCLUDE ARDUINO FOR INO SERIAL OR OTHER FUNCS
  #include <Arduino.h>
  #if defined(WYMAN_LORAV2)
    #if defined(WY_NODE_MASTER)
       #include "wy_v2_node_master.h"
    #elif defined(WY_NODE_DRONE)
       #include "lorav2/wy_v2_node_drone.h"
    #elif defined(WY_NODE_01)
       #include "lorav2/wy_v2_node_01.h"
    #endif
  #elif defined(WYMAN_LORAV3)
    #if defined(WY_NODE_MASTER)
       #include "wy_v3_node_master.h"
    #elif defined(WY_NODE_DRONE)
       #include "lorav3/wy_v3_node_drone.h"
    #elif defined(WY_NODE_01)
       #include "lorav3/wy_v3_node_01.h"
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
    #define BUTTON_PIN 0
    #define BATTERY_PIN 37 // For v2

  #elif defined(WYMAN_LORAV3)
    #define BUTTON_PIN 0
    #define BATTERY_PIN 13 // For v3 if doesnt work with battery we can try 37

  #elif defined(WYMAN_M5STICKCP2)
    // Placeholder

  #elif defined(WYMAN_ESP32C6147)
    // Placeholder

  //// COMPLETE DEVICE DEFINTION
  #endif
//// COMPLETED MAIN CONFIGURATION 
#endif
