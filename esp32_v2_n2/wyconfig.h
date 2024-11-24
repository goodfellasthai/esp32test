#pragma once
#ifndef wyconfig_h
// Called by all module headers
  #define wyconfig_h

  #define ENGLISH_POTATO

  //// BOARD TARGETS (COMMENT OUT AS NEEDED)
  #define WYMAN_LORAV2
  //#define WYMAN_LORAV3
  //#define WYMAN_M5STICKCP2
  //#define WYMAN_ESP32C6147

  //// LORA NODE NUMBER (COMMENT OUT AS NEEDED)
  //#define WY_NODE_01
  #define WY_NODE_02

  //// LORA DEBUG OPTIONS (COMMENT OUT AS NEEDED)
  //#define LORA_DEBUG
  //#define BATTERY_DEBUG
  #define ESPNOW_DEBUG

  #include "../master/globals.h"

#endif
