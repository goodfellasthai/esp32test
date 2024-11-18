#pragma once
// Main configuration file only required once and covered by pragma once
#include "wyconfig.h"

void setup() {
  // Initialize Serial communication at 115200 baud rate
  Serial.begin(115200);
  while (!Serial);
  Serial.println();
  Serial.print("Serial Initialized for " + String(HARDWARE_NAME));
  
  // Test unimportant procedure for project structure verification
  ic2scan();

  #if defined(WYMAN_LORAV2)
    // Placeholder
    wy_v2_factory_setup();

  #elif defined(WYMAN_LORAV3)
    // Placeholder
    wy_v3_simple_setup();

  #elif defined(WYMAN_LORA_GENERIC)
    // Placeholder
    wy_generic_setup();
    #if defined(WYMAN_LORA_GENERIC) && defined(WY_NODE_MASTER)
      //Serial.println("BEFORE NODE MASTER SETUP");
      wy_node_master_setup();
    #endif

  #elif defined(WYMAN_M5STICKCP2)
    // Placeholder
    wy_cp2_setup();

  #elif defined(WYMAN_ESP32C6147)
    // Placeholder

  #endif

}

void loop() {
  #if defined(WYMAN_LORAV2)
    wy_v2_factory_loop();

  #elif defined(WYMAN_LORAV3)
    wy_v3_simple_loop();

  #elif defined(WYMAN_LORA_GENERIC)
    wy_generic_loop();
    #if defined(WYMAN_LORA_GENERIC) && defined(WY_NODE_MASTER)
      wy_node_master_loop(); 
    #endif

  #elif defined(WYMAN_M5STICKCP2)
    wy_cp2_loop();

  #elif defined(WYMAN_ESP32C6147)
    // Placeholder

  #endif

}
