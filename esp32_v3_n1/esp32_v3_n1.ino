#pragma once
#include "wyconfig.h"

void setup() {
  // Initialize Serial communication at 115200 baud rate
  Serial.begin(115200);
  while (!Serial);
  Serial.println();
  Serial.println("Serial Initialized for " + String(HARDWARE_NAME));

  #if defined(WYMAN_LORAV2)
    #if defined(WY_NODE_01) || defined(WY_NODE_02)
       wy_v2_node_master_setup();   
    #endif
  #elif defined(WYMAN_LORAV3)
    #if defined(WY_NODE_01) || defined(WY_NODE_02)
       wy_v3_node_master_setup();
    #endif
  #endif
}

void loop() {
  #if defined(WYMAN_LORAV2)
    #if defined(WY_NODE_01) || defined(WY_NODE_02)
       wy_v2_node_master_loop();   
    #endif
  #elif defined(WYMAN_LORAV3)
    #if defined(WY_NODE_01) || defined(WY_NODE_02)
       wy_v3_node_master_loop();
    #endif
  #endif
}
