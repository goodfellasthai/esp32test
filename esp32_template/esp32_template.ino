#pragma once
#include "wyconfig.h"

void setup() {
  // Initialize Serial communication at 115200 baud rate
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Serial Initialized for " + String(HARDWARE_NAME));
  
  // Test unimportant procedure for project structure verification
  ic2scan();

}

void loop() {
  #if defined(WYMAN_LORAV2)
    // Placeholder

  #elif defined(WYMAN_LORAV3)
    // Placeholder

  #elif defined(WYMAN_M5STICKCP2)
    // Placeholder

  #elif defined(WYMAN_ESP32C6147)
    // Placeholder

  #endif

}
