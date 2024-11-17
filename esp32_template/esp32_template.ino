#include "wyconfig.h"

void setup() {
  // Initialize Serial communication at 115200 baud rate
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Serial Initialized for " + String(HARDWARE_NAME));
  
}

void loop() {
  #if defined(WYMAN_LORAV2)
    // Placeholder    
  #endif

  #if defined(WYMAN_LORAV2)
    // Placeholder    
  #endif
}
