#pragma once
#include "wy_ic2scan.h"

void ic2scan() {
  Wire.begin(4,15);

  Serial.println("\nI2C Scanner");
  
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      delay(10);
    }
  }
  Serial.println("Scan complete.");
}
