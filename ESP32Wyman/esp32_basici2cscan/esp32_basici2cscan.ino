#include <Arduino.h>
#include <esp_system.h>

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    // Wait for Serial to initialize
  }
  Serial.println("\nESP32 Chip Information:");

  // Retrieve chip information
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);

  // Display chip model
  Serial.print("Model: ");
  switch (chip_info.model) {
    case CHIP_ESP32:
      Serial.println("ESP32");
      break;
    case CHIP_ESP32S2:
      Serial.println("ESP32-S2");
      break;
    case CHIP_ESP32S3:
      Serial.println("ESP32-S3");
      break;
    case CHIP_ESP32C3:
      Serial.println("ESP32-C3");
      break;
    case CHIP_ESP32H2:
      Serial.println("ESP32-H2");
      break;
    default:
      Serial.println("Unknown");
      break;
  }

  // Display number of cores
  Serial.print("Cores: ");
  Serial.println(chip_info.cores);

  // Display chip revision
  Serial.print("Revision: ");
  Serial.println(chip_info.revision);

  // Display flash size
  Serial.print("Flash Size: ");
  Serial.print(ESP.getFlashChipSize() / (1024 * 1024));
  Serial.println(" MB");

  // Display features
  Serial.print("Features: WiFi");
  if (chip_info.features & CHIP_FEATURE_BT) {
    Serial.print("/BT");
  }
  if (chip_info.features & CHIP_FEATURE_BLE) {
    Serial.print("/BLE");
  }
  if (chip_info.features & CHIP_FEATURE_EMB_FLASH) {
    Serial.print(", Embedded Flash");
  }
  if (chip_info.features & CHIP_FEATURE_EMB_PSRAM) {
    Serial.print(", Embedded PSRAM");
  }
  Serial.println();
}

void loop() {
  // No actions needed in loop
}
