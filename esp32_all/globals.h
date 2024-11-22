  #ifndef BATTERY_STATUS_H
  #define BATTERY_STATUS_H
    struct BatteryStatus {
        uint8_t percentage; // Battery percentage
        float voltage;      // Battery voltage
    };
  #endif // BATTERY_STATUS_H