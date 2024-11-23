#ifndef BATTERY_STATUS_H
#define BATTERY_STATUS_H
  struct BatteryStatus {
      uint8_t percentage; // Battery percentage
      float voltage;      // Battery voltage
  };
#endif // BATTERY_STATUS_H

#ifndef ESPN_SENSOR_H
#define ESPN_SENSOR_H
// Define the structure for sensor data
#pragma pack(push, 1)  // Align structure to 1-byte boundaries
typedef struct {
    float gps_latitude;
    float gps_longitude;
    float altitude;
    float pressure;
} SensorData;
#pragma pack(pop)
#endif // ESPN_SENSOR_H
