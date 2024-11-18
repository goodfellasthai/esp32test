#include "wy_generic.h"
#if defined(WYMAN_LORA_GENERIC)
#pragma once
#include <HT_SSD1306Wire.h>

// Initialize the display
SSD1306Wire generic_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// Constants for battery voltage calculation
const float MAX_BATTERY_VOLTAGE = 4.2; // Maximum voltage of a fully charged battery
const float MIN_BATTERY_VOLTAGE = 3.0; // Minimum voltage of a discharged battery

unsigned long lastActivityTime = 0; // To track the last button press
const unsigned long inactivityTimeout = 10000; // 10 seconds inactivity threshold
bool isDisplayOn = true; // Track the display power state

// Function prototypes
void VextON(void);
void VextOFF(void);
bool isButtonPressed(); // Detect button activity
float readBatteryVoltage(); // Read battery voltage
int calculateBatteryPercentage(float voltage); // Calculate battery percentage
void drawBatteryStatus(); // Draw battery status on the display
void updateDisplay(); // Updates the display dynamically

void logo() {
    generic_display.clear();
    //generic_display.drawXbm(0, 5, logo_width, logo_height, (const unsigned char *)logo_bits);
    generic_display.drawXbm(0, 5, wy_width, wy_height, (const unsigned char *)wy_bits);
    generic_display.display();
}

// Allows power to the OLED
void VextON(void) {
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);
}

// Stops power to the OLED
void VextOFF(void) {
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, HIGH);
}

// Check if a button is pressed
bool isButtonPressed() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    return digitalRead(BUTTON_PIN) == LOW;
}

// Read battery voltage
float readBatteryVoltage() {
    int raw = analogRead(BATTERY_PIN);
    float voltage = raw * (3.3 / 4095.0) * 2; // Assuming a voltage divider with equal resistors
    return voltage;
}

// Calculate battery percentage
int calculateBatteryPercentage(float voltage) {
    if (voltage >= MAX_BATTERY_VOLTAGE) return 100;
    if (voltage <= MIN_BATTERY_VOLTAGE) return 0;
    return (int)((voltage - MIN_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE) * 100);
}

// Draw battery status on the display
void drawBatteryStatus() {
    float voltage = readBatteryVoltage();
    int percentage = calculateBatteryPercentage(voltage);

    // Battery rectangle dimensions
    int rectWidth = 20;    // Width of the battery rectangle
    int rectHeight = 10;   // Height of the battery rectangle
    int rectX = 128 - rectWidth - 2;  // X position (aligned to the right)
    int rectY = 2;                    // Y position (top of the display)

    // Calculate the width of the filled portion based on the percentage
    int fillWidth = (percentage * rectWidth) / 100;

    // Flashing logic for low battery
    static unsigned long lastFlashTime = 0;
    static bool isVisible = true;
    unsigned long currentMillis = millis();

    if (percentage < 5) {
        if (currentMillis - lastFlashTime >= 1000) { // Flash every second
            isVisible = !isVisible; // Toggle visibility
            lastFlashTime = currentMillis;
        }
    } else {
        isVisible = true; // Always visible if battery is not low
    }

    // Draw the battery rectangle only if visible
    if (isVisible) {
        // Draw the outline
        generic_display.drawRect(rectX, rectY, rectWidth, rectHeight);

        // Fill the battery proportionally
        if (fillWidth > 0) {
            generic_display.fillRect(rectX + 1, rectY + 1, fillWidth, rectHeight - 2);
        }
    }

    // Always show the battery percentage text
    String batteryText = String(percentage) + "%";
    int textWidth = generic_display.getStringWidth(batteryText);
    int textX = rectX - textWidth - 5; // Position the text to the left of the rectangle
    int textY = 0;                     // Align to the top of the screen
    generic_display.drawString(textX, textY, batteryText);
}

// Updates the display dynamically
void updateDisplay() {
    generic_display.clear();

    // Display "Hello World"
    generic_display.setFont(ArialMT_Plain_10);
    generic_display.drawString(0, 25, "Hello World");

    // Update and display battery status
    drawBatteryStatus();

    // Refresh the display
    generic_display.display();
}

void turnDisplayOn() {
    if (!isDisplayOn) {
        VextON(); // Turn on power to the display
        generic_display.init(); // Reinitialize the display
        isDisplayOn = true;
    }
}

void turnDisplayOff() {
    if (isDisplayOn) {
        generic_display.clear();
        generic_display.display(); // Clear display content
        VextOFF(); // Turn off power to the display
        isDisplayOn = false;
    }
}

void wy_generic_setup() {
    VextON();
    generic_display.init();

    // Show logo for 5 seconds
    logo();
    delay(5000);

    // Reset activity timer
    lastActivityTime = millis();
}

void wy_generic_loop() {
    static unsigned long lastUpdateTime = 0; // Track last display update time
    const unsigned long updateInterval = 100; // Refresh every 100ms

    // Check for button activity
    if (isButtonPressed()) {
        lastActivityTime = millis(); // Reset activity timer
        turnDisplayOn();             // Turn the display back on if it's off
    }

    // Turn off the display if inactive for 10 seconds
    if (millis() - lastActivityTime > inactivityTimeout) {
        turnDisplayOff();
    }

    // Update the display periodically if it's on
    if (isDisplayOn && millis() - lastUpdateTime > updateInterval) {
        updateDisplay();
        lastUpdateTime = millis(); // Update the timestamp
    }

    // LoRa and Wi-Fi functionalities will continue to work here
    // Insert LoRa/Wi-Fi-related code below
}

#endif
