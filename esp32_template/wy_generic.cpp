#include "wy_generic.h"
#if defined(WYMAN_LORA_GENERIC)
#pragma once
#include <HT_SSD1306Wire.h>

SSD1306Wire generic_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

unsigned long lastActivityTime = 0; // To track the last button press
const unsigned long inactivityTimeout = 10000; // 10 seconds inactivity threshold
bool isDisplayOn = true; // Track the display power state

// Function prototypes
void VextON(void);
void VextOFF(void);
bool isButtonPressed(); // Detect button activity
void showHelloWorld(); // Function to display "Hello World"

void logo() {
    generic_display.clear();
    generic_display.drawXbm(0, 5, logo_width, logo_height, (const unsigned char *)logo_bits);
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
    // Replace `BUTTON_PIN` with the GPIO number for the button
    pinMode(BUTTON_PIN, INPUT_PULLUP); 
    return digitalRead(BUTTON_PIN) == LOW;
}

// Function to display "Hello World"
void showHelloWorld() {
    generic_display.clear();
    generic_display.drawString(0, 25, "Hello World");
    generic_display.display();
}

void turnDisplayOn() {
    if (!isDisplayOn) {
        VextON(); // Turn on power to the display
        generic_display.init(); // Reinitialize the display
        showHelloWorld(); // Display "Hello World"
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

    // Display "Hello World" using the dedicated function
    showHelloWorld();

    // Reset activity timer
    lastActivityTime = millis();
}

void wy_generic_loop() {
    // Check for button activity
    if (isButtonPressed()) {
        lastActivityTime = millis(); // Reset activity timer
        turnDisplayOn(); // Turn the display back on if it's off
    }

    // Turn off the display if inactive for 10 seconds
    if (millis() - lastActivityTime > inactivityTimeout) {
        turnDisplayOff();
    }

    // LoRa and Wi-Fi functionalities will continue to work here
    // Insert LoRa/Wi-Fi-related code below
}

#endif
