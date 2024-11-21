#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// OLED display dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
// I2C address for the OLED
#define OLED_ADDRESS 0x3C // Replace with your OLED's I2C address
// PINS for the OLED LoRaV2 I2C
#define OLED_RESET 16 // Set to -1 if not used LoRaV2
#define OLED_SDA 4 // LoRaV2
#define OLED_SCL 15 // LoRaV2
// Create an SSD1306 object for I2C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);
    Serial.println("Initializing OLED...");
    // Set custom SDA and SCL pins for I2C
    Wire.begin(OLED_SDA, OLED_SCL); 
    // Initialize the display with the specified I2C address
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;); // Loop forever if initialization fails
    }
    // Clear the buffer
    display.clearDisplay();
    // Display a "Hello World" message
    display.setTextSize(1);              // Set text size (1 = small, 2 = medium, etc.)
    display.setTextColor(SSD1306_WHITE); // Set text color to white
    display.setCursor(0, 0);             // Set cursor to top-left corner
    display.println(F("Hello, LoRa V2 I2C!")); // Print text
    display.display();                   // Send buffer to the display

    //delay(2000); // Pause for 2 seconds
}

void loop() {
    // Clear the display and show another message
    //display.clearDisplay();
    //display.setCursor(0, 0);
    display.println(F("Display working!"));
    display.display();

    delay(10000); // Update every 10 seconds
}
