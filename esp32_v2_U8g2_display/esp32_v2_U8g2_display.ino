#include <U8g2lib.h>
#include <SPI.h>

// PINS for the OLED LoRaV2 I2C
#define OLED_RESET 16 // Set to -1 if not used LoRaV2
#define OLED_SDA 4 // LoRaV2
#define OLED_SCL 15 // LoRaV2
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ OLED_RESET, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA);

void setup() {
    u8g2.begin(); // Initialize the display
    u8g2.clearBuffer(); // Clear the internal buffer
    u8g2.setFont(u8g2_font_ncenB08_tr); // Set font
    u8g2.drawStr(0, 24, "Hello, LoRa V2 I2C!"); // Draw string at position (0,24)
    u8g2.sendBuffer(); // Send the buffer to the display
}

void loop() {
    // Nothing to do here
}
