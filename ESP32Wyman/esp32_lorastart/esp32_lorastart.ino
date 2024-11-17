#include "wyconfig.h"
// Define the display object
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// Define the counter variable
int counter = 0;

void setup() {
  // Initialize Serial communication at 115200 baud rate
  Serial.begin(115200);
  while (!Serial);

  #if defined(WYMAN_LORAV2)
    //ic2scan();

    // Initialize OLED display
    Wire.begin(4, 15); // SDA on GPIO 8, SCL on GPIO 9

    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
      Serial.println("OLED initialization failed!");
      while (true); // Stop program if OLED initialization fails
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Display and Serial Initialized. LoRa v2");
    display.display();

    // Setup LoRa transceiver module with the pins
    LoRa.setPins(SS, RST, DI0);
    
    if (!LoRa.begin(915E6)) { // Initialize LoRa module to 915MHz
      Serial.println("Starting LoRa failed!");
      display.setCursor(0, 10);
      display.println("LoRa init failed!");
      display.display();
      while (true); // Stop program if LoRa initialization fails
    }

    display.setCursor(0, 10);
    display.println("LoRa Initialized");
    display.display();

  #elif defined(WYMAN_LORAV3)
    Serial.println("Serial Initialized. LoRa v3");

  #endif
}

void loop() {
  #if defined(WYMAN_LORAV2)
    // Display and Serial Print
    Serial.print("Sending packet: ");
    Serial.println(counter);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("LoRa Sender");
    display.setCursor(0, 10);
    display.print("Packet: ");
    display.println(counter);
    display.display();

    // Send LoRa packet
    LoRa.beginPacket();
    LoRa.print("Hello LoRa v3 from LoRa v2 Packet: ");
    LoRa.print(counter);
    LoRa.endPacket();

    counter++; // Increment counter

    delay(2000); // Delay 2 seconds before next packet
    
  #endif
}
