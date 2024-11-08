void setup() {
  // Start the serial communication
  Serial.begin(115200);
  // Wait for the serial port to connect. Needed for native USB port only
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  Serial.println("Serial input echo initialized. Type something:");
}

void loop() {
  // Check if data is available to read from the serial port
  if (Serial.available() > 0) {
    // Read the incoming byte
    char incomingByte = Serial.read();

    // Echo the received byte back to the serial monitor
    Serial.print("You typed: ");
    Serial.println(incomingByte);
  }
}
