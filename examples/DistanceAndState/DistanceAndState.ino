#include <Arduino.h>
#include <RD_03E.h>

// Create an instance of the RD_03E class
RD_03E rd03e(Serial1);

void setup() {
  // Initialize the Serial Monitor for debugging
  Serial.begin(115200);


  Serial.println("Initializing RD-03E sensor...");

  // Initialize the RD-03E sensor with default settings
  rd03e.begin();  // Uses default pins (RX -> GPIO16, TX -> GPIO17 for ESP32) and default baud rate of 256000

  // Initialize the RD-03E sensor with specified RX and TX pins
  // rd03e.begin(16, 17);  // RX pin -> GPIO16, TX pin -> GPIO17

  // Initialize the RD-03E sensor with a specified baud rate
  // rd03e.begin(256000); // Baud rate -> 256000

  // Initialize the RD-03E sensor with specified baud rate and RX/TX pins
  // rd03e.begin(256000, 16, 17); // Baud rate -> 256000, RX pin -> GPIO16, TX pin -> GPIO17

  Serial.println("RD-03E sensor initialized!");
}
void loop() {
  // Run the RD_03E processing logic
  rd03e.run();

  // Check if a successful reading is available
  float distance = rd03e.getDistance();
  int status = rd03e.getStatus();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print("m, Status: ");

  switch (status) {
    case 0:
      Serial.print("HUMAN_ABSENT");
      break;
    case 1:
      Serial.print("HUMAN_MOVING");
      break;
    case 2:
      Serial.print("HUMAN_STATIONARY");
      break;
    default:
      Serial.print("UNKNOWN");
      break;
  }

  if (rd03e.isHumanPresent()) {
    Serial.println(" (Human detected)");
  } else {
    Serial.println(" (No human detected)");
  }

  Serial.print("Last successful read time: ");
  Serial.println(rd03e.getLastSucessfulRead());

  delay(100);  // Add a delay to avoid flooding the serial monitor
}
