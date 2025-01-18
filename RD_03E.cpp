#include "Arduino.h"
#include "HardwareSerial.h"
#include "RD_03E.h"

RD_03E::RD_03E(HardwareSerial& serial)
  : hardwareSerial(serial) {
}

void RD_03E::begin() {
  hardwareSerial.begin(RD_03E_BAUD_RATE);
}

void RD_03E::begin(unsigned long baudRate) {
  hardwareSerial.begin(baudRate);
}

void RD_03E::begin(int rxPin, int txPin) {
  RD_03E::begin(RD_03E_BAUD_RATE, rxPin, txPin);
}

void RD_03E::begin(unsigned long baudRate, int rxPin, int txPin) {
#if defined(ESP32)
  hardwareSerial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
#else
  RD_03E::begin(baudRate);
#endif
}


void RD_03E::run() {

  static uint8_t RX_count = 0;

  // Capture all bytes until the buffer is full or no more data is available
  while (hardwareSerial.available() && RX_count < sizeof(RX_BUF)) {
    RX_temp = hardwareSerial.read();  // Read a byte
    RX_BUF[RX_count++] = RX_temp;     // Store it in the buffer
  }

  // Process the buffer
  for (int i = 0; i < RX_count - 6; ++i) {
    // Check for a valid packet starting from the current index
    if (RX_BUF[i] == 0xAA && RX_BUF[i + 1] == 0xAA && RX_BUF[i + 5] == 0x55 && RX_BUF[i + 6] == 0x55) {
      uint16_t range = (RX_BUF[i + 4] << 8) | RX_BUF[i + 3];  // Combine distance bytes
      distance = static_cast<float>(range) / 100;             // Convert cm to meters
      status = RX_BUF[i + 2];
      lastSucessfulRead = millis();
      break;
    }
  }
  memset(RX_BUF, 0x00, sizeof(RX_BUF));  // Reset buffer
}

float RD_03E::getDistance() {
  return distance;
}

int RD_03E::getStatus() {
  return status;
}

unsigned long RD_03E::getLastSucessfulRead() {
  return lastSucessfulRead;
}

bool RD_03E::isHumanPresent() {
  return status >= 1;
}
