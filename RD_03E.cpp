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

  static int rxCount = 0;
  int dataCount = hardwareSerial.available();

  while (dataCount--) {
    uint8_t rxByte = hardwareSerial.read();
    rxBuffer[rxCount] = rxByte;
    rxCount = (rxCount + 1) % rxBufferSize;

    if (rxCount >= 7) {
      int startIndex = (rxCount - 7);
      if (rxBuffer[startIndex] == 0xAA && rxBuffer[(startIndex + 1)] == 0xAA && rxBuffer[(startIndex + 5)] == 0x55 && rxBuffer[(startIndex + 6)] == 0x55) {
        uint16_t range = (rxBuffer[(startIndex + 4)] << 8) | rxBuffer[(startIndex + 3)];
        distance = static_cast<float>(range) / 100;
        status = rxBuffer[startIndex + 2];
        lastSucessfulRead = millis();
        break;
      }
    }
  }
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