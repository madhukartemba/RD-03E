#pragma once
#include "HardwareSerial.h"

#define RD_03E_BAUD_RATE 256000

class RD_03E {
private:
  HardwareSerial& hardwareSerial;
  float distance;
  unsigned long lastSucessfulRead = 0;
  int status = 0;
  static constexpr int rxBufferSize = 32;
  uint8_t rxBuffer[rxBufferSize] = { 0 };

public:
  RD_03E(HardwareSerial& serial);

  void begin();
  void begin(unsigned long baudRate);
  void begin(int rxPin, int txPin);
  void begin(unsigned long baudRate, int rxPin, int txPin);

  void run();

  float getDistance();

  int getStatus();

  unsigned long getLastSucessfulRead();

  bool isHumanPresent();
};