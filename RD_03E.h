#pragma once
#include "HardwareSerial.h"

#define RD_03E_BAUD_RATE 256000

class RD_03E {
private:
  HardwareSerial& hardwareSerial;
  float distance;
  int status = 0;
  uint8_t RX_BUF[64] = { 0 };
  uint8_t RX_count = 0;
  uint8_t RX_temp;

public:
  RD_03E(HardwareSerial& serial);

  void begin();
  void begin(unsigned long baudRate);
  void begin(int rxPin, int txPin);
  void begin(unsigned long baudRate, int rxPin, int txPin);

  void run();

  float getDistance();

  int getStatus();

  bool isHumanPresent();
};
