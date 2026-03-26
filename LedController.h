#pragma once

#include <Arduino.h>

class LedController {
public:
  void begin();
  bool handleCommand(const String& cmd, String* response = nullptr);
  void off();

private:
  void setColor_(uint8_t r, uint8_t g, uint8_t b);

  int colorIndex_ = -1;
};
