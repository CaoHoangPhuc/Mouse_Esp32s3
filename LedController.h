#pragma once

#include <Arduino.h>

class LedController {
public:
  enum class State : uint8_t {
    OFF = 0,
    RED,
    GREEN,
    BLUE,
    YELLOW,
    MAGENTA,
    CYAN,
    WHITE
  };

  void begin();
  bool handleCommand(const String& cmd, String* response = nullptr);
  void setState(State state);
  void off();
  void setCyan();
  void setWhite();
  void setRed();
  void setGreen();
  void setBlue();
  void setYellow();
  void setMagenta();

private:
  void setColor_(uint8_t r, uint8_t g, uint8_t b);

  int colorIndex_ = -1;
};
