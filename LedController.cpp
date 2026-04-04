#include "LedController.h"

#include <Adafruit_NeoPixel.h>

#ifndef RGB_PIN
#define RGB_PIN 48
#endif

#ifndef NUM_PIXELS
#define NUM_PIXELS 1
#endif

namespace {
Adafruit_NeoPixel gPixels(NUM_PIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

struct Rgb {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

constexpr Rgb kCycleColors[] = {
  {255,   0,   0},
  {  0, 255,   0},
  {  0,   0, 255},
  {255, 255,   0},
  {  0, 255, 255},
  {255,   0, 255},
  {255, 255, 255},
  {  0,   0,   0},
};
}

void LedController::begin() {
  gPixels.begin();
  gPixels.setBrightness(255);
  gPixels.show();
  colorIndex_ = -1;
}

void LedController::setState(State state) {
  switch (state) {
    case State::OFF: off(); return;
    case State::RED: setColor_(255, 0, 0); return;
    case State::GREEN: setColor_(0, 255, 0); return;
    case State::BLUE: setColor_(0, 0, 255); return;
    case State::YELLOW: setColor_(255, 255, 0); return;
    case State::MAGENTA: setColor_(255, 0, 255); return;
    case State::CYAN: setColor_(0, 255, 255); return;
    case State::WHITE: setColor_(255, 255, 255); return;
  }
}

void LedController::setRed() {
  setState(State::RED);
}

void LedController::setGreen() {
  setState(State::GREEN);
}

void LedController::setBlue() {
  setState(State::BLUE);
}

void LedController::setYellow() {
  setState(State::YELLOW);
}

void LedController::setMagenta() {
  setState(State::MAGENTA);
}

void LedController::setCyan() {
  setState(State::CYAN);
}

void LedController::setWhite() {
  setState(State::WHITE);
}

bool LedController::handleCommand(const String& rawCmd, String* response) {
  String cmd = rawCmd;
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "cycle" || cmd == "rotate") {
    colorIndex_ = (colorIndex_ + 1) % (int)(sizeof(kCycleColors) / sizeof(kCycleColors[0]));
    const Rgb& c = kCycleColors[colorIndex_];
    setColor_(c.r, c.g, c.b);
    if (response) {
      *response = String("[LED] cycle idx=") + colorIndex_ +
                  " rgb=(" + c.r + "," + c.g + "," + c.b + ")";
    }
    return true;
  }

  if (cmd == "off") {
    off();
    if (response) *response = "[LED] off";
    return true;
  }

  if (cmd == "red") {
    setRed();
    if (response) *response = "[LED] red";
    return true;
  }

  if (cmd == "green") {
    setGreen();
    if (response) *response = "[LED] green";
    return true;
  }

  if (cmd == "blue") {
    setBlue();
    if (response) *response = "[LED] blue";
    return true;
  }

  if (cmd == "yellow") {
    setYellow();
    if (response) *response = "[LED] yellow";
    return true;
  }

  if (cmd == "magenta") {
    setMagenta();
    if (response) *response = "[LED] magenta";
    return true;
  }

  if (cmd == "cyan" || cmd == "bluegreen") {
    setCyan();
    if (response) *response = "[LED] cyan";
    return true;
  }

  if (cmd == "white") {
    setWhite();
    if (response) *response = "[LED] white";
    return true;
  }

  return false;
}

void LedController::off() {
  setColor_(0, 0, 0);
}

void LedController::setColor_(uint8_t r, uint8_t g, uint8_t b) {
  gPixels.setPixelColor(0, gPixels.Color(r, g, b));
  gPixels.show();
}
