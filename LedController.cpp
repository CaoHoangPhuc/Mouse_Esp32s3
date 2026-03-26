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
    setColor_(255, 0, 0);
    if (response) *response = "[LED] red";
    return true;
  }

  if (cmd == "green") {
    setColor_(0, 255, 0);
    if (response) *response = "[LED] green";
    return true;
  }

  if (cmd == "blue") {
    setColor_(0, 0, 255);
    if (response) *response = "[LED] blue";
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
