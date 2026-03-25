#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class WiFiOtaWebSerial {
public:
  struct Config {
    const char* ssid       = nullptr;
    const char* pass       = nullptr;

    const char* hostname   = "esp32";
    const char* otaPassword = "";     // empty => no password
    uint16_t    port       = 80;

    // Web log ring buffer size
    size_t      logBufferBytes = 16 * 1024;

    // Service task
    uint32_t    serviceDelayMs = 10;
    uint32_t    wifiTaskStack  = 6144;   // bytes
    UBaseType_t wifiTaskPrio   = 1;
    BaseType_t  wifiCore       = 0;
  };

  WiFiOtaWebSerial();
  ~WiFiOtaWebSerial();

  bool begin(const Config& cfg);
  void end();

  // In this design, WiFi/OTA/Web runs in pinned task. Call this in loop if you want (no-op).
  void loopNoService();

  // Logging API
  void print(const String& s, bool mirrorToSerial = true);
  void println(const String& s, bool mirrorToSerial = true);
  void clear();

  // Info
  String ip() const;

private:
  class WebServerWrapper;   // forward declaration
  WebServerWrapper* web_ = nullptr;

  Config cfg_;
  bool started_ = false;

  // log buffer + mutex
  String log_;
  SemaphoreHandle_t logMutex_ = nullptr;

  // FreeRTOS task
  TaskHandle_t wifiTask_ = nullptr;

  bool atLineStart_ = true;
  
  volatile bool otaInProgress = false;

private:
  static void wifiTaskThunk_(void* arg);
  void wifiTaskLoop_();

  void setupWiFi_();
  void setupOta_();
  void setupWeb_();

  void appendLog_(const String& s);
};
