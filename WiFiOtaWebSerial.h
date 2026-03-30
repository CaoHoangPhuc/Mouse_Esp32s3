#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class WiFiOtaWebSerial {
public:
  using LedCommandHandler = bool (*)(const String& cmd, String& response);
  using TelnetReconnectHandler = bool (*)();
  using HealthJsonProvider = String (*)();
  using SerialOutputAllowedFn = bool (*)();

  struct Config {
    const char* ssid       = nullptr;
    const char* pass       = nullptr;

    const char* hostname   = "esp32";
    uint16_t    port       = 80;
    bool        enableWeb  = true;
    uint16_t    debugTcpPort = 2323;
    uint16_t    uploadPort = 82;
    bool        enableUploadWeb = true;

    // Web log ring buffer size
    size_t      logBufferBytes = 16 * 1024;

    // Service task
    uint32_t    serviceDelayMs = 10;
    uint32_t    wifiTaskStack  = 6144;   // bytes
    UBaseType_t wifiTaskPrio   = 1;
    BaseType_t  wifiCore       = 0;
    uint32_t    wifiConnectTimeoutMs = 5000;
    uint32_t    wifiReconnectIntervalMs = 1000;
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
  void setLedCommandHandler(LedCommandHandler handler) { ledCommandHandler_ = handler; }
  void setTelnetReconnectHandler(TelnetReconnectHandler handler) { telnetReconnectHandler_ = handler; }
  void setHealthJsonProvider(HealthJsonProvider provider) { healthJsonProvider_ = provider; }
  void setSerialOutputAllowedHandler(SerialOutputAllowedFn fn) { serialOutputAllowedFn_ = fn; }

  // Info
  String ip() const;
  bool isOtaInProgress() const { return otaInProgress; }
  bool isUpdateInProgress() const { return otaInProgress || webUploadInProgress_ || rebootPending_; }

private:
  class WebServerWrapper;   // forward declaration
  class UploadWsWrapper;
  WebServerWrapper* web_ = nullptr;
  WebServerWrapper* uploadWeb_ = nullptr;
  UploadWsWrapper* uploadWs_ = nullptr;

  Config cfg_;
  bool started_ = false;

  // log buffer + mutex
  String log_;
  SemaphoreHandle_t logMutex_ = nullptr;

  // FreeRTOS task
  TaskHandle_t wifiTask_ = nullptr;

  bool atLineStart_ = true;
  
  volatile bool otaInProgress = false;
  volatile bool webUploadInProgress_ = false;
  volatile bool rebootPending_ = false;
  uint32_t rebootAtMs_ = 0;
  LedCommandHandler ledCommandHandler_ = nullptr;
  TelnetReconnectHandler telnetReconnectHandler_ = nullptr;
  HealthJsonProvider healthJsonProvider_ = nullptr;
  SerialOutputAllowedFn serialOutputAllowedFn_ = nullptr;

private:
  static void wifiTaskThunk_(void* arg);
  void wifiTaskLoop_();

  void setupWiFi_();
  void configureSta_();
  void restartSta_();
  void onWiFiConnected_();
  bool ensureWiFiConnected_(uint32_t timeoutMs);
  void setupOta_();
  void setupWeb_();
  void setupUploadWeb_();
  void setupUploadWs_();
  void serviceUploadWs_();
  bool handleUploadWsHandshake_();
  bool readUploadWsFrameHeader_(uint8_t& opcode, uint64_t& len, bool& masked, uint8_t mask[4]);
  bool readUploadWsTextFrame_(String& payload, uint8_t opcode, uint64_t len, bool masked, const uint8_t mask[4]);
  bool handleUploadWsBinaryFrame_(uint64_t len, bool masked, const uint8_t mask[4]);
  void sendUploadWsText_(const String& text);
  uint16_t uploadWsPort_() const { return (uint16_t)(cfg_.uploadPort + 2); }
  void setLedState_(const String& cmd);
  void serviceUpdateLed_();
  String extractBatteryText_(const String& json) const;

  void appendLog_(const String& s);

  uint32_t ledBlinkMs_ = 0;
  bool ledBlinkOn_ = false;
  bool chunkUploadActive_ = false;
  size_t chunkExpectedOffset_ = 0;
  size_t chunkTotalSize_ = 0;
  size_t chunkRequestOffset_ = 0;
  size_t chunkRequestBytes_ = 0;
  bool chunkRequestOk_ = false;
  bool chunkRequestSkip_ = false;
  String chunkRequestError_;
  uint32_t uploadWsLastActivityMs_ = 0;
};
