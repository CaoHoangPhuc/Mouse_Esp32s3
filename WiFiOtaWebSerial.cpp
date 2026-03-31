#include "WiFiOtaWebSerial.h"
#include "Config.h"

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <esp_err.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>

// Wrapper to keep WebServer out of header
class WiFiOtaWebSerial::WebServerWrapper {
public:
  WebServerWrapper(uint16_t port) : server(port) {}
  WebServer server;
};

static const char* kIndexHtml PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>%HOSTNAME% Control</title>
  <style>
    :root { color-scheme: light; }
    body { font-family: "Segoe UI", Arial, sans-serif; margin: 0; background: #eef3ea; color: #102017; }
    .wrap { max-width: 720px; margin: 0 auto; padding: 24px; }
    .card { background: #fbfdf9; border-radius: 18px; box-shadow: 0 10px 28px rgba(0,0,0,0.08); padding: 22px; }
    h1 { margin: 0 0 8px; font-size: 1.5rem; }
    .sub { color: #4f5f55; margin-bottom: 20px; }
    .meta { display: grid; grid-template-columns: 110px 1fr; gap: 8px 14px; margin-bottom: 20px; }
    .meta b { color: #355040; }
    .row { display:flex; gap:12px; align-items:center; flex-wrap:wrap; }
    h2 { margin: 24px 0 10px; font-size: 1.05rem; color: #1d3b2b; }
    .hint { margin: 0 0 12px; color: #4f5f55; line-height: 1.5; }
    .guide { display: grid; gap: 12px; margin-top: 8px; }
    .group { background: #f4f8f1; border: 1px solid #d9e6d7; border-radius: 14px; padding: 14px; }
    .group h3 { margin: 0 0 8px; font-size: 0.98rem; color: #284735; }
    .group ul { margin: 0; padding-left: 18px; }
    .group li { margin: 6px 0; line-height: 1.4; }
    code { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace; background: #e8efe6; padding: 1px 5px; border-radius: 6px; }
    button { padding: 11px 14px; cursor: pointer; border: 0; border-radius: 12px; font-weight: 600; }
    .primary { background: #1d6b45; color: white; }
    .secondary { background: #1d4f6b; color: white; }
    #status { margin-top: 16px; min-height: 1.3em; color: #44584a; }
  </style>
</head>
<body>
<div class="wrap">
  <div class="card">
    <h1>%HOSTNAME%</h1>
    <div class="sub">Simple robot control page on port 80.</div>
    <div class="meta">
      <b>Hostname</b><span>%HOSTNAME%</span>
      <b>IP</b><span id="robotIp">%IP%</span>
      <b>Telnet</b><span id="telnetTarget">%IP%:%TELNET_PORT%</span>
      <b>Battery</b><span id="batteryNow">%BATTERY_TEXT%</span>
    </div>
    <div class="row">
      <button class="primary" onclick="reconnectTelnet()">Reconnect Telnet</button>
      <button class="secondary" onclick="cycleLed()">Cycle LED</button>
      <button class="secondary" onclick="openFloodfill()">Open Floodfill</button>
      <button class="secondary" onclick="openUpload()">Open Upload</button>
    </div> 
    <div id="status">Ready.</div>
    <h2>How To Use</h2>
    <p class="hint">Use <b>Reconnect Telnet</b> to open the live CLI in your telnet app. The commands below are the main ones the robot accepts, grouped by purpose so you can quickly find the right action.</p>
    <div class="guide">
      <div class="group">
        <h3>Run Control</h3>
        <ul>
          <li><code>help</code> - print the command list in the robot console.</li>
          <li><code>status</code> - show mode, pose, battery, motion, and wall state.</li>
          <li><code>explore [n]</code> - explore the maze and learn walls until the shortest path is known, or stop after <code>n</code> forward cell moves.</li>
          <li><code>speedrun [1-4]</code> - start the speed-run mode when ready; <code>speedrun</code> means phase 1, phase 2 is the one-way home-to-goal profile, and phases 3-4 inherit the previous phase until tuned.</li>
          <li><code>idle</code> - switch the robot back to idle mode.</li>
          <li><code>restart</code> - reboot the robot after closing the debug client.</li>
        </ul>
      </div>
      <div class="group">
        <h3>Motion</h3>
        <ul>
          <li><code>move [n]</code> - move forward 1 cell by default, or move exactly <code>n</code> cells in a straight corridor.</li>
          <li><code>back</code> - move backward using the configured short reverse distance.</li>
          <li><code>left</code> - turn left 90 degrees.</li>
          <li><code>right</code> - turn right 90 degrees.</li>
          <li><code>uturn</code> - turn 180 degrees.</li>
          <li><code>testsnap</code> - run the snap-center combo for alignment tuning.</li>
          <li><code>stop</code> - stop with the normal coast-to-idle path.</li>
          <li><code>brake</code> - apply the active brake immediately for bench testing.</li>
        </ul>
      </div>
      <div class="group">
        <h3>Maze And Pose</h3>
        <ul>
          <li><code>maze</code> - print the current maze memory.</li>
          <li><code>clearmaze</code> - clear remembered maze walls only; keep the current pose and remove saved wall memory from SPIFFS.</li>
          <li><code>resetpose x y h</code> - set pose manually for the current runtime; heading is the numeric enum used by the firmware.</li>
          <li><code>setgoal x y w h</code> - set the active goal rectangle origin and size for the current runtime.</li>
        </ul>
      </div>
      <div class="group">
        <h3>LED Control</h3>
        <ul>
          <li><code>led cycle</code> - step through LED states to identify the robot.</li>
          <li><code>led rotate</code> - rotate LED colors.</li>
          <li><code>led off</code>, <code>led red</code>, <code>led green</code>, <code>led blue</code>, <code>led yellow</code>, <code>led cyan</code>, <code>led magenta</code>, <code>led white</code> - force a specific LED state.</li>
        </ul>
      </div>
      <div class="group">
        <h3>Test And Debug</h3>
        <ul>
          <li><code>test</code> - enter test mode.</li>
          <li><code>test off</code> - leave test mode.</li>
          <li><code>test battery</code> - print battery diagnostics.</li>
          <li><code>test sensors</code> - print interpreted wall-sensor readings.</li>
          <li><code>test sensorsraw</code> - print raw TOF sensor readings.</li>
          <li><code>test encoders</code> - print encoder diagnostics.</li>
          <li><code>test motorl</code>, <code>test motorr</code> - spin a single motor for bench checks.</li>
          <li><code>test motor both</code> - flip both motors between +100% and -100% every second for bench testing.</li>
          <li><code>test loop status|battery|sensors|sensorsraw|encoders|maze|off</code> - start or stop periodic debug printing.</li>
        </ul>
      </div>
    </div>
  </div>
</div>
<script>
let busy = false;
function setStatus(s){ document.getElementById('status').textContent = s; }

async function cycleLed(){
  if (busy) return;
  busy = true;
  try {
    const res = await fetch('/led', {
      method: 'POST',
      headers: { 'Content-Type':'application/x-www-form-urlencoded' },
      body: 'cmd=' + encodeURIComponent('cycle'),
      cache: 'no-store'
    });
    if (!res.ok) throw new Error('HTTP ' + res.status);
    setStatus('LED cycle command sent.');
  } catch (e) {
    setStatus('LED command failed: ' + e);
  } finally {
    busy = false;
  }
}

async function reconnectTelnet(){
  if (busy) return;
  busy = true;
  try {
    const res = await fetch('/telnet/reconnect', { method: 'POST', cache: 'no-store' });
    if (!res.ok) throw new Error('HTTP ' + res.status);
    const info = await res.json();
    document.getElementById('robotIp').textContent = info.ip;
    document.getElementById('telnetTarget').textContent = info.ip + ':' + info.port;
    if (info.batteryText) {
      document.getElementById('batteryNow').textContent = info.batteryText;
    }
    setStatus('Reconnecting telnet to ' + info.ip + ':' + info.port + ' ...');
    window.location.href = 'telnet://' + info.ip + ':' + info.port;
  } catch (e) {
    setStatus('Telnet reconnect failed: ' + e);
  } finally {
    busy = false;
  }
}

function openFloodfill(){
  const ip = document.getElementById('robotIp').textContent || location.hostname;
  setStatus('Opening floodfill viewer on ' + ip + ':81 ...');
  window.location.href = 'http://' + ip + ':81/';
}

function openUpload(){
  const ip = document.getElementById('robotIp').textContent || location.hostname;
  setStatus('Opening upload page on ' + ip + ':%UPLOAD_PORT% ...');
  window.location.href = 'http://' + ip + ':%UPLOAD_PORT%/';
}
</script>
</body>
</html>
)HTML";

static const char* kUploadHtml PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>ESP32 Upload</title>
  <style>
    body { font-family: ui-sans-serif, Segoe UI, Arial, sans-serif; margin: 24px; background: #f6f3ea; color: #1b1b1b; }
    .card { max-width: 560px; padding: 20px; border-radius: 16px; background: #fffdf7; box-shadow: 0 8px 30px rgba(0,0,0,0.08); }
    h1 { margin-top: 0; font-size: 1.4rem; }
    input, button { font: inherit; }
    input[type=file] { display: block; margin: 12px 0; }
    button { padding: 10px 14px; border: 0; border-radius: 10px; background: #165d47; color: white; cursor: pointer; }
    progress { width: 100%; height: 18px; margin-top: 16px; }
    #msg { margin-top: 12px; white-space: pre-wrap; }
    .muted { color: #5d5d5d; font-size: 0.95rem; }
  </style>
</head>
<body>
    <div class="card">
    <h1>Wireless Firmware Upload</h1>
    <div class="muted">Chunked HTTP upload with retry. Upload a compiled firmware binary (`.bin`). The mouse will reboot after a successful update.</div>
    <form id="fwForm">
      <input id="fwFile" name="firmware" type="file" accept=".bin,application/octet-stream" required />
      <button type="submit">Upload Firmware</button>
    </form>
    <progress id="prog" value="0" max="100" hidden></progress>
    <div id="msg"></div>
  </div>
<script>
const form = document.getElementById('fwForm');
const fileInput = document.getElementById('fwFile');
const prog = document.getElementById('prog');
const msg = document.getElementById('msg');
const CHUNK_SIZE = 64 * 1024; 
const MAX_CHUNK_SIZE = 64 * 1024;
const MIN_CHUNK_SIZE = 8192;
const MAX_RETRIES = 5;
const RETRY_BACKOFF_MS = 250;
const CHUNK_SUCCESS_PAUSE_MS = 20;
const GROW_CHUNK_AFTER_SUCCESS = 4;

async function postStart(totalSize){
  const res = await fetch('/upload/start?size=' + totalSize, {
    method: 'POST',
    cache: 'no-store'
  });
  const text = await res.text();
  if (!res.ok) throw new Error(text || ('HTTP ' + res.status));
  return JSON.parse(text);
}

async function postChunk(file, offset, chunkSize){
  const end = Math.min(offset + chunkSize, file.size);
  const chunk = file.slice(offset, end);
  const form = new FormData();
  form.append('chunk', chunk, file.name + '.part');
  const res = await fetch('/upload/chunk?offset=' + offset, {
    method: 'POST',
    body: form,
    cache: 'no-store'
  });
  const text = await res.text();
  if (!res.ok) throw new Error(text || ('HTTP ' + res.status));
  return JSON.parse(text);
}

async function postFinish(){
  const res = await fetch('/upload/finish', {
    method: 'POST',
    cache: 'no-store'
  });
  const text = await res.text();
  if (!res.ok) throw new Error(text || ('HTTP ' + res.status));
  return JSON.parse(text);
}

form.addEventListener('submit', async (ev) => {
  ev.preventDefault();
  const file = fileInput.files[0];
  if (!file) {
    msg.textContent = 'Choose a .bin file first.';
    return;
  }

  prog.hidden = false;
  prog.value = 0;
  msg.textContent = 'Starting chunked upload...';

  try {
    let info = await postStart(file.size);
    let offset = Number(info.nextOffset || 0);
    let chunkSize = CHUNK_SIZE;
    let successStreak = 0;

    while (offset < file.size) {
      let done = false;
      let lastErr = '';
      for (let attempt = 1; attempt <= MAX_RETRIES; ++attempt) {
        try {
          info = await postChunk(file, offset, chunkSize);
          offset = Number(info.nextOffset || 0);
          prog.value = Math.round((offset / file.size) * 100);
          msg.textContent = 'Uploading... ' + prog.value + '%';
          successStreak++;
          if (successStreak >= GROW_CHUNK_AFTER_SUCCESS && chunkSize < MAX_CHUNK_SIZE) {
            chunkSize = Math.min(MAX_CHUNK_SIZE, chunkSize * 2);
            successStreak = 0;
          }
          await new Promise((resolve) => setTimeout(resolve, CHUNK_SUCCESS_PAUSE_MS));
          done = true;
          break;
        } catch (err) {
          lastErr = String(err);
          successStreak = 0;
          msg.textContent = 'Chunk retry ' + attempt + '/' + MAX_RETRIES + ' at offset ' + offset + ' failed: ' + lastErr + ' (chunk=' + chunkSize + ')';
          await new Promise((resolve) => setTimeout(resolve, RETRY_BACKOFF_MS));
        }
      }
      if (!done) {
        if (chunkSize > MIN_CHUNK_SIZE) {
          chunkSize = Math.max(MIN_CHUNK_SIZE, Math.floor(chunkSize / 2));
          msg.textContent = 'Network unstable at offset ' + offset + '. Reducing chunk size to ' + chunkSize + ' and retrying...';
          continue;
        }
        throw new Error('Chunk upload failed at offset ' + offset + ': ' + lastErr);
      }
    }

    await postFinish();
    prog.value = 100;
    msg.textContent = 'Upload complete. Device will reboot.';
  } catch (err) {
    msg.textContent = 'Upload failed: ' + err;
  }
});
</script>
</body>
</html>
)HTML";

static constexpr uint32_t kUploadStallTimeoutMs = 10000;
static constexpr bool kUploadPreEraseEnabled = true;

WiFiOtaWebSerial::WiFiOtaWebSerial() {}

WiFiOtaWebSerial::~WiFiOtaWebSerial() {
  end();
}

bool WiFiOtaWebSerial::begin(const Config& cfg) {
  cfg_ = cfg;
  if (!cfg_.ssid || cfg_.ssid[0] == '\0') return false;
  otaStarted_ = false;
  webServerStarted_ = false;
  uploadWebServerStarted_ = false;

  // server
  if (web_) { delete web_; web_ = nullptr; }
  if (uploadWeb_) { delete uploadWeb_; uploadWeb_ = nullptr; }
  if (cfg_.enableWeb) {
    web_ = new WebServerWrapper(cfg_.port);
  }
  if (cfg_.enableUploadWeb) {
    uploadWeb_ = new WebServerWrapper(cfg_.uploadPort);
  }

  setupWiFi_();
  setupOta_();
  if (cfg_.enableWeb && web_) {
    setupWeb_();
  }
  if (cfg_.enableUploadWeb && uploadWeb_) {
    setupUploadWeb_();
  }

  started_ = true;

  // start service task pinned to core
  if (wifiTask_) {
    vTaskDelete(wifiTask_);
    wifiTask_ = nullptr;
  }
  if (uploadTask_) {
    vTaskDelete(uploadTask_);
    uploadTask_ = nullptr;
  }

  BaseType_t ok = xTaskCreatePinnedToCore(
    &WiFiOtaWebSerial::wifiTaskThunk_,
    "wifiTask",
    cfg_.wifiTaskStack / sizeof(StackType_t),
    this,
    cfg_.wifiTaskPrio,
    &wifiTask_,
    cfg_.wifiCore
  );

  if (ok != pdPASS) {
    wifiTask_ = nullptr;
    started_ = false;
    return false;
  }

  if (cfg_.enableUploadWeb && uploadWeb_) {
    BaseType_t uploadOk = xTaskCreatePinnedToCore(
      &WiFiOtaWebSerial::uploadTaskThunk_,
      "uploadTask",
      cfg_.uploadTaskStack / sizeof(StackType_t),
      this,
      cfg_.uploadTaskPrio,
      &uploadTask_,
      cfg_.uploadCore
    );
    if (uploadOk != pdPASS) {
      uploadTask_ = nullptr;
      vTaskDelete(wifiTask_);
      wifiTask_ = nullptr;
      started_ = false;
      return false;
    }
  }

  println(String("OTA Hostname: ") + cfg_.hostname);
  return true;
}

void WiFiOtaWebSerial::end() {
  if (wifiTask_) {
    vTaskDelete(wifiTask_);
    wifiTask_ = nullptr;
  }
  if (uploadTask_) {
    vTaskDelete(uploadTask_);
    uploadTask_ = nullptr;
  }
  started_ = false;
  otaStarted_ = false;
  webServerStarted_ = false;
  uploadWebServerStarted_ = false;

  if (web_) {
    delete web_;
    web_ = nullptr;
  }
  if (uploadWeb_) {
    delete uploadWeb_;
    uploadWeb_ = nullptr;
  }
}

void WiFiOtaWebSerial::loopNoService() {
  // Intentionally empty: WiFi/OTA/Web runs in pinned task.
  // Keep your main app logic in Arduino loop().
  
  if (otaStarted_) {
    ArduinoOTA.handle();
  }
}

void WiFiOtaWebSerial::wifiTaskThunk_(void* arg) {
  static_cast<WiFiOtaWebSerial*>(arg)->wifiTaskLoop_();
}

void WiFiOtaWebSerial::uploadTaskThunk_(void* arg) {
  static_cast<WiFiOtaWebSerial*>(arg)->uploadTaskLoop_();
}

void WiFiOtaWebSerial::wifiTaskLoop_() {
  uint32_t lastReconnectMs = 0;
  uint32_t disconnectSinceMs = 0;
  bool urlsAnnounced = false;
  for (;;) {
    if (started_) {
      serviceUpdateLed_();
      const wl_status_t wifiStatus = WiFi.status();
      if (wifiStatus != WL_CONNECTED) {
        urlsAnnounced = false;
        otaStarted_ = false;
        if (otaInProgress || webUploadInProgress_ || rebootPending_) {
          vTaskDelay(pdMS_TO_TICKS(50));
          continue;
        }
        const bool reconnectAllowed =
          reconnectAllowedFn_ ? reconnectAllowedFn_() : true;
        if (!reconnectAllowed) {
          disconnectSinceMs = 0;
          lastReconnectMs = 0;
          vTaskDelay(pdMS_TO_TICKS(250));
          continue;
        }
        if (disconnectSinceMs == 0) disconnectSinceMs = millis();
        const uint32_t now = millis();
        const bool shouldReconnect =
          wifiStatus == WL_DISCONNECTED ||
          wifiStatus == WL_CONNECTION_LOST ||
          wifiStatus == WL_CONNECT_FAILED ||
          wifiStatus == WL_NO_SSID_AVAIL;
        if (shouldReconnect && now - lastReconnectMs >= cfg_.wifiReconnectIntervalMs) {
          lastReconnectMs = now;
          const bool serialOutputAllowed =
            serialOutputAllowedFn_ ? serialOutputAllowedFn_() : AppConfig::Debug::ENABLE_SERIAL_OUTPUT;
          if (serialOutputAllowed) Serial.println("[WiFi] reconnecting...");
          WiFi.reconnect();
        }
        if ((uint32_t)(now - disconnectSinceMs) >= cfg_.wifiConnectTimeoutMs) {
          disconnectSinceMs = now;
          restartSta_();
        }
        vTaskDelay(pdMS_TO_TICKS(250));
        continue;
      }
      if (disconnectSinceMs != 0) {
        disconnectSinceMs = 0;
        onWiFiConnected_();
      }
      if (!urlsAnnounced) {
        const String currentIp = WiFi.localIP().toString();
        if (cfg_.enableWeb) {
          println(String("HTTP: http://") + currentIp + "/  (or http://" + cfg_.hostname + ".local/ )");
        }
        if (cfg_.enableUploadWeb) {
          println(String("Upload: http://") + currentIp + ":" + String(cfg_.uploadPort) + "/");
        }
        urlsAnnounced = true;
      }
      if (otaInProgress) {
        // OTA priority mode
        if (otaStarted_) ArduinoOTA.handle();
        if (cfg_.enableUploadWeb && uploadWeb_ && uploadWebServerStarted_ && !uploadServerOwnedByUploadTask_()) {
          uploadWeb_->server.handleClient();
        }
        serviceUploadSession_();
        // Optional: skip web server to reduce load
        // if (web_) web_->server.handleClient();
        vTaskDelay(1);
        continue;
      }
      if (webUploadInProgress_) {
        if (cfg_.enableUploadWeb && uploadWeb_ && uploadWebServerStarted_ && !uploadServerOwnedByUploadTask_()) {
          uploadWeb_->server.handleClient();
        }
        serviceUploadSession_();
        vTaskDelay(1);
        continue;
      }
      // Normal mode
      if (otaStarted_) ArduinoOTA.handle();
      if (cfg_.enableWeb && web_ && webServerStarted_) web_->server.handleClient();
      if (cfg_.enableUploadWeb && uploadWeb_ && uploadWebServerStarted_ && !uploadServerOwnedByUploadTask_()) {
        uploadWeb_->server.handleClient();
      }
      serviceUploadSession_();
      if (rebootPending_ && static_cast<int32_t>(millis() - rebootAtMs_) >= 0) {
        println("[WEB OTA] Rebooting now...");
        vTaskDelay(pdMS_TO_TICKS(50));
        ESP.restart();
      }
      vTaskDelay(pdMS_TO_TICKS(cfg_.serviceDelayMs));
    } else {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

void WiFiOtaWebSerial::uploadTaskLoop_() {
  for (;;) {
    if (!started_ || !cfg_.enableUploadWeb || !uploadWeb_ || !uploadWebServerStarted_) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    if (!uploadServerOwnedByUploadTask_()) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    uploadWeb_->server.handleClient();
    serviceUploadSession_();
    vTaskDelay(pdMS_TO_TICKS(cfg_.uploadServiceDelayMs));
  }
}

bool WiFiOtaWebSerial::uploadServerOwnedByUploadTask_() const {
  return uploadTask_ != nullptr && (webUploadInProgress_ || chunkUploadActive_);
}

void WiFiOtaWebSerial::print(const String& s, bool mirrorToSerial) {
  const bool serialOutputAllowed =
    serialOutputAllowedFn_ ? serialOutputAllowedFn_() : AppConfig::Debug::ENABLE_SERIAL_OUTPUT;
  if (mirrorToSerial && serialOutputAllowed) Serial.print(s);
}

void WiFiOtaWebSerial::println(const String& s, bool mirrorToSerial) {
  const bool serialOutputAllowed =
    serialOutputAllowedFn_ ? serialOutputAllowedFn_() : AppConfig::Debug::ENABLE_SERIAL_OUTPUT;
  if (mirrorToSerial && serialOutputAllowed) Serial.println(s);
}

String WiFiOtaWebSerial::ip() const {
  return WiFi.isConnected() ? WiFi.localIP().toString() : String("0.0.0.0");
}

void WiFiOtaWebSerial::setupWiFi_() {
  configureSta_();
  WiFi.begin(cfg_.ssid, cfg_.pass);
  const bool serialOutputAllowed =
    serialOutputAllowedFn_ ? serialOutputAllowedFn_() : AppConfig::Debug::ENABLE_SERIAL_OUTPUT;
  if (serialOutputAllowed) Serial.println("[WiFi] start connect (non-blocking)");
}

void WiFiOtaWebSerial::configureSta_() {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
}

void WiFiOtaWebSerial::restartSta_() {
  const bool serialOutputAllowed =
    serialOutputAllowedFn_ ? serialOutputAllowedFn_() : AppConfig::Debug::ENABLE_SERIAL_OUTPUT;
  if (serialOutputAllowed) Serial.println("[WiFi] restarting STA...");
  MDNS.end();
  WiFi.disconnect(false, true);
  vTaskDelay(pdMS_TO_TICKS(100));
  configureSta_();
  WiFi.begin(cfg_.ssid, cfg_.pass);
}

void WiFiOtaWebSerial::onWiFiConnected_() {
  MDNS.end();
  if (MDNS.begin(cfg_.hostname)) {
    if (cfg_.enableWeb) {
      MDNS.addService("http", "tcp", cfg_.port);
    }
    if (cfg_.enableUploadWeb) {
      MDNS.addService("http", "tcp", cfg_.uploadPort);
    }
  }
  startWebServersIfNeeded_();
  setupOta_();
}

void WiFiOtaWebSerial::setupOta_() {
  if (!otaHandlersReady_) {
    ArduinoOTA.setHostname(cfg_.hostname);

    ArduinoOTA
      .onStart([this]() {
        otaInProgress = true;
        ledBlinkOn_ = true;
        setUploadLedActive_();

        println("[OTA] Start");

        // Enter OTA safe mode
        WiFi.setSleep(false);     // improve stability
      })

      .onEnd([this]() {
        println("[OTA] End");
        setUploadLedSuccess_();

        // Exit OTA mode
        otaInProgress = false;

      })

      .onProgress([this](unsigned int progress, unsigned int total) {
        static uint32_t lastMs = 0;
        if (millis() - lastMs > 1000) {
          lastMs = millis();
          int pct = (total > 0) ? (int)(progress * 100 / total) : 0;
          println(String("[OTA] ") + pct + "%");
        }
      })

      .onError([this](ota_error_t error) {
        println(String("[OTA] Error: ") + (int)error);
        setUploadLedError_();

        otaInProgress = false;
      });

    otaHandlersReady_ = true;
  }

  if (WiFi.status() == WL_CONNECTED && !otaStarted_) {
    ArduinoOTA.begin();
    otaStarted_ = true;
  }
}

void WiFiOtaWebSerial::setupWeb_() {
  auto& srv = web_->server;

  srv.on("/", HTTP_GET, [this]() {
    String html = FPSTR(kIndexHtml);
    html.replace("%HOSTNAME%", String(cfg_.hostname ? cfg_.hostname : "esp32"));
    html.replace("%IP%", ip());
    html.replace("%TELNET_PORT%", String(cfg_.debugTcpPort));
    html.replace("%UPLOAD_PORT%", String(cfg_.uploadPort));
    html.replace("%BATTERY_TEXT%", healthJsonProvider_ ? extractBatteryText_(healthJsonProvider_()) : String("unavailable"));
    web_->server.send(200, "text/html; charset=utf-8", html);
  });

  srv.on("/led", HTTP_POST, [this]() {
    String cmd = web_->server.arg("cmd");
    cmd.toLowerCase();

    if (!ledCommandHandler_) {
      web_->server.send(503, "text/plain", "LED unavailable");
      return;
    }

    String response;
    if (!ledCommandHandler_(cmd, response)) {
      web_->server.send(400, "text/plain", "Bad cmd");
      return;
    }

    if (response.length() > 0) {
      println(response);
    }
    web_->server.send(200, "text/plain", "OK");
  });

  srv.on("/telnet/reconnect", HTTP_POST, [this]() {
    if (!telnetReconnectHandler_) {
      web_->server.send(503, "application/json", "{\"ok\":false,\"error\":\"telnet unavailable\"}");
      return;
    }

    const bool ok = telnetReconnectHandler_();
    const String host = cfg_.hostname ? String(cfg_.hostname) : String("esp32");
    const String extra = healthJsonProvider_ ? (String(",") + healthJsonProvider_()) : String("");
    String body = String("{\"ok\":") + (ok ? "true" : "false") +
                  ",\"ip\":\"" + ip() + "\"" +
                  ",\"hostname\":\"" + host + "\"" +
                  ",\"port\":" + String(cfg_.debugTcpPort) +
                  extra + "}";
    web_->server.send(ok ? 200 : 500, "application/json", body);
  });

  srv.onNotFound([this]() {
    web_->server.send(404, "text/plain", "Not found");
  });

}

String WiFiOtaWebSerial::extractBatteryText_(const String& json) const {
  const String key = "\"batteryText\":\"";
  const int start = json.indexOf(key);
  if (start < 0) return "unavailable";
  const int valueStart = start + key.length();
  const int valueEnd = json.indexOf('"', valueStart);
  if (valueEnd < 0) return "unavailable";
  return json.substring(valueStart, valueEnd);
}

void WiFiOtaWebSerial::setupUploadWeb_() {
  auto& srv = uploadWeb_->server;

  srv.on("/", HTTP_GET, [this]() {
    String html = FPSTR(kUploadHtml);
    uploadWeb_->server.send(200, "text/html; charset=utf-8", html);
  });

  srv.on("/upload/start", HTTP_POST, [this]() {
    uploadWeb_->server.client().setNoDelay(true);
    const size_t totalSize = (size_t)uploadWeb_->server.arg("size").toInt();
    if (totalSize == 0) {
      uploadWeb_->server.send(400, "application/json", "{\"ok\":false,\"error\":\"invalid size\"}");
      return;
    }

    if (chunkUploadActive_) {
      Update.abort();
    }

    webUploadInProgress_ = true;
    chunkUploadActive_ = false;
    chunkExpectedOffset_ = 0;
    chunkTotalSize_ = totalSize;
    chunkRequestOffset_ = 0;
    chunkRequestBytes_ = 0;
    chunkRequestOk_ = false;
    chunkRequestSkip_ = false;
    chunkRequestError_ = "";
    uploadLastActivityMs_ = millis();
    rebootPending_ = false;
    rebootAtMs_ = 0;
    ledBlinkOn_ = true;
    setUploadLedActive_();
    println(String("[WEB OTA] Chunked start total=") + totalSize);

    if (kUploadPreEraseEnabled) {
      const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);
      if (next == nullptr) {
        setUploadLedError_();
        webUploadInProgress_ = false;
        chunkUploadActive_ = false;
        uploadWeb_->server.send(500, "application/json", "{\"ok\":false,\"error\":\"no ota partition\"}");
        return;
      }
      const size_t eraseSize =
        ((totalSize + SPI_FLASH_SEC_SIZE - 1) / SPI_FLASH_SEC_SIZE) * SPI_FLASH_SEC_SIZE;
      println(String("[WEB OTA] Pre-erase start bytes=") + eraseSize);
      const esp_err_t eraseErr = esp_partition_erase_range(next, 0, eraseSize);
      if (eraseErr != ESP_OK) {
        setUploadLedError_();
        webUploadInProgress_ = false;
        chunkUploadActive_ = false;
        uploadWeb_->server.send(
          500,
          "application/json",
          String("{\"ok\":false,\"error\":\"pre-erase failed:") + esp_err_to_name(eraseErr) + "\"}"
        );
        return;
      }
      println("[WEB OTA] Pre-erase done");
    }

    if (!Update.begin(totalSize)) {
      Update.printError(Serial);
      setUploadLedError_();
      webUploadInProgress_ = false;
      uploadWeb_->server.send(500, "application/json", "{\"ok\":false,\"error\":\"update begin failed\"}");
      return;
    }

    chunkUploadActive_ = true;
    uploadWeb_->server.send(200, "application/json", "{\"ok\":true,\"nextOffset\":0}");
  });

  srv.on("/upload/chunk", HTTP_POST,
    [this]() {
      uploadWeb_->server.client().setNoDelay(true);
      if (!chunkUploadActive_) {
        uploadWeb_->server.send(409, "application/json", "{\"ok\":false,\"error\":\"no active upload session\"}");
        return;
      }
      if (!chunkRequestOk_) {
        const String err = chunkRequestError_.length() > 0 ? chunkRequestError_ : String("chunk failed");
        uploadWeb_->server.send(409, "application/json",
                                String("{\"ok\":false,\"error\":\"") + err + "\"}");
        return;
      }
      uploadWeb_->server.send(200, "application/json",
                              String("{\"ok\":true,\"nextOffset\":") + chunkExpectedOffset_ + "}");
    },
    [this]() {
      HTTPUpload& upload = uploadWeb_->server.upload();

      if (upload.status == UPLOAD_FILE_START) {
        uploadLastActivityMs_ = millis();
        chunkRequestOffset_ = (size_t)uploadWeb_->server.arg("offset").toInt();
        chunkRequestBytes_ = 0;
        chunkRequestOk_ = false;
        chunkRequestHadWrite_ = false;
        chunkRequestSkip_ = false;
        chunkRequestError_ = "";
        if (!chunkUploadActive_) {
          chunkRequestError_ = "no active upload session";
          return;
        }
        if (chunkRequestOffset_ < chunkExpectedOffset_) {
          chunkRequestOk_ = true;
          chunkRequestSkip_ = true;
          return;
        }
        if (chunkRequestOffset_ != chunkExpectedOffset_) {
          chunkRequestError_ = String("offset mismatch expected ") + chunkExpectedOffset_ +
                               " got " + chunkRequestOffset_;
          return;
        }
        chunkRequestOk_ = true;
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (!chunkRequestOk_) return;
        if (chunkRequestSkip_) return;
        if (upload.buf == nullptr || upload.currentSize == 0) return;
        uploadLastActivityMs_ = millis();
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
          chunkRequestOk_ = false;
          chunkRequestError_ = "flash write failed";
          chunkUploadActive_ = false;
          webUploadInProgress_ = false;
          uploadLastActivityMs_ = 0;
          Update.abort();
          setUploadLedError_();
          println("[WEB OTA] Chunked session aborted (flash write failed)");
          return;
        }
        delay(0);
        chunkRequestHadWrite_ = true;
        chunkRequestBytes_ += upload.currentSize;
      } else if (upload.status == UPLOAD_FILE_END) {
        if (!chunkRequestOk_) return;
        uploadLastActivityMs_ = millis();
        if (chunkRequestSkip_) return;
        chunkExpectedOffset_ += chunkRequestBytes_;
        if (chunkExpectedOffset_ > chunkTotalSize_) {
          chunkRequestOk_ = false;
          chunkRequestError_ = "size overflow";
          setUploadLedError_();
          return;
        }
        if ((chunkExpectedOffset_ % (128 * 1024)) < chunkRequestBytes_) {
          println(String("[WEB OTA] Chunked received ") + chunkExpectedOffset_ + " bytes");
        }
      } else if (upload.status == UPLOAD_FILE_ABORTED) {
        chunkRequestOk_ = false;
        chunkRequestError_ = "chunk aborted, restart upload";
        if (chunkUploadActive_) {
          chunkUploadActive_ = false;
          webUploadInProgress_ = false;
          uploadLastActivityMs_ = 0;
          Update.abort();
          setUploadLedError_();
          println(String("[WEB OTA] Chunked session aborted (chunk aborted, wrote=") +
                  (chunkRequestHadWrite_ ? "1" : "0") + ")");
        }
        uploadLastActivityMs_ = 0;
      }
    }
  );

  srv.on("/upload/finish", HTTP_POST, [this]() {
    if (!chunkUploadActive_) {
      uploadWeb_->server.send(409, "application/json", "{\"ok\":false,\"error\":\"no active upload session\"}");
      return;
    }
    if (chunkExpectedOffset_ != chunkTotalSize_) {
      uploadWeb_->server.send(409, "application/json",
                              String("{\"ok\":false,\"error\":\"incomplete upload at ") +
                              chunkExpectedOffset_ + "/" + chunkTotalSize_ + "\"}");
      return;
    }
    const bool ok = Update.end(true);
    chunkUploadActive_ = false;
    webUploadInProgress_ = false;
    uploadLastActivityMs_ = 0;
    if (ok) {
      println(String("[WEB OTA] Chunked success: ") + chunkTotalSize_ + " bytes");
      setUploadLedSuccess_();
      rebootPending_ = true;
      rebootAtMs_ = millis() + 1500;
      uploadWeb_->server.send(200, "application/json", "{\"ok\":true}");
    } else {
      Update.printError(Serial);
      setUploadLedError_();
      println("[WEB OTA] Chunked finish failed");
      uploadWeb_->server.send(500, "application/json", "{\"ok\":false,\"error\":\"update end failed\"}");
    }
  });

  srv.on("/update", HTTP_POST,
    [this]() {
      const bool ok = !Update.hasError();
      if (ok) {
        uploadWeb_->server.send(200, "text/plain", "OK");
        println("[WEB OTA] Update complete, reboot scheduled...");
        setUploadLedSuccess_();
        rebootPending_ = true;
        rebootAtMs_ = millis() + 1500;
      } else {
        uploadWeb_->server.send(500, "text/plain", "FAIL");
        println("[WEB OTA] Update failed");
      }
      webUploadInProgress_ = false;
    },
    [this]() {
      HTTPUpload& upload = uploadWeb_->server.upload();

      if (upload.status == UPLOAD_FILE_START) {
        webUploadInProgress_ = true;
        rebootPending_ = false;
        rebootAtMs_ = 0;
        ledBlinkOn_ = true;
        setUploadLedActive_();
        println(String("[WEB OTA] Start: ") + upload.filename);
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          Update.printError(Serial);
          setUploadLedError_();
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (upload.buf == nullptr || upload.currentSize == 0) {
          return;
        }
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
          setUploadLedError_();
        } else {
          delay(0);
          if ((upload.totalSize % (128 * 1024)) < upload.currentSize) {
          println(String("[WEB OTA] Received ") + upload.totalSize + " bytes");
          }
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
          println(String("[WEB OTA] Success: ") + upload.totalSize + " bytes");
          setUploadLedSuccess_();
        } else {
          Update.printError(Serial);
          setUploadLedError_();
        }
      } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        webUploadInProgress_ = false;
        setUploadLedError_();
        println("[WEB OTA] Aborted");
      }
    }
  );

  srv.onNotFound([this]() {
    uploadWeb_->server.send(404, "text/plain", "Not found");
  });

}

void WiFiOtaWebSerial::startWebServersIfNeeded_() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (cfg_.enableWeb && web_ && !webServerStarted_) {
    web_->server.begin();
    webServerStarted_ = true;
  }
  if (cfg_.enableUploadWeb && uploadWeb_ && !uploadWebServerStarted_) {
    uploadWeb_->server.begin();
    uploadWebServerStarted_ = true;
  }
}

void WiFiOtaWebSerial::serviceUploadSession_() {
  if ((chunkUploadActive_ || webUploadInProgress_) &&
      uploadLastActivityMs_ != 0 &&
      (uint32_t)(millis() - uploadLastActivityMs_) >= kUploadStallTimeoutMs) {
    Update.abort();
    chunkUploadActive_ = false;
    webUploadInProgress_ = false;
    uploadLastActivityMs_ = 0;
    setUploadLedError_();
    println("[WEB OTA] Upload stalled");
  }
}

void WiFiOtaWebSerial::setLedState_(const String& cmd) {
  if (!ledCommandHandler_) return;
  String response;
  ledCommandHandler_(cmd, response);
}

void WiFiOtaWebSerial::setUploadLedActive_() {
  setLedState_("blue");
}

void WiFiOtaWebSerial::setUploadLedSuccess_() {
  setLedState_("off");
}

void WiFiOtaWebSerial::setUploadLedError_() {
  setLedState_("red");
}

void WiFiOtaWebSerial::serviceUpdateLed_() {
  if (!(otaInProgress || webUploadInProgress_)) return;
  if (!ledBlinkOn_) {
    ledBlinkOn_ = true;
    setUploadLedActive_();
  }
}



