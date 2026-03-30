#include "WiFiOtaWebSerial.h"
#include "Config.h"

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <mbedtls/base64.h>
#include <mbedtls/sha1.h>

// Wrapper to keep WebServer out of header
class WiFiOtaWebSerial::WebServerWrapper {
public:
  WebServerWrapper(uint16_t port) : server(port) {}
  WebServer server;
};

class WiFiOtaWebSerial::UploadWsWrapper {
public:
  explicit UploadWsWrapper(uint16_t port) : server(port) {}
  WiFiServer server;
  WiFiClient client;
  bool handshaken = false;
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
          <li><code>speedrun [1-4]</code> - start the speed-run mode when ready; <code>speedrun</code> means phase 1, and each later phase inherits the previous phase until tuned.</li>
          <li><code>idle</code> - switch the robot back to idle mode.</li>
          <li><code>restart</code> - reboot the robot after closing the debug client.</li>
        </ul>
      </div>
      <div class="group">
        <h3>Motion</h3>
        <ul>
          <li><code>move</code> - move forward one cell.</li>
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
    <div class="muted">WebSocket firmware upload. Upload a compiled firmware binary (`.bin`). The mouse will reboot after a successful update.</div>
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
const WS_PORT = %UPLOAD_WS_PORT%;
const CHUNK_SIZE = 32768;
const MAX_RETRIES = 5;
const ACK_TIMEOUT_MS = 5000;

function openUploadSocket(){
  const proto = (location.protocol === 'https:') ? 'wss://' : 'ws://';
  return new WebSocket(proto + location.hostname + ':' + WS_PORT + '/ota');
}

function waitForOpen(ws){
  return new Promise((resolve, reject) => {
    const onOpen = () => { cleanup(); resolve(); };
    const onError = () => { cleanup(); reject(new Error('websocket open failed')); };
    const cleanup = () => {
      ws.removeEventListener('open', onOpen);
      ws.removeEventListener('error', onError);
    };
    ws.addEventListener('open', onOpen, { once: true });
    ws.addEventListener('error', onError, { once: true });
  });
}

function waitForMessage(ws, timeoutMs){
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => {
      cleanup();
      reject(new Error('timeout waiting for upload ack'));
    }, timeoutMs);
    const onMessage = (ev) => {
      cleanup();
      resolve(String(ev.data || ''));
    };
    const onClose = () => {
      cleanup();
      reject(new Error('websocket closed'));
    };
    const onError = () => {
      cleanup();
      reject(new Error('websocket error'));
    };
    const cleanup = () => {
      clearTimeout(timer);
      ws.removeEventListener('message', onMessage);
      ws.removeEventListener('close', onClose);
      ws.removeEventListener('error', onError);
    };
    ws.addEventListener('message', onMessage, { once: true });
    ws.addEventListener('close', onClose, { once: true });
    ws.addEventListener('error', onError, { once: true });
  });
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
  msg.textContent = 'Starting WebSocket upload...';

  try {
    let ws = openUploadSocket();
    ws.binaryType = 'arraybuffer';
    await waitForOpen(ws);
    ws.send('start|' + file.size);
    let reply = await waitForMessage(ws, ACK_TIMEOUT_MS);
    if (!reply.startsWith('ready|')) {
      throw new Error(reply || 'upload start failed');
    }
    let offset = Number(reply.substring(6) || 0);

    while (offset < file.size) {
      let done = false;
      let lastErr = '';
      for (let attempt = 1; attempt <= MAX_RETRIES; ++attempt) {
        try {
          const end = Math.min(offset + CHUNK_SIZE, file.size);
          const chunk = await file.slice(offset, end).arrayBuffer();
          ws.send(chunk);
          reply = await waitForMessage(ws, ACK_TIMEOUT_MS);
          if (!reply.startsWith('ack|')) {
            throw new Error(reply || 'bad chunk reply');
          }
          offset = Number(reply.substring(4) || 0);
          prog.value = Math.round((offset / file.size) * 100);
          msg.textContent = 'Uploading... ' + prog.value + '%';
          done = true;
          break;
        } catch (err) {
          lastErr = String(err);
          msg.textContent = 'Chunk retry ' + attempt + '/' + MAX_RETRIES + ' at offset ' + offset + ' failed: ' + lastErr;
          if (ws.readyState !== WebSocket.OPEN) {
            ws = openUploadSocket();
            ws.binaryType = 'arraybuffer';
            await waitForOpen(ws);
            ws.send('start|' + file.size);
            reply = await waitForMessage(ws, ACK_TIMEOUT_MS);
            if (!reply.startsWith('ready|')) {
              throw new Error(reply || 'resume failed');
            }
            const resumedOffset = Number(reply.substring(6) || 0);
            if (resumedOffset !== offset) {
              throw new Error('resume offset mismatch ' + resumedOffset + ' vs ' + offset);
            }
          }
          await new Promise((resolve) => setTimeout(resolve, 250));
        }
      }
      if (!done) {
        ws.close();
        throw new Error('Chunk upload failed at offset ' + offset + ': ' + lastErr);
      }
    }

    ws.send('finish');
    reply = await waitForMessage(ws, ACK_TIMEOUT_MS);
    ws.close();
    if (reply !== 'done') {
      throw new Error(reply || 'upload finish failed');
    }
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

static inline String TS() {
  uint32_t t = millis();
  String s = String(t);
  while(s.length() < 10) s = " " + s;   // pad space trái đến 10
  return "[" + s + "] ";
}

WiFiOtaWebSerial::WiFiOtaWebSerial() {}

WiFiOtaWebSerial::~WiFiOtaWebSerial() {
  end();
}

bool WiFiOtaWebSerial::begin(const Config& cfg) {
  cfg_ = cfg;
  if (!cfg_.ssid || cfg_.ssid[0] == '\0') return false;

  // mutex
  if (!logMutex_) {
    logMutex_ = xSemaphoreCreateMutex();
  }

  // server
  if (web_) { delete web_; web_ = nullptr; }
  if (uploadWeb_) { delete uploadWeb_; uploadWeb_ = nullptr; }
  if (uploadWs_) { delete uploadWs_; uploadWs_ = nullptr; }
  if (cfg_.enableWeb) {
    web_ = new WebServerWrapper(cfg_.port);
  }
  if (cfg_.enableUploadWeb) {
    uploadWeb_ = new WebServerWrapper(cfg_.uploadPort);
    uploadWs_ = new UploadWsWrapper(uploadWsPort_());
  }

  // reserve log
  log_.reserve(cfg_.logBufferBytes);

  setupWiFi_();
  setupOta_();
  if (cfg_.enableWeb && web_) {
    setupWeb_();
  }
  if (cfg_.enableUploadWeb && uploadWeb_) {
    setupUploadWeb_();
    setupUploadWs_();
  }

  started_ = true;

  // start service task pinned to core
  if (wifiTask_) {
    vTaskDelete(wifiTask_);
    wifiTask_ = nullptr;
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

  println(String("OTA Hostname: ") + cfg_.hostname);
  return true;
}

void WiFiOtaWebSerial::end() {
  if (wifiTask_) {
    vTaskDelete(wifiTask_);
    wifiTask_ = nullptr;
  }
  started_ = false;

  if (web_) {
    delete web_;
    web_ = nullptr;
  }
  if (uploadWeb_) {
    delete uploadWeb_;
    uploadWeb_ = nullptr;
  }
  if (uploadWs_) {
    delete uploadWs_;
    uploadWs_ = nullptr;
  }

  if (logMutex_) {
    vSemaphoreDelete(logMutex_);
    logMutex_ = nullptr;
  }
}

void WiFiOtaWebSerial::loopNoService() {
  // Intentionally empty: WiFi/OTA/Web runs in pinned task.
  // Keep your main app logic in Arduino loop().
  

  ArduinoOTA.handle();   // run continuously
}

void WiFiOtaWebSerial::wifiTaskThunk_(void* arg) {
  static_cast<WiFiOtaWebSerial*>(arg)->wifiTaskLoop_();
}

void WiFiOtaWebSerial::wifiTaskLoop_() {
  uint32_t lastReconnectMs = 0;
  bool urlsAnnounced = false;
  for (;;) {
    if (started_) {
      serviceUpdateLed_();
      const wl_status_t wifiStatus = WiFi.status();
      if (wifiStatus != WL_CONNECTED) {
        urlsAnnounced = false;
        if (otaInProgress || webUploadInProgress_ || rebootPending_) {
          vTaskDelay(pdMS_TO_TICKS(50));
          continue;
        }
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
        vTaskDelay(pdMS_TO_TICKS(250));
        continue;
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
        ArduinoOTA.handle();
        if (cfg_.enableUploadWeb && uploadWeb_) uploadWeb_->server.handleClient();
        serviceUploadWs_();
        // Optional: skip web server to reduce load
        // if (web_) web_->server.handleClient();
        vTaskDelay(1);
        continue;
      }
      if (webUploadInProgress_) {
        if (cfg_.enableUploadWeb && uploadWeb_) uploadWeb_->server.handleClient();
        serviceUploadWs_();
        vTaskDelay(1);
        continue;
      }
      // Normal mode
      ArduinoOTA.handle();
      if (cfg_.enableWeb && web_) web_->server.handleClient();
      if (cfg_.enableUploadWeb && uploadWeb_) uploadWeb_->server.handleClient();
      serviceUploadWs_();
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

void WiFiOtaWebSerial::print(const String& s, bool mirrorToSerial) {
  if (cfg_.enableWeb) {
    if (atLineStart_) {
      appendLog_(TS());
      atLineStart_ = false;
    }
    appendLog_(s);
  } else {
    atLineStart_ = false;
  }

  const bool serialOutputAllowed =
    serialOutputAllowedFn_ ? serialOutputAllowedFn_() : AppConfig::Debug::ENABLE_SERIAL_OUTPUT;
  if (mirrorToSerial && serialOutputAllowed) Serial.print(s);
}

void WiFiOtaWebSerial::println(const String& s, bool mirrorToSerial) {
  if (cfg_.enableWeb) {
    if (atLineStart_) {
      appendLog_(TS());
    }
    appendLog_(s + "\n");
  }
  atLineStart_ = true;

  const bool serialOutputAllowed =
    serialOutputAllowedFn_ ? serialOutputAllowedFn_() : AppConfig::Debug::ENABLE_SERIAL_OUTPUT;
  if (mirrorToSerial && serialOutputAllowed) Serial.println(s);
}

void WiFiOtaWebSerial::clear() {
  if (logMutex_) xSemaphoreTake(logMutex_, portMAX_DELAY);
  log_ = "";
  if (logMutex_) xSemaphoreGive(logMutex_);
}

String WiFiOtaWebSerial::ip() const {
  return WiFi.isConnected() ? WiFi.localIP().toString() : String("0.0.0.0");
}

void WiFiOtaWebSerial::setupWiFi_() {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.begin(cfg_.ssid, cfg_.pass);
  ensureWiFiConnected_(cfg_.wifiConnectTimeoutMs);
}
bool WiFiOtaWebSerial::ensureWiFiConnected_(uint32_t timeoutMs) {
  const uint32_t start = millis();
  const bool serialOutputAllowed =
    serialOutputAllowedFn_ ? serialOutputAllowedFn_() : AppConfig::Debug::ENABLE_SERIAL_OUTPUT;
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(250));
    if (timeoutMs > 0 && millis() - start >= timeoutMs) {
      if (serialOutputAllowed) Serial.println("[WiFi] connect timeout");
      return false;
    }
  }
  if (serialOutputAllowed) {
    Serial.println("[WiFi] Connected");
    Serial.println(String("[WiFi] IP: ") + WiFi.localIP().toString());
  }
  if (MDNS.begin(cfg_.hostname)) {
    if (cfg_.enableWeb) {
      MDNS.addService("http", "tcp", cfg_.port);
    }
    if (cfg_.enableUploadWeb) {
      MDNS.addService("http", "tcp", cfg_.uploadPort);
    }
    MDNS.addService("arduino", "tcp", 3232);
  }
  return true;
}

void WiFiOtaWebSerial::setupOta_() {
  ArduinoOTA.setHostname(cfg_.hostname);

  ArduinoOTA
    .onStart([this]() {
      otaInProgress = true;
      ledBlinkMs_ = 0;
      ledBlinkOn_ = true;
      setLedState_("blue");

      println("[OTA] Start");

      // Enter OTA safe mode
      WiFi.setSleep(false);     // improve stability
    })

    .onEnd([this]() {
      println("[OTA] End");
      setLedState_("green");

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
      setLedState_("red");

      otaInProgress = false;
    });

  ArduinoOTA.begin();
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

  srv.begin();
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
    html.replace("%UPLOAD_WS_PORT%", String(uploadWsPort_()));
    uploadWeb_->server.send(200, "text/html; charset=utf-8", html);
  });

  srv.on("/upload/start", HTTP_POST, [this]() {
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
    rebootPending_ = false;
    rebootAtMs_ = 0;
    ledBlinkMs_ = 0;
    ledBlinkOn_ = true;
    setLedState_("blue");
    println(String("[WEB OTA] Chunked start total=") + totalSize);

    if (!Update.begin(totalSize)) {
      Update.printError(Serial);
      setLedState_("red");
      webUploadInProgress_ = false;
      uploadWeb_->server.send(500, "application/json", "{\"ok\":false,\"error\":\"update begin failed\"}");
      return;
    }

    chunkUploadActive_ = true;
    uploadWeb_->server.send(200, "application/json", "{\"ok\":true,\"nextOffset\":0}");
  });

  srv.on("/upload/chunk", HTTP_POST,
    [this]() {
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
        chunkRequestOffset_ = (size_t)uploadWeb_->server.arg("offset").toInt();
        chunkRequestBytes_ = 0;
        chunkRequestOk_ = false;
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
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
          chunkRequestOk_ = false;
          chunkRequestError_ = "flash write failed";
          setLedState_("red");
          return;
        }
        delay(0);
        chunkRequestBytes_ += upload.currentSize;
      } else if (upload.status == UPLOAD_FILE_END) {
        if (!chunkRequestOk_) return;
        if (chunkRequestSkip_) return;
        chunkExpectedOffset_ += chunkRequestBytes_;
        if (chunkExpectedOffset_ > chunkTotalSize_) {
          chunkRequestOk_ = false;
          chunkRequestError_ = "size overflow";
          setLedState_("red");
          return;
        }
        if ((chunkExpectedOffset_ % (128 * 1024)) < chunkRequestBytes_) {
          println(String("[WEB OTA] Chunked received ") + chunkExpectedOffset_ + " bytes");
        }
      } else if (upload.status == UPLOAD_FILE_ABORTED) {
        chunkRequestOk_ = false;
        chunkRequestError_ = "chunk aborted";
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
    if (ok) {
      println(String("[WEB OTA] Chunked success: ") + chunkTotalSize_ + " bytes");
      setLedState_("green");
      rebootPending_ = true;
      rebootAtMs_ = millis() + 1500;
      uploadWeb_->server.send(200, "application/json", "{\"ok\":true}");
    } else {
      Update.printError(Serial);
      setLedState_("red");
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
        ledBlinkMs_ = 0;
        ledBlinkOn_ = true;
        setLedState_("blue");
        println(String("[WEB OTA] Start: ") + upload.filename);
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          Update.printError(Serial);
          setLedState_("red");
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (upload.buf == nullptr || upload.currentSize == 0) {
          return;
        }
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
          setLedState_("red");
        } else {
          delay(0);
          if ((upload.totalSize % (128 * 1024)) < upload.currentSize) {
          println(String("[WEB OTA] Received ") + upload.totalSize + " bytes");
          }
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
          println(String("[WEB OTA] Success: ") + upload.totalSize + " bytes");
          setLedState_("green");
        } else {
          Update.printError(Serial);
          setLedState_("red");
        }
      } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        webUploadInProgress_ = false;
        setLedState_("red");
        println("[WEB OTA] Aborted");
      }
    }
  );

  srv.onNotFound([this]() {
    uploadWeb_->server.send(404, "text/plain", "Not found");
  });

  srv.begin();
}

void WiFiOtaWebSerial::setupUploadWs_() {
  if (!uploadWs_) return;
  uploadWs_->server.begin();
  uploadWs_->handshaken = false;
}

bool WiFiOtaWebSerial::handleUploadWsHandshake_() {
  if (!uploadWs_ || !uploadWs_->client || !uploadWs_->client.connected()) return false;

  uploadWs_->client.setTimeout(50);
  String key;
  while (uploadWs_->client.connected()) {
    String line = uploadWs_->client.readStringUntil('\n');
    if (line.length() == 0) break;
    line.trim();
    if (line.startsWith("Sec-WebSocket-Key:")) {
      key = line.substring(strlen("Sec-WebSocket-Key:"));
      key.trim();
    }
    if (line.length() == 0) break;
  }

  if (key.length() == 0) {
    uploadWs_->client.stop();
    return false;
  }

  const String acceptSrc = key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  unsigned char sha1[20];
  mbedtls_sha1((const unsigned char*)acceptSrc.c_str(), acceptSrc.length(), sha1);

  unsigned char base64Out[64];
  size_t outLen = 0;
  mbedtls_base64_encode(base64Out, sizeof(base64Out), &outLen, sha1, sizeof(sha1));
  const String acceptKey = String((const char*)base64Out).substring(0, outLen);

  uploadWs_->client.print(
    "HTTP/1.1 101 Switching Protocols\r\n"
    "Upgrade: websocket\r\n"
    "Connection: Upgrade\r\n"
    "Sec-WebSocket-Accept: " + acceptKey + "\r\n\r\n");

  uploadWs_->handshaken = true;
  return true;
}

bool WiFiOtaWebSerial::readUploadWsFrameHeader_(uint8_t& opcode, uint64_t& len, bool& masked, uint8_t mask[4]) {
  if (!uploadWs_ || !uploadWs_->client || !uploadWs_->client.connected() || uploadWs_->client.available() < 2) {
    return false;
  }

  const uint8_t b0 = uploadWs_->client.read();
  const uint8_t b1 = uploadWs_->client.read();
  opcode = b0 & 0x0F;
  len = b1 & 0x7F;

  if (len == 126) {
    while (uploadWs_->client.available() < 2) yield();
    len = ((uint16_t)uploadWs_->client.read() << 8) | (uint16_t)uploadWs_->client.read();
  } else if (len == 127) {
    while (uploadWs_->client.available() < 8) yield();
    len = 0;
    for (int i = 0; i < 8; ++i) {
      len = (len << 8) | (uint64_t)uploadWs_->client.read();
    }
  }

  masked = (b1 & 0x80) != 0;
  for (int i = 0; i < 4; ++i) mask[i] = 0;
  if (masked) {
    while (uploadWs_->client.available() < 4) yield();
    for (int i = 0; i < 4; ++i) mask[i] = uploadWs_->client.read();
  }
  return true;
}

bool WiFiOtaWebSerial::readUploadWsTextFrame_(String& payload, uint8_t opcode, uint64_t len, bool masked, const uint8_t mask[4]) {
  if (opcode != 0x1) return false;
  payload = "";
  payload.reserve((size_t)len);
  for (uint64_t i = 0; i < len; ++i) {
    while (uploadWs_->client.available() < 1) yield();
    char ch = (char)uploadWs_->client.read();
    if (masked) ch = (char)(ch ^ mask[i & 3]);
    payload += ch;
  }
  return true;
}

bool WiFiOtaWebSerial::handleUploadWsBinaryFrame_(uint64_t len, bool masked, const uint8_t mask[4]) {
  if (!chunkUploadActive_) {
    sendUploadWsText_("error|no active upload session");
    return false;
  }

  static uint8_t buf[256];
  size_t bufUsed = 0;
  for (uint64_t i = 0; i < len; ++i) {
    while (uploadWs_->client.available() < 1) yield();
    uint8_t ch = (uint8_t)uploadWs_->client.read();
    if (masked) ch ^= mask[i & 3];
    buf[bufUsed++] = ch;
    if (bufUsed == sizeof(buf)) {
      if (Update.write(buf, bufUsed) != bufUsed) {
        Update.printError(Serial);
        setLedState_("red");
        sendUploadWsText_("error|flash write failed");
        return false;
      }
      delay(0);
      chunkExpectedOffset_ += bufUsed;
      bufUsed = 0;
    }
  }

  if (bufUsed > 0) {
    if (Update.write(buf, bufUsed) != bufUsed) {
      Update.printError(Serial);
      setLedState_("red");
      sendUploadWsText_("error|flash write failed");
      return false;
    }
    delay(0);
    chunkExpectedOffset_ += bufUsed;
  }

  if (chunkExpectedOffset_ > chunkTotalSize_) {
    setLedState_("red");
    sendUploadWsText_("error|size overflow");
    return false;
  }

  if ((chunkExpectedOffset_ % (128 * 1024)) < len) {
    println(String("[WEB OTA] WS received ") + chunkExpectedOffset_ + " bytes");
  }
  sendUploadWsText_("ack|" + String(chunkExpectedOffset_));
  return true;
}

void WiFiOtaWebSerial::sendUploadWsText_(const String& text) {
  if (!uploadWs_ || !uploadWs_->client || !uploadWs_->client.connected() || !uploadWs_->handshaken) return;

  const size_t len = text.length();
  uploadWs_->client.write((uint8_t)0x81);
  if (len < 126) {
    uploadWs_->client.write((uint8_t)len);
  } else if (len < 65536) {
    uploadWs_->client.write((uint8_t)126);
    uploadWs_->client.write((uint8_t)((len >> 8) & 0xFF));
    uploadWs_->client.write((uint8_t)(len & 0xFF));
  } else {
    uploadWs_->client.write((uint8_t)127);
    for (int i = 7; i >= 0; --i) {
      uploadWs_->client.write((uint8_t)((((uint64_t)len) >> (i * 8)) & 0xFF));
    }
  }
  uploadWs_->client.write((const uint8_t*)text.c_str(), len);
}

void WiFiOtaWebSerial::serviceUploadWs_() {
  if (!uploadWs_) return;

  if ((!uploadWs_->client || !uploadWs_->client.connected()) && uploadWs_->server.hasClient()) {
    if (uploadWs_->client) uploadWs_->client.stop();
    uploadWs_->client = uploadWs_->server.accept();
    uploadWs_->handshaken = false;
  }

  if (!uploadWs_->client || !uploadWs_->client.connected()) return;

  if (!uploadWs_->handshaken) {
    if (!handleUploadWsHandshake_()) return;
  }

  while (uploadWs_->client.available() > 1) {
    uint8_t opcode = 0;
    uint64_t len = 0;
    bool masked = false;
    uint8_t mask[4] = {0, 0, 0, 0};
    if (!readUploadWsFrameHeader_(opcode, len, masked, mask)) break;

    if (opcode == 0x8) {
      uploadWs_->client.stop();
      uploadWs_->handshaken = false;
      return;
    }

    if (opcode == 0x9) {
      uploadWs_->client.write((uint8_t)0x8A);
      uploadWs_->client.write((uint8_t)0x00);
      continue;
    }

    if (opcode == 0x1) {
      String payload;
      if (!readUploadWsTextFrame_(payload, opcode, len, masked, mask)) {
        sendUploadWsText_("error|bad text frame");
        continue;
      }

      if (payload.startsWith("start|")) {
        const size_t totalSize = (size_t)payload.substring(6).toInt();
        if (totalSize == 0) {
          sendUploadWsText_("error|invalid size");
          continue;
        }

        if (chunkUploadActive_) {
          if (chunkTotalSize_ != totalSize) {
            Update.abort();
            chunkUploadActive_ = false;
            webUploadInProgress_ = false;
            chunkExpectedOffset_ = 0;
            chunkTotalSize_ = 0;
          } else {
            sendUploadWsText_("ready|" + String(chunkExpectedOffset_));
            continue;
          }
        }

        webUploadInProgress_ = true;
        chunkUploadActive_ = false;
        chunkExpectedOffset_ = 0;
        chunkTotalSize_ = totalSize;
        rebootPending_ = false;
        rebootAtMs_ = 0;
        ledBlinkMs_ = 0;
        ledBlinkOn_ = true;
        setLedState_("blue");
        println(String("[WEB OTA] WebSocket start total=") + totalSize);

        if (!Update.begin(totalSize)) {
          Update.printError(Serial);
          webUploadInProgress_ = false;
          setLedState_("red");
          sendUploadWsText_("error|update begin failed");
          continue;
        }

        chunkUploadActive_ = true;
        sendUploadWsText_("ready|0");
        continue;
      }

      if (payload == "finish") {
        if (!chunkUploadActive_) {
          sendUploadWsText_("error|no active upload session");
          continue;
        }
        if (chunkExpectedOffset_ != chunkTotalSize_) {
          sendUploadWsText_("error|incomplete upload");
          continue;
        }
        const bool ok = Update.end(true);
        chunkUploadActive_ = false;
        webUploadInProgress_ = false;
        if (ok) {
          println(String("[WEB OTA] WebSocket success: ") + chunkTotalSize_ + " bytes");
          setLedState_("green");
          rebootPending_ = true;
          rebootAtMs_ = millis() + 1500;
          sendUploadWsText_("done");
        } else {
          Update.printError(Serial);
          setLedState_("red");
          sendUploadWsText_("error|update end failed");
        }
        continue;
      }

      sendUploadWsText_("error|unknown command");
      continue;
    }

    if (opcode == 0x2) {
      if (!handleUploadWsBinaryFrame_(len, masked, mask)) {
        chunkUploadActive_ = false;
        webUploadInProgress_ = false;
        Update.abort();
      }
      continue;
    }
  }
}

void WiFiOtaWebSerial::appendLog_(const String& s) {
  if (logMutex_) xSemaphoreTake(logMutex_, portMAX_DELAY);

  log_ += s;

  if (log_.length() > cfg_.logBufferBytes) {
    const size_t over = log_.length() - cfg_.logBufferBytes;
    const size_t cut  = min(over + (cfg_.logBufferBytes / 10), log_.length());
    log_.remove(0, cut);
  }

  if (logMutex_) xSemaphoreGive(logMutex_);
}

void WiFiOtaWebSerial::setLedState_(const String& cmd) {
  if (!ledCommandHandler_) return;
  String response;
  ledCommandHandler_(cmd, response);
}

void WiFiOtaWebSerial::serviceUpdateLed_() {
  if (!(otaInProgress || webUploadInProgress_)) return;

  const uint32_t now = millis();
  if (ledBlinkMs_ == 0) {
    ledBlinkMs_ = now;
    return;
  }

  if ((now - ledBlinkMs_) < 250) {
    return;
  }

  ledBlinkMs_ = now;
  ledBlinkOn_ = !ledBlinkOn_;
  setLedState_(ledBlinkOn_ ? "blue" : "off");
}



