#include "WiFiOtaWebSerial.h"

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <Update.h>

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
  <title>%HOSTNAME% Terminal</title>
  <style>
    :root { color-scheme: light; }
    body { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace; margin: 0; background: #eef3ea; color: #102017; }
    .wrap { max-width: 980px; margin: 0 auto; padding: 24px; }
    .card { background: #fbfdf9; border-radius: 18px; box-shadow: 0 10px 28px rgba(0,0,0,0.08); padding: 22px; }
    h1 { margin: 0 0 8px; font-size: 1.5rem; font-family: "Segoe UI", Arial, sans-serif; }
    .sub { color: #4f5f55; margin-bottom: 18px; font-family: "Segoe UI", Arial, sans-serif; }
    .meta { display: grid; grid-template-columns: 110px 1fr; gap: 8px 14px; margin-bottom: 18px; font-family: "Segoe UI", Arial, sans-serif; }
    .meta b { color: #355040; }
    .toolbar { display:flex; gap:12px; align-items:center; flex-wrap:wrap; margin-bottom: 14px; }
    button { padding: 11px 14px; cursor: pointer; border: 0; border-radius: 12px; font-weight: 600; font-family: "Segoe UI", Arial, sans-serif; }
    .primary { background: #1d6b45; color: white; }
    .secondary { background: #1d4f6b; color: white; }
    #term { background: #122018; color: #d8f3dc; padding: 14px; border-radius: 14px; min-height: 360px; max-height: 58vh; overflow: auto; white-space: pre-wrap; word-break: break-word; }
    .cmdrow { display:flex; gap:10px; margin-top: 14px; }
    #cmd { flex: 1; padding: 12px 14px; border-radius: 12px; border: 1px solid #b6c9bb; font: inherit; }
    #status { margin-top: 12px; min-height: 1.3em; color: #44584a; font-family: "Segoe UI", Arial, sans-serif; }
  </style>
</head>
<body>
<div class="wrap">
  <div class="card">
    <h1>%HOSTNAME%</h1>
    <div class="sub">Browser terminal on port 80.</div>
    <div class="meta">
      <b>Hostname</b><span>%HOSTNAME%</span>
      <b>IP</b><span>%IP%</span>
      <b>CLI</b><span>Same commands as serial/telnet</span>
    </div>
    <div class="toolbar">
      <button class="secondary" onclick="cycleLed()">Cycle LED</button>
      <button class="secondary" onclick="clearTerminal()">Clear View</button>
    </div>
    <pre id="term"></pre>
    <div class="cmdrow">
      <input id="cmd" type="text" autocomplete="off" spellcheck="false" placeholder="Type a command like status, maze, explore, brake" />
      <button class="primary" onclick="sendCmd()">Send</button>
    </div>
    <div id="status">Ready.</div>
  </div>
</div>
<script>
let busy = false;
let logOffset = 0;
const term = document.getElementById('term');
const cmdInput = document.getElementById('cmd');
function setStatus(s){ document.getElementById('status').textContent = s; }
function scrollTerm(){ term.scrollTop = term.scrollHeight; }

async function fetchLog() {
  if (busy) return;
  try {
    const res = await fetch('/log?from=' + logOffset + '&t=' + Date.now(), { cache: 'no-store' });
    if (!res.ok) throw new Error('HTTP ' + res.status);
    const chunk = await res.text();
    if (!chunk) return;
    if (chunk.startsWith('RESET\n')) {
      term.textContent = '';
      logOffset = 0;
      const rest = chunk.slice(6);
      term.textContent += rest;
      logOffset += rest.length;
    } else {
      term.textContent += chunk;
      logOffset += chunk.length;
    }
    scrollTerm();
  } catch (e) {
    setStatus('Log fetch failed: ' + e);
  }
}

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
    await fetchLog();
  } catch (e) {
    setStatus('LED command failed: ' + e);
  } finally {
    busy = false;
  }
}

async function sendCmd(){
  const line = cmdInput.value.trim();
  if (!line || busy) return;
  busy = true;
  try {
    const res = await fetch('/cmd', {
      method: 'POST',
      headers: { 'Content-Type':'application/x-www-form-urlencoded' },
      body: 'line=' + encodeURIComponent(line),
      cache: 'no-store'
    });
    if (!res.ok) throw new Error('HTTP ' + res.status);
    cmdInput.value = '';
    setStatus('Sent: ' + line);
    await fetchLog();
  } catch (e) {
    setStatus('Command failed: ' + e);
  } finally {
    busy = false;
    cmdInput.focus();
  }
}

async function clearTerminal(){
  term.textContent = '';
  logOffset = 0;
  setStatus('Terminal view cleared.');
  await fetchLog();
}

cmdInput.addEventListener('keydown', (ev) => {
  if (ev.key === 'Enter') {
    ev.preventDefault();
    sendCmd();
  }
});

setInterval(fetchLog, 250);
fetchLog();
cmdInput.focus();
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
    <div class="muted">Upload a compiled firmware binary (`.bin`). The mouse will reboot after a successful update.</div>
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

form.addEventListener('submit', (ev) => {
  ev.preventDefault();
  const file = fileInput.files[0];
  if (!file) {
    msg.textContent = 'Choose a .bin file first.';
    return;
  }

  prog.hidden = false;
  prog.value = 0;
  msg.textContent = 'Uploading...';

  const xhr = new XMLHttpRequest();
  xhr.open('POST', '/update');
  xhr.upload.onprogress = (e) => {
    if (!e.lengthComputable) return;
    prog.value = Math.round((e.loaded / e.total) * 100);
  };
  xhr.onload = () => {
    msg.textContent = xhr.status === 200
      ? 'Upload complete. Device will reboot.'
      : 'Upload failed: HTTP ' + xhr.status + '\n' + xhr.responseText;
  };
  xhr.onerror = () => {
    msg.textContent = 'Upload failed: network error';
  };
  const data = new FormData();
  data.append('firmware', file, file.name);
  xhr.send(data);
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
  if (cfg_.enableWeb) {
    web_ = new WebServerWrapper(cfg_.port);
  }
  if (cfg_.enableUploadWeb) {
    uploadWeb_ = new WebServerWrapper(cfg_.uploadPort);
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

  if (cfg_.enableWeb) {
    println(String("HTTP: http://") + ip() + "/  (or http://" + cfg_.hostname + ".local/ )");
  }
  if (cfg_.enableUploadWeb) {
    println(String("Upload: http://") + ip() + ":" + String(cfg_.uploadPort) + "/");
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
  for (;;) {
    if (started_) {
      serviceUpdateLed_();
      const wl_status_t wifiStatus = WiFi.status();
      if (wifiStatus != WL_CONNECTED) {
        if (otaInProgress || webUploadInProgress_ || rebootPending_) {
          vTaskDelay(pdMS_TO_TICKS(50));
          continue;
        }
        const uint32_t now = millis();
        if (wifiStatus != WL_IDLE_STATUS && now - lastReconnectMs >= cfg_.wifiReconnectIntervalMs) {
          lastReconnectMs = now;
          Serial.println("[WiFi] reconnecting...");
          WiFi.reconnect();
        }
        vTaskDelay(pdMS_TO_TICKS(250));
        continue;
      }
      if (otaInProgress) {
        // OTA priority mode
        ArduinoOTA.handle();
        if (cfg_.enableUploadWeb && uploadWeb_) uploadWeb_->server.handleClient();
        // Optional: skip web server to reduce load
        // if (web_) web_->server.handleClient();
        vTaskDelay(1);
        continue;
      }
      if (webUploadInProgress_) {
        if (cfg_.enableUploadWeb && uploadWeb_) uploadWeb_->server.handleClient();
        vTaskDelay(1);
        continue;
      }
      // Normal mode
      ArduinoOTA.handle();
      if (cfg_.enableWeb && web_) web_->server.handleClient();
      if (cfg_.enableUploadWeb && uploadWeb_) uploadWeb_->server.handleClient();
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

  if (mirrorToSerial) Serial.print(s);
}

void WiFiOtaWebSerial::println(const String& s, bool mirrorToSerial) {
  if (cfg_.enableWeb) {
    if (atLineStart_) {
      appendLog_(TS());
    }
    appendLog_(s + "\n");
  }
  atLineStart_ = true;

  if (mirrorToSerial) Serial.println(s);
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
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(250));
    if (timeoutMs > 0 && millis() - start >= timeoutMs) {
      Serial.println("[WiFi] connect timeout");
      return false;
    }
  }
  Serial.println("[WiFi] Connected");
  Serial.println(String("[WiFi] IP: ") + WiFi.localIP().toString());
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
    web_->server.send(200, "text/html; charset=utf-8", html);
  });

  srv.on("/log", HTTP_GET, [this]() {
    size_t from = 0;
    if (web_->server.hasArg("from")) {
      from = (size_t)web_->server.arg("from").toInt();
    }

    String copy;
    if (logMutex_) xSemaphoreTake(logMutex_, portMAX_DELAY);
    copy = log_;
    if (logMutex_) xSemaphoreGive(logMutex_);

    if (from > copy.length()) {
      web_->server.send(200, "text/plain; charset=utf-8", String("RESET\n") + copy);
      return;
    }

    web_->server.send(200, "text/plain; charset=utf-8", copy.substring(from));
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

  srv.on("/cmd", HTTP_POST, [this]() {
    String line = web_->server.arg("line");
    line.trim();

    if (!commandHandler_) {
      web_->server.send(503, "application/json", "{\"ok\":false,\"error\":\"cli unavailable\"}");
      return;
    }

    String response;
    const bool ok = commandHandler_(line, response);
    String body = String("{\"ok\":") + (ok ? "true" : "false") +
                  ",\"response\":\"" + response + "\"}";
    web_->server.send(ok ? 200 : 500, "application/json", body);
  });

  srv.on("/health", HTTP_GET, [this]() {
    const String host = cfg_.hostname ? String(cfg_.hostname) : String("esp32");
    web_->server.send(200, "application/json",
                      String("{\"ip\":\"") + ip() +
                      "\",\"hostname\":\"" + host + "\"}");
  });

  srv.onNotFound([this]() {
    web_->server.send(404, "text/plain", "Not found");
  });

  srv.begin();
}

void WiFiOtaWebSerial::setupUploadWeb_() {
  auto& srv = uploadWeb_->server;

  srv.on("/", HTTP_GET, [this]() {
    uploadWeb_->server.send(200, "text/html; charset=utf-8", FPSTR(kUploadHtml));
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
        } else if ((upload.totalSize % (128 * 1024)) < upload.currentSize) {
          println(String("[WEB OTA] Received ") + upload.totalSize + " bytes");
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


