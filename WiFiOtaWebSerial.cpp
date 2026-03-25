#include "WiFiOtaWebSerial.h"

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>

#include <Adafruit_NeoPixel.h>

// ====== NeoPixel config (fixed pin) ======
#ifndef RGB_PIN
#define RGB_PIN 48
#endif
#ifndef NUM_PIXELS
#define NUM_PIXELS 1
#endif

static Adafruit_NeoPixel gPixels(NUM_PIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

static void ledSet(uint8_t r, uint8_t g, uint8_t b) {
  gPixels.setPixelColor(0, gPixels.Color(r, g, b));
  gPixels.show();
}

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
  <title>ESP32 Web Serial</title>
  <style>
    body { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace; margin: 0; }
    header { padding: 10px 12px; background: #111; color: #eee; position: sticky; top: 0; }
    header small { opacity: 0.8; }
    #log { white-space: pre-wrap; word-break: break-word; padding: 12px; }
    .row { display:flex; gap:8px; align-items:center; flex-wrap:wrap; }
    button { padding: 6px 10px; cursor:pointer; }
    input { width: 90px; }
  </style>
</head>
<body>
<header>
  <div class="row">
    <div><b>ESP32 Web Serial</b> <small id="status">connecting…</small></div>
    <div style="margin-left:auto" class="row">
      <label>Poll(ms)</label>
      <input id="poll" type="number" value="200" min="100" step="100"/>
      <button onclick="clearLog()">Clear</button>
      <button onclick="togglePause()" id="pauseBtn">Pause</button>
      <button onclick="cycleLed()">Cycle LED</button>
      <button onclick="ledOff()">LED OFF</button>
    </div>
  </div>
</header>
<pre id="log"></pre>
<script>
let paused = false;
let logOffset = 0;        // bytes already received
const MAX_UI = 200000;    // cap UI text to avoid browser lag

let busy = false;         // lock when sending POST commands
let stopPoll = false;
let pollMs = 200;

function setStatus(s){ document.getElementById('status').textContent = s; }

function getPollMs(){
  const p = parseInt(document.getElementById('poll').value || '200', 10);
  pollMs = Math.max(100, p|0);
  return pollMs;
}

async function fetchLog(fromCmd=false){
  if(paused) return;
  // không poll trong lúc đang gửi command (tránh đè offset)
  if(busy && !fromCmd) return;

  try {
    const res = await fetch('/log?from=' + logOffset + '&t=' + Date.now(), { cache: 'no-store' });
    if(!res.ok) throw new Error('HTTP ' + res.status);

    const chunk = await res.text();
    if(chunk && chunk.length){
      const el = document.getElementById('log');

      if(chunk.startsWith("RESET\n")){
        el.textContent = "";
        logOffset = 0;

        const rest = chunk.slice("RESET\n".length);
        if(rest.length){
          el.textContent += rest;
          logOffset += rest.length;
        }
      }else{
        el.textContent += chunk;
        logOffset += chunk.length;
      }

      // cap UI buffer
      if(el.textContent.length > MAX_UI){
        el.textContent = el.textContent.slice(el.textContent.length - MAX_UI);
      }

      window.scrollTo(0, document.body.scrollHeight);
    }

    setStatus('online');
  } catch(e){
    setStatus('offline: ' + e);
  }
}

async function clearLog(){
  if(busy) return;
  busy = true;
  try{
    const r = await fetch('/clear', { method: 'POST', cache:'no-store' });
    if(!r.ok) throw new Error('HTTP ' + r.status);

    // reset client state
    document.getElementById('log').textContent = "";
    logOffset = 0;

    await fetchLog(true);
  }catch(e){
    setStatus('offline: ' + e);
  }finally{
    busy = false;
  }
}

function togglePause(){
  paused = !paused;
  document.getElementById('pauseBtn').textContent = paused ? 'Resume' : 'Pause';
}

async function postLed(cmd){
  if(busy) return;
  busy = true;
  try{
    const res = await fetch('/led', {
      method: 'POST',
      headers: { 'Content-Type':'application/x-www-form-urlencoded' },
      body: 'cmd=' + encodeURIComponent(cmd),
      cache:'no-store'
    });
    if(!res.ok) throw new Error('HTTP ' + res.status);
    await fetchLog(true); // refresh sau command
  }catch(e){
    setStatus('offline: ' + e);
  }finally{
    busy = false;
  }
}

function cycleLed(){ postLed('cycle'); }
function ledOff(){ postLed('off'); }

// Polling tuần tự: fetch xong mới sleep rồi fetch tiếp (không overlap)
async function pollLoop(){
  while(!stopPoll){
    await fetchLog(false);
    await new Promise(res => setTimeout(res, getPollMs()));
  }
}

document.getElementById('poll').addEventListener('change', () => { getPollMs(); });

pollLoop();
fetchLog(true);
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
  web_ = new WebServerWrapper(cfg_.port);

  // reserve log
  log_.reserve(cfg_.logBufferBytes);

  // init neopixel (fixed pin)
  gPixels.begin();
  gPixels.setBrightness(255);
  gPixels.show(); // off

  setupWiFi_();
  setupOta_();
  setupWeb_();

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

  println(String("HTTP: http://") + ip() + "/  (or http://" + cfg_.hostname + ".local/ )");
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
  for (;;) {
    if (started_) {

      if (otaInProgress) {
        // 🔥 OTA priority mode

        ArduinoOTA.handle();   // run continuously

        // Optional: skip web server to reduce load
        // if (web_) web_->server.handleClient();

        vTaskDelay(1);  // yield only (VERY IMPORTANT for watchdog)
        continue;
      }

      // 🟢 Normal mode
      ArduinoOTA.handle();

      if (web_) web_->server.handleClient();

      vTaskDelay(pdMS_TO_TICKS(cfg_.serviceDelayMs));
    } else {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

void WiFiOtaWebSerial::print(const String& s, bool mirrorToSerial) {
  if (atLineStart_) {
    appendLog_(TS());
    atLineStart_ = false;
  }

  appendLog_(s);

  if (mirrorToSerial) Serial.print(s);
}

void WiFiOtaWebSerial::println(const String& s, bool mirrorToSerial) {
  if (atLineStart_) {
    appendLog_(TS());
  }

  appendLog_(s + "\n");
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
  WiFi.setSleep(false);

  // 🔥 Set max TX power
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  WiFi.begin(cfg_.ssid, cfg_.pass);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(200));

    if (millis() - start > 1000) break;  // give it more time
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi Connected");

    if (MDNS.begin(cfg_.hostname)) {
      MDNS.addService("http", "tcp", cfg_.port);
    }
  } else {
    Serial.println("WiFi Failed");
  }
}

void WiFiOtaWebSerial::setupOta_() {
  ArduinoOTA.setHostname(cfg_.hostname);
  if (cfg_.otaPassword && cfg_.otaPassword[0] != '\0') {
    ArduinoOTA.setPassword(cfg_.otaPassword);
  }

  ArduinoOTA
    .onStart([this]() {
      otaInProgress = true;

      println("[OTA] Start");

      // 🔥 Enter OTA safe mode
      WiFi.setSleep(false);     // improve stability
    })

    .onEnd([this]() {
      println("[OTA] End");

      // 🔥 Exit OTA mode
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

      otaInProgress = false;
    });

  ArduinoOTA.begin();
}

void WiFiOtaWebSerial::setupWeb_() {
  auto& srv = web_->server;

  srv.on("/", HTTP_GET, [this]() {
    web_->server.send(200, "text/html; charset=utf-8", FPSTR(kIndexHtml));
  });

  // ===== incremental log endpoint =====
  // GET /log?from=<offset>
  // returns:
  //  - "" (empty) if no new bytes
  //  - "RESET\n" + full_log if offset is beyond current buffer (rotated)
  //  - otherwise substring(log, from)
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

    String chunk = copy.substring(from);
    web_->server.send(200, "text/plain; charset=utf-8", chunk);
  });

  srv.on("/clear", HTTP_POST, [this]() {
    clear();
    web_->server.send(200, "text/plain; charset=utf-8", "OK");
  });

  // ===== LED control endpoint =====
  srv.on("/led", HTTP_POST, [this]() {
    String cmd = web_->server.arg("cmd"); // cmd=cycle/off/red/green/blue...
    cmd.toLowerCase();

    static int idx = -1;

    auto cycle = [&](){
      static const uint8_t colors[][3] = {
        {255,   0,   0}, // red
        {  0, 255,   0}, // green
        {  0,   0, 255}, // blue
        {255, 255,   0}, // yellow
        {  0, 255, 255}, // cyan
        {255,   0, 255}, // magenta
        {255, 255, 255}, // white
        {  0,   0,   0}, // off
      };
      const int n = (int)(sizeof(colors) / sizeof(colors[0]));
      idx = (idx + 1) % n;
      ledSet(colors[idx][0], colors[idx][1], colors[idx][2]);
      println(String("[LED] cycle idx=") + idx +
              " rgb=(" + colors[idx][0] + "," + colors[idx][1] + "," + colors[idx][2] + ")");
    };

    if (cmd == "cycle") {
      cycle();
      web_->server.send(200, "text/plain", "OK");
      return;
    }
    if (cmd == "off") {
      ledSet(0, 0, 0);
      println("[LED] off");
      web_->server.send(200, "text/plain", "OK");
      return;
    }
    if (cmd == "red")   { ledSet(255,0,0);   println("[LED] red");   web_->server.send(200,"text/plain","OK"); return; }
    if (cmd == "green") { ledSet(0,255,0);   println("[LED] green"); web_->server.send(200,"text/plain","OK"); return; }
    if (cmd == "blue")  { ledSet(0,0,255);   println("[LED] blue");  web_->server.send(200,"text/plain","OK"); return; }

    web_->server.send(400, "text/plain", "Bad cmd");
  });

  srv.on("/health", HTTP_GET, [this]() {
    web_->server.send(200, "application/json", String("{\"ip\":\"") + ip() + "\"}");
  });

  srv.onNotFound([this]() {
    web_->server.send(404, "text/plain", "Not found");
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
