#include "PersistenceStore.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <SPIFFS.h>
#include <cstring>

namespace PersistenceStore {

namespace {

constexpr uint32_t kMazeMagic = 0x4D415A31u;  // MAZ1
constexpr const char* kMazePath = "/maze_state.bin";

struct PersistedMazeData {
  uint32_t magic;
  uint8_t walls[FloodFillExplorer::N][FloodFillExplorer::N];
  uint8_t mask[FloodFillExplorer::N][FloodFillExplorer::N];
  uint8_t visited[FloodFillExplorer::N][FloodFillExplorer::N];
};

LogFn gLogFn = nullptr;
bool gInitialized = false;
bool gReady = false;
SemaphoreHandle_t gSpiMutex = nullptr;
PersistedMazeData gScratch{};

void logLine(const String& s) {
  if (gLogFn) gLogFn(s);
}

bool ensureMutex() {
  if (gSpiMutex) return true;
  gSpiMutex = xSemaphoreCreateMutex();
  if (!gSpiMutex) {
    logLine("[SPIFFS] mutex create failed");
    return false;
  }
  return true;
}

bool lockSpi() {
  if (!ensureMutex()) return false;
  return xSemaphoreTake(gSpiMutex, pdMS_TO_TICKS(2000)) == pdTRUE;
}

void unlockSpi() {
  if (gSpiMutex) xSemaphoreGive(gSpiMutex);
}

}  // namespace

void setLogger(LogFn fn) {
  gLogFn = fn;
}

bool begin() {
  if (!ensureMutex()) return false;
  if (!lockSpi()) return false;
  if (gInitialized) {
    const bool ready = gReady;
    unlockSpi();
    return ready;
  }
  gInitialized = true;
  gReady = SPIFFS.begin(true);
  logLine(String("[SPIFFS] ") + (gReady ? "ready" : "failed"));
  unlockSpi();
  return gReady;
}

bool loadMaze(FloodFillExplorer& explorer, uint8_t mouseX, uint8_t mouseY, FloodFillExplorer::Dir mouseH) {
  if (!begin()) return false;
  if (!lockSpi()) return false;
  File f = SPIFFS.open(kMazePath, FILE_READ);
  if (!f) {
    unlockSpi();
    return false;
  }

  memset(&gScratch, 0, sizeof(gScratch));
  const size_t n = f.readBytes(reinterpret_cast<char*>(&gScratch), sizeof(gScratch));
  f.close();
  unlockSpi();
  if (n != sizeof(gScratch) || gScratch.magic != kMazeMagic) {
    logLine("[SPIFFS] maze restore skipped (invalid file)");
    return false;
  }

  if (!explorer.importKnownMaze(gScratch.walls, gScratch.mask, gScratch.visited, mouseX, mouseY, mouseH)) {
    logLine("[SPIFFS] maze restore failed");
    return false;
  }

  logLine("[SPIFFS] restored maze memory");
  return true;
}

bool saveMaze(const FloodFillExplorer& explorer) {
  if (!begin()) return false;
  if (!lockSpi()) return false;
  if (SPIFFS.exists(kMazePath)) {
    SPIFFS.remove(kMazePath);
  }

  File f = SPIFFS.open(kMazePath, FILE_WRITE);
  if (!f) {
    unlockSpi();
    logLine("[SPIFFS] failed to open maze file for write");
    return false;
  }

  memset(&gScratch, 0, sizeof(gScratch));
  gScratch.magic = kMazeMagic;
  explorer.exportKnownMaze(gScratch.walls, gScratch.mask, gScratch.visited);

  const bool ok = f.write(reinterpret_cast<const uint8_t*>(&gScratch), sizeof(gScratch)) == sizeof(gScratch);
  f.close();
  unlockSpi();
  if (!ok) {
    logLine("[SPIFFS] failed to save maze memory");
    return false;
  }

  logLine("[SPIFFS] saved maze memory");
  return true;
}

bool clearMaze() {
  if (!begin()) return false;
  if (!lockSpi()) return false;
  if (!SPIFFS.exists(kMazePath)) {
    unlockSpi();
    return true;
  }
  const bool ok = SPIFFS.remove(kMazePath);
  unlockSpi();
  logLine(ok ? "[SPIFFS] cleared saved maze memory" : "[SPIFFS] failed to clear saved maze memory");
  return ok;
}

}  // namespace PersistenceStore
