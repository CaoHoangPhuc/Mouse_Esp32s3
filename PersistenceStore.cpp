#include "PersistenceStore.h"

#include <SPIFFS.h>

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

void logLine(const String& s) {
  if (gLogFn) gLogFn(s);
}

}  // namespace

void setLogger(LogFn fn) {
  gLogFn = fn;
}

bool begin() {
  if (gInitialized) return gReady;
  gInitialized = true;
  gReady = SPIFFS.begin(true);
  logLine(String("[SPIFFS] ") + (gReady ? "ready" : "failed"));
  return gReady;
}

bool loadMaze(FloodFillExplorer& explorer, uint8_t mouseX, uint8_t mouseY, FloodFillExplorer::Dir mouseH) {
  if (!begin()) return false;
  File f = SPIFFS.open(kMazePath, FILE_READ);
  if (!f) return false;

  PersistedMazeData data{};
  const size_t n = f.readBytes(reinterpret_cast<char*>(&data), sizeof(data));
  f.close();
  if (n != sizeof(data) || data.magic != kMazeMagic) {
    logLine("[SPIFFS] maze restore skipped (invalid file)");
    return false;
  }

  if (!explorer.importKnownMaze(data.walls, data.mask, data.visited, mouseX, mouseY, mouseH)) {
    logLine("[SPIFFS] maze restore failed");
    return false;
  }

  logLine("[SPIFFS] restored maze memory");
  return true;
}

bool saveMaze(const FloodFillExplorer& explorer) {
  if (!begin()) return false;
  if (SPIFFS.exists(kMazePath)) {
    SPIFFS.remove(kMazePath);
  }

  File f = SPIFFS.open(kMazePath, FILE_WRITE);
  if (!f) {
    logLine("[SPIFFS] failed to open maze file for write");
    return false;
  }

  PersistedMazeData data{};
  data.magic = kMazeMagic;
  explorer.exportKnownMaze(data.walls, data.mask, data.visited);

  const bool ok = f.write(reinterpret_cast<const uint8_t*>(&data), sizeof(data)) == sizeof(data);
  f.close();
  if (!ok) {
    logLine("[SPIFFS] failed to save maze memory");
    return false;
  }

  logLine("[SPIFFS] saved maze memory");
  return true;
}

bool clearMaze() {
  if (!begin()) return false;
  if (!SPIFFS.exists(kMazePath)) return true;
  const bool ok = SPIFFS.remove(kMazePath);
  logLine(ok ? "[SPIFFS] cleared saved maze memory" : "[SPIFFS] failed to clear saved maze memory");
  return ok;
}

}  // namespace PersistenceStore
