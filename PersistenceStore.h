#pragma once

#include <Arduino.h>

#include "FloodFillExplorer.h"

namespace PersistenceStore {

using LogFn = void(*)(const String&);

void setLogger(LogFn fn);
bool begin();
bool loadMaze(FloodFillExplorer& explorer, uint8_t mouseX, uint8_t mouseY, FloodFillExplorer::Dir mouseH);
bool saveMaze(const FloodFillExplorer& explorer);
bool clearMaze();

}  // namespace PersistenceStore
