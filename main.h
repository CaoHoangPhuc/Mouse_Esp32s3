#pragma once

#include <Arduino.h>

namespace MainApp {

void setupApp(TaskFunction_t userTaskFn, TaskFunction_t plannerTaskFn);
void loopApp();
void userTaskBody(void* arg);
void plannerTaskBody(void* arg);

}  // namespace MainApp
