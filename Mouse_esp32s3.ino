#include "main.h"

static void userTask(void* arg);
static void plannerTask(void* arg);

void setup() {
  MainApp::setupApp(userTask, plannerTask);
}

void loop() {
  MainApp::loopApp();
}

static void userTask(void* arg) {
  MainApp::userTaskBody(arg);
}

static void plannerTask(void* arg) {
  MainApp::plannerTaskBody(arg);
}
