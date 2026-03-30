#pragma once
#include <Arduino.h>

#if defined(ESP32)
  #include <WebServer.h>
#else
  #include <WebServer.h> // Depending on the core, you can switch to ESP8266WebServer if needed.
#endif

class FloodFillExplorer {
public:
  static constexpr int N = 16;

  enum Dir : uint8_t { NORTH=0, EAST=1, SOUTH=2, WEST=3 };

  enum : uint8_t {
    WALL_N = 1,
    WALL_E = 2,
    WALL_S = 4,
    WALL_W = 8
  };

  enum Action : uint8_t {
    ACT_NONE = 0,
    ACT_TURN_L,
    ACT_TURN_R,
    ACT_TURN_180,
    ACT_MOVE_F
  };

  struct PlanNode {
    uint8_t x;
    uint8_t y;
    uint8_t h;
  };

  static constexpr uint16_t kMaxPlan = 512;

  struct Config {
    uint16_t port = 80;
    uint16_t wsPort = 83;
    bool enableWeb = true;
    bool autoRun = false;

    // ACK timeout handling (ms). 0 = disable
    uint32_t ackTimeoutMs = 2000;

    // true: timeout -> pause (safe)
    // false: timeout -> discard pending and continue running
    bool pauseOnAckTimeout = true;
  };

  using LogFn = void(*)(const String&);
  using WebCommandFn = void(*)(const String&);
  using StateExtrasJsonFn = String(*)();

  FloodFillExplorer();
  ~FloodFillExplorer();

  bool begin(const Config& cfg);
  void loop();

  void setLog(LogFn fn) { logFn_ = fn; }
  void setWebCommandHandler(WebCommandFn fn) { webCommandFn_ = fn; }
  void setStateExtrasJsonProvider(StateExtrasJsonFn fn) { stateExtrasJsonFn_ = fn; }
  void setHardwareMode(bool en);
  void notifyStateChanged();

  void setStart(uint8_t x, uint8_t y, Dir h);
  void setHomeRect(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h);
  void setGoalRect(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h);
  void setRunning(bool en);
  void clearKnownMaze();
  void syncPose(uint8_t x, uint8_t y, Dir h, bool markVisited = true);
  void observeRelativeWalls(uint8_t x, uint8_t y, Dir heading,
                            bool leftWall, bool frontWall, bool rightWall,
                            bool leftValid = true, bool frontValid = true, bool rightValid = true);
  Action requestNextAction();
  Action requestNextActionNoAck();
  bool ackPendingActionExternal(bool ok, uint8_t x, uint8_t y, Dir h);

  // SIM truth walls (optional). If you don't call this, truthWalls_ default 0 (no walls),
  // robot mode should override senseCell_ later by real sensors.
  void setTruthFromWalls(const uint8_t walls16[N][N],
                         bool normalizePairs = true,
                         bool forceBoundaries = true);

  // optional external controls
  bool isRunning() const { return running_; }
  bool isWaitAck() const { return waitAck_; }
  uint32_t pendingSeq() const { return pendingSeq_; }
  Action pendingAction() const { return pendingAction_; }
  bool atGoal() const { return atActiveTarget_(); }
  void advanceTargetAfterReach();
  bool getKnownWall(uint8_t x, uint8_t y, Dir d, bool& known, bool& wall) const;
  String buildKnownMazeAscii(uint8_t mouseX = 255, uint8_t mouseY = 255, Dir mouseH = NORTH) const;
  uint16_t bestKnownCostOriginalStartToGoal() const;
  bool isInOriginalStart(uint8_t x, uint8_t y) const;
  bool isInOriginalGoal(uint8_t x, uint8_t y) const;
  void exportKnownMaze(uint8_t walls[N][N], uint8_t mask[N][N], uint8_t visited[N][N]) const;
  bool importKnownMaze(const uint8_t walls[N][N], const uint8_t mask[N][N], const uint8_t visited[N][N],
                       uint8_t mouseX, uint8_t mouseY, Dir mouseH);
  
private:
  // --- web handlers ---
  void setupWeb_();
  void setupWs_();
  void handleRoot_();
  void handleState_();
  void handleCmd_();
  void handleNext_();
  void handleAck_();
  void serviceWs_();
  bool handleWsHandshake_();
  bool readWsFrame_(String& payload, uint8_t& opcode);
  void sendWsText_(const String& text);
  void sendWsState_();
  void processWsMessage_(const String& msg);
  String jsonEscape_(const String& s) const;
  void sendWsReply_(const String& kind, const String& msg);

  // --- state / json ---
  void buildStateJson_();
  void markDirty_();
  const char* actionName_(Action a) const;

  // --- floodfill core ---
  bool inBounds_(int x,int y) const;
  bool isGoal_(int x,int y) const;
  bool atActiveTarget_() const;
  uint16_t computeBestKnownCost_(uint8_t startX0, uint8_t startY0,
                                 uint8_t startW, uint8_t startH,
                                 uint8_t goalX0, uint8_t goalY0,
                                 uint8_t goalW, uint8_t goalH) const;

  uint8_t bitForDir_(Dir d) const;
  Dir opposite_(Dir d) const;

  void clearKnown_();
  void applyBoundaryWalls_();
  bool truthHasWall_(int x,int y, Dir d) const;
  bool knownHasWall_(int x,int y, Dir d) const;
  void knownSetWallBoth_(int x,int y, Dir d, bool on);
  bool confirmObservedWall_(int x, int y, Dir d, bool on);

  void senseCell_(int x,int y);

  void reset();
  void computeFloodFill_();
  void computePlan_();

  // --- ack-driven action ---
  Action chooseNextAction_();
  void dispatchAction_(Action a);
  void commitPendingAction_();
  bool performStepMove_(String& reply);
  void onGoalReached_();

  void log_(const String& s);
  // --- remember original start/goal so we can toggle ---
  uint8_t origSx_ = 0, origSy_ = 0;
  Dir     origSh_ = NORTH;
  uint8_t origHx0_ = 0, origHy0_ = 15, origHw_ = 1, origHh_ = 1;
  uint8_t origGx0_ = 7, origGy0_ = 7, origGw_ = 2, origGh_ = 2;

  // true: currently targeting HOME (the original start), false: currently targeting the original GOAL (2x2)
  bool    targetHome_ = false;


private:
  WebServer* server_ = nullptr;
  class WsServerWrapper;
  WsServerWrapper* ws_ = nullptr;
  Config cfg_{};

  bool started_ = false;
  bool running_ = false;
  bool hardwareMode_ = false;

  uint8_t sx_ = 0, sy_ = 15, mx_ = 0, my_ = 15;
  Dir sh_ = NORTH, mh_ = NORTH;

  uint8_t hx0_ = 0, hy0_ = 15, hw_ = 1, hh_ = 1;
  uint8_t gx0_ = 7, gy0_ = 7, gw_ = 2, gh_ = 2;

  bool visited_[N][N]{};
  uint16_t dist_[N][N]{};

  uint8_t knownWalls_[N][N]{};
  uint8_t knownMask_[N][N]{};
  uint8_t truthWalls_[N][N]{};

  PlanNode plan_[kMaxPlan]{};
  uint16_t planLen_ = 0;

  uint32_t stateVer_ = 0;
  String stateJson_;
  volatile bool wsStatePending_ = false;

  // ACK state
  bool waitAck_ = false;
  uint32_t pendingSeq_ = 0;
  Action pendingAction_ = ACT_NONE;
  uint32_t pendingSinceMs_ = 0;

  LogFn logFn_ = nullptr;
  WebCommandFn webCommandFn_ = nullptr;
  StateExtrasJsonFn stateExtrasJsonFn_ = nullptr;
};

