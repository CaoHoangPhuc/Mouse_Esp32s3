#pragma once
#include <Arduino.h>

#if defined(ESP32)
  #include <WebServer.h>
#else
  #include <WebServer.h> // tuỳ core, bạn có thể đổi sang ESP8266WebServer nếu cần
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
    bool autoRun = false;

    // ACK timeout handling (ms). 0 = disable
    uint32_t ackTimeoutMs = 2000;

    // true: timeout -> pause (safe)
    // false: timeout -> discard pending and continue running
    bool pauseOnAckTimeout = true;
  };

  using LogFn = void(*)(const String&);

  FloodFillExplorer();
  ~FloodFillExplorer();

  bool begin(const Config& cfg);
  void loop();

  void setLog(LogFn fn) { logFn_ = fn; }
  void setHardwareMode(bool en);

  void setStart(uint8_t x, uint8_t y, Dir h);
  void setGoalRect(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h);
  void setRunning(bool en);
  void clearKnownMaze();
  void syncPose(uint8_t x, uint8_t y, Dir h, bool markVisited = true);
  void observeRelativeWalls(uint8_t x, uint8_t y, Dir heading,
                            bool leftWall, bool frontWall, bool rightWall,
                            bool leftValid = true, bool frontValid = true, bool rightValid = true);
  Action requestNextAction();
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
  bool atGoal() const { return isGoal_(mx_, my_); }
  
private:
  // --- web handlers ---
  void setupWeb_();
  void handleRoot_();
  void handleState_();
  void handleCmd_();
  void handleNext_();
  void handleAck_();

  // --- state / json ---
  void buildStateJson_();
  void markDirty_();
  const char* actionName_(Action a) const;

  // --- floodfill core ---
  bool inBounds_(int x,int y) const;
  bool isGoal_(int x,int y) const;

  uint8_t bitForDir_(Dir d) const;
  Dir opposite_(Dir d) const;

  void clearKnown_();
  void applyBoundaryWalls_();
  bool truthHasWall_(int x,int y, Dir d) const;
  bool knownHasWall_(int x,int y, Dir d) const;
  void knownSetWallBoth_(int x,int y, Dir d, bool on);

  void senseCell_(int x,int y);

  void reset();
  void computeFloodFill_();
  void computePlan_();

  // --- ack-driven action ---
  Action chooseNextAction_();
  void dispatchAction_(Action a);
  void commitPendingAction_();
  void onGoalReached_();

  void log_(const String& s);
  // --- remember original start/goal so we can toggle ---
  uint8_t origSx_ = 0, origSy_ = 0;
  Dir     origSh_ = NORTH;
  uint8_t origGx0_ = 7, origGy0_ = 7, origGw_ = 2, origGh_ = 2;

  bool    origCaptured_ = false;

  // true: đang nhắm về HOME (start gốc), false: đang nhắm về GOAL gốc (2x2)
  bool    targetHome_ = false;


private:
  WebServer* server_ = nullptr;
  Config cfg_{};

  bool started_ = false;
  bool running_ = false;
  bool hardwareMode_ = false;

  uint8_t sx_ = 0, sy_ = 15, mx_ = 0, my_ = 15;
  Dir sh_ = NORTH, mh_ = NORTH;

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

  // ACK state
  bool waitAck_ = false;
  uint32_t pendingSeq_ = 0;
  Action pendingAction_ = ACT_NONE;
  uint32_t pendingSinceMs_ = 0;

  LogFn logFn_ = nullptr;
};
