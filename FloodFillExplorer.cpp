#include "FloodFillExplorer.h"

// ============================ HTML UI ============================
// Improvements:
// - fetchWithTimeout() + AbortController to abort hung requests (1~2s)
// - stateInFlight prevents overlapping /state calls
// - 204 still runs doNextIfNeeded + autoAckIfEnabled (state machine keeps going)
// - seq snapshot for autoAck to avoid 409 seq mismatch
// - configurable poll/timeout/delay constants

static const char* kHtml PROGMEM = R"HTML(
<!doctype html><html>
<head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Flood Fill Explorer (ACK)</title>
<style>
  body{font-family:system-ui,Segoe UI,Arial;margin:0}
  header{background:#111;color:#eee;padding:10px 12px;position:sticky;top:0}
  .row{display:flex;gap:10px;align-items:center;flex-wrap:wrap}
  button{padding:8px 12px;cursor:pointer}
  input{padding:6px 8px;width:64px}
  canvas{display:block;margin:0 auto;border-top:1px solid #333}
  small{opacity:.85}
  .lbl{opacity:.85}
  .pill{padding:2px 8px;border-radius:999px;background:rgba(255,255,255,.12)}
  .warn{background:rgba(255,140,0,.18)}
</style>
</head>
<body>
<header>
  <div class="row">
    <b>Flood Fill Explorer</b>
    <small id="st" class="pill">loading…</small>

    <div style="margin-left:auto" class="row">
      <span class="lbl">Start:</span>
      <input id="sx" type="number" min="0" max="15" value="0" />
      <input id="sy" type="number" min="0" max="15" value="15" />
      <input id="sh" type="number" min="0" max="3"  value="0" />
      <button onclick="setStart()">Set Start</button>

      <button onclick="cmd('step')">Step</button>
      <button onclick="cmd('run')">Run</button>
      <button onclick="cmd('pause')">Pause</button>
      <button onclick="cmd('reset')">Reset</button>

      <span class="pill warn">
        AutoACK(sim)
        <input id="autoAck" type="checkbox" checked style="transform:scale(1.2);margin-left:6px"/>
      </span>
    </div>
  </div>
</header>

<canvas id="c" width="720" height="720"></canvas>

<script>
const N=16;
const c=document.getElementById('c');
const ctx=c.getContext('2d');
let S=null;

let busy=false;
let stopPoll=false;

let inputsInit=false;
let lastVer=0;

let nextInFlight=false;
let ackInFlight=false;
let stateInFlight=false;

// ---- tunables ----
const POLL_MS = 200;          // /state poll interval
const TICK_MS = 50;           // state machine tick (next/ack)
const STATE_TO = 300;         // /state timeout
const NEXT_TO  = 300;         // /next timeout
const ACK_TO   = 300;         // /ack timeout
const AUTO_ACK_DELAY_MS = 50; // simulate speed (increase -> slower)

function st(s){
  const el=document.getElementById('st');
  el.textContent=s;
}

function clampInt(v, lo, hi){
  v = (v|0);
  if(v<lo) v=lo;
  if(v>hi) v=hi;
  return v;
}

// --- fetch with abort timeout ---
async function fetchWithTimeout(url, opts={}, timeoutMs=1500){
  const ctrl = new AbortController();
  const t = setTimeout(() => ctrl.abort("timeout"), timeoutMs);
  try{
    return await fetch(url, {...opts, signal: ctrl.signal});
  } finally {
    clearTimeout(t);
  }
}

function initInputsOnce(){
  if(inputsInit) return;
  if(S && S.start){
    document.getElementById('sx').value = S.start.x;
    document.getElementById('sy').value = S.start.y;
    document.getElementById('sh').value = S.start.h;
    inputsInit = true;
  }
}

async function cmd(a){
  if(busy) return;
  busy=true;
  try{
    const r = await fetchWithTimeout('/cmd?a='+encodeURIComponent(a), {cache:'no-store'}, 1200);
    if(!r.ok) st("cmd failed HTTP "+r.status);
    await refresh(true);
  }catch(e){
    st("cmd err: "+e);
  }finally{
    busy=false;
  }
}

async function setStart(){
  if(busy) return;
  busy=true;
  try{
    const x = clampInt(parseInt(document.getElementById('sx').value||"0",10), 0, 15);
    const y = clampInt(parseInt(document.getElementById('sy').value||"15",10), 0, 15);
    const h = clampInt(parseInt(document.getElementById('sh').value||"0",10), 0, 3);

    document.getElementById('sx').value = x;
    document.getElementById('sy').value = y;
    document.getElementById('sh').value = h;

    const url = `/cmd?a=setstart&x=${x}&y=${y}&h=${h}`;
    const r = await fetchWithTimeout(url, {cache:'no-store'}, 1200);
    if(!r.ok) st("setstart failed HTTP "+r.status);

    await refresh(true);
  }catch(e){
    st("setstart err: "+e);
  }finally{
    busy=false;
  }
}

function cellGeom(){
  const W=c.width, H=c.height;
  const pad=20;
  const grid=Math.min(W,H)-pad*2;
  const cs=grid/N;
  const ox=(W-grid)/2, oy=(H-grid)/2;
  return {W,H,pad,grid,cs,ox,oy};
}

function cellCenter(x,y,geom){
  const {cs,ox,oy}=geom;
  return {cx: ox + x*cs + cs*0.5, cy: oy + y*cs + cs*0.5};
}

function drawPlanPath(geom, plan){
  if(!plan || plan.length < 2) return;
  const {cs}=geom;

  ctx.save();
  ctx.lineWidth = Math.max(1.2, cs*0.040);
  ctx.strokeStyle = "rgba(255, 130, 0, 0.65)";
  ctx.setLineDash([cs*0.16, cs*0.11]);

  ctx.beginPath();
  for(let i=0;i<plan.length;i++){
    const a = plan[i];
    const b = plan[i+1] || a;

    const ca = cellCenter(a.x, a.y, geom);
    const cb = cellCenter(b.x, b.y, geom);

    const sdx = Math.sign(b.x - a.x);
    const sdy = Math.sign(b.y - a.y);

    const off = cs * 0.22;
    const ax = ca.cx + sdy * off;
    const ay = ca.cy - sdx * off;
    const bx = cb.cx + sdy * off;
    const by = cb.cy - sdx * off;

    if(i === 0) ctx.moveTo(ax, ay);
    else ctx.lineTo(ax, ay);

    if(i === plan.length-1) ctx.lineTo(bx, by);
  }
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.restore();
}

function draw(){
  if(!S) return;
  const g = cellGeom();
  const {W,H,cs,ox,oy}=g;
  ctx.clearRect(0,0,W,H);

  initInputsOnce();

  // goal rect
  const gx0=S.goal.x0, gy0=S.goal.y0, gw=S.goal.w, gh=S.goal.h;
  ctx.fillStyle="rgba(0,0,0,0.06)";
  ctx.fillRect(ox+gx0*cs, oy+gy0*cs, gw*cs, gh*cs);

  // start marker
  const sx=S.start.x, sy=S.start.y;
  ctx.fillStyle="rgba(0,0,0,0.08)";
  ctx.fillRect(ox+sx*cs, oy+sy*cs, cs, cs);

  // visited shading
  for(let y=0;y<N;y++){
    for(let x=0;x<N;x++){
      if(S.visited[y][x]){
        ctx.fillStyle="rgba(0,0,0,0.03)";
        ctx.fillRect(ox+x*cs, oy+y*cs, cs, cs);
      }
    }
  }

  // grid
  ctx.strokeStyle="rgba(0,0,0,0.08)";
  for(let y=0;y<N;y++){
    for(let x=0;x<N;x++){
      ctx.strokeRect(ox+x*cs, oy+y*cs, cs, cs);
    }
  }

  // dist numbers
  ctx.textAlign="center";
  ctx.textBaseline="middle";
  ctx.font=Math.floor(cs*0.35)+"px ui-monospace,monospace";
  for(let y=0;y<N;y++){
    for(let x=0;x<N;x++){
      const d=S.dist[y][x];
      if(d!==65535){
        ctx.fillStyle = S.visited[y][x] ? "rgba(0,0,0,0.55)" : "rgba(0,0,0,0.25)";
        ctx.fillText(String(d), ox+x*cs+cs*0.5, oy+y*cs+cs*0.5);
      }
    }
  }

  // known walls only
  ctx.strokeStyle="rgba(0,0,0,0.9)";
  ctx.lineWidth=Math.max(2, cs*0.08);
  for(let y=0;y<N;y++){
    for(let x=0;x<N;x++){
      const w=S.knownWalls[y][x];
      const m=S.knownMask[y][x];
      const px=ox+x*cs, py=oy+y*cs;

      if((m&1) && (w&1)){ ctx.beginPath(); ctx.moveTo(px,py); ctx.lineTo(px+cs,py); ctx.stroke(); }
      if((m&2) && (w&2)){ ctx.beginPath(); ctx.moveTo(px+cs,py); ctx.lineTo(px+cs,py+cs); ctx.stroke(); }
      if((m&4) && (w&4)){ ctx.beginPath(); ctx.moveTo(px,py+cs); ctx.lineTo(px+cs,py+cs); ctx.stroke(); }
      if((m&8) && (w&8)){ ctx.beginPath(); ctx.moveTo(px,py); ctx.lineTo(px,py+cs); ctx.stroke(); }
    }
  }

  // plan
  if(S.plan) drawPlanPath(g, S.plan);

  // mouse
  const mx=S.mouse.x, my=S.mouse.y, h=S.mouse.h;
  const cc = cellCenter(mx,my,g);
  const cx=cc.cx, cy=cc.cy;

  ctx.fillStyle = "rgba(0, 0, 0, 0.18)";
  ctx.beginPath();
  ctx.arc(cx, cy, cs*0.30, 0, Math.PI*2);
  ctx.fill();

  ctx.strokeStyle = "rgba(0, 0, 0, 0.45)";
  ctx.lineWidth = Math.max(1.5, cs*0.05);
  ctx.beginPath();
  ctx.arc(cx, cy, cs*0.30, 0, Math.PI*2);
  ctx.stroke();

  const ax=[0,1,0,-1], ay=[-1,0,1,0];
  ctx.strokeStyle="rgba(0,0,0,0.35)";
  ctx.lineWidth=Math.max(2.2, cs*0.065);
  ctx.beginPath();
  ctx.moveTo(cx,cy);
  ctx.lineTo(cx + ax[h]*cs*0.42, cy + ay[h]*cs*0.42);
  ctx.stroke();

  const cur = S.dist[my][mx];
  const pa = S.pendingActionName || "-";
  st("ver="+(S.ver||0)+" | mouse=("+mx+","+my+") h="+h+
     " | running="+S.running+" | waitAck="+S.waitAck+
     " | seq="+(S.pendingSeq||0)+" | act="+pa+
     " | dist="+cur+" | planLen="+(S.planLen||0));
}

// ---------- ACTION STATE MACHINE (separate from refresh) ----------
async function doNextIfNeeded(){
  if(!S || !S.running || S.waitAck) return;
  if(nextInFlight || busy || ackInFlight || stateInFlight) return;
  nextInFlight = true;
  try{
    await fetchWithTimeout('/next', {cache:'no-store'}, NEXT_TO);
  }catch(e){}
  finally{
    nextInFlight = false;
  }
}

async function autoAckIfEnabled(){
  if(!S || !S.running || !S.waitAck) return;
  if(ackInFlight || busy || stateInFlight) return;

  const autoAck = document.getElementById('autoAck').checked;
  if(!autoAck) return;

  ackInFlight = true;
  const seqSnapshot = S.pendingSeq; // snapshot!

  setTimeout(async ()=>{
    try{
      await fetchWithTimeout('/ack?seq=' + encodeURIComponent(String(seqSnapshot)) + '&ok=1',
                             {cache:'no-store'}, ACK_TO);
    }catch(e){}
    finally{
      ackInFlight = false;
    }
  }, AUTO_ACK_DELAY_MS);
}

async function tickLoop(){
  if(!stopPoll){
    // tách action machine ra khỏi /state để tránh burst
    await doNextIfNeeded();
    await autoAckIfEnabled();
    setTimeout(tickLoop, TICK_MS);
  }
}

// ---------- STATE POLL (only fetch + draw) ----------
async function refresh(fromCmd=false){
  if(stateInFlight) return;
  if(busy && !fromCmd) return;

  stateInFlight = true;
  try{
    const url = '/state?since=' + encodeURIComponent(String(lastVer)) + '&t=' + Date.now();
    const r = await fetchWithTimeout(url, {cache:'no-store'}, STATE_TO);

    if(r.status === 204){
      // no changes -> do nothing here (tickLoop handles next/ack)
      return;
    }
    if(!r.ok){
      st("state HTTP "+r.status);
      return;
    }

    S = await r.json();
    lastVer = S.ver || lastVer;
    draw();

  }catch(e){
    const es = String(e);
    if(es.includes("AbortError") || es.includes("timeout")) st("state timeout");
    else st("offline: "+e);
  }finally{
    stateInFlight = false;
  }
}

async function pollLoop(){
  while(!stopPoll){
    await refresh(false);
    await new Promise(res => setTimeout(res, POLL_MS));
  }
}

pollLoop();
tickLoop();
refresh(true);
</script>
</body></html>
)HTML";

// ----------------- Core constants -----------------
static const int dx4[4] = {0, 1, 0, -1};
static const int dy4[4] = {-1, 0, 1, 0};

static inline uint8_t clampMask(uint8_t v){ return v & 0x0F; }

// ========================= Optional helpers for truth walls =========================
static void forceBoundaries16(uint8_t w[FloodFillExplorer::N][FloodFillExplorer::N]){
  const int N = FloodFillExplorer::N;
  for(int x=0;x<N;x++){
    w[0][x]   |= FloodFillExplorer::WALL_N;
    w[N-1][x] |= FloodFillExplorer::WALL_S;
  }
  for(int y=0;y<N;y++){
    w[y][0]   |= FloodFillExplorer::WALL_W;
    w[y][N-1] |= FloodFillExplorer::WALL_E;
  }
}

static void normalizePairs16(uint8_t w[FloodFillExplorer::N][FloodFillExplorer::N]){
  const int N = FloodFillExplorer::N;
  for(int y=0;y<N;y++){
    for(int x=0;x<N;x++){
      uint8_t a = w[y][x];
      if(a & FloodFillExplorer::WALL_N){ if(y>0)   w[y-1][x] |= FloodFillExplorer::WALL_S; }
      if(a & FloodFillExplorer::WALL_E){ if(x<N-1) w[y][x+1] |= FloodFillExplorer::WALL_W; }
      if(a & FloodFillExplorer::WALL_S){ if(y<N-1) w[y+1][x] |= FloodFillExplorer::WALL_N; }
      if(a & FloodFillExplorer::WALL_W){ if(x>0)   w[y][x-1] |= FloodFillExplorer::WALL_E; }
    }
  }
}

// ========================= Impl =========================

FloodFillExplorer::FloodFillExplorer() {}

FloodFillExplorer::~FloodFillExplorer(){
  if(server_){
    delete server_;
    server_ = nullptr;
  }
}

void FloodFillExplorer::log_(const String& s){
  if(logFn_) logFn_(s);
}

uint8_t FloodFillExplorer::bitForDir_(Dir d) const{
  switch(d){
    case NORTH: return WALL_N;
    case EAST:  return WALL_E;
    case SOUTH: return WALL_S;
    case WEST:  return WALL_W;
  }
  return WALL_N;
}

FloodFillExplorer::Dir FloodFillExplorer::opposite_(Dir d) const{
  return (Dir)(((uint8_t)d + 2) & 3);
}

bool FloodFillExplorer::inBounds_(int x,int y) const{
  return (x>=0 && x<N && y>=0 && y<N);
}

bool FloodFillExplorer::isGoal_(int x,int y) const{
  return (x >= gx0_ && x < (int)(gx0_ + gw_) &&
          y >= gy0_ && y < (int)(gy0_ + gh_));
}

uint16_t FloodFillExplorer::computeBestKnownCost_(uint8_t startX0, uint8_t startY0,
                                                  uint8_t startW, uint8_t startH,
                                                  uint8_t goalX0, uint8_t goalY0,
                                                  uint8_t goalW, uint8_t goalH) const {
  static constexpr uint16_t INF = 0xFFFF;
  uint16_t dist[N][N];
  for (int y = 0; y < N; ++y) {
    for (int x = 0; x < N; ++x) {
      dist[y][x] = INF;
    }
  }

  int qx[N * N], qy[N * N];
  int qh = 0, qt = 0;

  for (int y = goalY0; y < (int)(goalY0 + goalH); ++y) {
    for (int x = goalX0; x < (int)(goalX0 + goalW); ++x) {
      if (!inBounds_(x, y)) continue;
      dist[y][x] = 0;
      qx[qt] = x;
      qy[qt] = y;
      qt++;
    }
  }

  while (qh < qt) {
    int x = qx[qh];
    int y = qy[qh];
    qh++;
    const uint16_t base = dist[y][x];

    for (int di = 0; di < 4; ++di) {
      Dir d = (Dir)di;
      if (knownHasWall_(x, y, d)) continue;
      int nx = x + dx4[di];
      int ny = y + dy4[di];
      if (!inBounds_(nx, ny)) continue;
      if (dist[ny][nx] > base + 1) {
        dist[ny][nx] = base + 1;
        qx[qt] = nx;
        qy[qt] = ny;
        qt++;
      }
    }
  }

  uint16_t best = INF;
  for (int y = startY0; y < (int)(startY0 + startH); ++y) {
    for (int x = startX0; x < (int)(startX0 + startW); ++x) {
      if (!inBounds_(x, y)) continue;
      if (dist[y][x] < best) best = dist[y][x];
    }
  }
  return best;
}

uint16_t FloodFillExplorer::bestKnownCostOriginalStartToGoal() const {
  return computeBestKnownCost_(origHx0_, origHy0_, origHw_, origHh_,
                               origGx0_, origGy0_, origGw_, origGh_);
}

bool FloodFillExplorer::isInOriginalStart(uint8_t x, uint8_t y) const {
  return x >= origHx0_ && x < (uint8_t)(origHx0_ + origHw_) &&
         y >= origHy0_ && y < (uint8_t)(origHy0_ + origHh_);
}

bool FloodFillExplorer::isInOriginalGoal(uint8_t x, uint8_t y) const {
  return x >= origGx0_ && x < (uint8_t)(origGx0_ + origGw_) &&
         y >= origGy0_ && y < (uint8_t)(origGy0_ + origGh_);
}

const char* FloodFillExplorer::actionName_(Action a) const{
  switch(a){
    case ACT_NONE:   return "none";
    case ACT_TURN_L: return "turnL";
    case ACT_TURN_R: return "turnR";
    case ACT_TURN_180: return "turn180";
    case ACT_MOVE_F: return "moveF";
  }
  return "none";
}

void FloodFillExplorer::markDirty_(){
  stateVer_++;
  buildStateJson_();
}

void FloodFillExplorer::setHardwareMode(bool en) {
  hardwareMode_ = en;
  markDirty_();
}

void FloodFillExplorer::setStart(uint8_t x, uint8_t y, Dir h){
  if(x >= N || y >= N) return;
  sx_ = x; sy_ = y; sh_ = h;
  origSx_ = x; origSy_ = y; origSh_ = h;
  origCaptured_ = true;
  targetHome_ = false;
  markDirty_();
}

void FloodFillExplorer::setHomeRect(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h){
  if(w == 0 || h == 0) return;
  if(x0 >= N || y0 >= N) return;
  if(x0 + w > N) w = N - x0;
  if(y0 + h > N) h = N - y0;
  hx0_ = x0; hy0_ = y0; hw_ = w; hh_ = h;
  origHx0_ = x0; origHy0_ = y0; origHw_ = w; origHh_ = h;
  origCaptured_ = true;
  targetHome_ = false;
  markDirty_();
}

void FloodFillExplorer::setGoalRect(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h){
  if(w == 0 || h == 0) return;
  if(x0 >= N || y0 >= N) return;
  if(x0 + w > N) w = N - x0;
  if(y0 + h > N) h = N - y0;
  gx0_ = x0; gy0_ = y0; gw_ = w; gh_ = h;
  origGx0_ = x0; origGy0_ = y0; origGw_ = gw_; origGh_ = gh_;
  origCaptured_ = true;
  targetHome_ = false;
  markDirty_();
}

void FloodFillExplorer::setRunning(bool en) {
  running_ = en;
  markDirty_();
}

void FloodFillExplorer::clearKnownMaze() {
  memset(visited_, 0, sizeof(visited_));
  clearKnown_();
  mx_ = sx_;
  my_ = sy_;
  mh_ = sh_;
  visited_[my_][mx_] = true;
  waitAck_ = false;
  pendingAction_ = ACT_NONE;
  computeFloodFill_();
  computePlan_();
  markDirty_();
}

void FloodFillExplorer::syncPose(uint8_t x, uint8_t y, Dir h, bool markVisited) {
  if (x >= N || y >= N) return;
  mx_ = x;
  my_ = y;
  mh_ = h;
  if (markVisited) visited_[my_][mx_] = true;
  computeFloodFill_();
  computePlan_();
  markDirty_();
}

void FloodFillExplorer::observeRelativeWalls(uint8_t x, uint8_t y, Dir heading,
                                             bool leftWall, bool frontWall, bool rightWall,
                                             bool leftValid, bool frontValid, bool rightValid) {
  if (!inBounds_(x, y)) return;

  auto leftDir = (Dir)(((uint8_t)heading + 3) & 3);
  auto rightDir = (Dir)(((uint8_t)heading + 1) & 3);

  if (leftValid) confirmObservedWall_(x, y, leftDir, leftWall);
  if (frontValid) confirmObservedWall_(x, y, heading, frontWall);
  if (rightValid) confirmObservedWall_(x, y, rightDir, rightWall);

  visited_[y][x] = true;
  applyBoundaryWalls_();
  computeFloodFill_();
  computePlan_();
  markDirty_();
}

FloodFillExplorer::Action FloodFillExplorer::requestNextAction() {
  if (waitAck_) return pendingAction_;

  Action act = chooseNextAction_();
  if (act == ACT_NONE) {
    running_ = false;
    markDirty_();
    return ACT_NONE;
  }

  dispatchAction_(act);
  return act;
}

bool FloodFillExplorer::ackPendingActionExternal(bool ok, uint8_t x, uint8_t y, Dir h) {
  if (!waitAck_) return false;

  if (!ok) {
    running_ = false;
    waitAck_ = false;
    pendingAction_ = ACT_NONE;
    markDirty_();
    return false;
  }

  mx_ = x;
  my_ = y;
  mh_ = h;
  visited_[my_][mx_] = true;

  waitAck_ = false;
  pendingAction_ = ACT_NONE;

  bool reachedGoal = isGoal_(mx_, my_);
  if (reachedGoal) {
    onGoalReached_();
    return true;
  }

  computeFloodFill_();
  computePlan_();
  markDirty_();
  return false;
}

bool FloodFillExplorer::begin(const Config& cfg){
  cfg_ = cfg;

  if(server_){
    delete server_;
    server_ = nullptr;
  }
  if (cfg_.enableWeb) {
    server_ = new WebServer(cfg_.port);
  }

  clearKnown_();
  reset();

  targetHome_ = false; // start by targeting original goal

  if (cfg_.enableWeb && server_) {
    setupWeb_();
    server_->begin();
  }

  started_ = true;
  running_ = cfg_.autoRun;

  waitAck_ = false;
  pendingAction_ = ACT_NONE;
  pendingSeq_ = 0;

  if (cfg_.enableWeb) {
    log_("[Explorer] Web on port " + String(cfg_.port));
  } else {
    log_("[Explorer] Web disabled");
  }
  return true;
}

void FloodFillExplorer::loop() {
  if (!started_ || !server_) return;

  server_->handleClient();

  static uint32_t lastLogMs = 0;

  if (running_ && waitAck_) {
    const uint32_t now = millis();

    if (cfg_.ackTimeoutMs > 0 && (uint32_t)(now - pendingSinceMs_) > cfg_.ackTimeoutMs) {
      const uint32_t LOG_EVERY_MS = 2000;
      if ((uint32_t)(now - lastLogMs) > LOG_EVERY_MS) {
        log_("[ACK] timeout (still waiting). seq=" + String(pendingSeq_) +
             " act=" + String(actionName_(pendingAction_)));
        lastLogMs = now;
      }
    }
  }
}

// ========================= Truth / Known (simulator) =========================

void FloodFillExplorer::setTruthFromWalls(const uint8_t walls16[N][N],
                                         bool normalizePairs,
                                         bool forceBoundaries){
  for(int y=0;y<N;y++){
    for(int x=0;x<N;x++){
      truthWalls_[y][x] = clampMask(walls16[y][x]);
    }
  }

  if(forceBoundaries){
    forceBoundaries16(truthWalls_);
  }
  if(normalizePairs){
    normalizePairs16(truthWalls_);
    if(forceBoundaries) forceBoundaries16(truthWalls_);
  }

  reset();
  log_("[Truth] set from array");
}

void FloodFillExplorer::clearKnown_(){
  memset(knownWalls_, 0, sizeof(knownWalls_));
  memset(knownMask_,  0, sizeof(knownMask_));
  memset(pendingWalls_, 0, sizeof(pendingWalls_));
  memset(pendingMask_,  0, sizeof(pendingMask_));
  memset(pendingCounts_, 0, sizeof(pendingCounts_));
  applyBoundaryWalls_();
}

void FloodFillExplorer::applyBoundaryWalls_() {
  for (int x = 0; x < N; ++x) {
    knownSetWallBoth_(x, 0, NORTH, true);
    knownSetWallBoth_(x, N - 1, SOUTH, true);
  }
  for (int y = 0; y < N; ++y) {
    knownSetWallBoth_(0, y, WEST, true);
    knownSetWallBoth_(N - 1, y, EAST, true);
  }
}

bool FloodFillExplorer::truthHasWall_(int x,int y, Dir d) const{
  return (truthWalls_[y][x] & bitForDir_(d)) != 0;
}

bool FloodFillExplorer::knownHasWall_(int x,int y, Dir d) const{
  uint8_t b = bitForDir_(d);
  if((knownMask_[y][x] & b) == 0) return false; // unknown treated open
  return (knownWalls_[y][x] & b) != 0;
}

bool FloodFillExplorer::getKnownWall(uint8_t x, uint8_t y, Dir d, bool& known, bool& wall) const {
  if (!inBounds_(x, y)) {
    known = false;
    wall = false;
    return false;
  }
  const uint8_t bit = bitForDir_(d);
  known = (knownMask_[y][x] & bit) != 0;
  wall = (knownWalls_[y][x] & bit) != 0;
  return true;
}

String FloodFillExplorer::buildKnownMazeAscii(uint8_t mouseX, uint8_t mouseY, Dir mouseH) const {
  String out;
  out.reserve(N * N * 8);

  for (int y = 0; y < N; ++y) {
    for (int x = 0; x < N; ++x) {
      out += "+";
      bool known = false;
      bool wall = false;
      getKnownWall((uint8_t)x, (uint8_t)y, NORTH, known, wall);
      out += (known && wall) ? "---" : "   ";
    }
    out += "+\n";

    for (int x = 0; x < N; ++x) {
      bool known = false;
      bool wall = false;
      getKnownWall((uint8_t)x, (uint8_t)y, WEST, known, wall);
      out += (known && wall) ? "|" : " ";

      char cell[4] = {' ', ' ', ' ', '\0'};
      if (x == mouseX && y == mouseY) {
        static const char kHeading[4] = {'^', '>', 'v', '<'};
        cell[1] = kHeading[(uint8_t)mouseH & 3];
      } else if (isGoal_(x, y)) {
        cell[1] = 'G';
      } else if (visited_[y][x]) {
        cell[1] = '.';
      }
      out += cell;
    }
    bool known = false;
    bool wall = false;
    getKnownWall((uint8_t)(N - 1), (uint8_t)y, EAST, known, wall);
    out += (known && wall) ? "|\n" : " \n";
  }

  for (int x = 0; x < N; ++x) {
    out += "+";
    bool known = false;
    bool wall = false;
    getKnownWall((uint8_t)x, (uint8_t)(N - 1), SOUTH, known, wall);
    out += (known && wall) ? "---" : "   ";
  }
  out += "+\n";
  return out;
}

void FloodFillExplorer::knownSetWallBoth_(int x,int y, Dir d, bool on){
  auto setOne = [&](int cx,int cy, Dir cd, bool v){
    if(!inBounds_(cx,cy)) return;
    uint8_t b = bitForDir_(cd);

    knownMask_[cy][cx] |= b;
    if(v) knownWalls_[cy][cx] |= b;
    else  knownWalls_[cy][cx] &= ~b;
  };

  setOne(x,y,d,on);
  int nx = x + dx4[(int)d], ny = y + dy4[(int)d];
  if(inBounds_(nx,ny)){
    setOne(nx,ny, opposite_(d), on);
  }
}

void FloodFillExplorer::pendingSetWallBoth_(int x, int y, Dir d, bool on, uint8_t count) {
  auto setOne = [&](int cx, int cy, Dir cd, bool v, uint8_t c) {
    if (!inBounds_(cx, cy)) return;
    const uint8_t bit = bitForDir_(cd);
    pendingMask_[cy][cx] |= bit;
    if (v) pendingWalls_[cy][cx] |= bit;
    else   pendingWalls_[cy][cx] &= ~bit;
    pendingCounts_[cy][cx][(int)cd] = c;
  };

  setOne(x, y, d, on, count);
  const int nx = x + dx4[(int)d];
  const int ny = y + dy4[(int)d];
  if (inBounds_(nx, ny)) {
    setOne(nx, ny, opposite_(d), on, count);
  }
}

void FloodFillExplorer::pendingClearWallBoth_(int x, int y, Dir d) {
  auto clearOne = [&](int cx, int cy, Dir cd) {
    if (!inBounds_(cx, cy)) return;
    const uint8_t bit = bitForDir_(cd);
    pendingMask_[cy][cx] &= ~bit;
    pendingWalls_[cy][cx] &= ~bit;
    pendingCounts_[cy][cx][(int)cd] = 0;
  };

  clearOne(x, y, d);
  const int nx = x + dx4[(int)d];
  const int ny = y + dy4[(int)d];
  if (inBounds_(nx, ny)) {
    clearOne(nx, ny, opposite_(d));
  }
}

bool FloodFillExplorer::confirmObservedWall_(int x, int y, Dir d, bool on) {
  if (!inBounds_(x, y)) return false;

  const uint8_t bit = bitForDir_(d);
  const bool known = (knownMask_[y][x] & bit) != 0;
  const bool currentWall = (knownWalls_[y][x] & bit) != 0;

  if (known && currentWall == on) {
    pendingClearWallBoth_(x, y, d);
    return false;
  }
  pendingClearWallBoth_(x, y, d);
  knownSetWallBoth_(x, y, d, on);
  return true;
}

void FloodFillExplorer::senseCell_(int x,int y){
  if (hardwareMode_) return;
  // SIM: use truthWalls_
  for(int di=0; di<4; di++){
    Dir d = (Dir)di;
    bool w = truthHasWall_(x,y,d);
    knownSetWallBoth_(x,y,d,w);
  }
}

// ========================= Reset / Flood / Plan =========================

void FloodFillExplorer::reset(){
  mx_ = sx_; my_ = sy_; mh_ = sh_;

  memset(visited_, 0, sizeof(visited_));
  clearKnown_();

  senseCell_(mx_, my_);
  visited_[my_][mx_] = true;

  computeFloodFill_();
  computePlan_();

  running_ = false;
  waitAck_ = false;
  pendingAction_ = ACT_NONE;

  log_("[FF] reset");
  markDirty_();
}

void FloodFillExplorer::computeFloodFill_(){
  const uint16_t INF = 0xFFFF;
  for(int y=0;y<N;y++) for(int x=0;x<N;x++) dist_[y][x] = INF;

  int qx[N*N], qy[N*N];
  int qh=0, qt=0;

  for(int y=gy0_; y<(int)(gy0_+gh_); y++){
    for(int x=gx0_; x<(int)(gx0_+gw_); x++){
      dist_[y][x] = 0;
      qx[qt]=x; qy[qt]=y; qt++;
    }
  }

  while(qh<qt){
    int x=qx[qh], y=qy[qh]; qh++;
    uint16_t base = dist_[y][x];

    for(int di=0; di<4; di++){
      Dir d = (Dir)di;
      if(knownHasWall_(x,y,d)) continue;

      int nx = x + dx4[di], ny = y + dy4[di];
      if(!inBounds_(nx,ny)) continue;

      if(dist_[ny][nx] > base + 1){
        dist_[ny][nx] = base + 1;
        qx[qt]=nx; qy[qt]=ny; qt++;
      }
    }
  }
}

void FloodFillExplorer::computePlan_(){
  planLen_ = 0;

  uint8_t x = mx_, y = my_;
  Dir h = mh_;

  auto push = [&](uint8_t px, uint8_t py, Dir ph){
    if(planLen_ >= kMaxPlan) return;
    plan_[planLen_++] = PlanNode{px, py, (uint8_t)ph};
  };

  push(x,y,h);
  if(isGoal_(x,y)) return;

  static uint8_t seen[N][N][4];
  memset(seen, 0, sizeof(seen));
  seen[y][x][(uint8_t)h] = 1;

  auto orderFromHeading = [&](Dir hh, Dir out[4]){
    out[0] = hh;
    out[1] = (Dir)(((uint8_t)hh + 3) & 3);
    out[2] = (Dir)(((uint8_t)hh + 1) & 3);
    out[3] = (Dir)(((uint8_t)hh + 2) & 3);
  };

  const uint16_t maxSteps = N*N*4;
  for(uint16_t step=0; step<maxSteps; step++){
    uint16_t cur = dist_[y][x];
    if(cur == 0xFFFF) break;
    if(cur == 0) break;

    Dir order[4];
    orderFromHeading(h, order);

    bool moved=false;
    for(int i=0;i<4;i++){
      Dir d = order[i];
      int nx = (int)x + dx4[(int)d];
      int ny = (int)y + dy4[(int)d];
      if(!inBounds_(nx,ny)) continue;
      if(knownHasWall_(x,y,d)) continue;

      uint16_t nd = dist_[ny][nx];
      if(nd == 0xFFFF) continue;

      if(nd < cur){
        h = d;
        x = (uint8_t)nx;
        y = (uint8_t)ny;
        push(x,y,h);
        moved=true;
        break;
      }
    }

    if(!moved) break;
    if(isGoal_(x,y)) break;
    if(seen[y][x][(uint8_t)h]) break;
    seen[y][x][(uint8_t)h] = 1;
  }
}

// ========================= ACTION (ACK-driven) =========================

FloodFillExplorer::Action FloodFillExplorer::chooseNextAction_(){
  if (!hardwareMode_) {
    senseCell_(mx_, my_);
  }
  visited_[my_][mx_] = true;
  computeFloodFill_();
  computePlan_();

  if(isGoal_(mx_, my_)) return ACT_NONE;

  uint16_t cur = dist_[my_][mx_];
  if(cur == 0xFFFF) return ACT_NONE;

  Dir order[4];
  order[0] = mh_;
  order[1] = (Dir)((mh_ + 3) & 3);
  order[2] = (Dir)((mh_ + 1) & 3);
  order[3] = (Dir)((mh_ + 2) & 3);

  Dir best = mh_;
  bool found=false;

  for(int i=0;i<4;i++){
    Dir d = order[i];
    if(knownHasWall_(mx_, my_, d)) continue;
    int nx = mx_ + dx4[(int)d];
    int ny = my_ + dy4[(int)d];
    if(!inBounds_(nx,ny)) continue;
    uint16_t nd = dist_[ny][nx];
    if(nd < cur){
      best = d;
      found = true;
      break;
    }
  }
  if(!found) return ACT_NONE;

  if(best == mh_) return ACT_MOVE_F;

  uint8_t curh = (uint8_t)mh_;
  uint8_t tarh = (uint8_t)best;
  uint8_t diff = (tarh + 4 - curh) & 3;

  if(diff == 1) return ACT_TURN_R;
  if(diff == 3) return ACT_TURN_L;
  return ACT_TURN_180;
}

void FloodFillExplorer::dispatchAction_(Action a){
  pendingAction_ = a;
  pendingSeq_++;
  pendingSinceMs_ = millis();
  waitAck_ = true;
  markDirty_();
}

void FloodFillExplorer::commitPendingAction_(){
  if(pendingAction_ == ACT_TURN_L){
    mh_ = (Dir)(((uint8_t)mh_ + 3) & 3);
  }else if(pendingAction_ == ACT_TURN_R){
    mh_ = (Dir)(((uint8_t)mh_ + 1) & 3);
  }else if(pendingAction_ == ACT_TURN_180){
    mh_ = (Dir)(((uint8_t)mh_ + 2) & 3);
  }else if(pendingAction_ == ACT_MOVE_F){
    if(!knownHasWall_(mx_, my_, mh_)){
      int nx = mx_ + dx4[(int)mh_];
      int ny = my_ + dy4[(int)mh_];
      if(inBounds_(nx,ny)){
        mx_ = (uint8_t)nx;
        my_ = (uint8_t)ny;
        visited_[my_][mx_] = true;
        senseCell_(mx_, my_);
      }
    }
  }
  pendingAction_ = ACT_NONE;
}

void FloodFillExplorer::onGoalReached_(){
  // Stop running for safety
  running_ = false;
  waitAck_ = false;
  pendingAction_ = ACT_NONE;

  // --- Toggle target ---
  // Nếu vừa tới GOAL gốc (2x2) => đổi mục tiêu về HOME (start gốc 1 ô)
  // Nếu vừa tới HOME => đổi mục tiêu về GOAL gốc (2x2)
  targetHome_ = !targetHome_;

  if(targetHome_){
    // target HOME rectangle
    gx0_ = origHx0_;
    gy0_ = origHy0_;
    gw_  = origHw_;
    gh_  = origHh_;
    log_("[FF] reached target -> now GO HOME (original home rect)");
  }else{
    // target original GOAL rect
    gx0_ = origGx0_;
    gy0_ = origGy0_;
    gw_  = origGw_;
    gh_  = origGh_;
    log_("[FF] reached target -> now GO TO GOAL (original rect)");
  }

  // Recompute for next trip
  computeFloodFill_();
  computePlan_();
  markDirty_();
}

// ========================= Web =========================

void FloodFillExplorer::setupWeb_(){
  server_->on("/", HTTP_GET, [this](){ handleRoot_(); });
  server_->on("/state", HTTP_GET, [this](){ handleState_(); });
  server_->on("/cmd", HTTP_GET, [this](){ handleCmd_(); });
  server_->on("/next", HTTP_GET, [this](){ handleNext_(); });
  server_->on("/ack", HTTP_GET, [this](){ handleAck_(); });
}

void FloodFillExplorer::handleRoot_(){
  server_->sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server_->sendHeader("Pragma", "no-cache");
  server_->sendHeader("Expires", "0");
  server_->sendHeader("Connection", "close");
  server_->send(200, "text/html; charset=utf-8", FPSTR(kHtml));
}

void FloodFillExplorer::handleCmd_(){
  if(!server_->hasArg("a")){
    server_->sendHeader("Connection", "close");
    server_->send(400, "text/plain", "missing a=step|run|pause|reset|setstart");
    return;
  }

  const String a = server_->arg("a");

  // STEP (SIM): dispatch+commit immediately
  if(a == "step"){
    if(waitAck_){
      server_->sendHeader("Connection", "close");
      server_->send(200, "text/plain", "waiting ack");
      return;
    }

    Action act = chooseNextAction_();
    if(act == ACT_NONE){
      running_ = false;
      markDirty_();
      server_->sendHeader("Connection", "close");
      server_->send(200, "text/plain", "done");
      return;
    }

    dispatchAction_(act);
    commitPendingAction_();
    waitAck_ = false;
    pendingAction_ = ACT_NONE;

    if(isGoal_(mx_, my_)){
      onGoalReached_();
      server_->sendHeader("Connection", "close");
      server_->send(200, "text/plain", "step ok (GOAL)");
      return;
    }

    computeFloodFill_();
    computePlan_();
    markDirty_();

    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", "step ok");
    return;
  }

  if(a == "run"){
    running_ = true;

    if(!waitAck_){
      Action act = chooseNextAction_();
      if(act != ACT_NONE){
        dispatchAction_(act);
      }
    }

    markDirty_();
    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", "OK");
    return;
  }

  if(a == "pause"){
    running_ = false;
    markDirty_();
    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", "OK");
    return;
  }

  if(a == "reset"){
    reset();
    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", "OK");
    return;
  }

  if(a == "setstart"){
    if(!server_->hasArg("x") || !server_->hasArg("y") || !server_->hasArg("h")){
      server_->sendHeader("Connection", "close");
      server_->send(400, "text/plain", "missing x,y,h");
      return;
    }

    int x = server_->arg("x").toInt();
    int y = server_->arg("y").toInt();
    int h = server_->arg("h").toInt();

    if(x < 0) x = 0; if(x >= N) x = N - 1;
    if(y < 0) y = 0; if(y >= N) y = N - 1;
    h &= 3;

    running_ = false;
    waitAck_ = false;
    pendingAction_ = ACT_NONE;

    sx_ = (uint8_t)x;
    sy_ = (uint8_t)y;
    sh_ = (Dir)h;

    reset();

    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", "OK");
    return;
  }

  server_->sendHeader("Connection", "close");
  server_->send(400, "text/plain", "unknown a= (step|run|pause|reset|setstart)");
}

void FloodFillExplorer::handleNext_(){
  if(!running_){
    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", "not running");
    return;
  }
  if(waitAck_){
    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", "waiting ack");
    return;
  }

  Action act = chooseNextAction_();
  if(act == ACT_NONE){
    running_ = false;
    markDirty_();
    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", "done");
    return;
  }

  dispatchAction_(act);
  server_->sendHeader("Connection", "close");
  server_->send(200, "text/plain", "dispatched");
}

void FloodFillExplorer::handleAck_(){
  if(!server_->hasArg("seq")){
    server_->sendHeader("Connection", "close");
    server_->send(400, "text/plain", "missing seq");
    return;
  }
  uint32_t seq = (uint32_t) strtoul(server_->arg("seq").c_str(), nullptr, 10);
  bool ok = !server_->hasArg("ok") || server_->arg("ok") != "0";

  if(!waitAck_){
    server_->sendHeader("Connection", "close");
    server_->send(409, "text/plain", "no pending");
    return;
  }
  if(seq != pendingSeq_){
    server_->sendHeader("Connection", "close");
    server_->send(409, "text/plain", "seq mismatch");
    return;
  }

  if(!ok){
    running_ = false;
    waitAck_ = false;
    pendingAction_ = ACT_NONE;
    markDirty_();
    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", "ACK FAIL");
    return;
  }

  commitPendingAction_();
  waitAck_ = false;

  if(isGoal_(mx_, my_)){
    onGoalReached_();
    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", "ACK OK (GOAL)");
    return;
  }

  computeFloodFill_();
  computePlan_();
  markDirty_();

  server_->sendHeader("Connection", "close");
  server_->send(200, "text/plain", "ACK OK");
}

void FloodFillExplorer::handleState_() {
  uint32_t since = 0;
  if (server_->hasArg("since")) {
    since = (uint32_t)strtoul(server_->arg("since").c_str(), nullptr, 10);
  }

  // Always disable caching for state
  server_->sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server_->sendHeader("Pragma", "no-cache");
  server_->sendHeader("Expires", "0");

  // Close to avoid keep-alive stalls on ESP32 WebServer
  server_->sendHeader("Connection", "close");

  // Fast path: no changes
  if (since == stateVer_) {
    // 204 must not include body
    server_->send(204);
    return;
  }

  // Optional but helps browser + reduces weird stalls:
  server_->sendHeader("Content-Type", "application/json");
  server_->sendHeader("Content-Length", String(stateJson_.length()));

  server_->send(200, "application/json", stateJson_);
}

// ========================= JSON =========================

void FloodFillExplorer::buildStateJson_(){
  String out;
  out.reserve(12000); // giảm một chút để bớt pressure heap

  out += "{";
  out += "\"ver\":" + String(stateVer_) + ",";

  out += "\"mouse\":{\"x\":" + String(mx_) + ",\"y\":" + String(my_) + ",\"h\":" + String((int)mh_) + "},";
  out += "\"start\":{\"x\":" + String(sx_) + ",\"y\":" + String(sy_) + ",\"h\":" + String((int)sh_) + "},";
  out += "\"home\":{\"x0\":" + String(hx0_) + ",\"y0\":" + String(hy0_) + ",\"w\":" + String(hw_) + ",\"h\":" + String(hh_) + "},";
  out += "\"goal\":{\"x0\":" + String(gx0_) + ",\"y0\":" + String(gy0_) + ",\"w\":" + String(gw_) + ",\"h\":" + String(gh_) + "},";
  out += "\"running\":" + String(running_ ? "true":"false") + ",";
  out += "\"hardwareMode\":" + String(hardwareMode_ ? "true":"false") + ",";
  out += "\"waitAck\":" + String(waitAck_ ? "true":"false") + ",";
  out += "\"pendingSeq\":" + String((uint32_t)pendingSeq_) + ",";
  out += "\"pendingActionName\":\"" + String(actionName_(pendingAction_)) + "\",";

  out += "\"knownWalls\":[";
  for(int y=0;y<N;y++){
    if(y) out += ",";
    out += "[";
    for(int x=0;x<N;x++){
      if(x) out += ",";
      out += String((int)knownWalls_[y][x]);
    }
    out += "]";
  }
  out += "],";

  out += "\"knownMask\":[";
  for(int y=0;y<N;y++){
    if(y) out += ",";
    out += "[";
    for(int x=0;x<N;x++){
      if(x) out += ",";
      out += String((int)knownMask_[y][x]);
    }
    out += "]";
  }
  out += "],";

  out += "\"visited\":[";
  for(int y=0;y<N;y++){
    if(y) out += ",";
    out += "[";
    for(int x=0;x<N;x++){
      if(x) out += ",";
      out += (visited_[y][x] ? "true" : "false");
    }
    out += "]";
  }
  out += "],";

  out += "\"dist\":[";
  for(int y=0;y<N;y++){
    if(y) out += ",";
    out += "[";
    for(int x=0;x<N;x++){
      if(x) out += ",";
      uint16_t v = dist_[y][x];
      out += (v == 0xFFFF) ? "65535" : String(v);
    }
    out += "]";
  }
  out += "],";

  out += "\"planLen\":" + String((int)planLen_) + ",";
  out += "\"plan\":[";
  for(uint16_t i=0;i<planLen_; i++){
    if(i) out += ",";
    out += "{";
    out += "\"x\":" + String(plan_[i].x) + ",";
    out += "\"y\":" + String(plan_[i].y) + ",";
    out += "\"h\":" + String((int)plan_[i].h);
    out += "}";
  }
  out += "]";

  out += "}";

  stateJson_ = out;
}
