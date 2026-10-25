/*
 * Task 9 - ESP32 PID Line Follower + Depth-First-Search Maze Solver
 * -----------------------------------------------------------------
 * Hardware (Appendix A): ESP32-WROOM-32, 5 analog IR sensors (higher voltage
 * = darker = line), L298N dual H-bridge, 3S LiPo.
 *
 *  1. Line position   : weighted centroid of calibrated analog sensors,
 *                       weights {-2,-1,0,1,2}  -> position -2000..+2000
 *  2. Line following  : PID on the position error -> differential motor speed
 *  3. Lost line       : short gap  -> keep steering toward the last side seen
 *                       confirmed  -> treated as a DEAD END: U-turn + backtrack
 *                       U-turn can't re-find the line -> SAFE STOP
 *  4. Maze solving    : DFS on a grid of junction nodes, exit priority
 *                       LEFT -> FORWARD -> RIGHT. Visited nodes are never
 *                       re-entered; dead ends / fully explored nodes pop the
 *                       stack and drive back to the previous node.
 *  5. Startup         : min-max calibration of every sensor (12-bit ADC)
 *  6. Telemetry       : every control-loop iteration prints
 *                       s1,s2,s3,s4,s5,error,motorL,motorR  @115200
 *
 * SIMULATION = 1 : PID is fed hard-coded raw ADC arrays, then the DFS solves a
 *                  hard-coded maze in software (no hardware needed).
 * SIMULATION = 0 : runs on the robot.
 *
 * PID TUNING NOTES
 *  - Error = centroid position, range -2000..+2000. Loop = 100 Hz (10 ms), which
 *    keeps the 115200-baud telemetry stream (~35 B/line) well inside the UART limit.
 *  - Kp = 0.06 : full-scale error gives ~120 PWM of correction. Enough to take
 *                a 90 deg bend, not enough to saturate on every wobble.
 *  - Kd = 0.0004 (per second): damps oscillation; kept small because the
 *                error is quantised by only 5 sensors.
 *  - Ki = 0.02, clamped to +-40 PWM : only trims small steady drift. A big Ki
 *                winds up during turns and causes overshoot, so it stays tiny.
 *  - Tuning order: Ki=Kd=0, raise Kp until it just starts to wobble, back off
 *                ~30 %, add Kd until the wobble is gone, add Ki last (or leave 0).
 *  - L298N drops ~1.4 V per channel and motors stall below ~60 PWM, so
 *                BASE_SPEED is 150 and reverse is limited to -80.
 */

#include <Arduino.h>

#define SIMULATION 1
#define TELEMETRY  1        // stream values every loop iteration (spec requirement)
#define CAL_AUTO_SPIN 1     // 1: robot spins in place during calibration
                            // 0: you sweep the sensors over line + floor by hand

// ============================== PINS (Appendix A) =========================
const uint8_t SENSOR_PINS[5] = {34, 35, 32, 33, 25};   // S1 (left) ... S5 (right)

const uint8_t ENA = 14, IN1 = 27, IN2 = 26;            // left  motor
const uint8_t ENB = 13, IN3 = 12, IN4 = 11;            // right motor
const bool INVERT_LEFT = false, INVERT_RIGHT = false;  // flip if a motor runs backwards

// ============================ PARAMETERS =================================
const int   BASE_SPEED   = 150;      // 0..255
const int   MAX_SPEED    = 255;
const int   MIN_SPEED    = -80;
const int   TURN_SPEED   = 140;
const int   ADV_SPEED    = 120;      // creep speed over a junction
const int   CAL_SPEED    = 100;

const float KP = 0.06f;
const float KI = 0.02f;
const float KD = 0.0004f;
const float I_LIMIT = 2000.0f;       // KI * I_LIMIT = 40 PWM max integral action

const float ON_THRESH    = 0.50f;    // normalised value above which a sensor "sees" the line
const float FOUND_THRESH = 0.30f;    // strongest sensor must exceed this to say "line found"
const float NOISE_FLOOR  = 0.15f;    // ignore tiny readings in the centroid

const uint32_t LOOP_US          = 10000;  // 100 Hz
const uint32_t ADVANCE_MS       = 120;    // brings wheel axle over the node centre
const uint32_t LOST_CONFIRM_MS  = 60;     // no line this long => dead end
const uint32_t TURN_MIN_90_MS   = 120;    // must leave the old line first
const uint32_t TURN_MIN_180_MS  = 300;
const uint32_t TURN_TIMEOUT_MS  = 1500;   // line not re-found => safe stop
const uint32_t CAL_MS           = 3000;

// ============================ DATA TYPES =================================
struct Exits { bool left, fwd, right, goal; };

enum Dir : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
const int8_t DX[4] = {0, 1, 0, -1};
const int8_t DY[4] = {1, 0, -1, 0};
const char   DIR_CHAR[4] = {'N', 'E', 'S', 'W'};

const int GRID = 12;
bool     visited[GRID][GRID];
uint8_t  pathStack[GRID * GRID];      // heading used to ENTER each cell of the current path
int      sp = 0;
int      cx = GRID / 2, cy = GRID / 2;   // current cell (start in the middle of the grid)
uint8_t  heading = NORTH;                // robot starts facing "north"
bool     solved = false, failed = false;

// sensors / PID state
int   rawVals[5];
int   calMin[5], calMax[5];
float pidIntegral = 0, pidPrevErr = 0, lastPos = 0;

// ========================== MOTORS (L298N) ================================
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_SETUP()      do { } while (0)
  #define PWM_OUT(pin, ch, duty) analogWrite(pin, duty)
#else   // Arduino-ESP32 2.x has no analogWrite(): emulate it with LEDC
  #define PWM_SETUP()      do { ledcSetup(0, 5000, 8); ledcAttachPin(ENA, 0); \
                                ledcSetup(1, 5000, 8); ledcAttachPin(ENB, 1); } while (0)
  #define PWM_OUT(pin, ch, duty) ledcWrite(ch, duty)
#endif

void hardwareInit() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  PWM_SETUP();
  analogReadResolution(12);                       // 0..4095
  for (uint8_t i = 0; i < 5; i++) pinMode(SENSOR_PINS[i], INPUT);
}

void setMotor(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);
  if (INVERT_LEFT)  left  = -left;
  if (INVERT_RIGHT) right = -right;
  digitalWrite(IN1, left  >= 0); digitalWrite(IN2, left  < 0);   // left  dir A/B
  digitalWrite(IN3, right >= 0); digitalWrite(IN4, right < 0);   // right dir A/B
  PWM_OUT(ENA, 0, abs(left));
  PWM_OUT(ENB, 1, abs(right));
}

void stopMotors() { setMotor(0, 0); }

void safeStop(const char *why) {
  stopMotors();
  Serial.printf("SAFE STOP: %s\n", why);
  failed = true;
}

// ============================ SENSORS =====================================
// raw 0..4095 -> normalised 0..1 (1 = darkest seen = line) + on/off flag
void normalise(const int raw[5], float v[5], bool on[5]) {
  for (int i = 0; i < 5; i++) {
    float range = (float)max(1, calMax[i] - calMin[i]);
    float x = (raw[i] - calMin[i]) / range;
    v[i]  = constrain(x, 0.0f, 1.0f);
    on[i] = v[i] > ON_THRESH;
  }
}

void readSensors(float v[5], bool on[5]) {
  for (int i = 0; i < 5; i++) rawVals[i] = analogRead(SENSOR_PINS[i]);
  normalise(rawVals, v, on);
}

int countOn(const bool on[5]) {
  int n = 0;
  for (int i = 0; i < 5; i++) n += on[i];
  return n;
}

// Weighted centroid. Returns false if no sensor sees the line.
// pos: -2000 (line far left) ... 0 (centred) ... +2000 (line far right)
bool computePosition(const float v[5], float &pos) {
  static const int W[5] = {-2, -1, 0, 1, 2};
  float sum = 0, num = 0, maxv = 0;
  for (int i = 0; i < 5; i++) {
    float x = (v[i] < NOISE_FLOOR) ? 0.0f : v[i];
    sum += x; num += W[i] * x;
    maxv = fmaxf(maxv, v[i]);
  }
  if (maxv < FOUND_THRESH || sum <= 0) return false;
  pos = 1000.0f * num / sum;
  return true;
}

// ================================= PID ====================================
void resetPid() { pidIntegral = 0; pidPrevErr = 0; }

float pidStep(float err, float dt) {
  pidIntegral += err * dt;
  pidIntegral  = constrain(pidIntegral, -I_LIMIT, I_LIMIT);
  float d = (err - pidPrevErr) / dt;
  pidPrevErr = err;
  return KP * err + KI * pidIntegral + KD * d;
}

void motorSpeedsFromCorrection(float corr, int &l, int &r) {
  // line to the right (pos>0) -> corr>0 -> left faster, right slower -> steer right
  l = constrain((int)(BASE_SPEED + corr), MIN_SPEED, MAX_SPEED);
  r = constrain((int)(BASE_SPEED - corr), MIN_SPEED, MAX_SPEED);
}

void telemetry(float err, int l, int r) {
#if TELEMETRY
  Serial.printf("%d,%d,%d,%d,%d,%.0f,%d,%d\n",
                rawVals[0], rawVals[1], rawVals[2], rawVals[3], rawVals[4], err, l, r);
#endif
}

// ===================== PID SELF-TEST (hard-coded raw ADC arrays) ===========
void runPidSelfTest() {
  const int SAMPLES[][5] = {
    { 200,  250, 3900,  300,  180},   // centred
    { 200, 2200, 3600,  500,  190},   // drifting left
    { 250, 3800,  400,  200,  200},   // further left
    {2500, 3500,  300,  200,  200},   // far left
    {3900,  600,  200,  200,  200},   // extreme left
    { 200,  200,  200,  200,  200},   // LOST (all white)
    { 200,  200, 3500, 2800,  250},   // drifting right
    { 200,  200,  300, 3900,  500},   // right
    { 200,  200,  200, 3300, 3700},   // far right
    { 200,  200,  200,  300, 3900}    // extreme right
  };
  const int N = sizeof(SAMPLES) / sizeof(SAMPLES[0]);
  Serial.println("\n=== PID self-test (raw ADC -> position -> PID -> motors) ===");
  Serial.println("S1,S2,S3,S4,S5 | position | correction |  L   R");
  resetPid(); lastPos = 0;
  for (int i = 0; i < N; i++) {
    float v[5]; bool on[5]; float pos;
    normalise(SAMPLES[i], v, on);
    bool found = computePosition(v, pos);
    if (found) lastPos = pos;
    else       pos = (lastPos >= 0) ? 2000 : -2000;      // lost: steer toward last side seen
    float corr = pidStep(pos, LOOP_US / 1e6f);
    int l, r; motorSpeedsFromCorrection(corr, l, r);
    Serial.printf("%4d,%4d,%4d,%4d,%4d | %8.0f | %10.1f | %3d %3d%s\n",
                  SAMPLES[i][0], SAMPLES[i][1], SAMPLES[i][2], SAMPLES[i][3], SAMPLES[i][4],
                  pos, corr, l, r, found ? "" : "   <- LOST: recovery steering");
  }
}

// ========================= MOTION PRIMITIVES (REAL) ========================
#if !SIMULATION

// Min-max calibration. Robot spins left/right over line and floor.
void calibrateSensors() {
  for (int i = 0; i < 5; i++) { calMin[i] = 4095; calMax[i] = 0; }
  Serial.println("Calibrating... (sweep sensors over line and floor)");
  uint32_t t0 = millis();
  while (millis() - t0 < CAL_MS) {
#if CAL_AUTO_SPIN
    uint32_t p = millis() - t0;                  // right, left, right -> ends where it began
    int dir = (p < CAL_MS / 4 || p >= 3 * CAL_MS / 4) ? 1 : -1;
    setMotor(dir * CAL_SPEED, -dir * CAL_SPEED);
#endif
    for (int i = 0; i < 5; i++) {
      int r = analogRead(SENSOR_PINS[i]);
      calMin[i] = min(calMin[i], r);
      calMax[i] = max(calMax[i], r);
    }
    delay(2);
  }
  stopMotors();
  for (int i = 0; i < 5; i++) {
    Serial.printf("S%d min=%d max=%d\n", i + 1, calMin[i], calMax[i]);
    if (calMax[i] - calMin[i] < 400) safeStop("calibration range too small (sensor not seeing line/floor)");
  }
}

// Pivot until the centre sensor (S3) finds a line again. dir: +1 right, -1 left
bool pivotToLine(int dir, uint32_t minMs) {
  setMotor(dir * TURN_SPEED, -dir * TURN_SPEED);
  delay(minMs);                                   // leave the line we are on first
  uint32_t t0 = millis();
  float v[5]; bool on[5];
  while (millis() - t0 < TURN_TIMEOUT_MS) {
    readSensors(v, on);
    if (on[2]) { stopMotors(); return true; }
  }
  stopMotors();
  return false;
}

// diff: 1 = right 90, 2 = U-turn, 3 = left 90   (relative to current heading)
bool turnRobot(int diff) {
  if (diff == 1) return pivotToLine(+1, TURN_MIN_90_MS);
  if (diff == 3) return pivotToLine(-1, TURN_MIN_90_MS);
  if (diff == 2) return pivotToLine(+1, TURN_MIN_180_MS);
  return true;
}

// Follow the line with PID until the next node. Fills `ex`.
void driveToNextNode(Exits &ex) {
  ex = {false, false, false, false};
  resetPid();
  uint32_t lostSince = 0, last = micros();
  uint8_t  junctionCnt = 0;
  float v[5]; bool on[5];

  while (true) {
    while (micros() - last < LOOP_US) { }
    float dt = (micros() - last) / 1e6f; last = micros();

    readSensors(v, on);
    int n = countOn(on);
    float pos;

    if (computePosition(v, pos)) {
      lostSince = 0; lastPos = pos;
    } else {                                           // ---- lost-line handling ----
      if (lostSince == 0) lostSince = millis();
      if (millis() - lostSince > LOST_CONFIRM_MS) {    // confirmed: dead end
        stopMotors();
        telemetry(0, 0, 0);
        return;                                        // no exits -> DFS backtracks
      }
      pos = (lastPos >= 0) ? 2000 : -2000;             // ride out short gaps toward last side
    }

    // ---- junction detection: an outer sensor + at least one more, debounced ----
    junctionCnt = ((on[0] || on[4]) && n >= 2) ? junctionCnt + 1 : 0;
    if (junctionCnt >= 2) {
      bool L = on[0], R = on[4];
      setMotor(ADV_SPEED, ADV_SPEED);
      uint32_t t0 = millis();
      while (millis() - t0 < ADVANCE_MS) {             // latch branches while crossing
        readSensors(v, on); L |= on[0]; R |= on[4];
      }
      stopMotors();
      readSensors(v, on);
      if (countOn(on) == 5) { ex.goal = true; return; }   // solid black area = goal
      ex.left  = L;
      ex.right = R;
      ex.fwd   = on[1] || on[2] || on[3];
      return;
    }

    float corr = pidStep(pos, dt);
    int l, r; motorSpeedsFromCorrection(corr, l, r);
    setMotor(l, r);
    telemetry(pos, l, r);
  }
}

#else
// ========================= MOTION PRIMITIVES (SIM) =========================
// Hard-coded maze: bit d of simOpen[x][y] set => passage in direction d.
uint8_t simOpen[GRID][GRID];
const int SIM_OX = 5, SIM_OY = 6;               // sim (0,0) -> grid (5,6)
const int SIM_GOAL_X = 4 + SIM_OX, SIM_GOAL_Y = 0 + SIM_OY;

void simAddEdge(int x1, int y1, int x2, int y2) {
  x1 += SIM_OX; x2 += SIM_OX; y1 += SIM_OY; y2 += SIM_OY;
  for (int d = 0; d < 4; d++)
    if (x1 + DX[d] == x2 && y1 + DY[d] == y2) {
      simOpen[x1][y1] |= (1 << d);
      simOpen[x2][y2] |= (1 << ((d + 2) % 4));
    }
}

void simBuildMaze() {
  //  y=3  . - . - . - . - .          Start at (1,0) heading north, Goal at (4,0)
  //  y=2  .   . - .   .   .          (0,1) is a dead end on the LEFT of (1,1),
  //  y=1  . - . - . - . - .          (4,2) is a dead end below (4,3),
  //  y=0  .   S           G          so the left-first DFS must backtrack
  simAddEdge(1,0, 1,1);  simAddEdge(0,1, 1,1);  simAddEdge(1,1, 1,2);
  simAddEdge(1,2, 1,3);  simAddEdge(1,3, 2,3);  simAddEdge(2,3, 3,3);
  simAddEdge(3,3, 4,3);  simAddEdge(4,3, 4,2);  simAddEdge(4,1, 4,0);
  simAddEdge(1,2, 2,2);  simAddEdge(2,2, 2,1);  simAddEdge(2,1, 3,1);
  simAddEdge(3,1, 4,1);
}

bool simOpenDir(int x, int y, int d) { return simOpen[x][y] & (1 << d); }

bool turnRobot(int) { return true; }            // heading is updated by turnTo()

void driveToNextNode(Exits &ex) {
  int nx = cx + DX[heading], ny = cy + DY[heading];
  ex.left  = simOpenDir(nx, ny, (heading + 3) % 4);
  ex.fwd   = simOpenDir(nx, ny, heading);
  ex.right = simOpenDir(nx, ny, (heading + 1) % 4);
  ex.goal  = (nx == SIM_GOAL_X && ny == SIM_GOAL_Y);
}
#endif

// ============================== DFS LOGIC =================================
void turnTo(uint8_t target) {
  int diff = (target - heading + 4) % 4;
  if (diff != 0 && !turnRobot(diff)) { safeStop("could not re-find line after turn"); return; }
  heading = target;
}

bool inBounds(int x, int y) { return x >= 0 && y >= 0 && x < GRID && y < GRID; }

void printPath() {
  Serial.print("DFS path (headings from start): ");
  for (int i = 0; i < sp; i++) Serial.print(DIR_CHAR[pathStack[i]]);
  Serial.println();
}

// One DFS step = drive to the next node, then decide where to go from there.
void exploreStep() {
  Exits ex;
  driveToNextNode(ex);
  if (failed) return;

  cx += DX[heading]; cy += DY[heading];
  visited[cx][cy] = true;
  Serial.printf("Node (%d,%d) heading %c  exits: L=%d F=%d R=%d%s\n",
                cx, cy, DIR_CHAR[heading], ex.left, ex.fwd, ex.right,
                ex.goal ? "  GOAL" : "");

  if (ex.goal) { stopMotors(); solved = true; return; }

  // Priority: LEFT, then FORWARD, then RIGHT (relative to current heading)
  const int8_t rel[3]  = {3, 0, 1};                 // +3 = left, +0 = forward, +1 = right
  const bool   open[3] = {ex.left, ex.fwd, ex.right};
  const char  *name[3] = {"LEFT", "FORWARD", "RIGHT"};

  for (int i = 0; i < 3; i++) {
    if (!open[i]) continue;
    uint8_t h = (heading + rel[i]) % 4;
    int nx = cx + DX[h], ny = cy + DY[h];
    if (inBounds(nx, ny) && !visited[nx][ny]) {
      Serial.printf("  -> take %s (go %c)\n", name[i], DIR_CHAR[h]);
      turnTo(h);
      pathStack[sp++] = h;                          // remember how we entered the next cell
      return;
    }
  }

  // Nothing new here: backtrack to the cell we came from
  if (sp == 0) { stopMotors(); failed = true; Serial.println("No path to goal."); return; }
  uint8_t back = (pathStack[--sp] + 2) % 4;
  Serial.printf("  -> dead end / explored, BACKTRACK (go %c)\n", DIR_CHAR[back]);
  turnTo(back);
}

// ================================ SETUP ===================================
void setup() {
  Serial.begin(115200);
  delay(500);
  hardwareInit();
  memset(visited, 0, sizeof(visited));
  visited[cx][cy] = true;                           // start cell

#if SIMULATION
  for (int i = 0; i < 5; i++) { calMin[i] = 200; calMax[i] = 3800; }   // typical sample range
  runPidSelfTest();
  simBuildMaze();
  Serial.println("\n=== DFS maze simulation (priority L > F > R) ===");
#else
  Serial.println("Place robot on the start line. Calibration starts in 2 s...");
  delay(2000);
  calibrateSensors();
  if (failed) return;
  Serial.println("S1,S2,S3,S4,S5,error,motorL,motorR");
  delay(1000);
#endif
}

void loop() {
  if (solved || failed) {
    if (solved) { printPath(); Serial.println("Maze solved."); }
    while (true) delay(1000);                       // halt
  }
  exploreStep();
#if SIMULATION
  delay(50);
#endif
}
