#include <Arduino.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

#include <vector>
#include <queue>
#include <unordered_map>
#include <set>
#include <algorithm>   // for std::reverse

// ───────────────────────────────────────────────────────────────────────────────
//                                  Wi-Fi / HTTP
// ───────────────────────────────────────────────────────────────────────────────
const char* ssid     = "Maciek";
const char* password = "enidissexy";
WebServer server(80);  // HTTP server on port 80

// ───────────────────────────────────────────────────────────────────────────────
//                         Balance / Stepper / PID VARIABLES
// ───────────────────────────────────────────────────────────────────────────────
bool isTurningAnti   = true;
bool isTurningClock  = true;
bool turned          = false;
bool isAutonomous    = false;

#define MAX_CMD_LEN 32
char  cmdBuf[MAX_CMD_LEN];
uint8_t cmdPo = 0;

static unsigned long loopTimer    = 0;
static unsigned long printTimer   = 0;
static unsigned long odomToGridTimer = 0;  // <— new: for periodic odometry→grid

// IMU / stepper / odometry state
static float accelAngle       = 0;
static float spinAngle        = 0;
static float gyroAngle        = 0;
static float theta_n          = 0;
static float Turndrive        = 0;
static float current_speed    = 0;
static float WheelPos         = 0;
static float spinRad          = 0;
static float currentSpinAngle = 0;

// Continuous (real‐valued) odometry in meters (or cm)
static float CurXCoord        = 0;
static float CurYCoord        = 0;

// For PID loop
static float prev_theta_n = 0;
static float prev_speed   = 0;
static float prevspin     = 0;

// PID Gains (unchanged)
const float kp = 2200, kd = 62, ki = 1;
const float sp = 0.002, sd = 0.000004, si = 0;
const float tp = 10, td = 8, ti = 0;
const float c  = 0.98;

// “Autonomous” setpoints (only used if you send discrete commands)
float set_speed      = 0;
float turn_reference = 0;
float xdistance      = 0;
float ydistance      = 0;

// Abort flag for any in‐progress A* path
volatile bool abortPath = false;

// Stepper/MPU pins
const int STEPPER1_DIR_PIN   = 16;
const int STEPPER1_STEP_PIN  = 17;
const int STEPPER2_DIR_PIN   = 4;
const int STEPPER2_STEP_PIN  = 14;
const int STEPPER_EN_PIN     = 15;
const int TOGGLE_PIN         = 32;
const int STEPPER_INTERVAL_US = 20;

// IMU + stepper objects
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;
step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN,  STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN,  STEPPER2_DIR_PIN);

// ───────────────────────────────────────────────────────────────────────────────
//                              30×30 GRID + A* ROUTINES
// ───────────────────────────────────────────────────────────────────────────────
static const int GRID_W = 30;
static const int GRID_H = 30;

// 0 = FREE, 1 = OBSTACLE
uint8_t gridMap[GRID_H][GRID_W];

// Robot’s current grid cell (integers)
volatile int robotGX = GRID_W/2;
volatile int robotGY = GRID_H/2;

// ISR: run both steppers
bool TimerHandler(void *timerNo) {
  static bool toggle = false;
  step1.runStepper();
  step2.runStepper();
  digitalWrite(TOGGLE_PIN, toggle);
  toggle = !toggle;
  return true;
}

// A* Node struct
struct Node {
  int x, y, g, h, f;
  Node* parent;
  Node(int _x,int _y,int _g,int _h, Node* _p=nullptr) {
    x = _x; y = _y; g = _g; h = _h; f = _g + _h; parent = _p;
  }
};

struct NodeCompare {
  bool operator()(Node* a, Node* b) {
    return a->f > b->f;
  }
};

inline int heuristic(int x1,int y1,int x2,int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}
inline bool isFree(int x,int y) {
  return (x >= 0 && x < GRID_W && y >= 0 && y < GRID_H && gridMap[y][x] == 0);
}
std::vector<std::pair<int,int>> getNeighbors(int x,int y) {
  std::vector<std::pair<int,int>> nbrs;
  if (isFree(x+1,y)) nbrs.emplace_back(x+1,y);
  if (isFree(x-1,y)) nbrs.emplace_back(x-1,y);
  if (isFree(x,y+1)) nbrs.emplace_back(x,y+1);
  if (isFree(x,y-1)) nbrs.emplace_back(x,y-1);
  return nbrs;
}
inline int cellHash(int x,int y) { return y * GRID_W + x; }

std::vector<std::pair<int,int>> aStarPath(int sx,int sy,int gx,int gy) {
  std::priority_queue<Node*, std::vector<Node*>, NodeCompare> openSet;
  std::unordered_map<int, Node*> allNodes;
  std::set<std::pair<int,int>> closedSet;

  Node* start = new Node(sx, sy, 0, heuristic(sx,sy,gx,gy), nullptr);
  openSet.push(start);
  allNodes[cellHash(sx,sy)] = start;

  while (!openSet.empty()) {
    Node* cur = openSet.top();
    openSet.pop();

    if (cur->x == gx && cur->y == gy) {
      std::vector<std::pair<int,int>> path;
      for (Node* n = cur; n; n = n->parent) {
        path.emplace_back(n->x, n->y);
      }
      std::reverse(path.begin(), path.end());
      // clean up
      for (auto &kv : allNodes) delete kv.second;
      return path;
    }

    closedSet.insert({cur->x, cur->y});
    for (auto &nbr : getNeighbors(cur->x, cur->y)) {
      int nx = nbr.first, ny = nbr.second;
      if (closedSet.count({nx, ny})) continue;
      int tentativeG = cur->g + 1;
      int hkey = cellHash(nx, ny);
      if (!allNodes.count(hkey) || tentativeG < allNodes[hkey]->g) {
        Node* neighbor = new Node(nx, ny,
                                  tentativeG,
                                  heuristic(nx, ny, gx, gy),
                                  cur);
        allNodes[hkey] = neighbor;
        openSet.push(neighbor);
      }
    }
  }

  for (auto &kv : allNodes) delete kv.second;  // no path found
  return {};
}

// ───────────────────────────────────────────────────────────────────────────────
//            GLOBAL A* STATE (drive one cell at a time from loop())
// ───────────────────────────────────────────────────────────────────────────────
std::vector<std::pair<int,int>> currentPath;
size_t pathIndex = 0;
bool   pathActive = false;

// ───────────────────────────────────────────────────────────────────────────────
// ** Handler for /pose  (returns JSON of current robot cell + obstacles) **
// ───────────────────────────────────────────────────────────────────────────────
void handlePose() {
  server.sendHeader("Access-Control-Allow-Origin", "*"); // CORS

  StaticJsonDocument<600> doc;
  doc["x"] = robotGX;
  doc["y"] = robotGY;
  JsonArray arr = doc.createNestedArray("obstacles");
  for (int r = 0; r < GRID_H; r++) {
    for (int c = 0; c < GRID_W; c++) {
      if (gridMap[r][c] == 1) {
        JsonArray pair = arr.createNestedArray();
        pair.add(c);
        pair.add(r);
      }
    }
  }
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// ───────────────────────────────────────────────────────────────────────────────
// ** Handler for /goto?gx=<>&gy=<>  (compute A* path, store globally, return ) **
// ───────────────────────────────────────────────────────────────────────────────
void handleGoto() {
  server.sendHeader("Access-Control-Allow-Origin", "*"); // CORS

  if (!server.hasArg("gx") || !server.hasArg("gy")) {
    server.send(400, "text/plain", "Missing gx or gy");
    return;
  }
  int gx = server.arg("gx").toInt();
  int gy = server.arg("gy").toInt();
  gx = max(0, min(GRID_W - 1, gx));
  gy = max(0, min(GRID_H - 1, gy));

  // Return immediately so browser isn’t blocked
  server.send(200, "text/plain", "OK");

  // Build the path once and store it
  abortPath        = false;
  currentPath.clear();
  pathIndex        = 0;

  auto p = aStarPath(robotGX, robotGY, gx, gy);
  if (p.empty()) {
    Serial.println("A*: no path found!");
    return;
  }
  currentPath = std::move(p);
  pathActive  = true;
}

// ───────────────────────────────────────────────────────────────────────────────
// ** Handler for /f, /r, /a, /c, /s  (immediate discrete commands) **
// ───────────────────────────────────────────────────────────────────────────────
void handleDiscreteCommand(const char* cmd) {
  Serial.printf("handleDiscreteCommand: %c\n", cmd[0]);

  if (cmd[0] == 's') {
    // Immediately abort any in-progress A* path
    set_speed      = 0;
    isTurningClock = false;
    isTurningAnti  = false;
    abortPath      = true;
    return;
  }
  // (We ignore “x<>y<>” here; front-end always goes /goto instead)
  switch (cmd[0]) {
    case 'f':
      set_speed = -10;
      isTurningClock = false;
      isTurningAnti  = false;
      break;
    case 'r':
      set_speed = 13;
      isTurningClock = false;
      isTurningAnti  = false;
      break;
    case 'c':
      if (!isTurningClock) {
        turn_reference += 1.57;
        isTurningClock  = true;
        isTurningAnti   = false;
      }
      break;
    case 'a':
      if (!isTurningAnti) {
        turn_reference -= 1.57;
        isTurningAnti   = true;
        isTurningClock  = false;
      }
      break;
  }
}

// ───────────────────────────────────────────────────────────────────────────────
//            Register HTTP routes (call once from setup())
// ───────────────────────────────────────────────────────────────────────────────
void setupHttpRoutes() {
  // 1) /pose
  server.on("/pose", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    handlePose();
  });

  // 2) /goto?gx=<>&gy=<>
  server.on("/goto", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    handleGoto();
  });

  // 3) /f, /r, /a, /c, /s
  server.on("/f", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    handleDiscreteCommand("f");
    server.send(200, "text/plain", "OK");
  });
  server.on("/r", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    handleDiscreteCommand("r");
    server.send(200, "text/plain", "OK");
  });
  server.on("/a", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    handleDiscreteCommand("a");
    server.send(200, "text/plain", "OK");
  });
  server.on("/c", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    handleDiscreteCommand("c");
    server.send(200, "text/plain", "OK");
  });
  server.on("/s", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    handleDiscreteCommand("s");
    server.send(200, "text/plain", "OK");
  });

  // 4) 404 for everything else
  server.onNotFound([]() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(404, "text/plain", "Not Found");
  });
}

// ───────────────────────────────────────────────────────────────────────────────
//                          WiFi & setup() / loop()
// ───────────────────────────────────────────────────────────────────────────────
void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nWiFi IP = %s\n", WiFi.localIP().toString().c_str());
}

void setup() {
  Serial.begin(115200);
  pinMode(TOGGLE_PIN, OUTPUT);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  CurXCoord = CurYCoord = 0;

  // Start stepper interrupt
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
  }
  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, LOW);

  // Initialize 30×30 occupancy grid (all free=0)
  for (int r = 0; r < GRID_H; r++) {
    for (int c = 0; c < GRID_W; c++) {
      gridMap[r][c] = 0;
    }
  }
  // Example obstacles (replace later by real sensors)
  gridMap[12][10] = 1;
  gridMap[12][11] = 1;
  gridMap[12][12] = 1;
  gridMap[13][12] = 1;
  gridMap[14][12] = 1;

  setupWiFi();
  setupHttpRoutes();
  server.begin();
  Serial.println("HTTP server started");
}

void checkSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdPos > 0) {
        cmdBuf[cmdPos] = '\0';
        handleDiscreteCommand(cmdBuf);
        cmdPos = 0;
      }
    } else {
      if (cmdPos < MAX_CMD_LEN - 1) {
        cmdBuf[cmdPos++] = c;
      }
    }
  }
}

void loop() {
  unsigned long now = millis();

  // ─────────────────────────────────────────────────────────────────────────────
  // 1) Balance + PID (every ~5 ms) – exactly same as before
  // ─────────────────────────────────────────────────────────────────────────────
  if (now > loopTimer) {
    loopTimer += 5;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    accelAngle = (a.acceleration.z / 9.67f) - 0.040f;
    spinAngle  = (g.gyro.roll)    + 0.021f;
    gyroAngle  = (g.gyro.pitch);

    WheelPos = step1.getPositionRad();
    currentSpinAngle += spinAngle;
    spinRad = (currentSpinAngle * DEG_TO_RAD) / 4.0f;

    // Update continuous odometry (in meters or cm):
    CurXCoord += WheelPos * cos(spinRad) * 10.0f;
    CurYCoord += WheelPos * sin(spinRad) * 10.0f;

    // Speed control
    current_speed = step1.getSpeedRad();
    float speed_error = set_speed - current_speed;
    float Ps = speed_error * sp;
    float Ds = -((speed_error - prev_speed) / 0.005f) * sd;
    float Is = (speed_error + prev_speed) * si * 0.005f;
    float reference = Ps + Ds + Is - 0.004f;

    // Balance control
    theta_n = (1 - c) * accelAngle
            + c * (((gyroAngle - 0.02f) * 0.005f) + prev_theta_n);
    float spinComp = (spinAngle * 0.005f) + prevspin;

    float error     = reference - theta_n;
    float prev_err  = reference - prev_theta_n;
    float P         = error * kp;
    float D         = -((error - prev_err) / 0.005f) * kd;
    float I         = (error + prev_err) * ki * 0.005f;
    prev_theta_n    = P + D + I;

    float turn_error    = turn_reference - spinComp;
    float prev_turn_err = turn_reference - prevspin;
    float Pt            = turn_error * tp;
    float Dt            = ((turn_error - prev_turn_err) / 0.005f) * td;
    float It            = (turn_error + prev_turn_err) * ti * 0.005f;
    Turndrive           = Pt + Dt + It;

    if ((theta_n < 0.02f) && (theta_n > -0.02f)) {
      step1.setAccelerationRad(-prev_theta_n - Turndrive);
      step2.setAccelerationRad(prev_theta_n - Turndrive);
    } else {
      step1.setAccelerationRad(-prev_theta_n);
      step2.setAccelerationRad(prev_theta_n);
    }

    if (prev_theta_n > 0) {
      step1.setTargetSpeedRad(15);
      step2.setTargetSpeedRad(-15);
    } else {
      step1.setTargetSpeedRad(-15);
      step2.setTargetSpeedRad(15);
    }

    prev_theta_n = theta_n;
    prev_speed   = current_speed;
    prevspin     = spinComp;
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // 2) Print odometry once every 200 ms (unchanged)
  // ─────────────────────────────────────────────────────────────────────────────
  if (now > printTimer) {
    printTimer += 200;
    Serial.print("X:"); Serial.print(CurXCoord, 2);
    Serial.print(", Y:"); Serial.print(CurYCoord, 2);
    Serial.print(", spinRad:"); Serial.println(spinRad, 2);
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // 3) Handle any Serial‐over‐USB commands
  // ─────────────────────────────────────────────────────────────────────────────
  checkSerialInput();

  // ─────────────────────────────────────────────────────────────────────────────
  // 4) Handle any pending HTTP requests
  // ─────────────────────────────────────────────────────────────────────────────
  server.handleClient();

  // ─────────────────────────────────────────────────────────────────────────────
  // 5) If an A* path is active, drive one cell at a time, interleaving server.handleClient()
  // ─────────────────────────────────────────────────────────────────────────────
  if (pathActive && pathIndex < currentPath.size()) {
    int nx = currentPath[pathIndex].first;
    int ny = currentPath[pathIndex].second;

    float wx = nx * 0.2f;
    float wy = ny * 0.2f;
    char buf[32];
    int  n = snprintf(buf, sizeof(buf), "x%.2fy%.2f\n", wx, wy);
    Serial2.write((uint8_t*)buf, n);

    bool reached = false;
    String incoming = "";
    unsigned long t0 = millis();
    while (!reached && !abortPath) {
      server.handleClient();   // ← Let the server see a /s if user hits STOP

      if (Serial2.available()) {
        char c = Serial2.read();
        incoming += c;
        if (c == '\n') {
          float rx = 0, ry = 0;
          if (sscanf(incoming.c_str(), "X:%f,Y:%f", &rx, &ry) == 2) {
            int cx = int(rx / 0.2f + 0.5f);
            int cy = int(ry / 0.2f + 0.5f);
            if (cx == nx && cy == ny) {
              reached = true;
              robotGX = cx;  // ← Update grid‐cell at each step
              robotGY = cy;
            }
          }
          incoming = "";
        }
      }
      if (millis() - t0 > 1000) {
        Serial.println("Timeout waiting for Serial2 reply");
        break;
      }
    }

    if (abortPath) {
      pathActive = false;
      Serial.println("Path aborted by /s");
    }
    else if (reached) {
      pathIndex++;
      if (pathIndex >= currentPath.size()) {
        pathActive = false;
        Serial.println("Reached final goal");
      }
    }
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // 6) NEW: Every 100 ms, update robotGX/robotGY from free odometry (CurXCoord,CurYCoord)
  //    This ensures that even if you drive manually (via /f,/r,/a,/c) the pose on /pose is up‐to‐date.
  // ─────────────────────────────────────────────────────────────────────────────
  if (now > odomToGridTimer) {
    odomToGridTimer = now + 100;  // poll every 100 ms

    // Convert continuous odometry (in meters) to grid indices [0..29]
    int gx = int(CurXCoord / 0.2f + 0.5f);
    int gy = int(CurYCoord / 0.2f + 0.5f);

    // clamp
    gx = max(0, min(GRID_W - 1, gx));
    gy = max(0, min(GRID_H - 1, gy));

    robotGX = gx;
    robotGY = gy;
  }
}
