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
#include <algorithm>

const char* ssid     = "Aurel iPhone";
const char* password = "12345678";
WebServer server(80);

bool isTurningAnti   = true;
bool isTurningClock  = true;
bool turned          = false;
bool isAutonomous    = false;

#define MAX_CMD_LEN 32
char  cmdBuf[MAX_CMD_LEN];
uint8_t cmdPos = 0;

static unsigned long loopTimer       = 0;
static unsigned long printTimer      = 0;
static unsigned long odomToGridTimer = 0;

static float accelAngle       = 0;
static float spinAngle        = 0;
static float gyroAngle        = 0;
static float theta_n          = 0;
static float Turndrive        = 0;
static float current_speed    = 0;
static float WheelPos         = 0;
static float spinRad          = 0;
static float currentSpinAngle = 0;

static float CurXCoord        = 0;
static float CurYCoord        = 0;

static float prev_theta_n = 0;
static float prev_speed   = 0;
static float prevspin     = 0;

static float PrevWheelPos = 0;

const float kp = 2000, kd = 62, ki = 1;
const float sp = 0.002, sd = 0.000004, si = 0;
const float tp = 10, td = 8,  ti = 0;
const float c  = 0.98;

float set_speed      = 0;
float turn_reference = 0;
float xdistance      = 0;
float ydistance      = 0;

const int STEPPER1_DIR_PIN    = 16;
const int STEPPER1_STEP_PIN   = 17;
const int STEPPER2_DIR_PIN    = 4;
const int STEPPER2_STEP_PIN   = 14;
const int STEPPER_EN_PIN      = 15;
const int TOGGLE_PIN          = 32;
const int STEPPER_INTERVAL_US = 20;

ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;
step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

enum class CellState { UNKNOWN, FREE, ROBOT };

static const int gridWidth        = 30;
static const int gridHeight       = 30;
const float physicalCellSize = 0.25;

std::pair<int,int> worldToGrid(float x, float y) {
    int gx = static_cast<int>(x / physicalCellSize + gridWidth  / 2.0);        //  f
    int gy = static_cast<int>(y / physicalCellSize + gridHeight / 2.0);        //  f
    // Serial.print("GX: ");
    // Serial.print(gx);
    // Serial.print("GY: ");
    // Serial.print(gy);
    gx = std::max(0, std::min(gridWidth  - 1, gx));
    gy = std::max(0, std::min(gridHeight - 1, gy));
    // Serial.print("GX After: ");
    // Serial.print(gx);
    // Serial.print("GY After: ");
    // Serial.print(gy);
    return { gx, gy };

}

static std::vector<std::vector<CellState>> grid(
  gridHeight, std::vector<CellState>(gridWidth, CellState::UNKNOWN)
);

int prevRobotX = -1, prevRobotY = -1;

struct Node {
    int x, y;
    float f, g, h;
    Node* parent;
    Node(int x_, int y_, float g_, float h_, Node* parent_ = nullptr)
        : x(x_), y(y_), g(g_), h(h_), f(g_ + h_), parent(parent_) {}
    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

std::vector<std::pair<int,int>> getNeighbors(
    int x, int y,
    const std::vector<std::vector<CellState>>& gridRef
) {
    std::vector<std::pair<int,int>> neighbors;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (abs(dx) + abs(dy) == 1) {
                int nx = x + dx, ny = y + dy;
                if (nx >= 0 && nx < gridWidth &&
                    ny >= 0 && ny < gridHeight &&
                    (gridRef[ny][nx] == CellState::FREE || gridRef[ny][nx] == CellState::ROBOT)) {
                    neighbors.emplace_back(nx, ny);
                }
            }
        }
    }
    return neighbors;
}

float heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

std::vector<std::pair<int,int>> aStarPath(
    int sx, int sy,
    int gx, int gy,
    const std::vector<std::vector<CellState>>& gridRef
) {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
    std::unordered_map<int, Node*> allNodes;
    std::set<std::pair<int,int>> closedSet;

    auto hash = [&](int x, int y) { return y * gridWidth + x; };

    Node* startNode = new Node(sx, sy, 0.0f, heuristic(sx, sy, gx, gy));
    openSet.push(*startNode);
    allNodes[ hash(sx, sy) ] = startNode;

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        if (current.x == gx && current.y == gy) {
            std::vector<std::pair<int,int>> result;
            for (Node* n = &current; n != nullptr; n = n->parent) {
                result.emplace_back(n->x, n->y);
            }
            std::reverse(result.begin(), result.end());
            for (auto& kv : allNodes) delete kv.second;
            return result;
        }

        closedSet.insert({current.x, current.y});
        for (auto [nx, ny] : getNeighbors(current.x, current.y, gridRef)) {
            if (closedSet.count({nx, ny})) continue;
            float tentativeG = current.g + 1.0f;
            int key = hash(nx, ny);

            if (!allNodes.count(key) || tentativeG < allNodes[key]->g) {
                Node* neighbor = new Node(
                    nx, ny,
                    tentativeG,
                    heuristic(nx, ny, gx, gy),
                    allNodes[ hash(current.x, current.y) ]
                );
                allNodes[key] = neighbor;
                openSet.push(*neighbor);
            }
        }
    }

    for (auto& kv : allNodes) delete kv.second;
    return {};
}

static std::vector<std::pair<int,int>> path;
static size_t pathIndex = 0;
static bool pathActive = false;

bool TimerHandler(void* timerNo) {
    static bool toggle = false;
    step1.runStepper();
    step2.runStepper();
    digitalWrite(TOGGLE_PIN, toggle);
    toggle = !toggle;
    return true;
}

static float CurrentXDistance = 0, PrevXDistance = 0;
static float CurrentYDistance = 0, PrevYDistance = 0;
static float spinComp         = 0;

void setco() {
    if (xdistance == 0 && ydistance == 0) {
        set_speed    = 0;
        isAutonomous = false;
        pathActive   = false;
        return;
    }
    if (xdistance != 0) {
        float errorX = xdistance - WheelPos;
        if (WheelPos < xdistance && errorX >= 0.11f) {
            set_speed = -15;
        }
        else if (WheelPos < xdistance && errorX <= 0.15f) {
            set_speed = -15 * fabs(errorX) * 2;
        }
        else {
            set_speed = 0;
            xdistance = 0;
            if (!turned) {
                if (ydistance < 0) {
                    turn_reference += 1.57f;
                } else if (ydistance > 0) {
                    turn_reference -= 1.57f;
                } else {
                    turn_reference = 0;
                }
                turned = true;
                CurrentXDistance = 0;
                PrevXDistance    = 0;
            }
        }
        return;
    }
    if (ydistance != 0) {
        if (fabs(WheelPos) < fabs(ydistance) &&
            spinComp > turn_reference - 0.05f &&
            spinComp < turn_reference + 0.05f) {
            set_speed = -13;
        } else {
            set_speed = 0;
            ydistance = 0;
            turned    = false;
            CurrentYDistance = 0;
            PrevYDistance    = 0;
        }
    }
}

void handleCommand(const char* cmd) {
    Serial.print("handleCommand received raw: ");
    Serial.println(cmd);

    if (cmd[0] == 'x') {
        float x = 0, y = 0;
        const char* pX = strchr(cmd, 'x');
        const char* pY = strchr(cmd, 'y');
        if (pX) x = atof(pX + 1);
        if (pY) y = atof(pY + 1);

        xdistance   = x;
        ydistance   = y;
        turned      = false;
        isAutonomous = true;

        pathActive = false; // interrupt any ongoing path

        Serial.print("Parsed target X: ");
        Serial.print(x);
        Serial.print("  Y: ");
        Serial.println(y);

        Serial.print("Autonomous activated: ");
        Serial.println(isAutonomous);
    } else {
        Serial.print("Received non-x command: ");
        Serial.println(cmd[0]);

        switch (cmd[0]) {
            case 'f':
                set_speed = -15;
                isTurningClock = false;
                isTurningAnti = false;
                pathActive = false; // interrupt path
                break;
            case 'r':
                set_speed = 15;
                isTurningClock = false;
                isTurningAnti = false;
                pathActive = false;
                break;
            case 'c':
                if (!isTurningClock) {
                    turn_reference += 1.57f;
                    isTurningClock = true;
                    isTurningAnti = false;
                    pathActive = false;
                }
                break;
            case 'a':
                if (!isTurningAnti) {
                    turn_reference -= 1.57f;
                    isTurningAnti = true;
                    isTurningClock = false;
                    pathActive = false;
                }
                break;
            case 's':
                set_speed = 0;
                isTurningClock = false;
                isTurningAnti = false;
                pathActive = false;
                break;
            case 'q':
                set_speed = 0.0f;
                isTurningClock = false;
                isTurningAnti = false;
                isAutonomous = false;
                CurXCoord = CurYCoord = 0.0f;
                PrevXDistance = PrevYDistance = 0.0f;
                PrevWheelPos = CurrentXDistance = CurrentYDistance = currentSpinAngle = prevspin = 0.0f;
                path.clear();
                pathIndex = 0;
                pathActive = false;
                Serial.println("Reset command received.");
                break;
        }
    }
}

void checkSerialInput() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmdPos > 0) {
                cmdBuf[cmdPos] = '\0';
                handleCommand(cmdBuf);
                cmdPos = 0;
            }
        } else {
            if (cmdPos < MAX_CMD_LEN - 1) {
                cmdBuf[cmdPos++] = c;
            }
        }
    }
}

void handlePose() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    auto [robotGX, robotGY] = worldToGrid(CurXCoord, CurYCoord);

    StaticJsonDocument<200> doc;
    doc["x"] = robotGX;
    doc["y"] = robotGY;

    // Always return an empty obstacles array:
    JsonArray arr = doc.createNestedArray("obstacles");
    (void)arr;  // no obstacles for now

    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
}


void handleGoto() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    if (!server.hasArg("gx") || !server.hasArg("gy")) {
        server.send(400, "text/plain", "Missing gx or gy");
        return;
    }
    int gx = server.arg("gx").toInt();
    int gy = server.arg("gy").toInt();
    gx = constrain(gx, 0, gridWidth - 1);
    gy = constrain(gy, 0, gridHeight - 1);
    server.send(200, "text/plain", "OK");

    auto [startGX, startGY] = worldToGrid(CurXCoord, CurYCoord);
    path = aStarPath(startGX, startGY, gx, gy, grid);
    if (path.empty()) {
        Serial.println("A*: no path found!");
        pathActive = false;
        return;
    }

    pathIndex = 0;
    pathActive = true;
    Serial.printf("A*: path length = %d\n", (int)path.size());

    if (path.size() > 1) {
        auto [nx, ny] = path[1];
        float targetX = nx * physicalCellSize;
        float targetY = ny * physicalCellSize;
        char cmd[32];
        int len = snprintf(cmd, sizeof(cmd), "x%.2fy%.2f", targetX, targetY);
        cmd[len] = '\0';
        handleCommand(cmd);
        pathIndex = 1;
    }
}

void handleDiscreteCommand(const char* c) {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    handleCommand(c);
    server.send(200, "text/plain", "OK");
}

void setupHttpRoutes() {
    server.on("/pose", HTTP_GET, [](){ handlePose(); });
    server.on("/goto", HTTP_GET, [](){ handleGoto(); });
    server.on("/f", HTTP_GET, [](){ handleDiscreteCommand("f"); });
    server.on("/r", HTTP_GET, [](){ handleDiscreteCommand("r"); });
    server.on("/a", HTTP_GET, [](){ handleDiscreteCommand("a"); });
    server.on("/c", HTTP_GET, [](){ handleDiscreteCommand("c"); });
    server.on("/s", HTTP_GET, [](){ handleDiscreteCommand("s"); });
    server.on("/q", HTTP_GET, [](){ handleDiscreteCommand("q"); });
    server.onNotFound([](){
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(404, "text/plain", "Not Found");
    });
}

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
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) delay(10);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

    CurXCoord = CurYCoord = 0.0f;
    prevRobotX = prevRobotY = -1;
    PrevWheelPos = 0.0f;
    prev_theta_n = prev_speed = prevspin = 0.0f;

    if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
        Serial.println("Failed to start stepper interrupt");
        while (1) delay(10);
    }
    pinMode(STEPPER_EN_PIN, OUTPUT);
    digitalWrite(STEPPER_EN_PIN, LOW);

    for (int r = 0; r < gridHeight; r++) {
        for (int c = 0; c < gridWidth; c++) {
            grid[r][c] = CellState::UNKNOWN;
        }
    }

    setupWiFi();
    setupHttpRoutes();
    server.begin();
    Serial.println("HTTP server started");
}

void loop() {
    unsigned long now = millis();

    if (now > loopTimer) {
        loopTimer += 5;
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        accelAngle = (a.acceleration.z / 9.67f) +0;
        spinAngle  = (g.gyro.roll) + 0.031f;
        gyroAngle  = (g.gyro.pitch);

        // WheelPos = step1.getPositionRad();
        // float deltaWheel  = newWheelPos - PrevWheelPos;
        // PrevWheelPos      = newWheelPos;

        // const float R_wheel = 0.05f;
        // float dMeters = R_wheel * deltaWheel;

        // currentSpinAngle += spinAngle;
        // spinRad = (currentSpinAngle * DEG_TO_RAD) / 4.0f;
        // spinComp = (spinAngle * 0.005f) + prevspin;

        // CurXCoord += dMeters * cos(spinRad);
        // CurYCoord += dMeters * sin(spinRad);

        WheelPos = step1.getPositionRad();

        // Convert spinAngle from degrees to radians
        currentSpinAngle += spinAngle;
        spinRad = (currentSpinAngle * DEG_TO_RAD)/4;

        // Update X and Y using delta
        CurXCoord += WheelPos * cos(spinRad) * 10;
        CurYCoord += WheelPos * sin(spinRad) * 10;

        if (isAutonomous) {
            setco();
        }

        current_speed = step1.getSpeedRad();
        float s_error = set_speed - current_speed;
        float Ps = s_error * sp;
        float Ds = -((s_error - prev_speed) / 0.005f) * sd;
        float Is = (s_error + prev_speed) * si * 0.005f;
        float reference = Ps + Ds + Is - 0.004f;

        theta_n = (1 - c) * accelAngle
                + c * (((gyroAngle - 0.02f) * 0.005f) + prev_theta_n);

        float err      = reference - theta_n;
        float prev_err = reference - prev_theta_n;
        float Pout     = err * kp;
        float Dout     = -((err - prev_err) / 0.005f) * kd;
        float Iout     = (err + prev_err) * ki * 0.005f;
        prev_theta_n   = Pout + Dout + Iout;

        float t_error = turn_reference - spinComp;
        float prev_te = turn_reference - prevspin;
        float Pto     = t_error * tp;
        float Dto     = ((t_error - prev_te) / 0.005f) * td;
        float Ito     = (t_error + prev_te) * ti * 0.005f;
        Turndrive     = Pto + Dto + Ito;

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

    if (now > printTimer) {
        printTimer += 200;
        Serial.print("X:"); Serial.print(CurXCoord, 2);
        Serial.print(", Y:"); Serial.print(CurYCoord, 2);
        Serial.print(", spinRad:"); Serial.println(spinRad, 2);

    }

    if (now > odomToGridTimer) {
        odomToGridTimer = now + 100;
        auto [newGX, newGY] = worldToGrid(CurXCoord, CurYCoord);

        if (prevRobotX >= 0 && prevRobotY >= 0 &&
            grid[prevRobotY][prevRobotX] == CellState::ROBOT) {
            grid[prevRobotY][prevRobotX] = CellState::FREE;
        }
        if (grid[newGY][newGX] == CellState::UNKNOWN) {
            grid[newGY][newGX] = CellState::FREE;
        }
        grid[newGY][newGX] = CellState::ROBOT;
        prevRobotX = newGX;
        prevRobotY = newGY;
    }

    if (pathActive && pathIndex < path.size()) {
        auto [tx, ty]   = path[pathIndex];
        auto [rgx, rgy] = worldToGrid(CurXCoord, CurYCoord);

        if (rgx == tx && rgy == ty) {
            if (pathIndex + 1 < path.size()) {
                pathIndex++;
                auto [nx, ny] = path[pathIndex];
                float nextX = nx * physicalCellSize;
                float nextY = ny * physicalCellSize;
                char cmd[32];
                int len = snprintf(cmd, sizeof(cmd), "x%.2fy%.2f", nextX, nextY);
                cmd[len] = '\0';
                handleCommand(cmd);
            } else {
                pathActive = false;
                Serial.println("A*: reached goal");
            }
        }
    }

    checkSerialInput();
    server.handleClient();
}
