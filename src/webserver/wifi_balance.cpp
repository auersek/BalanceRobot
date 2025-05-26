#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include <Arduino.h>

// === Network credentials – change these to your Wi-Fi SSID & password ===
const char* ssid     = "YOUR_SSID";
const char* password = "YOUR_PASS";

// === HTTP server on port 80 ===
AsyncWebServer server(80);

// === Your existing globals ===
bool isTurningAnti  = true;
bool isTurningClock = true;
bool turned         = false;

// Timing and sensor state
static unsigned long printTimer = 0;
static unsigned long loopTimer  = 0;
static float AccelAngle = 0, SpinAngle = 0, GyroAngle = 0;
static float current=0, Turndrive=0, currentspeed=0, WheelPos=0;
static float CurrentXDistance, PrevXDistance, CurrentYDistance, PrevYDistance=0;

// Errors and PID state
static float prev, prevspeed, prevspin;
static float error, speederror, turnerror;
static float preverror, prevspeederror, prevturnerror;
static float P,D,I,Ps,Ds,Is,Pt,Dt,It;

// Stepper & MPU6050 objects
const int STEPPER1_DIR_PIN  = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15;
const int TOGGLE_PIN        = 32;

ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;
step step1(/*interval_us=*/20, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(/*interval_us=*/20, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

// PID Gains
const float kp = 2000, kd = 65, ki = 5;
const float sp = 0.002, sd = 0, si = 0;
const float tp = 10, td = 8, ti = 0;
const float c  = 0.98;

// Motion targets
char currentOperation = 'S';
float setpoint, setspeed=0, setturn=0, xdistance=0, ydistance=0;

// ISR for driving steppers
bool TimerHandler(void *) {
  step1.runStepper();
  step2.runStepper();
  digitalWrite(TOGGLE_PIN, !digitalRead(TOGGLE_PIN));
  return true;
}

// Autonomous control stub (unused for manual API)
void setco() {
  // your existing autonomous code, if any
}

void setup() {
  // Serial for debugging
  Serial.begin(115200);
  pinMode(TOGGLE_PIN, OUTPUT);

  // Initialize MPU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  Serial.println("MPU6050 Initialized");

  // Attach stepper ISR
  if (!ITimer.attachInterruptInterval(20, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
  }
  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, LOW);

  // --- Wi-Fi setup ---
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // --- HTTP API routes ---
  // Map GET /f → forward, /r → reverse, /a → anticlockwise, /c → clockwise, /s → stop
  server.on("/f", HTTP_GET, [](AsyncWebServerRequest *req){
    currentOperation = 'f';
    req->send(200, "text/plain", "OK");
    Serial.println("HTTP command: f");
  });
  server.on("/r", HTTP_GET, [](AsyncWebServerRequest *req){
    currentOperation = 'r';
    req->send(200, "text/plain", "OK");
    Serial.println("HTTP command: r");
  });
  server.on("/a", HTTP_GET, [](AsyncWebServerRequest *req){
    currentOperation = 'a';
    req->send(200, "text/plain", "OK");
    Serial.println("HTTP command: a");
  });
  server.on("/c", HTTP_GET, [](AsyncWebServerRequest *req){
    currentOperation = 'c';
    req->send(200, "text/plain", "OK");
    Serial.println("HTTP command: c");
  });
  server.on("/s", HTTP_GET, [](AsyncWebServerRequest *req){
    currentOperation = 's';
    req->send(200, "text/plain", "OK");
    Serial.println("HTTP command: s");
  });

  // Optional status page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send(200, "text/html",
             "<h1>ESP32 Balance API</h1><p>Use /f, /r, /a, /c, /s</p>");
  });

  server.begin();
}

void loop() {
  // Control loop timing
  if (millis() > loopTimer) {
    loopTimer += 5;  // LOOP_INTERVAL

    // Read sensors
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    AccelAngle = (a.acceleration.z/9.67) - 0.036;
    SpinAngle  = (g.gyro.roll) + 0.001;
    GyroAngle  = (g.gyro.pitch);

    setco();
    WheelPos = step1.getPosition();

    // Speed PID
    currentspeed   = step1.getSpeedRad();
    speederror     = setspeed - currentspeed;
    prevspeederror = speederror;  // simple derivative
    Ps = speederror * sp;
    Ds = -((speederror - prevspeederror)/0.005) * sd;
    Is = (speederror + prevspeederror) * si * 0.005;
    setpoint = Ps + Ds + Is - 0.004;

    // Complementary filter for balance
    current = (1 - c) * AccelAngle
            + c * ((GyroAngle - 0.02)*0.005 + prev);

    SpinComp = (SpinAngle)*0.005 + prevspin;
    turnerror = setturn - SpinComp;
    error     = setpoint - current;
    preverror      = error;
    prevturnerror  = turnerror;

    // Balance PID
    P = error * kp;
    D = -((error - preverror)/0.005) * kd;
    I = (error + preverror) * ki * 0.005;
    prev = P + D + I;

    // Turn PID
    Pt = turnerror * tp;
    Dt = ((turnerror - prevturnerror)/0.005) * td;
    It = (turnerror + prevturnerror) * ti * 0.005;
    Turndrive = Pt + Dt + It;

    // Drive steppers
    if (abs(current) < 0.02) {
      step1.setAccelerationRad(-prev - Turndrive);
      step2.setAccelerationRad( prev - Turndrive);
    } else {
      step1.setAccelerationRad(-prev);
      step2.setAccelerationRad( prev);
    }

    if (prev > 0) {
      step1.setTargetSpeedRad( 15);
      step2.setTargetSpeedRad(-15);
    } else {
      step1.setTargetSpeedRad(-15);
      step2.setTargetSpeedRad( 15);
    }

    prev = current;
    prevspeed = currentspeed;
    prevspin  = SpinComp;
  }

  // Print telemetry
  if (millis() > printTimer) {
    printTimer += 200;  // PRINT_INTERVAL
    Serial.print(AccelAngle, 4);  Serial.print(",");
    Serial.print(GyroAngle, 4);   Serial.print(",");
    Serial.print(SpinAngle, 4);   Serial.print(",");
    Serial.print(currentspeed, 4); Serial.print(",");
    Serial.println(prev - Turndrive, 4);
  }

  // Apply the last HTTP‐set `currentOperation`
  switch (currentOperation) {
    case 'f': setspeed = -7;  isTurningClock = isTurningAnti = false; break;
    case 'r': setspeed =  9;  isTurningClock = isTurningAnti = false; break;
    case 'c':
      if (!isTurningClock) { setturn += 1.57; isTurningClock=true; isTurningAnti=false; }
      break;
    case 'a':
      if (!isTurningAnti)  { setturn -= 1.57; isTurningAnti=true;  isTurningClock=false; }
      break;
    case 's': setspeed = 0;   isTurningClock = isTurningAnti = false; break;
  }
}
