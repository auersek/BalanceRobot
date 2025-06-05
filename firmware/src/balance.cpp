#include <Arduino.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include <stdio.h>
#include <stdlib.h>
#include <HardwareSerial.h>
bool isTurningAnti=true;
bool isTurningClock=true;
bool turned = false;
bool isAutonomous = false;
// Power monitoring constants
#define SAMPLES 64 // Multisampling
#define CELLS 12 // Battery cells
#define MAX_CMD_LEN 32

char cmdBuf[MAX_CMD_LEN];
uint8_t cmdPos = 0;

const int bufferSize = 32; // Set buffer size to 32 bytes

static unsigned long coordinateTimer = 0;  
static unsigned long printTimer = 0;       //time of the next print
static unsigned long loopTimer = 0;        //time of the next control update
static float accelAngle = 0;
static float spinAngle = 0;                //theta_ntilt angle
static float gyroAngle = 0;                //rate of change of tilt angle 
static float theta_n= 0;
static float Turndrive = 0;
static float current_speed = 0;
static float WheelPos = 0;
static float CurrentXDistance, PrevXDistance, CurrentYDistance, PrevYDistance = 0;
//Errors
static float prev_theta_n, prev_speed, prevspin;
static float error, speed_error, turn_error; 
static float prev_error, prev_speed_error, prev_turn_error;    
//PID Gains 
static float P, D, I, Ps, Ds, Is, Pt, Dt, It;
//Components
static float GyroComp, AccelComp, spinComp; 
static float turn_coord = 0;

// Coordinates
static float PrevWheelPos = 0;
static float deltaWheel = 0;
static float CurXCoord = 0, PrevXCoord = 0;
static float CurYCoord = 0, PrevYCoord = 0;
static float deltaDistance = 0;
static float spinRad = 0;
static float currentSpinAngle = 0;
static float spinAngleStep = 0; 
static float currentSpinAngleStep = 0;
static float spinRadStep = 0;
static float WheelPosX = 0;
static float WheelPosY = 0;

// The Stepper pins
const int STEPPER1_DIR_PIN  = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15; 

//ADC pins
const int ADC_CS_PIN        = 5;
const int ADC_SCK_PIN       = 18;
const int ADC_MISO_PIN      = 19;
const int ADC_MOSI_PIN      = 23;

// Diagnostic pin for oscilloscope
const int TOGGLE_PIN        = 32;

const int COORDINATE_INTERVAL = 200;
const int PRINT_INTERVAL = 200;
const int LOOP_INTERVAL = 5;
const int STEPPER_INTERVAL_US = 20;
char currentOperation='S';

//PID values
const float kp = 2000;        // with battery  2200
const float kd = 60;          // with battery  64
const float ki = 0.1;         // make 0 later if in a bad mood, ki is the worst constant in the game no question

const float sp = 0.002;       // with battery 0.002
const float sd = 0.00001;    // with battery 0.00001
const float si = 0;

const float tp = 10;
const float td = 8;
const float ti = 0;

//time const
const float c = 0.98;

//static
float reference;// = 0.0135;
float set_speed = 0;         
float turn_reference = 0;
float xdistance = 0;  //1m = 2000
float ydistance = 0;

//Global objects
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;         //Default pins for I2C are SCL: IO22/Arduino D3, SDA: IO21/Arduino D4

step step1(STEPPER_INTERVAL_US,STEPPER1_STEP_PIN,STEPPER1_DIR_PIN );
step step2(STEPPER_INTERVAL_US,STEPPER2_STEP_PIN,STEPPER2_DIR_PIN );

//Interrupt Service Routine for motor update
//Note: ESP32 doesn't support floating point calculations in an ISR

bool TimerHandler(void * timerNo)
{
  static bool toggle = false;

  //Update the stepper motors
  step1.runStepper();
  step2.runStepper();

  //Indicate that the ISR is running
  digitalWrite(TOGGLE_PIN,toggle);  
  toggle = !toggle;
  return true;
}

void setup()
{
  const int bufferSize = 32; // Set buffer size to 32 bytes
  Serial.begin(115200);
  pinMode(TOGGLE_PIN,OUTPUT);

  // Try to initialize Accelerometer/Gyroscope
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  CurXCoord = 0;
  CurYCoord = 0;

  //Attach motor update ISR to timer to run every STEPPER_INTERVAL_US μs
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
    }
  Serial.println("Initialised Interrupt for Stepper");

  //Enable the stepper motor drivers
  pinMode(STEPPER_EN_PIN,OUTPUT);
  digitalWrite(STEPPER_EN_PIN, false);
}
     
void setco() {
  // If both distances have been reached, stop and exit autonomous mode
  if (xdistance == 0 && ydistance == 0) {
    set_speed = 0;                            // Possibliy change to accelAngle bias
    isAutonomous = false;
    return;
  }
  // Update X movement if not yet reached
  if (xdistance != 0) {
    // Update current X distance based on wheel movement
    // float deltaX = WheelPos;     //  / 200.0) * 6.5
    // CurrentXDistance += deltaX;
    // PrevXDistance = CurrentXDistance;
   
    float errorX = xdistance - WheelPos;

    // Move forward in X if within heading tolerance
    if (WheelPos < xdistance && errorX >= 0.11) {     // CurrentXDistance         && spinComp > turn_reference - 0.05 && spinComp < turn_reference + 0.05
      set_speed = -15;
    } 
    else if (WheelPos < xdistance && errorX <= 0.15) {     // CurrentXDistance         && spinComp > turn_reference - 0.05 && spinComp < turn_reference + 0.05
      set_speed = -15*abs(errorX)*2;
    } 
    // else if (WheelPos > xdistance) {   //  && spinComp > turn_reference - 0.05 && spinComp < turn_reference + 0.05
    //   set_speed = 15*abs(errorX);
    // } 
    else {
      // Stop X movement and begin turning
      set_speed = 0;
      xdistance = 0;

      if (!turned) {
        if (ydistance < 0) {
          turn_reference += 1.57;  // +90 degrees
        } else if (ydistance > 0) {
          turn_reference -= 1.57;  // -90 degrees
        } else{
          turn_reference = 0;
        } 
        turned = true;
        CurrentXDistance = 0;
        PrevXDistance = 0;
      }
    }
    return;  // Exit for now — Y handled after X
  }
  // If turning is done and ydistance remains
  if (ydistance != 0) {
    // Update current Y distance
    // float deltaY = WheelPos;     //  / 200.0) * 6.5
    // CurrentYDistance += deltaY;
    // PrevYDistance = CurrentYDistance;

    if (abs(WheelPos) < abs(ydistance) &&
        spinComp > turn_reference - 0.05 &&
        spinComp < turn_reference + 0.05) {
      set_speed = -13;
    } else {
      // Y target reached
      set_speed = 0;
      ydistance = 0;
      turned = false;
      CurrentYDistance = 0;
      PrevYDistance = 0;
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

    xdistance = x;
    ydistance = y;
    turned = false;
    isAutonomous = true;

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
      case 'f': set_speed = -15; currentOperation = 'f'; isTurningClock = false; isTurningAnti = false; break;
      case 'r': set_speed = 15;  currentOperation = 'r'; isTurningClock = false; isTurningAnti = false; break;
      case 'c': if (!isTurningClock) { turn_reference += 1.57; currentOperation = 'c'; isTurningClock = true; isTurningAnti = false; } break;
      case 'a': if (!isTurningAnti)  { turn_reference -= 1.57; currentOperation = 'a'; isTurningAnti = true; isTurningClock = false; } break;
      case 's': set_speed = 0;   currentOperation = 's'; isTurningClock = false; isTurningAnti = false; break;
      case 'q': set_speed = 0.0;
                isTurningClock = false;
                isTurningAnti = false;
                isAutonomous = false;
                CurXCoord = CurYCoord = PrevXCoord = PrevYCoord = 0.0;
                PrevWheelPos = CurrentXDistance = CurrentYDistance = currentSpinAngle = prevspin = 0.0;
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

void loop()
{
  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;

    // Fetch data from MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    //Calculate accelerometer Tilt using sin x = x approximation for a small tilt angle and measure gyroscope tilt
    accelAngle = (a.acceleration.z/9.67) + 0.010;   // greater minus, moves towards R
    spinAngle = (g.gyro.roll) + 0.001;     // on other robot + 0.0721     // greater plus, moves clockwise
    gyroAngle = (g.gyro.pitch);

  
    if (isAutonomous) {
      setco();
    }

    // spinAngleStep = (g.gyro.roll) + 0.076; 
    // currentSpinAngleStep += spinAngleStep;
    // spinRadStep = (currentSpinAngleStep * DEG_TO_RAD)/4;

    WheelPos = step1.getPositionRad();

    // deltaWheel = WheelPos - PrevWheelPos;
    // PrevWheelPos = WheelPos;

    // deltaDistance = (WheelPos / 200.0) * 6.5; // cm per rotation segment
    // Convert spinAngle from degrees to radians
    currentSpinAngle += spinAngle;
    spinRad = (currentSpinAngle * DEG_TO_RAD)/4;

    // Update X and Y using delta
    CurXCoord += WheelPos * cos(spinRad) * 10;
    CurYCoord += WheelPos * sin(spinRad) * 10;


//Speed Control
    current_speed = step1.getSpeedRad();
    speed_error = set_speed - current_speed;
    prev_speed_error = set_speed - prev_speed_error;
    Ps = speed_error * sp;
    Ds = -((speed_error - prev_speed_error) / 0.005) * sd;  
    Is = (speed_error + prev_speed_error) * si * 0.005;

    reference = Ps + Ds + Is - 0.004;

//Balance control

    //complementary sensitivity filter
    theta_n= (1 - c)*(accelAngle) + c*((gyroAngle - 0.02) *0.005 + prev_theta_n);    

    //Turning angle
    spinComp = (spinAngle) * 0.005 + prevspin;      // - 1.00
     
    //errors
    error = reference - theta_n;
    prev_error = reference - prev_theta_n;      
    turn_error = turn_reference - spinComp;
    prev_turn_error = turn_reference - prevspin;

    //Balance controller
    P = error*kp;
    D = -((error - prev_error) / 0.005) * kd;
    I = (error + prev_error) * ki * 0.005;

    prev_theta_n = P + D + I;

    //Turn controller
    Pt = turn_error*tp;
    Dt = ((turn_error-prev_turn_error)/0.005)*td;
    It = (turn_error+prev_turn_error)*ti*0.005;

    Turndrive = Pt + Dt + It;

  //Change acceleration according to PID output
  if((theta_n < 0.02) && (theta_n > -0.02)){ 
  step1.setAccelerationRad(-prev_theta_n - Turndrive);
  step2.setAccelerationRad( prev_theta_n - Turndrive);
  }

  else{
  step1.setAccelerationRad(-prev_theta_n);
  step2.setAccelerationRad( prev_theta_n);
  }
 
  //Keep target speed constant depending on the sign of the PID output
  if(prev_theta_n>0){ 
   step1.setTargetSpeedRad( 15);          // Changed from 20 to 15
   step2.setTargetSpeedRad(-15);         
  }

  else{
   step1.setTargetSpeedRad(-15);
   step2.setTargetSpeedRad( 15);
  }
  
  //Feedback
  prev_theta_n = theta_n;
  prev_speed = current_speed;
  prevspin = spinComp;

  }
  
  
  if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;
    // Serial.print(xdistance); Serial.print(", ");
    // Serial.print(set_speed); Serial.print(", ");
    // // Serial.print(currentSpinAngle/4); Serial.print(",   ");
    // Serial.print(" CurXCoord: "); Serial.print(CurXCoord);
    // Serial.print(" CurYCoord: "); Serial.print(CurYCoord);
    // Serial.print(" Angle: "); Serial.print(spinRad);
    // Serial.println();
    Serial.print("X:"); 
    Serial.print(CurXCoord, 2);   // two decimal places
    Serial.print(",Y:");
    Serial.print(CurYCoord, 2);
    Serial.print(",spinRad:");
    Serial.println(spinRad, 2);
  }

  checkSerialInput();
}