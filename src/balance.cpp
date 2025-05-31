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

const int bufferSize = 32; // Set buffer size to 32 bytes

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

const int PRINT_INTERVAL = 200;
const int LOOP_INTERVAL = 5;
const int STEPPER_INTERVAL_US = 20;
char currentOperation='S';

//PID values
const float kp = 2200;
const float kd = 64;
const float ki = 7;

const float sp = 0.002;
const float sd = 0.00001;
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


//Autonomous control function
void setco(){
  if((CurrentXDistance < xdistance) && (spinComp > turn_reference) - 0.05 && (spinComp < turn_reference + 0.05))
  { 
    set_speed = -13;
    CurrentXDistance = (WheelPos/200)*6.5 + PrevXDistance;
    PrevXDistance = CurrentXDistance;
  }
  //if arrived at x coordinate
  else{
    set_speed = 0;
    xdistance = 0;
  //If there is a y coordinate, turn to face it, else do noting 
  if((ydistance < 0) && !turned){
    turn_reference = turn_reference + 1.57;
    turned = true;
  }
  else if((ydistance > 0) && !turned){
    turn_reference = turn_reference - 1.57;
    turned = true;
  }
  else if (ydistance == 0){
    set_speed = 0;
  }
  //Go to y position
  if((abs(CurrentYDistance) < abs(ydistance)) && (spinComp > turn_reference - 0.05) && (spinComp < turn_reference + 0.05)){      
    set_speed = -13; 
    CurrentYDistance = (WheelPos/200)*6.5 + PrevYDistance;
    PrevYDistance = CurrentYDistance;
  }
 //Arrived at y location
  else if(abs(CurrentYDistance) >= abs(ydistance)){                      
    set_speed = 0;
    ydistance = 0;
    turned = false;
  }
  }
}

#define MAX_CMD_LEN 32
char cmdBuf[MAX_CMD_LEN];
uint8_t cmdPos = 0;

void handleCommand(const char* cmd) {
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
    Serial.print("Received target: X=");
    Serial.print(x);
    Serial.print(" Y=");
    Serial.println(y);
  } else {
    switch (cmd[0]) {
      case 'f': set_speed = -10; isTurningClock = false; isTurningAnti = false; break;
      case 'r': set_speed = 13;  isTurningClock = false; isTurningAnti = false; break;
      case 'c': if (!isTurningClock) { turn_reference += 1.57; isTurningClock = true; isTurningAnti = false; } break;
      case 'a': if (!isTurningAnti) { turn_reference -= 1.57; isTurningAnti = true; isTurningClock = false; } break;
      case 's': set_speed = 0; isTurningClock = false; isTurningAnti = false; break;
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
  //static float i = 0;
  //Run the control loop every LOOP_INTERVAL ms
  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;

    // Fetch data from MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    //Calculate accelerometer Tilt using sin x = x approximation for a small tilt angle and measure gyroscope tilt
    accelAngle = (a.acceleration.z/9.67) - 0.005;   // was - 0.037 
    spinAngle = (g.gyro.roll) + 0.076;     // on other robot + 0.0721
    gyroAngle = (g.gyro.pitch);

    if (isAutonomous) {
      setco();
      isAutonomous = false;
    }

    WheelPos = step1.getPosition();

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
    theta_n= (1 - c)*(accelAngle) + c*((gyroAngle - 0.02) *0.005 + prev_theta_n);       // Theta_n = theta_n also removed the 0.4 gyroAngle+0.4

    //Turning angle
    spinComp = (spinAngle) * 0.005 + prevspin;      // - 1.00
     
    //errors
    turn_error = turn_reference - spinComp;
    error = reference - theta_n;
    prev_error = reference - prev_theta_n;      
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
   step1.setTargetSpeedRad( 15);          // Changed from 20 to 5
   step2.setTargetSpeedRad(-15);          // Also flipped signs between step1 and step2 
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
  
  //Print updates every PRINT_INTERVAL ms
  
  if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;
  }

//  if (Serial.available()>0) {
//     char incomingByte = Serial.read();
//     currentOperation=incomingByte;
//     Serial.print(incomingByte);
//   }
//   switch (currentOperation) {
//     case 'f': // Forward
//       if (set_speed != -10) {
//         // Serial.println("Forward");
//       }
//       set_speed = -10;
//       isTurningClock = false;
//       isTurningAnti = false;
//       spinAngle = 0;
//       break;
//     case 'r': // Reverse
//       if (set_speed != 13) {
//         // Serial.println("Reverse");
//       }
//       set_speed = 13;
//       isTurningClock = false;
//       isTurningAnti = false;
//       spinAngle = 0;
//       break;

//     case 'c': // Clockwise Turn
//       if (!isTurningClock) {
//         // Serial.println("Clockwise");
//         turn_reference += 1.57;
//         isTurningClock = true;
//         isTurningAnti = false;
//       }
//       break;

//     case 'a': // Anti-clockwise Turn
//       if (!isTurningAnti) {
//         // Serial.println("Anti-Clockwise");
//         turn_reference -= 1.57;
//         isTurningAnti = true;
//         isTurningClock = false;
//       }
//       break;
//     case 's': // Stop
//       if (set_speed != 0) {
//         // Serial.println("Stop");
//       }
//       set_speed = 0;
//       isTurningClock = false;
//       isTurningAnti = false;
//       spinAngle = 0;
//       break;
//      case 'x':
//         int x = 0;
//         char position[50];
//         position[0]='X';
//         while(Serial.available()){
//           x++;
//           position[x]=Serial.read();
//         }
//         const char* posX = strchr(position, 'x');
//         const char* posY = strchr(position, 'y');
//         // Extract the substring after 'X' and convert to integer
//         xdistance = atoi(posX + 1);
//         // Extract the substring after 'Y' and convert to integer
//         ydistance = atoi(posY + 1);
//         // setco();
//         break;
//     } 

 checkSerialInput();
}