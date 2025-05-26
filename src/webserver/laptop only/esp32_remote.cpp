#include <Arduino.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include <stdio.h>
#include <stdlib.h>
#include <HardwareSerial.h>
bool isTurning=true;
bool turned = false;
// Power monitoring constants
#define SAMPLES 64 // Multisampling
#define CELLS 12 // Battery cells

const int bufferSize = 32; // Set buffer size to 32 bytes

static unsigned long printTimer = 0;       //time of the next print
  static unsigned long loopTimer = 0;        //time of the next control update
  static float AccelAngle = 0;
  static float SpinAngle = 0;                //current tilt angle
  static float GyroAngle = 0;                //rate of change of tilt angle 
  static float current = 0;
  static float Turndrive = 0;
  static float currentspeed = 0;
  static float WheelPos = 0;
  static float CurrentXDistance, PrevXDistance, CurrentYDistance, PrevYDistance = 0;
  //Errors
  static float prev, prevspeed, prevspin;
  static float error, speederror, turnerror; 
  static float preverror, prevspeederror, prevturnerror;    
  //PID Gains 
  static float P, D, I, Ps, Ds, Is, Pt, Dt, It;
  //Components
  static float GyroComp, AccelComp, SpinComp; 
  

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
const int  STEPPER_INTERVAL_US = 20;
char currentOperation='S';

//PID values
const float kp = 2000;
const float kd = 55;
const float ki = 5;

const float sp = 0.002;
const float sd = 0;
const float si = 0;

const float tp = 10;
const float td = 8;
const float ti = 0;

//time const
const float c = 0.98;

//static
float setpoint;// = 0.0135;
float setspeed = 0;
float setturn = 0;
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
  /*  if((CurrentXDistance < xdistance) && (SpinComp > setturn) - 0.05 && (SpinComp < setturn + 0.05)){      
    setspeed = -13;
    CurrentXDistance = (WheelPos/200)*6.5 + PrevXDistance;
    PrevXDistance = CurrentXDistance;
  }
  //if arrived at x coordinate
  else{
    setspeed = 0;
    xdistance = 0;
  //If there is a y coordinate, turn to face it, else do noting 
  if((ydistance < 0) && !turned){
    setturn = setturn + 1.57;
    turned = true;
  }
  else if((ydistance > 0) && !turned){
    setturn = setturn - 1.57;
    turned = true;
  }
  else if (ydistance == 0){
    setspeed = 0;
  }
  //Go to y position
  if((abs(CurrentYDistance) < abs(ydistance)) && (SpinComp > setturn - 0.05) && (SpinComp < setturn + 0.05)){      
    setspeed = -13; 
    CurrentYDistance = (WheelPos/200)*6.5 + PrevYDistance;
    PrevYDistance = CurrentYDistance;
  }
 //Arrived at y location
  else if(abs(CurrentYDistance) >= abs(ydistance)){                      
    setspeed = 0;
    ydistance = 0;
    turned = false;
  }
  }*/
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
    AccelAngle = (a.acceleration.z/9.67) - 0.017;   // was - 0.037 
    SpinAngle = (g.gyro.roll) + 0.0721;
    GyroAngle = (g.gyro.pitch);

  setco();
  WheelPos = step1.getPosition();

//Speed Control
    currentspeed = step1.getSpeedRad();
    speederror = setspeed - currentspeed;
    prevspeederror = setspeed - prevspeederror;
    Ps = speederror*sp;
    Ds = -((speederror-prevspeederror)/0.005)*sd;
    Is = (speederror+prevspeederror)*si*0.005;

    setpoint = Ps + Ds + Is - 0.004;

//Balance control

    //complementary sensitivity filter
    current = (1 - c)*(AccelAngle) + c*((GyroAngle - 0.02) *0.005 + prev);       // Theta_n = current  also removed the 0.4 GyroAngle+0.4
    // AccelComp = (AccelAngle+0.3);                                             // not Used
    // GyroComp = ((GyroAngle+0.4) *0.005 + prev);

    //Turning angle
    SpinComp = (SpinAngle) * 0.005 + prevspin;      // - 1.00
     
    //errors
    turnerror = setturn - SpinComp;
    error = setpoint - current;
    preverror = setpoint - prev;      
    prevturnerror = setturn - prevspin;


    //Balance controller
    P = error*kp;
    D = -((error-preverror)/0.005)*kd;
    I = (error+preverror)*ki*0.005;

    prev = P + D + I;

    //Turn controller
    Pt = turnerror*tp;
    Dt = ((turnerror-prevturnerror)/0.005)*td;
    It = (turnerror+prevturnerror)*ti*0.005;

    Turndrive = Pt + Dt + It;

  //Change acceleration according to PID output
  if((current < 0.02) && (current > -0.02)){ 
  step1.setAccelerationRad(-prev - Turndrive);
  step2.setAccelerationRad( prev - Turndrive);
  }

  else{
  step1.setAccelerationRad(-prev);
  step2.setAccelerationRad( prev);
  }
 
  //Keep target speed constant depending on the sign of the PID output
  if(prev>0){ 
   step1.setTargetSpeedRad( 15);          // Changed from 20 to 5
   step2.setTargetSpeedRad(-15);          // Also flipped signs between step1 and step2 
  }

  else{
   step1.setTargetSpeedRad(-15);
   step2.setTargetSpeedRad( 15);
  }
  
 
  //Feedback
  prev = current;
  prevspeed = currentspeed;
  // prevspin = SpinComp;

  }
  
  //Print updates every PRINT_INTERVAL ms
  S
  if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;
    Serial.print("Spin Comp: ");
    Serial.print(SpinComp, 4);
    Serial.print(" SpinAngle: ");
    Serial.print(SpinAngle, 4);
    Serial.print(" turnerror: ");
    Serial.print(turnerror, 4);
    Serial.print(" AccelAngle: ");
    Serial.print(AccelAngle, 4);
    Serial.println();
  }
  


 if (Serial.available()>0) {
    char incomingByte = Serial.read();
    currentOperation=incomingByte;
    Serial.print(incomingByte);

  }
 switch  (currentOperation) {
      case 'F':
        isTurning=false;
        Serial.println("Forward");
        setspeed =-17;
        break;
      case 'R':
        isTurning=false;
        //REVERSE
        Serial.println("Reverse");
        setspeed=13;
        break;
      case 'C':
        //CLOCKWISE
        Serial.println("Clockwise");
        if (!isTurning)
        {
          setturn = setturn + 1.57;
        }
        isTurning=true;
        break;
      case 'A':
        //ANTICLOCKWISE
        Serial.println("Anti-Clockwise");
        if (!isTurning)
        {
          setturn = setturn - 1.57;
        }
        isTurning=true;
        break;
      case 'S':
        //moving=false;
        isTurning=false;
        //STOP
        setspeed=0;
        Serial.println("Stop");
        break;
     case 'X':
        int x = 0;
        char position[50];
        position[0]='X';
        while(Serial.available()){
          x++;
          position[x]=Serial.read();
        }
        const char* posX = strchr(position, 'X');
        const char* posY = strchr(position, 'Y');
        // Extract the substring after 'X' and convert to integer
        xdistance = atoi(posX + 1);
        // Extract the substring after 'Y' and convert to integer
        ydistance = atoi(posY + 1);
        setco();
        break;
    } 
}