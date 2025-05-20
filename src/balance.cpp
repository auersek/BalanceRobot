#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

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

const int PRINT_INTERVAL    = 500;
const int LOOP_INTERVAL     = 5;
const int STEPPER_INTERVAL_US = 10;

const float kx = 20.0;
const float VREF = 4.096;

//Global objects
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;         //Default pins for I2C are SCL: IO22, SDA: IO21

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

uint16_t readADC(uint8_t channel) {
  uint8_t TXByte0 = 0x06 | (channel >> 2);  // Command Byte 0 = Start bit + single-ended mode + MSB of channel
  uint8_t TXByte1 = (channel & 0x03) << 6;  // Command Byte 1 = Remaining 2 bits of channel

  digitalWrite(ADC_CS_PIN, LOW); 

  SPI.transfer(TXByte0);                    // Send Command Byte 0
  uint8_t RXByte0 = SPI.transfer(TXByte1);      // Send Command Byte 1 and receive high byte of result
  uint8_t RXByte1 = SPI.transfer(0x00);     // Send dummy byte and receive low byte of result

  digitalWrite(ADC_CS_PIN, HIGH); 

  uint16_t result = ((RXByte0 & 0x0F) << 8) | RX1; // Combine high and low byte into 12-bit result
  return result;
}


void setup()
{
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

  //Set motor acceleration values
  step1.setAccelerationRad(15.0);
  step2.setAccelerationRad(15.0);

  //Enable the stepper motor drivers
  pinMode(STEPPER_EN_PIN,OUTPUT);
  digitalWrite(STEPPER_EN_PIN, false);

  //Set up ADC and SPI
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);
}


// //Constants - need to use trial and error to fix them
// float Kp = 1;
// float Ki = 1;
// float Kd = 1;
// float c = 0.9;
// long previous_time = millis();

// void loop(){

//     sensors_event_t a, g, temp;
//     mpu.getEvent(&a, &g, &temp);

//     //approximation for now - probably smarter to change to more accurate angle reading
//     //using the same as base code - sin x = x and approximating the graviation
//     float tiltx = a.acceleration.z/9.67;
//     float tilty = a.acceleration.y/9.67;
//     float tiltz = a.acceleration.x/9.67;

//     //complementary filter

//     float tilt_angle = a.acceleration.x/9.67; //accelerometer
//     float differential = g.gyro.y; //gyroscope

//     unsigned long current_time = millis();
//     float dt = (current_time - previous_time)/1000;
//     previous_time = current_time;

//     float theta_n = (1-c)*tilt_angle + c*(differential*dt+theta_n);
    
//     //PID
//     float reference = 0;

//     float derivative = 
//     float integral = 

//     float error = reference - output;
//     float uoutput = Kp*error + Ki*integral + Kd+derivative;

//     step1.setTargetSpeedRad(uoutput);
//     step2.setTargetSpeedRad(-uoutput);

//     Serial.print(tilt_angle);
//     Serial.print(' ');
//     Serial.print(theta_n);
//     Serial.print(' ');
//     Serial.print(uoutput);





// }



void loop(){
    static float Kp = 100.0;
    static float Ki = 0.0;
    static float Kd = 0.0;
    static float c = 0.9;
    static unsigned long previous_time = millis();
    static unsigned long printTimer = 0;
    static unsigned long loopTimer = 0;
    static float theta_n = 0.0;
    static float integral = 0.0;
    static float uoutput = 0.0;
    static float tilt_angle = 0.0;


    if (millis() > loopTimer){
        loopTimer += LOOP_INTERVAL;
        //complementary function

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        tilt_angle = a.acceleration.x/9.67; //accelerometer
        //tilt_angle = atan2(a.acceleration.x, a.acceleration.z) - 2.1;
        float gyrogyro = g.gyro.y; //gyroscope

        unsigned long current_time = millis();
        float dt = (current_time - previous_time)/1000;
        previous_time = current_time;

        theta_n = (1-c)*tilt_angle + c*(gyrogyro*dt+theta_n);

        //PID

        float reference = 0.02;
        float error = reference - theta_n;

        float derivative = -gyrogyro;
        integral += error*dt;

        uoutput = Kp*error + Ki*integral + Kd*derivative;

        step1.setTargetSpeedRad(uoutput);
        step2.setTargetSpeedRad(-uoutput);
    }

    if (millis() > printTimer){
        printTimer += PRINT_INTERVAL;

        Serial.print(tilt_angle);
        Serial.print(' ');
        Serial.print(theta_n);
        Serial.print(' ');
        Serial.print(uoutput);
        Serial.println();
    }


}